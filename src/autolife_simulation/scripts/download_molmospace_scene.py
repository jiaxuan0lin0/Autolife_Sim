#!/usr/bin/env python3
"""Install the pinned MolmoSpaces scene and its referenced object assets."""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "molmospace_scene.json"
OBJECT_REFERENCE_PATTERN = re.compile(
    rb"objects/(thor|objaverse)/([^@\x00\r\n\t \"'<>()]+)"
)
USD_SUFFIXES = {".usd", ".usda", ".usdc"}


def _default_cache_dir() -> Path:
    configured = os.environ.get("AUTOLIFE_MOLMOSPACE_CACHE")
    if configured:
        return Path(configured).expanduser()
    return WORKSPACE_ROOT / ".cache/molmospaces"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Download the pinned MolmoSpaces scene and only the THOR / Objaverse "
            "archives referenced by that scene."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Pinned scene configuration JSON.",
    )
    parser.add_argument(
        "--asset-root",
        type=Path,
        default=None,
        help=(
            "User-facing MolmoSpaces asset directory. The config's workspace-relative "
            "asset_root is used by default."
        ),
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=_default_cache_dir(),
        help="Versioned download cache; it must be outside --asset-root.",
    )
    return parser.parse_args()


def _load_config(path: Path) -> dict:
    path = path.expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)

    required_sections = {"scene_model", "asset_root", "remote", "scene", "objects", "physics"}
    missing = sorted(required_sections - config.keys())
    if missing:
        raise ValueError(f"Missing MolmoSpaces config fields in {path}: {missing}")
    return config


def _resolve_asset_root(config: dict, override: Path | None) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    configured = Path(config["asset_root"]).expanduser()
    if configured.is_absolute():
        return configured.resolve()
    return (WORKSPACE_ROOT / configured).resolve()


def _find_object_references(scene_dir: Path) -> dict[str, list[Path]]:
    references: dict[str, set[Path]] = {"thor": set(), "objaverse": set()}
    for usd_path in scene_dir.rglob("*"):
        if not usd_path.is_file() or usd_path.suffix.lower() not in USD_SUFFIXES:
            continue
        for match in OBJECT_REFERENCE_PATTERN.finditer(usd_path.read_bytes()):
            source = match.group(1).decode("ascii")
            relative_path = Path(match.group(2).decode("utf-8"))
            references[source].add(relative_path)
    return {source: sorted(paths) for source, paths in references.items()}


def _validate_referenced_files(asset_root: Path, references: dict[str, list[Path]]) -> None:
    missing = [
        asset_root / "objects" / source / relative_path
        for source, paths in references.items()
        for relative_path in paths
        if not (asset_root / "objects" / source / relative_path).is_file()
    ]
    if missing:
        preview = "\n".join(f"  {path}" for path in missing[:20])
        raise RuntimeError(
            f"{len(missing)} referenced object files are missing after installation:\n{preview}"
        )


def _ensure_cache_scene_object_view(cache_dir: Path, config: dict) -> None:
    """Keep USD relative references valid after per-file scene links resolve."""
    scene_object_root = cache_dir / "scenes" / "objects"
    scene_object_root.mkdir(parents=True, exist_ok=True)

    for source, version_key in (
        ("thor", "thor_version"),
        ("objaverse", "objaverse_version"),
    ):
        target = cache_dir / "objects" / source / config["objects"][version_key]
        link = scene_object_root / source
        if link.is_symlink():
            if link.resolve() == target.resolve():
                continue
            raise RuntimeError(
                f"Refusing to replace an unexpected MolmoSpaces cache link: {link}"
            )
        if link.exists():
            raise RuntimeError(
                f"MolmoSpaces cache object view already exists and is not a link: {link}"
            )
        relative_target = Path(os.path.relpath(target, start=link.parent))
        link.symlink_to(relative_target, target_is_directory=True)


def _relativize_asset_links(asset_root: Path, cache_dir: Path) -> int:
    """Make generated links portable when the cache lives inside the repository."""
    cache_dir = cache_dir.resolve()
    converted = 0
    for link in asset_root.rglob("*"):
        if not link.is_symlink():
            continue
        target = link.resolve(strict=False)
        try:
            target.relative_to(cache_dir)
        except ValueError:
            continue
        if not link.readlink().is_absolute():
            continue
        relative_target = Path(os.path.relpath(target, start=link.parent))
        target_is_directory = target.is_dir()
        link.unlink()
        link.symlink_to(relative_target, target_is_directory=target_is_directory)
        converted += 1
    return converted


def _validate_with_openusd(scene_usd: Path) -> None:
    try:
        from pxr import Usd, UsdUtils
    except ImportError:
        print("OpenUSD validation skipped: pxr is not available in this Python environment.")
        return

    stage = Usd.Stage.Open(str(scene_usd), load=Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"OpenUSD failed to open {scene_usd}")
    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(scene_usd))
    if unresolved:
        preview = "\n".join(f"  {path}" for path in unresolved[:20])
        raise RuntimeError(f"OpenUSD found {len(unresolved)} unresolved dependencies:\n{preview}")
    print(f"OpenUSD validation passed: {sum(1 for _ in stage.Traverse())} prims, 0 unresolved dependencies")


def main() -> int:
    args = _parse_args()
    config = _load_config(args.config)
    asset_root = _resolve_asset_root(config, args.asset_root)
    cache_dir = args.cache_dir.expanduser().resolve()

    try:
        from molmospaces_resources import HFRemoteStorage, ResourceManager
        from molmospaces_resources.behaviors import InstallMode
    except ImportError as exc:
        raise RuntimeError(
            "molmospaces-resources is required. Install it with: "
            "python3 -m pip install molmospaces-resources==0.0.1b4"
        ) from exc

    scene_config = config["scene"]
    scene_source = scene_config["source"]
    versions = {
        "objects": {
            "thor": config["objects"]["thor_version"],
            "objaverse": config["objects"]["objaverse_version"],
        },
        "scenes": {scene_source: scene_config["version"]},
    }
    manager = ResourceManager(
        remote_storage=HFRemoteStorage(
            repo_id=config["remote"]["repo_id"],
            repo_prefix=config["remote"]["repo_prefix"],
            token=os.environ.get("HF_TOKEN"),
        ),
        data_type_to_source_to_version=versions,
        symlink_dir=asset_root,
        cache_dir=cache_dir,
        source_overrides={
            # The library normally treats THOR as eager. This scene installer must
            # download only archives that the selected scene actually references.
            ("objects", "thor"): {"install_mode": InstallMode.ON_DEMAND},
        },
        force_install=True,
    )
    manager.setup()

    scene_relative_path = Path(scene_config["directory"]) / scene_config["entrypoint"]
    scene_packages = manager.find_archives("scenes", scene_source, [scene_relative_path])
    manager.install_packages("scenes", {scene_source: scene_packages})

    scene_dir = asset_root / "scenes" / scene_source / scene_config["directory"]
    scene_usd = scene_dir / scene_config["entrypoint"]
    if not scene_usd.is_file():
        raise RuntimeError(f"Scene entrypoint was not installed: {scene_usd}")

    references = _find_object_references(scene_dir)
    object_packages = {
        source: manager.find_archives("objects", source, paths)
        for source, paths in references.items()
        if paths
    }
    manager.install_packages("objects", object_packages)

    # OpenUSD resolves the individual scene-file symlinks to their physical
    # cache paths. Mirror the selected object versions under scenes/objects so
    # the scene's ../../../../objects/... references remain valid there too.
    _ensure_cache_scene_object_view(cache_dir, config)
    converted_links = _relativize_asset_links(asset_root, cache_dir)
    _validate_referenced_files(asset_root, references)
    _validate_with_openusd(scene_usd)

    print(f"Installed scene model: {config['scene_model']}")
    print(f"Scene USD: {scene_usd}")
    for source in ("thor", "objaverse"):
        print(
            f"{source}: {len(references[source])} referenced files in "
            f"{len(object_packages.get(source, []))} archives"
        )
    print(f"Cache directory: {cache_dir}")
    print(f"Portable asset links converted: {converted_links}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
