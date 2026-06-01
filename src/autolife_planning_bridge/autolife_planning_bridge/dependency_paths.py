"""Runtime path setup for the standalone Autolife-Planning package."""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
from pathlib import Path


DEFAULT_PLANNER_ROOT = "/data/jiaxuanLin/Autolife-Planning"
DEFAULT_PLANNER_PYTHON_SITE = (
    "/home/sutai/home/envs/autolife-planning/lib/python3.12/site-packages"
)


def ensure_autolife_planning_importable() -> None:
    """Expose editable Autolife-Planning sources and compiled extensions.

    ROS 2 entry points run with /usr/bin/python3. Adding conda site-packages
    through PYTHONPATH is not enough because .pth files are not processed there.
    The editable install hook is imported explicitly so compiled modules such as
    autolife_planning._ompl_vamp are visible from the source package.
    """

    planner_root = os.environ.get("AUTOLIFE_PLANNING_ROOT", DEFAULT_PLANNER_ROOT)
    planner_site = os.environ.get(
        "AUTOLIFE_PLANNING_PYTHON_SITE",
        DEFAULT_PLANNER_PYTHON_SITE,
    )
    cmeel_site = str(
        Path(planner_site) / "cmeel.prefix" / "lib" / "python3.12" / "site-packages"
    )

    _prepend_sys_path([planner_site, cmeel_site, planner_root])

    if importlib.util.find_spec("_autolife_planning_editable") is not None:
        importlib.import_module("_autolife_planning_editable")

    _extend_loaded_package_path(planner_site)


def _prepend_sys_path(paths: list[str]) -> None:
    existing = [path for path in sys.path if path not in paths]
    valid = [path for path in paths if path and Path(path).exists()]
    sys.path[:] = valid + existing


def _extend_loaded_package_path(planner_site: str) -> None:
    package = sys.modules.get("autolife_planning")
    if package is None or not hasattr(package, "__path__"):
        return
    extension_package_path = str(Path(planner_site) / "autolife_planning")
    if Path(extension_package_path).exists() and extension_package_path not in package.__path__:
        package.__path__.insert(0, extension_package_path)
