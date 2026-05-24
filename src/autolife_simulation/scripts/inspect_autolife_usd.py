#!/usr/bin/env python3
"""Inspect the Autolife USD before using it in an Isaac Sim or OmniGibson stage.

Run this with Isaac Sim's Python, for example:

    /data/jiaxuanLin/isaacsim/python.sh \
        /data/jiaxuanLin/autolife_ws/src/autolife_simulation/scripts/inspect_autolife_usd.py
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from pathlib import Path
from typing import Any


DEFAULT_USD = Path(
    "/data/jiaxuanLin/autolife_ws/src/asset/usd/autolife/autolife.usd"
)

CONTROLLABLE_JOINTS = [
    "Joint_Ground_Vehicle_X",
    "Joint_Ground_Vehicle_Y",
    "Joint_Ground_Vehicle_Z",
    "Joint_Ankle",
    "Joint_Knee",
    "Joint_Waist_Pitch",
    "Joint_Waist_Yaw",
    "Joint_Left_Shoulder_Inner",
    "Joint_Left_Shoulder_Outer",
    "Joint_Left_UpperArm",
    "Joint_Left_Elbow",
    "Joint_Left_Forearm",
    "Joint_Left_Wrist_Upper",
    "Joint_Left_Wrist_Lower",
    "Joint_Right_Shoulder_Inner",
    "Joint_Right_Shoulder_Outer",
    "Joint_Right_UpperArm",
    "Joint_Right_Elbow",
    "Joint_Right_Forearm",
    "Joint_Right_Wrist_Upper",
    "Joint_Right_Wrist_Lower",
    "Joint_Neck_Roll",
    "Joint_Neck_Pitch",
    "Joint_Neck_Yaw",
    "Joint_Left_Gripper",
    "Joint_Right_Gripper",
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect an existing Autolife USD for simulation readiness."
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD,
        help=f"USD to inspect. Default: {DEFAULT_USD}",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write the full inspection report as JSON.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero if warnings are produced.",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=12,
        help="Maximum sample paths to print for each category.",
    )
    return parser.parse_args()


def _has_api(prim: Any, api_cls: Any, api_name: str) -> bool:
    try:
        if prim.HasAPI(api_cls):
            return True
    except TypeError:
        pass
    return any(api_name in schema for schema in prim.GetAppliedSchemas())


def _attr_value(attr: Any) -> Any:
    if not attr or not attr.IsValid():
        return None
    value = attr.Get()
    try:
        json.dumps(value)
        return value
    except TypeError:
        return str(value)


def _path_list(prims: list[Any], limit: int) -> list[str]:
    return [str(prim.GetPath()) for prim in prims[:limit]]


def _joint_drive_summary(prim: Any) -> dict[str, dict[str, Any]]:
    from pxr import UsdPhysics

    drives = {}
    for drive_name in ("angular", "linear"):
        drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
        if not drive or not drive.GetPrim().IsValid():
            continue
        if not any(
            attr.IsValid()
            for attr in (
                drive.GetStiffnessAttr(),
                drive.GetDampingAttr(),
                drive.GetMaxForceAttr(),
                drive.GetTargetPositionAttr(),
                drive.GetTargetVelocityAttr(),
            )
        ):
            continue
        drives[drive_name] = {
            "stiffness": _attr_value(drive.GetStiffnessAttr()),
            "damping": _attr_value(drive.GetDampingAttr()),
            "max_force": _attr_value(drive.GetMaxForceAttr()),
            "target_position": _attr_value(drive.GetTargetPositionAttr()),
            "target_velocity": _attr_value(drive.GetTargetVelocityAttr()),
        }
    return drives


def _inspect_stage(usd_path: Path, max_samples: int) -> dict[str, Any]:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {usd_path}")

    default_prim = stage.GetDefaultPrim()
    root_layer = stage.GetRootLayer()

    type_counts: Counter[str] = Counter()
    articulation_roots = []
    rigid_bodies = []
    collision_prims = []
    mesh_prims = []
    joints = []
    sensors = []
    instance_proxy_prims = []
    drive_joints = {}

    traversal = Usd.PrimRange.Stage(
        stage, Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
    )
    for prim in traversal:
        if not prim.IsValid():
            continue

        type_name = prim.GetTypeName() or "<untyped>"
        type_counts[type_name] += 1
        prim_name = prim.GetName()
        prim_path = str(prim.GetPath())
        lowered = f"{type_name} {prim_name} {prim_path}".lower()

        if prim.IsInstanceProxy():
            instance_proxy_prims.append(prim)
        if _has_api(prim, UsdPhysics.ArticulationRootAPI, "ArticulationRootAPI"):
            articulation_roots.append(prim)
        if _has_api(prim, UsdPhysics.RigidBodyAPI, "RigidBodyAPI"):
            rigid_bodies.append(prim)
        if _has_api(prim, UsdPhysics.CollisionAPI, "CollisionAPI"):
            collision_prims.append(prim)
        if type_name == "Mesh":
            mesh_prims.append(prim)
        if "Joint" in type_name or prim_name.startswith("Joint_"):
            joints.append(prim)
            drives = _joint_drive_summary(prim)
            if drives:
                drive_joints[prim_name] = {
                    "path": prim_path,
                    "type": type_name,
                    "drives": drives,
                }
        if type_name == "Camera" or any(
            token in lowered for token in ("camera", "lidar", "imu", "sensor")
        ):
            sensors.append(prim)

    joint_by_name = {prim.GetName(): prim for prim in joints}
    missing_controllable_joints = [
        joint for joint in CONTROLLABLE_JOINTS if joint not in joint_by_name
    ]
    controllable_without_drive = [
        joint
        for joint in CONTROLLABLE_JOINTS
        if joint in joint_by_name and joint not in drive_joints
    ]

    warnings = []
    if not default_prim or not default_prim.IsValid():
        warnings.append("USD has no defaultPrim.")
    if not articulation_roots:
        warnings.append("No UsdPhysics.ArticulationRootAPI prim was found.")
    if not rigid_bodies:
        warnings.append("No UsdPhysics.RigidBodyAPI prims were found.")
    if not collision_prims:
        warnings.append("No UsdPhysics.CollisionAPI prims were found.")
    if missing_controllable_joints:
        warnings.append(
            "Missing expected controllable joints: "
            + ", ".join(missing_controllable_joints)
        )
    if controllable_without_drive:
        warnings.append(
            "Expected controllable joints without detected DriveAPI: "
            + ", ".join(controllable_without_drive)
        )
    if not sensors:
        warnings.append("No obvious camera/lidar/imu/sensor prims were detected.")

    root_layer_path = Path(root_layer.realPath) if root_layer.realPath else usd_path
    sublayers = []
    for sublayer in root_layer.subLayerPaths:
        sublayer_path = Path(sublayer)
        if not sublayer_path.is_absolute():
            sublayer_path = root_layer_path.parent / sublayer_path
        sublayers.append(
            {
                "path": str(sublayer_path),
                "exists": sublayer_path.exists(),
            }
        )

    report = {
        "usd": str(usd_path),
        "root_layer": root_layer.identifier,
        "default_prim": str(default_prim.GetPath())
        if default_prim and default_prim.IsValid()
        else None,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "sublayers": sublayers,
        "counts": {
            "prim_types": dict(type_counts.most_common()),
            "articulation_roots": len(articulation_roots),
            "rigid_bodies": len(rigid_bodies),
            "collision_prims": len(collision_prims),
            "mesh_prims": len(mesh_prims),
            "joints": len(joints),
            "drive_joints": len(drive_joints),
            "sensors_or_sensor_named_prims": len(sensors),
            "instance_proxy_prims": len(instance_proxy_prims),
        },
        "samples": {
            "articulation_roots": _path_list(articulation_roots, max_samples),
            "rigid_bodies": _path_list(rigid_bodies, max_samples),
            "collision_prims": _path_list(collision_prims, max_samples),
            "joints": _path_list(joints, max_samples),
            "sensors_or_sensor_named_prims": _path_list(sensors, max_samples),
            "instance_proxy_prims": _path_list(instance_proxy_prims, max_samples),
        },
        "controllable_joints": {
            "expected": CONTROLLABLE_JOINTS,
            "missing": missing_controllable_joints,
            "without_detected_drive": controllable_without_drive,
            "with_detected_drive": [
                joint for joint in CONTROLLABLE_JOINTS if joint in drive_joints
            ],
        },
        "drive_joints": drive_joints,
        "warnings": warnings,
    }

    # Keep Sdf imported so Isaac's Python initializes all relevant USD plugins.
    _ = Sdf
    return report


def _print_report(report: dict[str, Any]) -> None:
    def emit(text: str = "") -> None:
        os.write(1, f"{text}\n".encode("utf-8"))

    emit("\n=== Autolife USD Inspection ===")
    emit(f"USD: {report['usd']}")
    emit(f"defaultPrim: {report['default_prim']}")
    emit(f"upAxis: {report['up_axis']}")
    emit(f"metersPerUnit: {report['meters_per_unit']}")

    emit("\nSublayers:")
    if not report["sublayers"]:
        emit("  none")
    for sublayer in report["sublayers"]:
        status = "ok" if sublayer["exists"] else "missing"
        emit(f"  [{status}] {sublayer['path']}")

    counts = report["counts"]
    emit("\nPhysics and control counts:")
    for key in (
        "articulation_roots",
        "rigid_bodies",
        "collision_prims",
        "mesh_prims",
        "joints",
        "drive_joints",
        "sensors_or_sensor_named_prims",
        "instance_proxy_prims",
    ):
        emit(f"  {key}: {counts[key]}")

    emit("\nControllable joint check:")
    missing = report["controllable_joints"]["missing"]
    without_drive = report["controllable_joints"]["without_detected_drive"]
    emit(f"  expected: {len(report['controllable_joints']['expected'])}")
    emit(f"  missing: {len(missing)}")
    if missing:
        emit(f"    {', '.join(missing)}")
    emit(f"  without detected DriveAPI: {len(without_drive)}")
    if without_drive:
        emit(f"    {', '.join(without_drive)}")

    emit("\nSample paths:")
    for label, paths in report["samples"].items():
        emit(f"  {label}:")
        if not paths:
            emit("    none")
        for path in paths:
            emit(f"    {path}")

    emit("\nTop prim types:")
    for type_name, count in list(report["counts"]["prim_types"].items())[:12]:
        emit(f"  {type_name}: {count}")

    emit("\nWarnings:")
    if not report["warnings"]:
        emit("  none")
    for warning in report["warnings"]:
        emit(f"  - {warning}")
    emit("=== End Autolife USD Inspection ===\n")


def main() -> int:
    args = _parse_args()
    usd_path = args.usd.expanduser().resolve()
    if not usd_path.exists():
        raise FileNotFoundError(usd_path)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        report = _inspect_stage(usd_path, args.max_samples)
        _print_report(report)
        if args.json_out:
            args.json_out.expanduser().resolve().write_text(
                json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
    finally:
        app.close()

    if args.strict and report["warnings"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
