#!/usr/bin/env python3
"""Load a BEHAVIOR-1K interactive scene and add the Autolife USD robot.

Run from an Isaac Sim / OmniGibson Python environment, for example:

    /data/jiaxuanLin/isaacsim/python.sh \
        /data/jiaxuanLin/autolife_ws/src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py

This script intentionally does not register Autolife as an OmniGibson Robot. It
uses OmniGibson as the scene host and Isaac Sim ROS2 bridge for control.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


DEFAULT_OMNIGIBSON_ROOT = Path("/data/jiaxuanLin/BEHAVIOR-1K/OmniGibson")
DEFAULT_AUTOLIFE_USD = Path("/data/jiaxuanLin/autolife_ws/src/asset/usd/autolife/autolife.usd")
DEFAULT_AUTOLIFE_CONFIG = Path("/data/jiaxuanLin/autolife_ws/src/autolife_simulation/config/autolife.json")
DEFAULT_SENSOR_OVERLAY_USD = Path("/data/jiaxuanLin/autolife_ws/src/asset/usd/world.usd")
DEFAULT_REALSENSE_USD = Path("/data/jiaxuanLin/autolife_ws/src/asset/usd/realsense/d435i/realsense_d435i.usd")
DEFAULT_ROBOT_PRIM_PATH = "/World/autolife"
DEFAULT_GRAPH_PATH = "/ActionGraph"
DEFAULT_SENSOR_GRAPH_PATH = "/AutolifeSensorGraph"
DEFAULT_SENSOR_MANIFEST_PATH = Path("/tmp/autolife_sensor_manifest.json")
DEFAULT_ROS2_JOINT_COMMAND_TOPIC = "/autolife/joint_command"


TUNED_PID_VALUES = {
    "GV_MOTOR_PID": {
        "Kp": [100000.0, 100000.0, 100000.0],
        "Kd": [50000.0, 50000.0, 50000.0],
    },
    "NECK_MOTOR_PID": {
        "Kp": [10000.0, 4.0, 4.0],
        "Kd": [5000.0, 0.0, 0.0],
    },
    "LEG_WAIST_MOTOR_PID": {
        "Kp": [10000.0, 10000.0, 10000.0, 10000.0],
        "Kd": [5000.0, 5000.0, 5000.0, 5000.0],
    },
    "LEFT_ARM_MOTOR_PID": {
        "Kp": [10000.0, 10000.0, 10000.0, 10000.0, 6.0, 6.0, 6.0],
        "Kd": [5000.0, 5000.0, 5000.0, 0.0, 0.0, 0.0, 0.0],
    },
    "RIGHT_ARM_MOTOR_PID": {
        "Kp": [10000.0, 10000.0, 6.0, 10000.0, 6.0, 6.0, 6.0],
        "Kd": [5000.0, 5000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    "LEFT_GRIPPER_MOTOR_PID": {
        "Kp": [20.0],
        "Kd": [1.0],
    },
    "RIGHT_GRIPPER_MOTOR_PID": {
        "Kp": [20.0],
        "Kd": [1.0],
    },
}


PID_JOINT_MAP = {
    "GV_MOTOR_PID": [
        "Joint_Ground_Vehicle_X",
        "Joint_Ground_Vehicle_Y",
        "Joint_Ground_Vehicle_Z",
    ],
    "NECK_MOTOR_PID": [
        "Joint_Neck_Roll",
        "Joint_Neck_Pitch",
        "Joint_Neck_Yaw",
    ],
    "LEG_WAIST_MOTOR_PID": [
        "Joint_Ankle",
        "Joint_Knee",
        "Joint_Waist_Pitch",
        "Joint_Waist_Yaw",
    ],
    "LEFT_ARM_MOTOR_PID": [
        "Joint_Left_Shoulder_Inner",
        "Joint_Left_Shoulder_Outer",
        "Joint_Left_UpperArm",
        "Joint_Left_Elbow",
        "Joint_Left_Forearm",
        "Joint_Left_Wrist_Upper",
        "Joint_Left_Wrist_Lower",
    ],
    "RIGHT_ARM_MOTOR_PID": [
        "Joint_Right_Shoulder_Inner",
        "Joint_Right_Shoulder_Outer",
        "Joint_Right_UpperArm",
        "Joint_Right_Elbow",
        "Joint_Right_Forearm",
        "Joint_Right_Wrist_Upper",
        "Joint_Right_Wrist_Lower",
    ],
    "LEFT_GRIPPER_MOTOR_PID": ["Joint_Left_Gripper"],
    "RIGHT_GRIPPER_MOTOR_PID": ["Joint_Right_Gripper"],
}


SPEED_LIMIT_JOINT_MAP = {
    "GV_SPEED_LIMIT": [
        "Joint_Ground_Vehicle_X",
        "Joint_Ground_Vehicle_Y",
        "Joint_Ground_Vehicle_Z",
    ],
    "NECK_SYNC_SPEED_LIMIT": [
        "Joint_Neck_Roll",
        "Joint_Neck_Pitch",
        "Joint_Neck_Yaw",
    ],
    "LEG_WAIST_SYNC_SPEED_LIMIT": [
        "Joint_Ankle",
        "Joint_Knee",
        "Joint_Waist_Pitch",
        "Joint_Waist_Yaw",
    ],
    "LEFT_ARM_SYNC_SPEED_LIMIT": [
        "Joint_Left_Shoulder_Inner",
        "Joint_Left_Shoulder_Outer",
        "Joint_Left_UpperArm",
        "Joint_Left_Elbow",
        "Joint_Left_Forearm",
        "Joint_Left_Wrist_Upper",
        "Joint_Left_Wrist_Lower",
    ],
    "RIGHT_ARM_SYNC_SPEED_LIMIT": [
        "Joint_Right_Shoulder_Inner",
        "Joint_Right_Shoulder_Outer",
        "Joint_Right_UpperArm",
        "Joint_Right_Elbow",
        "Joint_Right_Forearm",
        "Joint_Right_Wrist_Upper",
        "Joint_Right_Wrist_Lower",
    ],
}


GV_FORCE_LIMITS = {
    "Joint_Ground_Vehicle_X": 200.0,
    "Joint_Ground_Vehicle_Y": 200.0,
    "Joint_Ground_Vehicle_Z": 20.0,
}


GRIPPER_FORCE_LIMITS = {
    "Joint_Left_Gripper": 20.0,
    "Joint_Right_Gripper": 20.0,
}


SENSOR_OVERLAY_SOURCE_ROBOT_PATH = "/World/autolife"
SENSOR_OVERLAY_RELATIVE_ROOTS = [
    (
        "Link_Camera_Gripper_Left/SG8S_AR0820C_5300_G2A_H120YA",
        "Link_Camera_Gripper_Left/SG8S_AR0820C_5300_G2A_H120YA",
    ),
    (
        "Link_Camera_Head_Back/SG8S_AR0820C_5300_G2A_H120YA",
        "Link_Camera_Head_Back/SG8S_AR0820C_5300_G2A_H120YA",
    ),
    ("Link_Camera_Head_Forehead/realsense_d435i", "Link_Camera_Head_Forehead/realsense_d435i"),
    ("Link_Camera_Head_Left_Eye/SG8S_AR0820C_5300_G2A_H120YA", "Link_Camera_Head_Left_Eye/Left_Eye"),
    ("Link_Camera_Head_Right_Eye/Right_Eye", "Link_Camera_Head_Right_Eye/Right_Eye"),
    (
        "Link_Camera_Gripper_Right/SG8S_AR0820C_5300_G2A_H120YA",
        "Link_Camera_Gripper_Right/SG8S_AR0820C_5300_G2A_H120YA",
    ),
    ("Link_IMU/Imu_Sensor", "Link_IMU/Imu_Sensor"),
    ("Link_Lidar_Back/World", "Link_Lidar_Back/World"),
    ("Link_Lidar_Front/World", "Link_Lidar_Front/World"),
]
REALSENSE_SENSOR_RELATIVE_ROOT = "Link_Camera_Head_Forehead/realsense_d435i"
SENSOR_TYPE_NAMES = {"Camera", "IsaacImuSensor", "OmniLidar"}


def _emit(message: str) -> None:
    os.write(1, f"{message}\n".encode("utf-8"))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Load Wainscott_0_int in OmniGibson and add the Autolife USD robot."
    )
    parser.add_argument("--omnigibson-root", type=Path, default=DEFAULT_OMNIGIBSON_ROOT)
    parser.add_argument("--scene-model", default="Wainscott_0_int")
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_AUTOLIFE_USD)
    parser.add_argument("--autolife-config", type=Path, default=DEFAULT_AUTOLIFE_CONFIG)
    parser.add_argument("--sensor-overlay-usd", type=Path, default=DEFAULT_SENSOR_OVERLAY_USD)
    parser.add_argument("--realsense-usd", type=Path, default=DEFAULT_REALSENSE_USD)
    parser.add_argument("--robot-prim-path", default=DEFAULT_ROBOT_PRIM_PATH)
    parser.add_argument("--graph-path", default=DEFAULT_GRAPH_PATH)
    parser.add_argument("--sensor-graph-path", default=DEFAULT_SENSOR_GRAPH_PATH)
    parser.add_argument(
        "--sensor-manifest-path",
        type=Path,
        default=DEFAULT_SENSOR_MANIFEST_PATH,
        help="Runtime JSON manifest written after creating the ROS2 sensor ActionGraph.",
    )
    parser.add_argument("--ros2-joint-command-topic", default=DEFAULT_ROS2_JOINT_COMMAND_TOPIC)
    parser.add_argument(
        "--robot-position",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.05],
        metavar=("X", "Y", "Z"),
        help="Initial robot root translation in world coordinates.",
    )
    parser.add_argument(
        "--robot-orientation-wxyz",
        nargs=4,
        type=float,
        default=[1.0, 0.0, 0.0, 0.0],
        metavar=("W", "X", "Y", "Z"),
        help="Initial robot root orientation as a USD quaternion.",
    )
    parser.add_argument(
        "--quick-load",
        action="store_true",
        help="Only load scene structure categories for faster smoke tests.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Launch OmniGibson without the GUI viewport.",
    )
    parser.add_argument(
        "--no-ros2-bridge",
        action="store_true",
        help="Skip creating the ROS2 joint state / command ActionGraph.",
    )
    parser.add_argument(
        "--no-ros2-sensors",
        action="store_true",
        help="Skip creating the ROS2 sensor ActionGraph.",
    )
    parser.add_argument(
        "--no-apply-drive-config",
        action="store_true",
        help="Do not override Autolife USD drive gains, force limits, and speed limits after import.",
    )
    parser.add_argument(
        "--no-sensor-overlay",
        action="store_true",
        help="Do not copy the sensor prims authored in the original Autolife world.usd.",
    )
    parser.add_argument(
        "--no-eye-camera-roll-correction",
        action="store_true",
        help="Deprecated no-op. Eye camera orientation is copied from --sensor-overlay-usd.",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=640,
        help="ROS2 camera render product width in pixels.",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=480,
        help="ROS2 camera render product height in pixels.",
    )
    parser.add_argument(
        "--sensor-frame-skip",
        type=int,
        default=0,
        help="Number of simulation frames to skip between sensor ROS2 messages.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=-1,
        help="Number of simulation steps to run. Use -1 to run until Ctrl+C.",
    )
    return parser.parse_args()


def _add_local_omnigibson_to_path(omnigibson_root: Path) -> None:
    omnigibson_root = omnigibson_root.expanduser().resolve()
    if not (omnigibson_root / "omnigibson").is_dir():
        raise FileNotFoundError(f"OmniGibson package directory not found under {omnigibson_root}")
    sys.path.insert(0, str(omnigibson_root))


def _enable_ros2_bridge_extensions() -> None:
    from isaacsim.core.utils.extensions import enable_extension

    for extension_name in (
        "omni.graph.action",
        "isaacsim.core.nodes",
        "isaacsim.sensors.physics",
        "isaacsim.sensors.rtx",
        "isaacsim.ros2.bridge",
    ):
        enable_extension(extension_name)


def _assert_ros2_bridge_env() -> None:
    ros_distro = os.environ.get("ROS_DISTRO")
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")

    conda_prefix = Path(sys.prefix)
    internal_jazzy_lib = (
        conda_prefix
        / "lib"
        / f"python{sys.version_info.major}.{sys.version_info.minor}"
        / "site-packages"
        / "isaacsim"
        / "exts"
        / "isaacsim.ros2.bridge"
        / "jazzy"
        / "lib"
    )

    if ros_distro and not os.environ.get("RMW_IMPLEMENTATION"):
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    if ros_distro:
        expected_paths = [f"/opt/ros/{ros_distro}/lib"]
        internal_lib = internal_jazzy_lib if ros_distro == "jazzy" else None
        if internal_lib is not None:
            expected_paths.append(str(internal_lib))
        if any(path in ld_library_path for path in expected_paths):
            return

    raise RuntimeError(
        "ROS2 bridge environment is not configured before Isaac Sim startup.\n"
        "Use one of these before running this script with ROS2 bridge enabled:\n\n"
        "  source /opt/ros/jazzy/setup.bash\n\n"
        "or, for Isaac Sim's bundled Jazzy libraries:\n\n"
        "  export ROS_DISTRO=jazzy\n"
        "  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
        f"  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{internal_jazzy_lib}\n\n"
        "For a scene-only smoke test, pass --no-ros2-bridge."
    )


def _set_root_pose(stage, prim_path: str, position, orientation_wxyz) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Robot prim does not exist: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    prop_names = prim.GetPropertyNames()

    if "xformOp:translate" not in prop_names:
        translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    else:
        translate_op = UsdGeom.XformOp(prim.GetAttribute("xformOp:translate"))

    if "xformOp:orient" not in prop_names:
        orient_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    else:
        orient_op = UsdGeom.XformOp(prim.GetAttribute("xformOp:orient"))

    if "xformOp:scale" not in prop_names:
        scale_op = xformable.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble)
        scale_op.Set(Gf.Vec3d(1.0, 1.0, 1.0))
    else:
        scale_op = UsdGeom.XformOp(prim.GetAttribute("xformOp:scale"))

    xformable.SetXformOpOrder([translate_op, orient_op, scale_op])
    translate_op.Set(Gf.Vec3d(*position))
    orient_op.Set(Gf.Quatd(orientation_wxyz[0], Gf.Vec3d(*orientation_wxyz[1:])))


def _find_robot_prim_by_name(stage, robot_prim_path: str, name: str):
    from pxr import Usd

    root = stage.GetPrimAtPath(robot_prim_path)
    if not root.IsValid():
        raise RuntimeError(f"Robot prim does not exist: {robot_prim_path}")

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim

    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        if prim_path == robot_prim_path or prim_path.startswith(f"{robot_prim_path}/"):
            if prim.GetName() == name:
                return prim

    raise RuntimeError(f"Cannot find Autolife joint prim under {robot_prim_path}: {name}")


def _get_drive_name(prim) -> str:
    from pxr import UsdPhysics

    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "linear"
    if prim.IsA(UsdPhysics.RevoluteJoint):
        return "angular"
    raise RuntimeError(f"{prim.GetPath()} is not a prismatic or revolute joint.")


def _apply_pd(stage, robot_prim_path: str) -> int:
    from pxr import UsdPhysics

    count = 0
    for group_name, joint_names in PID_JOINT_MAP.items():
        kp_values = TUNED_PID_VALUES[group_name]["Kp"]
        kd_values = TUNED_PID_VALUES[group_name]["Kd"]
        if len(joint_names) != len(kp_values) or len(joint_names) != len(kd_values):
            raise RuntimeError(f"{group_name}: PID value count does not match joint count")

        for joint_name, kp, kd in zip(joint_names, kp_values, kd_values):
            prim = _find_robot_prim_by_name(stage, robot_prim_path, joint_name)
            drive_name = _get_drive_name(prim)
            drive_api = UsdPhysics.DriveAPI.Get(prim, drive_name)
            if not drive_api:
                drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_name)
            drive_api.CreateStiffnessAttr(float(kp), True)
            drive_api.CreateDampingAttr(float(kd), True)
            count += 1

    return count


def _apply_force_limits(stage, robot_prim_path: str) -> int:
    from pxr import UsdPhysics

    count = 0
    for joint_name, max_force in {**GV_FORCE_LIMITS, **GRIPPER_FORCE_LIMITS}.items():
        prim = _find_robot_prim_by_name(stage, robot_prim_path, joint_name)
        drive_name = _get_drive_name(prim)
        drive_api = UsdPhysics.DriveAPI.Get(prim, drive_name)
        if not drive_api:
            drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_name)
        drive_api.CreateMaxForceAttr(float(max_force), True)
        count += 1

    return count


def _load_speed_limit_values(config_path: Path) -> dict[str, list[float]]:
    config_path = config_path.expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)

    return {
        "GV_SPEED_LIMIT": [
            cfg["GV_SPEED_LIMIT"][0],
            cfg["GV_SPEED_LIMIT"][1],
            cfg["GV_SPEED_LIMIT"][2] * 180.0 / 3.141592653589793,
        ],
        "NECK_SYNC_SPEED_LIMIT": cfg["NECK_SYNC_SPEED_LIMIT"],
        "LEG_WAIST_SYNC_SPEED_LIMIT": cfg["LEG_WAIST_SYNC_SPEED_LIMIT"],
        "LEFT_ARM_SYNC_SPEED_LIMIT": cfg["ARM_SYNC_SPEED_LIMIT"],
        "RIGHT_ARM_SYNC_SPEED_LIMIT": cfg["ARM_SYNC_SPEED_LIMIT"],
    }


def _apply_speed_limits(stage, robot_prim_path: str, config_path: Path) -> int:
    from pxr import PhysxSchema

    speed_limit_values = _load_speed_limit_values(config_path)
    count = 0

    for group_name, joint_names in SPEED_LIMIT_JOINT_MAP.items():
        limits = speed_limit_values[group_name]
        if len(joint_names) != len(limits):
            raise RuntimeError(f"{group_name}: speed limit count does not match joint count")
        for joint_name, limit in zip(joint_names, limits):
            prim = _find_robot_prim_by_name(stage, robot_prim_path, joint_name)
            joint_api = PhysxSchema.PhysxJointAPI.Get(stage, prim.GetPath())
            if not joint_api:
                joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
            joint_api.CreateMaxJointVelocityAttr(float(limit), True)
            count += 1

    return count


def _apply_autolife_drive_config(stage, robot_prim_path: str, config_path: Path) -> dict[str, int]:
    return {
        "pd": _apply_pd(stage, robot_prim_path),
        "max_force": _apply_force_limits(stage, robot_prim_path),
        "speed_limit": _apply_speed_limits(stage, robot_prim_path, config_path),
    }


def _ensure_parent_spec(layer, prim_path) -> None:
    from pxr import Sdf

    if layer.GetPrimAtPath(prim_path) is None:
        Sdf.CreatePrimInLayer(layer, prim_path)


def _count_typed_sensors(stage, robot_prim_path: str) -> dict[str, int]:
    from pxr import Usd

    counts = {type_name: 0 for type_name in sorted(SENSOR_TYPE_NAMES)}
    root = stage.GetPrimAtPath(robot_prim_path)
    if not root.IsValid():
        return counts

    predicate = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
    for prim in Usd.PrimRange.Stage(stage, predicate):
        prim_path = str(prim.GetPath())
        if prim_path != robot_prim_path and not prim_path.startswith(f"{robot_prim_path}/"):
            continue
        type_name = prim.GetTypeName()
        if type_name in counts:
            counts[type_name] += 1

    return counts


def _copy_sensor_overlay(
    stage,
    sensor_overlay_usd: Path,
    realsense_usd: Path,
    robot_prim_path: str,
) -> dict[str, int]:
    from pxr import Sdf, Usd

    sensor_overlay_usd = sensor_overlay_usd.expanduser().resolve()
    if not sensor_overlay_usd.exists():
        raise FileNotFoundError(sensor_overlay_usd)
    realsense_usd = realsense_usd.expanduser().resolve()
    if not realsense_usd.exists():
        raise FileNotFoundError(realsense_usd)

    source_stage = Usd.Stage.Open(str(sensor_overlay_usd))
    if source_stage is None:
        raise RuntimeError(f"Failed to open sensor overlay USD: {sensor_overlay_usd}")
    source_layer = source_stage.GetRootLayer()
    target_layer = stage.GetEditTarget().GetLayer()

    copied = 0
    for source_relative_path, target_relative_path in SENSOR_OVERLAY_RELATIVE_ROOTS:
        source_path = Sdf.Path(f"{SENSOR_OVERLAY_SOURCE_ROBOT_PATH}/{source_relative_path}")
        target_path = Sdf.Path(f"{robot_prim_path}/{target_relative_path}")
        if source_layer.GetPrimAtPath(source_path) is None:
            raise RuntimeError(f"Sensor overlay source prim is missing: {source_path}")

        _ensure_parent_spec(target_layer, target_path.GetParentPath())
        if not Sdf.CopySpec(source_layer, source_path, target_layer, target_path):
            raise RuntimeError(f"Failed to copy sensor overlay prim: {source_path} -> {target_path}")

        if source_relative_path == REALSENSE_SENSOR_RELATIVE_ROOT:
            target_prim = stage.GetPrimAtPath(target_path)
            if not target_prim.IsValid():
                target_prim = stage.OverridePrim(target_path)
            references = target_prim.GetReferences()
            references.ClearReferences()
            references.AddReference(str(realsense_usd))
        copied += 1

    sensor_counts = _count_typed_sensors(stage, robot_prim_path)
    sensor_counts["subtrees"] = copied
    return sensor_counts


def _add_autolife_usd(og, robot_usd: Path, robot_prim_path: str, position, orientation_wxyz) -> str:
    from omnigibson.utils.usd_utils import add_asset_to_stage

    robot_usd = robot_usd.expanduser().resolve()
    if not robot_usd.exists():
        raise FileNotFoundError(robot_usd)

    stage = og.sim.stage
    if stage.GetPrimAtPath(robot_prim_path).IsValid():
        with og.sim.editing_usd():
            stage.RemovePrim(robot_prim_path)

    add_asset_to_stage(asset_path=str(robot_usd), prim_path=robot_prim_path)
    with og.sim.editing_usd():
        _set_root_pose(stage, robot_prim_path, position, orientation_wxyz)

    root_joint_path = f"{robot_prim_path}/root_joint"
    if not stage.GetPrimAtPath(root_joint_path).IsValid():
        raise RuntimeError(f"Autolife articulation root was not found at {root_joint_path}")
    return root_joint_path


def _setup_ros2_joint_bridge(og, graph_path: str, robot_path: str, command_topic: str) -> None:
    script_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(script_dir))

    from joint_state_bridge import setup_joint_state_bridge

    with og.sim.editing_usd():
        if og.sim.stage.GetPrimAtPath(graph_path).IsValid():
            og.sim.stage.RemovePrim(graph_path)
        setup_joint_state_bridge(
            graph_path=graph_path,
            robot_path=robot_path,
            joint_command_topic=command_topic,
        )


def _setup_ros2_sensor_bridge(
    og,
    graph_path: str,
    robot_prim_path: str,
    robot_articulation_path: str,
    camera_width: int,
    camera_height: int,
    frame_skip_count: int,
    manifest_path: Path,
) -> dict[str, int]:
    script_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(script_dir))

    from sensor_bridge import setup_sensor_bridge

    with og.sim.editing_usd():
        if og.sim.stage.GetPrimAtPath(graph_path).IsValid():
            og.sim.stage.RemovePrim(graph_path)
        return setup_sensor_bridge(
            stage=og.sim.stage,
            graph_path=graph_path,
            robot_prim_path=robot_prim_path,
            robot_articulation_path=robot_articulation_path,
            camera_width=camera_width,
            camera_height=camera_height,
            frame_skip_count=frame_skip_count,
            manifest_path=manifest_path,
        )


def main() -> int:
    args = _parse_args()
    _add_local_omnigibson_to_path(args.omnigibson_root)

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "True"
    if not args.no_ros2_bridge:
        _assert_ros2_bridge_env()

    import omnigibson as og
    from omnigibson.macros import gm

    if args.quick_load:
        from omnigibson.utils.constants import STRUCTURE_CATEGORIES

    cfg = {
        "scene": {
            "type": "InteractiveTraversableScene",
            "scene_model": args.scene_model,
        },
    }
    if args.quick_load:
        cfg["scene"]["load_object_categories"] = list(STRUCTURE_CATEGORIES)

    _emit(f"Loading OmniGibson scene: {args.scene_model}")
    env = og.Environment(configs=cfg)

    # Add the external USD while simulation is stopped, then play again after
    # PhysX has the articulation and bridge graph in the stage.
    og.sim.stop()
    root_joint_path = _add_autolife_usd(
        og=og,
        robot_usd=args.robot_usd,
        robot_prim_path=args.robot_prim_path,
        position=args.robot_position,
        orientation_wxyz=args.robot_orientation_wxyz,
    )
    _emit(f"Added Autolife USD at {args.robot_prim_path}")
    _emit(f"Autolife articulation root: {root_joint_path}")

    if not args.no_sensor_overlay:
        with og.sim.editing_usd():
            sensor_counts = _copy_sensor_overlay(
                stage=og.sim.stage,
                sensor_overlay_usd=args.sensor_overlay_usd,
                realsense_usd=args.realsense_usd,
                robot_prim_path=args.robot_prim_path,
            )
        _emit(
            "Copied Autolife sensor overlay: "
            f"subtrees={sensor_counts['subtrees']}, "
            f"Camera={sensor_counts['Camera']}, "
            f"IsaacImuSensor={sensor_counts['IsaacImuSensor']}, "
            f"OmniLidar={sensor_counts['OmniLidar']}"
        )

    if not args.no_apply_drive_config:
        with og.sim.editing_usd():
            drive_counts = _apply_autolife_drive_config(
                stage=og.sim.stage,
                robot_prim_path=args.robot_prim_path,
                config_path=args.autolife_config,
            )
        _emit(
            "Applied Autolife drive config: "
            f"PD={drive_counts['pd']}, "
            f"maxForce={drive_counts['max_force']}, "
            f"speedLimit={drive_counts['speed_limit']}"
        )

    if not args.no_ros2_bridge:
        usd_guard_was_enabled = getattr(og.sim, "_usd_guard_enabled", False)
        if usd_guard_was_enabled:
            og.sim._disable_usd_guard()
        try:
            _enable_ros2_bridge_extensions()
            # Let newly enabled extensions register their OmniGraph node types.
            if og.app is not None:
                og.app.update()
        finally:
            if usd_guard_was_enabled:
                og.sim._enable_usd_guard()

        _setup_ros2_joint_bridge(
            og=og,
            graph_path=args.graph_path,
            robot_path=root_joint_path,
            command_topic=args.ros2_joint_command_topic,
        )
        _emit(
            f"Created ROS2 ActionGraph at {args.graph_path}: "
            f"publishing /joint_states and subscribing {args.ros2_joint_command_topic}"
        )

        if not args.no_ros2_sensors:
            sensor_graph_counts = _setup_ros2_sensor_bridge(
                og=og,
                graph_path=args.sensor_graph_path,
                robot_prim_path=args.robot_prim_path,
                robot_articulation_path=root_joint_path,
                camera_width=args.camera_width,
                camera_height=args.camera_height,
                frame_skip_count=args.sensor_frame_skip,
                manifest_path=args.sensor_manifest_path,
            )
            _emit(
                f"Created ROS2 sensor graph at {args.sensor_graph_path}: "
                f"cameras={sensor_graph_counts['cameras']} "
                f"cameraOutputs={sensor_graph_counts['camera_outputs']} "
                f"imus={sensor_graph_counts['imus']} "
                f"lidars={sensor_graph_counts['lidars']} "
                f"lidarOutputs={sensor_graph_counts['lidar_outputs']} "
                f"tfTargets={sensor_graph_counts['tf_targets']}"
            )
            _emit(f"Wrote ROS2 sensor manifest: {args.sensor_manifest_path}")

    og.sim.play()

    if not gm.HEADLESS:
        og.sim.enable_viewer_camera_teleoperation()

    _emit("Simulation is running. Press Ctrl+C to exit.")
    step = 0
    try:
        while args.steps < 0 or step < args.steps:
            og.sim.step()
            step += 1
    except KeyboardInterrupt:
        _emit("Stopping simulation.")
    finally:
        og.shutdown()

    # og.shutdown exits in normal standalone usage, but return 0 for callers
    # that embed this script.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
