#!/usr/bin/env python3
"""
Start Autolife simulation with a BEHAVIOR or MolmoSpaces scene.

BEHAVIOR scenes use OmniGibson. MolmoSpaces scenes run directly in Isaac Sim
without importing OmniGibson. Both paths normally run in the unified
``autolife_sim`` Conda environment:

    source scripts/activate_autolife_sim.sh
    python src/autolife_simulation/scripts/start_autolife_simulation.py

The adjacent standalone Isaac Sim ``python.sh`` remains a compatibility
fallback for direct-USD scenes.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_BEHAVIOR_ROOT = Path(
    os.environ.get("AUTOLIFE_BEHAVIOR_ROOT", WORKSPACE_ROOT / ".deps/BEHAVIOR-1K")
)
DEFAULT_OMNIGIBSON_ROOT = DEFAULT_BEHAVIOR_ROOT / "OmniGibson"
DEFAULT_MOLMOSPACE_CONFIG = Path(__file__).resolve().parents[1] / "config" / "molmospace_scene.json"
DEFAULT_DESK_CONFIG = Path(__file__).resolve().parents[1] / "config" / "desk.json"
DEFAULT_AUTOLIFE_USD = WORKSPACE_ROOT / "src/asset/usd/autolife/autolife.usd"
DEFAULT_AUTOLIFE_CONFIG = WORKSPACE_ROOT / "src/autolife_simulation/config/autolife.json"
DEFAULT_SENSOR_OVERLAY_USD = WORKSPACE_ROOT / "src/asset/usd/world.usd"
DEFAULT_DRIVE_PROFILE_USD = WORKSPACE_ROOT / "src/asset/usd/world.usd"
DEFAULT_REALSENSE_USD = WORKSPACE_ROOT / "src/asset/usd/realsense/d435i/realsense_d435i.usd"
DEFAULT_ROBOT_POSITION = (0.0, 0.0, 0.05)
DEFAULT_ROBOT_ORIENTATION_WXYZ = (1.0, 0.0, 0.0, 0.0)
DEFAULT_LIGHTWHEEL_MICROWAVE_USD = (
    WORKSPACE_ROOT
    / ".deps/Lightwheel_OpenSource/Manipulation/Microwave035/Microwave035.usd"
)
DEFAULT_ROBOT_PRIM_PATH = "/World/autolife"
DEFAULT_LIGHTWHEEL_MICROWAVE_PRIM_PATH = "/World/lightwheel_microwave_035"
DEFAULT_LIGHTWHEEL_MICROWAVE_POSITION = (0, 0.0, 0.18)
DEFAULT_LIGHTWHEEL_MICROWAVE_ORIENTATION_WXYZ = (1.0, 0.0, 0.0, 0.0)
DEFAULT_GRAPH_PATH = "/ActionGraph"
DEFAULT_SENSOR_GRAPH_PATH = "/AutolifeSensorGraph"
DEFAULT_SENSOR_MANIFEST_PATH = WORKSPACE_ROOT / ".cache/runtime/autolife_sensor_manifest.json"
DEFAULT_ROS2_JOINT_COMMAND_TOPIC = "/autolife/joint_command"
MOLMOSPACE_SCENE_MODEL = "molmospace_scene"
DESK_SCENE_MODEL = "desk"
DIRECT_ISAAC_SCENE_MODELS = {MOLMOSPACE_SCENE_MODEL, DESK_SCENE_MODEL}
DEFAULT_PHYSX_MOUSE_PICKING_FORCE = 10.0
ISAAC_SIM_PYTHON_ENV = "AUTOLIFE_ISAAC_SIM_PYTHON"

BASE_JOINT_NAMES = (
    "Joint_Ground_Vehicle_X",
    "Joint_Ground_Vehicle_Y",
    "Joint_Ground_Vehicle_Z",
)

# The main joint Drive values are copied from the previous BEHAVIOR world.usd.
# Keep the session overlay off those Drive attributes so Property/Gain Tuner
# edits take effect immediately.  Only solver and mimic-joint stability values
# are overlaid in the disposable session layer.
DEFAULT_ROBOT_MOUSE_INTERACTION = {
    "enabled": True,
    "capture_base_target_during_drag": True,
    # The exported five-bar fingers use compliant PhysX mimic joints.  Their
    # source damping ratio is 0.005, so a viewport impulse rings visibly even
    # when the master gripper drive is well damped.
    "minimum_mimic_joint_damping_ratio": 1.0,
    "minimum_solver_velocity_iteration_count": 4,
    "minimum_drive_damping": {
        "Joint_Left_Elbow": 50.0,
        "Joint_Right_UpperArm": 2.0,
        "Joint_Right_Elbow": 50.0,
        "Joint_Left_Forearm": 2.0,
        "Joint_Right_Forearm": 2.0,
        "Joint_Left_Wrist_Upper": 2.0,
        "Joint_Left_Wrist_Lower": 2.0,
        "Joint_Right_Wrist_Upper": 2.0,
        "Joint_Right_Wrist_Lower": 2.0,
        "Joint_Left_Gripper": 5.0,
        "Joint_Right_Gripper": 5.0,
    },
}


DEFAULT_MOLMOSPACE_ARTICULATION_STABILIZATION = {
    "enabled": True,
    "joint_friction": 0.05,
    "revolute": {
        "damping": 0.08,
        "max_force": 6.0,
        "max_velocity": 120.0,
    },
    "prismatic": {
        "damping": 6.0,
        "max_force": 20.0,
        "max_velocity": 0.5,
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


def _find_isaac_sim_python() -> Path | None:
    """Find an Isaac Sim python.sh without importing any Kit modules."""
    candidates = []
    configured = os.environ.get(ISAAC_SIM_PYTHON_ENV)
    if configured:
        candidates.append(Path(configured).expanduser())

    for environment_name in ("ISAAC_PATH", "ISAACSIM_PATH"):
        install_root = os.environ.get(environment_name)
        if install_root:
            candidates.append(Path(install_root).expanduser() / "python.sh")

    candidates.extend(
        (
            WORKSPACE_ROOT / ".deps/isaacsim/python.sh",
        )
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    return None


def _resolve_isaac_sim_experience(configured: Path | None, headless: bool) -> Path | None:
    """Resolve an explicitly requested Kit experience; base.python is the safe default."""
    if configured is None:
        return None
    experience = configured.expanduser().resolve()
    if not experience.is_file():
        raise FileNotFoundError(f"Isaac Sim Kit experience does not exist: {experience}")
    return experience


def _enable_isaac_sim_editor_extensions(simulation_app) -> tuple[str, ...]:
    """Add the joint-property editor and Gain Tuner to the stable Python GUI."""
    import omni.kit.app

    extension_ids = (
        "omni.usdphysics.ui",
        "omni.physx.ui",
        "isaacsim.gui.property",
        "isaacsim.robot_setup.gain_tuner",
    )
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    newly_enabled = []
    for extension_id in extension_ids:
        if extension_manager.is_extension_enabled(extension_id):
            continue
        if not extension_manager.set_extension_enabled_immediate(extension_id, True):
            raise RuntimeError(f"Failed to enable Isaac Sim editor extension: {extension_id}")
        newly_enabled.append(extension_id)
    simulation_app.update()
    return tuple(newly_enabled)


def _ensure_isaac_sim_python() -> None:
    """Re-exec MolmoSpaces with Isaac Sim's interpreter when necessary."""
    if importlib.util.find_spec("isaacsim") is not None:
        return

    isaac_sim_python = _find_isaac_sim_python()
    if isaac_sim_python is None:
        raise ModuleNotFoundError(
            "The current Python cannot import 'isaacsim', and no Isaac Sim python.sh "
            f"was found. Set {ISAAC_SIM_PYTHON_ENV}=/path/to/isaacsim/python.sh or "
            "run this script with that python.sh explicitly."
        )

    script_path = Path(__file__).resolve()
    _emit(
        "Current Python cannot import Isaac Sim; restarting with "
        f"{isaac_sim_python}"
    )
    os.execv(
        str(isaac_sim_python),
        [str(isaac_sim_python), str(script_path), *sys.argv[1:]],
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start Autolife simulation with a BEHAVIOR or pinned MolmoSpaces scene."
    )
    parser.add_argument("--omnigibson-root", type=Path, default=DEFAULT_OMNIGIBSON_ROOT)
    parser.add_argument(
        "--scene-model",
        default="Wainscott_0_int",
        help=(
            "BEHAVIOR model name, 'molmospace_scene' for the pinned furnished "
            "home, or 'desk' for the independent single-table scene."
        ),
    )
    parser.add_argument(
        "--molmospace-config",
        type=Path,
        default=DEFAULT_MOLMOSPACE_CONFIG,
        help="Pinned MolmoSpaces scene, asset, and physics configuration.",
    )
    parser.add_argument(
        "--molmospace-asset-root",
        type=Path,
        default=None,
        help="Override the MolmoSpaces install directory from the pinned config.",
    )
    parser.add_argument(
        "--gpu-total-aggregate-pairs-capacity",
        type=int,
        default=None,
        help="Override PhysX GPU total aggregate pairs capacity.",
    )
    parser.add_argument(
        "--gpu-found-lost-aggregate-pairs-capacity",
        type=int,
        default=None,
        help="Override PhysX GPU found/lost aggregate pairs capacity.",
    )
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_AUTOLIFE_USD)
    parser.add_argument("--autolife-config", type=Path, default=DEFAULT_AUTOLIFE_CONFIG)
    parser.add_argument(
        "--drive-profile-usd",
        type=Path,
        default=DEFAULT_DRIVE_PROFILE_USD,
        help=(
            "USD whose Autolife joint Drive values are copied at startup; defaults "
            "to the previously validated BEHAVIOR world.usd."
        ),
    )
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
        default=None,
        metavar=("X", "Y", "Z"),
        help="Initial robot root translation; MolmoSpaces uses the pinned scene spawn by default.",
    )
    parser.add_argument(
        "--robot-orientation-wxyz",
        nargs=4,
        type=float,
        default=None,
        metavar=("W", "X", "Y", "Z"),
        help="Initial robot root orientation; MolmoSpaces uses the pinned scene spawn by default.",
    )
    parser.add_argument(
        "--quick-load",
        action="store_true",
        help="Only load scene structure categories for faster smoke tests.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Launch the selected simulation backend without the GUI viewport.",
    )
    parser.add_argument(
        "--isaac-sim-experience",
        type=Path,
        default=None,
        help=(
            "Optional Kit experience override. The default stable Python experience "
            "explicitly enables the Physics Property UI and Gain Tuner."
        ),
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
        "--lightwheel-microwave-usd",
        type=Path,
        default=DEFAULT_LIGHTWHEEL_MICROWAVE_USD,
        help="Optional Lightwheel Microwave035 USD; skipped when the file is unavailable.",
    )
    parser.add_argument(
        "--no-lightwheel-microwave",
        action="store_true",
        help="Do not add the fixed Lightwheel Microwave035 asset to the loaded scene.",
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


def _load_molmospace_config(path: Path) -> dict:
    path = path.expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("scene_model") not in DIRECT_ISAAC_SCENE_MODELS:
        raise ValueError(
            "Direct Isaac config scene_model must be one of "
            f"{sorted(DIRECT_ISAAC_SCENE_MODELS)!r}: {path}"
        )
    return config


def _resolve_molmospace_asset_root(config: dict, override: Path | None) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    configured = Path(config["asset_root"]).expanduser()
    if configured.is_absolute():
        return configured.resolve()
    return (WORKSPACE_ROOT / configured).resolve()


def _resolve_molmospace_scene_usd(config: dict, asset_root: Path) -> Path:
    scene = config["scene"]
    scene_usd = (
        asset_root
        / "scenes"
        / scene["source"]
        / scene["directory"]
        / scene["entrypoint"]
    )
    if not scene_usd.is_file():
        downloader = Path(__file__).resolve().parent / "download_molmospace_scene.py"
        raise FileNotFoundError(
            f"MolmoSpaces scene is not installed: {scene_usd}\n"
            f"Install the pinned scene first:\n  python3 {downloader}"
        )
    return scene_usd


def _patch_isaac_sim_gui_selection() -> bool:
    """
    Guard Isaac Sim 5.1's transform manipulator against unresolved paths.

    The 5.1 manipulator can convert an otherwise valid viewport selection to
    ``None`` while USD/Fabric is still resolving an instance proxy.  It then
    forwards that value to ``get_prim_at_path`` and raises ``KeyError:
    <class 'NoneType'>``.  Keep all resolvable selections and let the stock
    implementation handle them normally; unresolved paths simply disable the
    transform gizmo for that selection event.
    """
    try:
        from omni.kit.manipulator.prim.core.prim_transform_manipulator import (
            PrimTransformManipulator,
        )
    except ImportError:
        _emit(
            "Isaac Sim GUI selection guard was not needed: transform manipulator is unavailable"
        )
        return False

    if getattr(PrimTransformManipulator, "_autolife_selection_guard", False):
        return False

    original_on_selection_changed = PrimTransformManipulator.on_selection_changed

    def _safe_on_selection_changed(self, stage, selection, *args, **kwargs):
        if selection and self.model and self.model.get_da().is_ready():
            data_accessor = self.model.get_da()
            filtered_selection = []
            for sdf_path in selection:
                if sdf_path is None or not str(sdf_path):
                    continue
                try:
                    prioritized_path = data_accessor.get_sdf_path_by_priority(sdf_path)
                except (KeyError, TypeError, ValueError):
                    continue
                if prioritized_path is not None:
                    filtered_selection.append(sdf_path)
            selection = filtered_selection
        return original_on_selection_changed(self, stage, selection, *args, **kwargs)

    PrimTransformManipulator.on_selection_changed = _safe_on_selection_changed
    PrimTransformManipulator._autolife_selection_guard = True
    _emit("Applied Isaac Sim 5.1 GUI selection compatibility guard")
    return True


def _enable_physx_ui_mouse_interaction(simulation_app, config: dict) -> bool:
    """Enable the viewport PhysX grab gesture omitted by the Python experience."""
    interaction_config = config.get("interaction", {})
    if not interaction_config.get("enable_physx_ui", True):
        _emit("PhysX viewport mouse interaction is disabled by MolmoSpaces config")
        return False

    picking_force = float(
        interaction_config.get("mouse_picking_force", DEFAULT_PHYSX_MOUSE_PICKING_FORCE)
    )
    if not 0.0 < picking_force <= 10.0:
        raise ValueError("interaction.mouse_picking_force must be in the range (0, 10]")

    import carb.settings
    import omni.kit.app
    import omni.physx.bindings._physx as physx_bindings

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if not extension_manager.is_extension_enabled("omni.physx.ui"):
        if not extension_manager.set_extension_enabled_immediate("omni.physx.ui", True):
            raise RuntimeError(
                "Failed to enable omni.physx.ui; Shift+left-drag physics interaction "
                "is unavailable in this Isaac Sim installation"
            )
        # Let the extension attach its gesture overlay to the active viewport.
        simulation_app.update()

    if not extension_manager.is_extension_enabled("omni.physx.ui"):
        raise RuntimeError("omni.physx.ui did not remain enabled after startup")

    settings = carb.settings.get_settings()
    settings.set_bool(physx_bindings.SETTING_MOUSE_INTERACTION_ENABLED, True)
    settings.set_bool(physx_bindings.SETTING_MOUSE_GRAB, True)
    settings.set_bool(physx_bindings.SETTING_MOUSE_GRAB_WITH_FORCE, True)
    settings.set_float(physx_bindings.SETTING_MOUSE_PICKING_FORCE, picking_force)
    _emit(
        "Enabled omni.physx.ui viewport interaction: "
        f"Shift+left-drag, pickingForce={picking_force:g}"
    )
    return True


def _robot_mouse_interaction_config(config: dict) -> dict:
    configured = config.get("robot_mouse_interaction", {})
    defaults = DEFAULT_ROBOT_MOUSE_INTERACTION
    damping = dict(defaults["minimum_drive_damping"])
    damping.update(configured.get("minimum_drive_damping", {}))
    result = {
        "enabled": bool(configured.get("enabled", defaults["enabled"])),
        "capture_base_target_during_drag": bool(
            configured.get(
                "capture_base_target_during_drag",
                defaults["capture_base_target_during_drag"],
            )
        ),
        "minimum_mimic_joint_damping_ratio": float(
            configured.get(
                "minimum_mimic_joint_damping_ratio",
                defaults["minimum_mimic_joint_damping_ratio"],
            )
        ),
        "minimum_solver_velocity_iteration_count": int(
            configured.get(
                "minimum_solver_velocity_iteration_count",
                defaults["minimum_solver_velocity_iteration_count"],
            )
        ),
        "minimum_drive_damping": {
            str(joint_name): float(value) for joint_name, value in damping.items()
        },
    }
    for joint_name, value in result["minimum_drive_damping"].items():
        if value < 0.0:
            raise ValueError(
                "robot_mouse_interaction.minimum_drive_damping values must be "
                f"non-negative: {joint_name}={value}"
            )
    if result["minimum_mimic_joint_damping_ratio"] < 0.0:
        raise ValueError(
            "robot_mouse_interaction.minimum_mimic_joint_damping_ratio must be non-negative"
        )
    if result["minimum_solver_velocity_iteration_count"] < 1:
        raise ValueError(
            "robot_mouse_interaction.minimum_solver_velocity_iteration_count must be positive"
        )
    return result


def _stabilize_autolife_mouse_interaction(
    stage, robot_prim_path: str, config: dict
) -> dict[str, int]:
    """Stabilize editable drives and session-only finger/solver internals."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    interaction = _robot_mouse_interaction_config(config)
    if not interaction["enabled"]:
        return {"drive_damping": 0, "mimic_damping": 0, "solver_iterations": 0}

    counts = {"drive_damping": 0, "mimic_damping": 0, "solver_iterations": 0}
    # Author the main Drive damping in the normal editable stage layer so the
    # Property panel and Gain Tuner can still change it at runtime.
    for joint_name, damping_floor in interaction["minimum_drive_damping"].items():
        prim = _find_robot_prim_by_name(stage, robot_prim_path, joint_name)
        drive_name = _get_drive_name(prim)
        drive_api = UsdPhysics.DriveAPI.Get(prim, drive_name)
        if not drive_api:
            drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_name)
        damping_attr = drive_api.CreateDampingAttr()
        current_damping = damping_attr.Get()
        if current_damping is None or current_damping < damping_floor:
            damping_attr.Set(damping_floor)
            counts["drive_damping"] += 1

    # Mimic and solver values are internal stabilization details rather than
    # user-facing PID gains; keep these disposable and stronger than assets.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        robot_root = stage.GetPrimAtPath(robot_prim_path)
        mimic_damping_floor = interaction["minimum_mimic_joint_damping_ratio"]
        solver_iteration_floor = interaction["minimum_solver_velocity_iteration_count"]
        for prim in Usd.PrimRange(robot_root):
            for property_name in prim.GetPropertyNames():
                if not (
                    property_name.startswith("physxMimicJoint:")
                    and property_name.endswith(":dampingRatio")
                ):
                    continue
                damping_attr = prim.GetAttribute(property_name)
                current_damping = damping_attr.Get()
                if current_damping is None or current_damping < mimic_damping_floor:
                    damping_attr.Set(mimic_damping_floor)
                    counts["mimic_damping"] += 1

            if "PhysxArticulationAPI" not in prim.GetAppliedSchemas():
                continue
            articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, prim.GetPath())
            velocity_iterations_attr = (
                articulation_api.CreateSolverVelocityIterationCountAttr()
            )
            current_iterations = velocity_iterations_attr.Get()
            if current_iterations is None or current_iterations < solver_iteration_floor:
                velocity_iterations_attr.Set(solver_iteration_floor)
                counts["solver_iterations"] += 1
    return counts


class _MouseDragBasePoseRetargeter:
    """Treat Shift+left-drag as a base teach operation instead of a disturbance."""

    def __init__(self, root_joint_path: str):
        import carb.input
        import omni.appwindow
        import numpy as np
        from isaacsim.core.prims import SingleArticulation

        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            raise RuntimeError("Cannot initialize robot mouse retargeting without an app window")

        self._carb_input = carb.input
        self._input = carb.input.acquire_input_interface()
        self._keyboard = app_window.get_keyboard()
        self._mouse = app_window.get_mouse()
        self._np = np
        self._was_dragging = False
        self._last_targets = None
        self._robot = SingleArticulation(
            prim_path=root_joint_path,
            name="autolife_mouse_retarget",
            reset_xform_properties=False,
        )
        self._robot.initialize()
        self._joint_indices = np.asarray(
            [self._robot.get_dof_index(name) for name in BASE_JOINT_NAMES],
            dtype=np.int32,
        )

    def _capture_current_base_target(self):
        from isaacsim.core.utils.types import ArticulationAction

        positions = self._robot.get_joint_positions(joint_indices=self._joint_indices)
        if positions is None:
            return None
        positions = self._np.asarray(positions, dtype=self._np.float32)
        if not self._np.all(self._np.isfinite(positions)):
            return None
        self._robot.apply_action(
            ArticulationAction(
                joint_positions=positions,
                joint_velocities=self._np.zeros_like(positions),
                joint_indices=self._joint_indices,
            )
        )
        self._last_targets = positions.copy()
        return positions

    def update(self) -> None:
        mouse_down = (
            self._input.get_mouse_value(
                self._mouse, self._carb_input.MouseInput.LEFT_BUTTON
            )
            > 0.5
        )
        shift_down = (
            self._input.get_keyboard_value(
                self._keyboard, self._carb_input.KeyboardInput.LEFT_SHIFT
            )
            > 0.5
            or self._input.get_keyboard_value(
                self._keyboard, self._carb_input.KeyboardInput.RIGHT_SHIFT
            )
            > 0.5
        )
        dragging = mouse_down and shift_down

        if dragging:
            self._capture_current_base_target()
            self._was_dragging = True
        elif self._was_dragging:
            targets = self._capture_current_base_target()
            self._was_dragging = False
            if targets is not None:
                _emit(
                    "Captured manual robot base target after mouse drag: "
                    f"x={targets[0]:.3f}, y={targets[1]:.3f}, yaw={targets[2]:.3f}"
                )


def _filter_molmospace_scene_content(stage, config: dict) -> int:
    """Deactivate random Objaverse filler while preserving native THOR furniture."""
    if not config.get("scene_content", {}).get("disable_objaverse_objects", False):
        return 0

    from pxr import Usd

    default_prim = stage.GetDefaultPrim()
    geometry_prim = default_prim.GetChild("Geometry") if default_prim else None
    if not geometry_prim:
        raise RuntimeError("MolmoSpaces stage has no Geometry prim to filter")

    objaverse_roots = [
        prim
        for prim in geometry_prim.GetChildren()
        if prim.GetName().lower().startswith("obja")
    ]
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in objaverse_roots:
            prim.SetActive(False)
    return len(objaverse_roots)


def _molmospace_stabilization_config(config: dict) -> dict:
    configured = config.get("articulated_object_stabilization", {})
    defaults = DEFAULT_MOLMOSPACE_ARTICULATION_STABILIZATION
    result = {
        "enabled": configured.get("enabled", defaults["enabled"]),
        "joint_friction": float(
            configured.get("joint_friction", defaults["joint_friction"])
        ),
    }
    for joint_type in ("revolute", "prismatic"):
        configured_type = configured.get(joint_type, {})
        default_type = defaults[joint_type]
        result[joint_type] = {
            key: float(configured_type.get(key, default_type[key]))
            for key in ("damping", "max_force", "max_velocity")
        }

    if result["joint_friction"] < 0.0:
        raise ValueError("articulated_object_stabilization.joint_friction must be non-negative")
    for joint_type in ("revolute", "prismatic"):
        for key, value in result[joint_type].items():
            if value < 0.0:
                raise ValueError(
                    f"articulated_object_stabilization.{joint_type}.{key} "
                    "must be non-negative"
                )
    return result


def _stabilize_molmospace_articulations(stage, config: dict) -> dict[str, int]:
    """Add passive friction and damping without changing source MolmoSpaces USDs."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    stabilization = _molmospace_stabilization_config(config)
    counts = {
        "revolute": 0,
        "prismatic": 0,
        "drives_added": 0,
        "existing_drives_preserved": 0,
    }
    if not stabilization["enabled"]:
        return counts

    joints = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints.append((prim, "revolute", "angular"))
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            joints.append((prim, "prismatic", "linear"))

    # The session layer is discarded when the app closes. The downloaded
    # MolmoSpaces cache and the stable project asset view remain byte-for-byte
    # unchanged.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim, joint_type, drive_name in joints:
            values = stabilization[joint_type]
            joint_api = PhysxSchema.PhysxJointAPI.Get(stage, prim.GetPath())
            if not joint_api:
                joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)

            friction_attr = joint_api.CreateJointFrictionAttr()
            current_friction = friction_attr.Get()
            if current_friction is None or current_friction < stabilization["joint_friction"]:
                friction_attr.Set(stabilization["joint_friction"])

            velocity_attr = joint_api.CreateMaxJointVelocityAttr()
            current_max_velocity = velocity_attr.Get()
            if current_max_velocity is None or current_max_velocity > values["max_velocity"]:
                velocity_attr.Set(values["max_velocity"])

            drive_api = UsdPhysics.DriveAPI.Get(prim, drive_name)
            if drive_api:
                counts["existing_drives_preserved"] += 1
            else:
                drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_name)
                drive_api.CreateTypeAttr("force", True)
                drive_api.CreateStiffnessAttr(0.0, True)
                drive_api.CreateDampingAttr(values["damping"], True)
                drive_api.CreateTargetVelocityAttr(0.0, True)
                drive_api.CreateMaxForceAttr(values["max_force"], True)
                counts["drives_added"] += 1

            counts[joint_type] += 1

    return counts


def _add_local_omnigibson_to_path(omnigibson_root: Path) -> None:
    omnigibson_root = omnigibson_root.expanduser().resolve()
    if not (omnigibson_root / "omnigibson").is_dir():
        if importlib.util.find_spec("omnigibson") is not None:
            return
        raise FileNotFoundError(
            f"OmniGibson package directory not found under {omnigibson_root}. "
            "Run scripts/setup_autolife_sim_env.sh first."
        )
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


def _remove_incompatible_python_paths() -> tuple[str, ...]:
    """Hide Python packages built for a different minor interpreter.

    ROS 2 Jazzy on Ubuntu 24.04 installs Python 3.12 packages.  The unified
    ``autolife_sim`` environment and Isaac Sim 5.1 use Python 3.11.  Sourcing
    ROS before starting the simulator therefore exposes an incompatible rclpy
    package to Kit.  The ROS bridge communicates over DDS and does not need the
    system rclpy package in the simulator process, so remove only paths that
    explicitly name another Python minor version.
    """
    current_version = (sys.version_info.major, sys.version_info.minor)
    version_pattern = re.compile(r"(?:^|/)python(\d+)\.(\d+)(?:/|$)")

    def is_compatible(path: str) -> bool:
        match = version_pattern.search(path)
        if match is None:
            return True
        return (int(match.group(1)), int(match.group(2))) == current_version

    removed = tuple(path for path in sys.path if path and not is_compatible(path))
    if removed:
        sys.path[:] = [path for path in sys.path if not path or is_compatible(path)]

    python_path = os.environ.get("PYTHONPATH")
    if python_path:
        entries = python_path.split(os.pathsep)
        compatible_entries = [path for path in entries if is_compatible(path)]
        if compatible_entries:
            os.environ["PYTHONPATH"] = os.pathsep.join(compatible_entries)
        else:
            os.environ.pop("PYTHONPATH", None)
    return removed


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

    # Prefer Isaac Sim's Python-3.11 Jazzy bundle when it exists.  The system
    # Jazzy installation targets Python 3.12 and must remain in the separate
    # ROS controller / RViz processes.
    if ros_distro == "jazzy" and internal_jazzy_lib.is_dir():
        library_entries = [
            path
            for path in ld_library_path.split(os.pathsep)
            if path and path != str(internal_jazzy_lib)
        ]
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
            [str(internal_jazzy_lib), *library_entries]
        )
        removed_paths = _remove_incompatible_python_paths()
        if removed_paths:
            _emit(
                "Removed incompatible Python paths before Isaac ROS2 bridge startup: "
                + ",".join(removed_paths)
            )
        return

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


def _copy_autolife_drive_profile(
    stage, robot_prim_path: str, profile_usd: Path
) -> dict[str, int]:
    """Copy the validated Drive/PID values from an existing Autolife USD stage."""
    from pxr import Usd, UsdPhysics

    profile_usd = profile_usd.expanduser().resolve()
    if not profile_usd.is_file():
        raise FileNotFoundError(f"Autolife Drive profile USD does not exist: {profile_usd}")
    profile_stage = Usd.Stage.Open(str(profile_usd))
    if profile_stage is None:
        raise RuntimeError(f"Cannot open Autolife Drive profile USD: {profile_usd}")

    source_robot_path = SENSOR_OVERLAY_SOURCE_ROBOT_PATH
    if not profile_stage.GetPrimAtPath(source_robot_path).IsValid():
        for prim in profile_stage.Traverse():
            if prim.GetName().lower() == "autolife":
                source_robot_path = str(prim.GetPath())
                break
        else:
            raise RuntimeError(f"No Autolife prim was found in Drive profile: {profile_usd}")

    counts = {"joints": 0, "attributes": 0}
    joint_names = [name for names in PID_JOINT_MAP.values() for name in names]
    for joint_name in joint_names:
        source_prim = _find_robot_prim_by_name(
            profile_stage, source_robot_path, joint_name
        )
        destination_prim = _find_robot_prim_by_name(stage, robot_prim_path, joint_name)
        source_drive_name = _get_drive_name(source_prim)
        destination_drive_name = _get_drive_name(destination_prim)
        source_drive = UsdPhysics.DriveAPI.Get(source_prim, source_drive_name)
        if not source_drive:
            raise RuntimeError(
                f"Drive profile has no {source_drive_name} Drive on {source_prim.GetPath()}"
            )
        destination_drive = UsdPhysics.DriveAPI.Get(
            destination_prim, destination_drive_name
        )
        if not destination_drive:
            destination_drive = UsdPhysics.DriveAPI.Apply(
                destination_prim, destination_drive_name
            )

        for source_getter, destination_creator in (
            (source_drive.GetStiffnessAttr, destination_drive.CreateStiffnessAttr),
            (source_drive.GetDampingAttr, destination_drive.CreateDampingAttr),
            (source_drive.GetMaxForceAttr, destination_drive.CreateMaxForceAttr),
            (source_drive.GetTypeAttr, destination_drive.CreateTypeAttr),
        ):
            value = source_getter().Get()
            if value is not None:
                destination_creator(value, True)
                counts["attributes"] += 1
        counts["joints"] += 1
    return counts


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


def _apply_autolife_drive_config(
    stage,
    robot_prim_path: str,
    config_path: Path,
    drive_profile_usd: Path,
) -> dict[str, int]:
    profile_counts = _copy_autolife_drive_profile(
        stage, robot_prim_path, drive_profile_usd
    )
    return {
        "profile_joints": profile_counts["joints"],
        "profile_attributes": profile_counts["attributes"],
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
    from pxr import Sdf, Usd, UsdPhysics

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
        if source_relative_path == REALSENSE_SENSOR_RELATIVE_ROOT:
            # Sanitize the relative payload in an anonymous layer before it can
            # be composed against the destination scene's directory.
            scratch_layer = Sdf.Layer.CreateAnonymous("autolife_realsense_overlay.usda")
            scratch_path = Sdf.Path("/RealsenseOverlay")
            if not Sdf.CopySpec(source_layer, source_path, scratch_layer, scratch_path):
                raise RuntimeError(f"Failed to stage Realsense overlay prim: {source_path}")
            scratch_spec = scratch_layer.GetPrimAtPath(scratch_path)
            scratch_spec.payloadList.ClearEdits()
            scratch_spec.referenceList.ClearEdits()
            scratch_spec.referenceList.explicitItems = [Sdf.Reference(str(realsense_usd))]
            copied_ok = Sdf.CopySpec(scratch_layer, scratch_path, target_layer, target_path)
        else:
            copied_ok = Sdf.CopySpec(source_layer, source_path, target_layer, target_path)
        if not copied_ok:
            raise RuntimeError(f"Failed to copy sensor overlay prim: {source_path} -> {target_path}")
        copied += 1

    # Sensor assets are mounted below existing robot rigid links.  In
    # particular, the referenced RealSense asset contains its own rigid-body
    # and collision schemas; leaving those nested schemas active makes PhysX
    # create an invalid rigid-body hierarchy and can inject motion into the
    # head/arms.  Sensors are visual/data-producing attachments here, so strip
    # only their nested dynamics while preserving every typed sensor prim.
    stripped_physics = {"rigid_body": 0, "mass": 0, "collision": 0}
    api_types = (
        ("rigid_body", UsdPhysics.RigidBodyAPI),
        ("mass", UsdPhysics.MassAPI),
        ("collision", UsdPhysics.CollisionAPI),
    )
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for _, target_relative_path in SENSOR_OVERLAY_RELATIVE_ROOTS:
            root = stage.GetPrimAtPath(f"{robot_prim_path}/{target_relative_path}")
            if not root.IsValid():
                continue
            for prim in Usd.PrimRange(root):
                for label, api_type in api_types:
                    if prim.HasAPI(api_type):
                        prim.RemoveAPI(api_type)
                        stripped_physics[label] += 1

    sensor_counts = _count_typed_sensors(stage, robot_prim_path)
    sensor_counts["subtrees"] = copied
    sensor_counts.update(stripped_physics)
    return sensor_counts


def _add_usd_reference(stage, asset_usd: Path, prim_path: str, asset_label: str):
    asset_usd = asset_usd.expanduser().resolve()
    if not asset_usd.is_file():
        raise FileNotFoundError(asset_usd)
    if stage.GetPrimAtPath(prim_path).IsValid():
        stage.RemovePrim(prim_path)
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(asset_usd))
    if not prim.IsValid():
        raise RuntimeError(f"Failed to reference {asset_label} at {prim_path}: {asset_usd}")
    return prim


def _add_autolife_usd(
    stage,
    robot_usd: Path,
    robot_prim_path: str,
    position,
    orientation_wxyz,
) -> str:
    robot_usd = robot_usd.expanduser().resolve()
    _add_usd_reference(stage, robot_usd, robot_prim_path, "Autolife robot")
    _set_root_pose(stage, robot_prim_path, position, orientation_wxyz)

    root_joint_path = f"{robot_prim_path}/root_joint"
    if not stage.GetPrimAtPath(root_joint_path).IsValid():
        raise RuntimeError(f"Autolife articulation root was not found at {root_joint_path}")
    return root_joint_path


def _configure_gpu_aggregate_pair_capacities(
    physics_context,
    total_capacity: int,
    found_lost_capacity: int,
) -> tuple[int, int]:
    if total_capacity <= 0 or found_lost_capacity <= 0:
        raise ValueError("GPU aggregate pair capacities must be positive integers")
    physics_context.set_gpu_total_aggregate_pairs_capacity(total_capacity)
    physics_context.set_gpu_found_lost_aggregate_pairs_capacity(found_lost_capacity)
    actual_total = physics_context.get_gpu_total_aggregate_pairs_capacity()
    actual_found_lost = physics_context.get_gpu_found_lost_aggregate_pairs_capacity()
    if actual_total != total_capacity or actual_found_lost != found_lost_capacity:
        raise RuntimeError(
            "PhysX GPU aggregate pair capacity verification failed: "
            f"requested total={total_capacity}, found/lost={found_lost_capacity}; "
            f"actual total={actual_total}, found/lost={actual_found_lost}"
        )
    return actual_total, actual_found_lost


def _add_lightwheel_microwave_usd(stage, microwave_usd: Path) -> str:
    microwave_usd = microwave_usd.expanduser().resolve()
    prim_path = DEFAULT_LIGHTWHEEL_MICROWAVE_PRIM_PATH
    _add_usd_reference(stage, microwave_usd, prim_path, "Lightwheel microwave")
    _set_root_pose(
        stage=stage,
        prim_path=prim_path,
        position=DEFAULT_LIGHTWHEEL_MICROWAVE_POSITION,
        orientation_wxyz=DEFAULT_LIGHTWHEEL_MICROWAVE_ORIENTATION_WXYZ,
    )

    door_joint_path = f"{prim_path}/Microwave035_door/microjoint"
    if not stage.GetPrimAtPath(door_joint_path).IsValid():
        raise RuntimeError(f"Lightwheel microwave door joint was not found at {door_joint_path}")
    return door_joint_path


def _setup_ros2_joint_bridge(stage, graph_path: str, robot_path: str, command_topic: str) -> None:
    script_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(script_dir))

    from joint_state_bridge import setup_joint_state_bridge

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)
    setup_joint_state_bridge(
        graph_path=graph_path,
        robot_path=robot_path,
        joint_command_topic=command_topic,
    )


def _setup_ros2_sensor_bridge(
    stage,
    graph_path: str,
    robot_prim_path: str,
    robot_articulation_path: str,
    camera_width: int,
    camera_height: int,
    frame_skip_count: int,
    manifest_path: Path,
) -> dict:
    script_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(script_dir))

    from sensor_bridge import setup_sensor_bridge

    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)
    return setup_sensor_bridge(
        stage=stage,
        graph_path=graph_path,
        robot_prim_path=robot_prim_path,
        robot_articulation_path=robot_articulation_path,
        camera_width=camera_width,
        camera_height=camera_height,
        frame_skip_count=frame_skip_count,
        manifest_path=manifest_path,
    )


def _populate_autolife_stage(stage, args: argparse.Namespace) -> str:
    root_joint_path = _add_autolife_usd(
        stage=stage,
        robot_usd=args.robot_usd,
        robot_prim_path=args.robot_prim_path,
        position=args.robot_position,
        orientation_wxyz=args.robot_orientation_wxyz,
    )
    _emit(f"Added Autolife USD at {args.robot_prim_path}")
    _emit(f"Autolife articulation root: {root_joint_path}")

    if not args.no_lightwheel_microwave:
        microwave_usd = args.lightwheel_microwave_usd.expanduser().resolve()
        if microwave_usd.is_file():
            microwave_door_joint_path = _add_lightwheel_microwave_usd(stage, microwave_usd)
            _emit(f"Added Lightwheel microwave at {DEFAULT_LIGHTWHEEL_MICROWAVE_PRIM_PATH}")
            _emit(f"Lightwheel microwave door joint: {microwave_door_joint_path}")
        else:
            _emit(f"Optional Lightwheel microwave is unavailable; skipping: {microwave_usd}")

    if not args.no_sensor_overlay:
        sensor_counts = _copy_sensor_overlay(
            stage=stage,
            sensor_overlay_usd=args.sensor_overlay_usd,
            realsense_usd=args.realsense_usd,
            robot_prim_path=args.robot_prim_path,
        )
        _emit(
            "Copied Autolife sensor overlay: "
            f"subtrees={sensor_counts['subtrees']}, "
            f"Camera={sensor_counts['Camera']}, "
            f"IsaacImuSensor={sensor_counts['IsaacImuSensor']}, "
            f"OmniLidar={sensor_counts['OmniLidar']}, "
            "nestedPhysicsRemoved="
            f"{sensor_counts['rigid_body'] + sensor_counts['mass'] + sensor_counts['collision']}"
        )

    if not args.no_apply_drive_config:
        drive_counts = _apply_autolife_drive_config(
            stage=stage,
            robot_prim_path=args.robot_prim_path,
            config_path=args.autolife_config,
            drive_profile_usd=args.drive_profile_usd,
        )
        _emit(
            f"Copied Autolife Drive profile from {args.drive_profile_usd}: "
            f"joints={drive_counts['profile_joints']}, "
            f"attributes={drive_counts['profile_attributes']}, "
            f"speedLimit={drive_counts['speed_limit']}"
        )
    return root_joint_path


def _create_ros2_graphs(stage, args: argparse.Namespace, root_joint_path: str) -> list[str]:
    _setup_ros2_joint_bridge(
        stage=stage,
        graph_path=args.graph_path,
        robot_path=root_joint_path,
        command_topic=args.ros2_joint_command_topic,
    )
    _emit(
        f"Created ROS2 ActionGraph at {args.graph_path}: "
        f"publishing /joint_states and subscribing {args.ros2_joint_command_topic}"
    )

    deferred_init_nodes = []
    if not args.no_ros2_sensors:
        sensor_graph_counts = _setup_ros2_sensor_bridge(
            stage=stage,
            graph_path=args.sensor_graph_path,
            robot_prim_path=args.robot_prim_path,
            robot_articulation_path=root_joint_path,
            camera_width=args.camera_width,
            camera_height=args.camera_height,
            frame_skip_count=args.sensor_frame_skip,
            manifest_path=args.sensor_manifest_path,
        )
        deferred_init_nodes = sensor_graph_counts["deferred_init_nodes"]
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
    return deferred_init_nodes


def _initialize_ros2_sensor_outputs(
    graph_path: str,
    deferred_init_nodes: list[str],
    step_once,
) -> None:
    """Create render products and ROS2 writers serially after timeline start."""
    if not deferred_init_nodes:
        return

    import omni.graph.core as og

    graph_path = graph_path.rstrip("/")
    total = len(deferred_init_nodes)
    _emit(f"Initializing ROS2 render products and writers serially: nodes={total}")
    # In GUI mode the Layers window locks specifiers during the first rendered
    # stage synchronization. Run that frame without TF traversal or Replicator
    # authoring before enabling any sensor node.
    step_once()
    for index, relative_attribute in enumerate(deferred_init_nodes, start=1):
        attribute_path = f"{graph_path}/{relative_attribute}"
        enabled_attribute = og.Controller.attribute(attribute_path)
        if not enabled_attribute.is_valid():
            raise RuntimeError(f"ROS2 sensor initialization attribute is invalid: {attribute_path}")
        enabled_attribute.set(True)
        # A rendered frame lets the newly enabled node finish all Replicator
        # stage authoring before another render product or writer starts.
        step_once()
        node_name = relative_attribute.split(".", 1)[0]
        _emit(f"Initialized ROS2 sensor node {index}/{total}: {node_name}")


def _run_behavior(args: argparse.Namespace) -> int:
    if args.robot_position is None:
        args.robot_position = DEFAULT_ROBOT_POSITION
    if args.robot_orientation_wxyz is None:
        args.robot_orientation_wxyz = DEFAULT_ROBOT_ORIENTATION_WXYZ

    _add_local_omnigibson_to_path(args.omnigibson_root)

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "True"
    if not args.no_ros2_bridge:
        _assert_ros2_bridge_env()

    import omnigibson as og
    from omnigibson.macros import gm

    gpu_total_capacity = args.gpu_total_aggregate_pairs_capacity
    gpu_found_lost_capacity = args.gpu_found_lost_aggregate_pairs_capacity
    if gpu_total_capacity is not None or gpu_found_lost_capacity is not None:
        if gpu_total_capacity is None:
            gpu_total_capacity = gm.GPU_AGGR_PAIRS_CAPACITY
        if gpu_found_lost_capacity is None:
            gpu_found_lost_capacity = gm.GPU_AGGR_PAIRS_CAPACITY
        if gpu_total_capacity <= 0 or gpu_found_lost_capacity <= 0:
            raise ValueError("GPU aggregate pair capacities must be positive integers")
        # OmniGibson reads this macro while constructing its PhysicsContext. Use
        # the larger value during construction, then apply and verify each exact
        # value below while the simulation is stopped.
        gm.GPU_AGGR_PAIRS_CAPACITY = max(gpu_total_capacity, gpu_found_lost_capacity)

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
    _emit(f"Backend: OmniGibson (BEHAVIOR scene {args.scene_model})")

    og.Environment(configs=cfg)

    # Add the external USD while simulation is stopped, then play again after
    # PhysX has the articulation and bridge graph in the stage.
    og.sim.stop()
    if gpu_total_capacity is not None and gpu_found_lost_capacity is not None:
        actual_total, actual_found_lost = _configure_gpu_aggregate_pair_capacities(
            physics_context=og.sim.get_physics_context(),
            total_capacity=gpu_total_capacity,
            found_lost_capacity=gpu_found_lost_capacity,
        )
        _emit(
            "Configured PhysX GPU aggregate pair capacities: "
            f"total={actual_total}, foundLost={actual_found_lost}"
        )

    with og.sim.editing_usd():
        root_joint_path = _populate_autolife_stage(og.sim.stage, args)

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

        with og.sim.editing_usd():
            deferred_sensor_nodes = _create_ros2_graphs(og.sim.stage, args, root_joint_path)
    else:
        deferred_sensor_nodes = []

    og.sim.play()
    _initialize_ros2_sensor_outputs(
        args.sensor_graph_path,
        deferred_sensor_nodes,
        og.sim.step,
    )

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


def _run_molmospace(args: argparse.Namespace) -> int:
    if args.quick_load:
        raise ValueError("--quick-load is only supported for BEHAVIOR / OmniGibson scenes")

    config = _load_molmospace_config(args.molmospace_config)
    if config["scene_model"] != args.scene_model:
        raise ValueError(
            f"Scene alias {args.scene_model!r} cannot use a "
            f"{config['scene_model']!r} config: {args.molmospace_config}"
        )
    robot_spawn = config.get("robot_spawn", {})
    if args.robot_position is None:
        args.robot_position = robot_spawn.get("position", DEFAULT_ROBOT_POSITION)
    if args.robot_orientation_wxyz is None:
        args.robot_orientation_wxyz = robot_spawn.get(
            "orientation_wxyz", DEFAULT_ROBOT_ORIENTATION_WXYZ
        )
    asset_root = _resolve_molmospace_asset_root(config, args.molmospace_asset_root)
    scene_usd = _resolve_molmospace_scene_usd(config, asset_root)
    if (
        not args.no_lightwheel_microwave
        and not config.get("additional_assets", {}).get("lightwheel_microwave", False)
    ):
        args.no_lightwheel_microwave = True
        _emit("MolmoSpaces config keeps native furnishing; Lightwheel microwave disabled")
    physics_config = config["physics"]
    physics_frequency_hz = float(physics_config.get("physics_frequency_hz", 120.0))
    rendering_frequency_hz = float(physics_config.get("rendering_frequency_hz", 30.0))
    if physics_frequency_hz <= 0.0 or rendering_frequency_hz <= 0.0:
        raise ValueError("Physics and rendering frequencies must be positive")
    if rendering_frequency_hz > physics_frequency_hz:
        raise ValueError("Rendering frequency cannot exceed physics frequency")
    total_capacity = args.gpu_total_aggregate_pairs_capacity
    found_lost_capacity = args.gpu_found_lost_aggregate_pairs_capacity
    if total_capacity is None:
        total_capacity = physics_config["gpu_total_aggregate_pairs_capacity"]
    if found_lost_capacity is None:
        found_lost_capacity = physics_config["gpu_found_lost_aggregate_pairs_capacity"]
    if total_capacity <= 0 or found_lost_capacity <= 0:
        raise ValueError("GPU aggregate pair capacities must be positive integers")

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    if not args.no_ros2_bridge:
        _assert_ros2_bridge_env()

    # This branch deliberately imports only Isaac Sim APIs. MolmoSpaces is a
    # complete USD stage and does not need an OmniGibson Scene wrapper.
    from isaacsim import SimulationApp

    _emit("Backend: Isaac Sim direct USD (OmniGibson is not imported)")
    _emit(f"Opening MolmoSpaces-compatible USD stage: {scene_usd}")
    experience = _resolve_isaac_sim_experience(
        args.isaac_sim_experience, args.headless
    )
    if experience is not None:
        _emit(
            f"Isaac Sim GUI experience override: {experience}"
        )
    simulation_app = SimulationApp(
        {"headless": args.headless, "sync_loads": True},
        experience="" if experience is None else str(experience),
    )
    simulation_context = None
    try:
        if not args.headless:
            enabled_editor_extensions = _enable_isaac_sim_editor_extensions(
                simulation_app
            )
            _emit(
                "Isaac Sim editor UI is ready: Physics Property panels and "
                "Tools > Robotics > Asset Editors > Gain Tuner; enabledNow="
                f"{','.join(enabled_editor_extensions) if enabled_editor_extensions else 'already-enabled'}"
            )
            _patch_isaac_sim_gui_selection()

        from isaacsim.core.api import SimulationContext
        from isaacsim.core.utils.stage import get_current_stage, is_stage_loading, open_stage

        if not args.no_ros2_bridge:
            _enable_ros2_bridge_extensions()
            simulation_app.update()

        # Do not resolve this symlink. Its relative payload paths are anchored
        # in the ResourceManager install view beside the installed objects.
        if not open_stage(str(scene_usd.expanduser().absolute())):
            raise RuntimeError(f"Isaac Sim failed to open the MolmoSpaces stage: {scene_usd}")
        while is_stage_loading():
            if not simulation_app.is_running():
                raise RuntimeError("Isaac Sim stopped while opening the MolmoSpaces stage")
            simulation_app.update()

        stage = get_current_stage()
        if stage is None:
            raise RuntimeError(f"Isaac Sim did not open the MolmoSpaces stage: {scene_usd}")
        filtered_objaverse_count = _filter_molmospace_scene_content(stage, config)
        if filtered_objaverse_count:
            _emit(
                "Disabled random Objaverse filler in the session layer: "
                f"roots={filtered_objaverse_count}; native THOR furniture preserved"
            )
        if not args.headless:
            # Stage loading resets per-stage PhysX settings, so enable and
            # configure the viewport grabber only after the final stage exists.
            _enable_physx_ui_mouse_interaction(simulation_app, config)

        simulation_context = SimulationContext(
            physics_dt=1.0 / physics_frequency_hz,
            rendering_dt=1.0 / rendering_frequency_hz,
            stage_units_in_meters=1.0,
        )
        _emit(
            "Simulation timing matches the previous BEHAVIOR baseline: "
            f"physics={physics_frequency_hz:g} Hz, rendering={rendering_frequency_hz:g} Hz"
        )
        simulation_context.stop()
        actual_total, actual_found_lost = _configure_gpu_aggregate_pair_capacities(
            physics_context=simulation_context.get_physics_context(),
            total_capacity=total_capacity,
            found_lost_capacity=found_lost_capacity,
        )
        _emit(
            "Configured PhysX GPU aggregate pair capacities: "
            f"total={actual_total}, foundLost={actual_found_lost}"
        )

        stabilization_counts = _stabilize_molmospace_articulations(stage, config)
        if config.get("articulated_object_stabilization", {}).get("enabled", True):
            _emit(
                "Applied passive MolmoSpaces articulation stabilization in the session layer: "
                f"revolute={stabilization_counts['revolute']}, "
                f"prismatic={stabilization_counts['prismatic']}, "
                f"drivesAdded={stabilization_counts['drives_added']}, "
                "existingDrivesPreserved="
                f"{stabilization_counts['existing_drives_preserved']}"
            )

        root_joint_path = _populate_autolife_stage(stage, args)
        robot_stabilization = _stabilize_autolife_mouse_interaction(
            stage, args.robot_prim_path, config
        )
        if _robot_mouse_interaction_config(config)["enabled"]:
            _emit(
                "Applied Autolife mouse-drag stabilization: "
                f"editableDriveDampingRaised={robot_stabilization['drive_damping']}, "
                f"sessionMimicDampingRaised={robot_stabilization['mimic_damping']}, "
                "sessionSolverIterationsRaised="
                f"{robot_stabilization['solver_iterations']}"
            )
        if not args.no_ros2_bridge:
            deferred_sensor_nodes = _create_ros2_graphs(stage, args, root_joint_path)
        else:
            deferred_sensor_nodes = []

        simulation_context.play()
        _initialize_ros2_sensor_outputs(
            args.sensor_graph_path,
            deferred_sensor_nodes,
            lambda: simulation_context.step(render=True),
        )
        base_retargeter = None
        robot_mouse_config = _robot_mouse_interaction_config(config)
        if (
            not args.headless
            and robot_mouse_config["enabled"]
            and robot_mouse_config["capture_base_target_during_drag"]
        ):
            base_retargeter = _MouseDragBasePoseRetargeter(root_joint_path)
            _emit(
                "Enabled manual robot base teach mode: Shift+left-drag updates "
                "the base target and release holds the new pose"
            )
        _emit("Simulation is running. Press Ctrl+C to exit.")
        step = 0
        try:
            while simulation_app.is_running() and (args.steps < 0 or step < args.steps):
                simulation_context.step(render=not args.headless)
                if base_retargeter is not None:
                    base_retargeter.update()
                step += 1
        except KeyboardInterrupt:
            _emit("Stopping simulation.")
    finally:
        if simulation_context is not None:
            simulation_context.stop()
            from isaacsim.core.api import SimulationContext

            SimulationContext.clear_instance()
        simulation_app.close()
    return 0


def main() -> int:
    """Parse command-line arguments and run the selected scene backend."""
    args = _parse_args()
    if args.scene_model in DIRECT_ISAAC_SCENE_MODELS:
        if (
            args.scene_model == DESK_SCENE_MODEL
            and args.molmospace_config == DEFAULT_MOLMOSPACE_CONFIG
        ):
            args.molmospace_config = DEFAULT_DESK_CONFIG
        _ensure_isaac_sim_python()
        return _run_molmospace(args)
    return _run_behavior(args)


if __name__ == "__main__":
    raise SystemExit(main())
