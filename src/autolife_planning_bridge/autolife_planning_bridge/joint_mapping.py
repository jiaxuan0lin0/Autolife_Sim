"""Joint and group mapping between Autolife_Sim and Autolife-Planning."""

from __future__ import annotations

from collections.abc import Iterable, Sequence

import numpy as np


PLANNING_JOINTS = (
    "Joint_Virtual_X",
    "Joint_Virtual_Y",
    "Joint_Virtual_Theta",
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
    "Joint_Neck_Roll",
    "Joint_Neck_Pitch",
    "Joint_Neck_Yaw",
    "Joint_Right_Shoulder_Inner",
    "Joint_Right_Shoulder_Outer",
    "Joint_Right_UpperArm",
    "Joint_Right_Elbow",
    "Joint_Right_Forearm",
    "Joint_Right_Wrist_Upper",
    "Joint_Right_Wrist_Lower",
)

PLANNING_TO_CONTROL = {
    "Joint_Virtual_X": "Joint_Ground_Vehicle_X",
    "Joint_Virtual_Y": "Joint_Ground_Vehicle_Y",
    "Joint_Virtual_Theta": "Joint_Ground_Vehicle_Z",
}
CONTROL_TO_PLANNING = {
    control_name: planning_name
    for planning_name, control_name in PLANNING_TO_CONTROL.items()
}

CONTROL_JOINTS = tuple(
    PLANNING_TO_CONTROL.get(joint_name, joint_name) for joint_name in PLANNING_JOINTS
)

PLANNING_JOINT_SET = frozenset(PLANNING_JOINTS)
CONTROL_JOINT_SET = frozenset(CONTROL_JOINTS)
UNPLANNED_CONTROL_JOINTS = frozenset({"Joint_Left_Gripper", "Joint_Right_Gripper"})

_BASE = ("Joint_Virtual_X", "Joint_Virtual_Y", "Joint_Virtual_Theta")
_LEGS = ("Joint_Ankle", "Joint_Knee")
_WAIST = ("Joint_Waist_Pitch", "Joint_Waist_Yaw")
_LEFT_ARM = (
    "Joint_Left_Shoulder_Inner",
    "Joint_Left_Shoulder_Outer",
    "Joint_Left_UpperArm",
    "Joint_Left_Elbow",
    "Joint_Left_Forearm",
    "Joint_Left_Wrist_Upper",
    "Joint_Left_Wrist_Lower",
)
_NECK = ("Joint_Neck_Roll", "Joint_Neck_Pitch", "Joint_Neck_Yaw")
_RIGHT_ARM = (
    "Joint_Right_Shoulder_Inner",
    "Joint_Right_Shoulder_Outer",
    "Joint_Right_UpperArm",
    "Joint_Right_Elbow",
    "Joint_Right_Forearm",
    "Joint_Right_Wrist_Upper",
    "Joint_Right_Wrist_Lower",
)
GROUP_JOINTS = {
    "autolife_base": frozenset(_BASE),
    "autolife_height": frozenset(("Joint_Ankle", "Joint_Knee", "Joint_Waist_Pitch")),
    "autolife_left_arm": frozenset(_LEFT_ARM),
    "autolife_right_arm": frozenset(_RIGHT_ARM),
    "autolife_dual_arm": frozenset(_LEFT_ARM + _RIGHT_ARM),
    "autolife_torso_left_arm": frozenset(_WAIST + _LEFT_ARM),
    "autolife_torso_right_arm": frozenset(_WAIST + _RIGHT_ARM),
    "autolife_torso_dual_arm": frozenset(_WAIST + _LEFT_ARM + _RIGHT_ARM),
    "autolife_leg_torso_dual_arm": frozenset(_LEGS + _WAIST + _LEFT_ARM + _RIGHT_ARM),
    "autolife_body": frozenset(_LEGS + _WAIST + _LEFT_ARM + _NECK + _RIGHT_ARM),
    "autolife": frozenset(PLANNING_JOINTS),
}

AUTO_GROUP_ORDER = (
    "autolife_base",
    "autolife_left_arm",
    "autolife_right_arm",
    "autolife_dual_arm",
    "autolife_height",
    "autolife_torso_left_arm",
    "autolife_torso_right_arm",
    "autolife_torso_dual_arm",
    "autolife_leg_torso_dual_arm",
    "autolife_body",
    "autolife",
)

CHAIN_JOINTS = {
    "left_arm": _LEFT_ARM,
    "right_arm": _RIGHT_ARM,
    "whole_body_left": _LEGS + _WAIST + _LEFT_ARM,
    "whole_body_right": _LEGS + _WAIST + _RIGHT_ARM,
    "whole_body_base_left": _BASE + _LEGS + _WAIST + _LEFT_ARM,
    "whole_body_base_right": _BASE + _LEGS + _WAIST + _RIGHT_ARM,
}

CHAIN_DEFAULT_GROUP = {
    "left_arm": "autolife_left_arm",
    "right_arm": "autolife_right_arm",
    "whole_body_left": "autolife_body",
    "whole_body_right": "autolife_body",
    "whole_body_base_left": "autolife",
    "whole_body_base_right": "autolife",
}


def planning_to_control_name(joint_name: str) -> str:
    return PLANNING_TO_CONTROL.get(joint_name, joint_name)


def control_to_planning_name(joint_name: str) -> str:
    return CONTROL_TO_PLANNING.get(joint_name, joint_name)


def planning_to_control_names(joint_names: Iterable[str]) -> list[str]:
    return [planning_to_control_name(joint_name) for joint_name in joint_names]


def control_to_planning_names(joint_names: Iterable[str]) -> list[str]:
    return [control_to_planning_name(joint_name) for joint_name in joint_names]


def infer_group(planning_joint_names: Iterable[str]) -> str:
    requested = frozenset(planning_joint_names)
    if not requested:
        raise ValueError("Cannot infer a planner group from an empty joint set")

    unknown = sorted(requested - PLANNING_JOINT_SET)
    if unknown:
        raise ValueError(f"Unknown planning joints: {unknown}")

    # Autolife-Planning has no pure head or pure torso subgroup.
    # Use the neutral body planner instead of a torso+arm subgroup for these
    # module-level commands.
    if requested <= frozenset(_NECK) or requested <= frozenset(_WAIST):
        return "autolife_body"

    for group in AUTO_GROUP_ORDER:
        if requested <= GROUP_JOINTS[group]:
            return group
    return "autolife"


def validate_group(group: str, planning_joint_names: Iterable[str]) -> None:
    if group not in GROUP_JOINTS:
        raise ValueError(f"Unknown planner group: {group}")

    requested = frozenset(planning_joint_names)
    unsupported = sorted(requested - GROUP_JOINTS[group])
    if unsupported:
        raise ValueError(
            f"Planner group {group!r} does not include requested joints: {unsupported}"
        )


def resolve_group(requested_group: str, planning_joint_names: Iterable[str]) -> str:
    if requested_group in ("", "auto"):
        return infer_group(planning_joint_names)
    validate_group(requested_group, planning_joint_names)
    return requested_group


def resolve_chain_key(chain: str, side: str = "") -> str:
    chain = chain.strip()
    side = side.strip()

    if not chain:
        raise ValueError("PoseControl.chain is required")
    if side:
        candidate = f"{chain}_{side}"
        if candidate in CHAIN_JOINTS:
            return candidate
    if chain in CHAIN_JOINTS:
        return chain
    raise ValueError(
        f"Unknown IK chain {chain!r} side {side!r}; supported: {sorted(CHAIN_JOINTS)}"
    )


def chain_factory_args(chain_key: str) -> tuple[str, str | None]:
    if chain_key in ("left_arm", "right_arm"):
        return chain_key, None
    for prefix in ("whole_body_base", "whole_body"):
        if chain_key.startswith(prefix + "_"):
            return prefix, chain_key.removeprefix(prefix + "_")
    raise ValueError(f"Unsupported chain key: {chain_key}")


def chain_joint_names(chain_key: str) -> tuple[str, ...]:
    if chain_key not in CHAIN_JOINTS:
        raise ValueError(f"Unsupported chain key: {chain_key}")
    return CHAIN_JOINTS[chain_key]


def default_group_for_chain(chain_key: str) -> str:
    if chain_key not in CHAIN_DEFAULT_GROUP:
        raise ValueError(f"Unsupported chain key: {chain_key}")
    return CHAIN_DEFAULT_GROUP[chain_key]


def state_to_planning_vector(
    positions_by_control_name: dict[str, float],
) -> tuple[np.ndarray | None, list[str]]:
    values = []
    missing = []
    for planning_name in PLANNING_JOINTS:
        control_name = planning_to_control_name(planning_name)
        if control_name not in positions_by_control_name:
            missing.append(control_name)
            values.append(0.0)
            continue
        values.append(float(positions_by_control_name[control_name]))

    if missing:
        return None, missing
    return np.asarray(values, dtype=np.float64), []


def merge_goal_positions(
    start_config: np.ndarray,
    planning_joint_names: Sequence[str],
    positions: Sequence[float],
) -> np.ndarray:
    if len(planning_joint_names) != len(positions):
        raise ValueError(
            "planning_joint_names and positions must have the same length "
            f"({len(planning_joint_names)} != {len(positions)})"
        )

    target = np.asarray(start_config, dtype=np.float64).copy()
    joint_to_index = {
        joint_name: index for index, joint_name in enumerate(PLANNING_JOINTS)
    }
    for joint_name, position in zip(planning_joint_names, positions):
        if joint_name not in joint_to_index:
            raise ValueError(f"Unknown planning joint: {joint_name}")
        target[joint_to_index[joint_name]] = float(position)
    return target


def extract_positions(
    config: np.ndarray,
    planning_joint_names: Sequence[str],
) -> list[float]:
    joint_to_index = {
        joint_name: index for index, joint_name in enumerate(PLANNING_JOINTS)
    }
    return [float(config[joint_to_index[joint_name]]) for joint_name in planning_joint_names]
