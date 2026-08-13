import json
from pathlib import Path

import omni.usd
from pxr import UsdPhysics
from pxr import PhysxSchema

# Run this script inside Isaac Sim after opening the target USD stage.

# ========================================================================================
# ========================== Selection for the config writing ============================
# ========================================================================================
WRITE_PD = True
WRITE_GV_MAX_FORCE = True
WRITE_ARM_MAX_FORCE = False
WRITE_GRIPPER_MAX_FORCE = True
WRITE_SYNC_SPEED_LIMITS = True
SAVE_STAGE = True
CONFIG = Path(__file__).resolve().parents[1] / "config" / "autolife.json"
USD_CONTEXT = omni.usd.get_context()

# Baseline Isaac drive gains selected from the URDF effort limits and current USD
# behavior. The virtual ground-vehicle X/Y joints use higher maxForce than URDF
# because they move the whole articulation; physical arm/head/torso force limits
# remain in the USD/URDF effort range.
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

# =======================================================================
# ========================== Load the config ============================
# =======================================================================

with CONFIG.open("r", encoding="utf-8") as f:
    cfg = json.load(f)

stage = USD_CONTEXT.get_stage()
if stage is None:
    raise RuntimeError("No USD stage is open. Open the target USD stage in Isaac Sim first.")


# =======================================================================
# =========================== Set Joint Mapping =========================
# =======================================================================

# set joint name mapping
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

SPEED_LIMIT_VALUES = {
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

FORCE_LIMIT_JOINT_MAP = {
    "LEFT_ARM_TORQUE_LIMIT": [
        "Joint_Left_Shoulder_Inner",
        "Joint_Left_Shoulder_Outer",
        "Joint_Left_UpperArm",
        "Joint_Left_Elbow",
        "Joint_Left_Forearm",
        "Joint_Left_Wrist_Upper",
        "Joint_Left_Wrist_Lower",
    ],

    "RIGHT_ARM_TORQUE_LIMIT": [
        "Joint_Right_Shoulder_Inner",
        "Joint_Right_Shoulder_Outer",
        "Joint_Right_UpperArm",
        "Joint_Right_Elbow",
        "Joint_Right_Forearm",
        "Joint_Right_Wrist_Upper",
        "Joint_Right_Wrist_Lower",
    ],
}

FORCE_LIMIT_VALUES = {
    "LEFT_ARM_TORQUE_LIMIT": cfg["ARM_TORQUE_LIMIT"],
    "RIGHT_ARM_TORQUE_LIMIT": cfg["ARM_TORQUE_LIMIT"],
}

GV_FORCE_LIMIT_JOINTS = [
    "Joint_Ground_Vehicle_X",
    "Joint_Ground_Vehicle_Y",
    "Joint_Ground_Vehicle_Z",
]

GV_FORCE_LIMIT_VALUES = [200.0, 200.0, 20.0]

GRIPPER_FORCE_LIMIT_JOINTS = [
    "Joint_Left_Gripper",
    "Joint_Right_Gripper",
]

GRIPPER_FORCE_LIMIT_VALUES = [20.0, 20.0]

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

PID_VALUES = TUNED_PID_VALUES


# =========================================================================================
# ========================== Write the config to the USD stage ============================
# =========================================================================================

# ========================== set speed limits =============================================
def set_speed_limit(stage, joint_name, speed_limit):
    prim = find_prim_by_name(stage, joint_name)
    joint_api = PhysxSchema.PhysxJointAPI.Get(stage, prim.GetPath())
    if not joint_api:
        joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)

    joint_api.CreateMaxJointVelocityAttr(float(speed_limit), True)
    print(f"{joint_name}: maxJointVelocity = {speed_limit}")

def apply_speed_limits():
    for group_name, joint_names in SPEED_LIMIT_JOINT_MAP.items():
        limits = SPEED_LIMIT_VALUES[group_name]
        if len(joint_names) != len(limits):
            raise RuntimeError(
                f"{group_name}: joint count {len(joint_names)} != limit count {len(limits)}"
            )
        for joint_name, limit in zip(joint_names, limits):
            set_speed_limit(stage, joint_name, limit)


# ========================== set_max_force ============================
def set_max_force(stage, joint_name, max_force):
    prim = find_prim_by_name(stage, joint_name)
    drive_name = get_drive_name(prim)
    drive_api = UsdPhysics.DriveAPI.Get(prim, drive_name)
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_name)

    drive_api.CreateMaxForceAttr(float(max_force), True)
    print(f"{joint_name}: {drive_name} maxForce = {max_force}")

def apply_max_force():
    for group_name, joint_names in FORCE_LIMIT_JOINT_MAP.items():
        limits = FORCE_LIMIT_VALUES[group_name]
        if len(joint_names) != len(limits):
            raise RuntimeError(
                f"{group_name}: joint count {len(joint_names)} != limit count {len(limits)}"
            )
        for joint_name, limit in zip(joint_names, limits):
            set_max_force(stage, joint_name, limit)


def apply_gv_max_force():
    for joint_name, limit in zip(GV_FORCE_LIMIT_JOINTS, GV_FORCE_LIMIT_VALUES):
        set_max_force(stage, joint_name, limit)


def apply_gripper_max_force():
    for joint_name, limit in zip(GRIPPER_FORCE_LIMIT_JOINTS, GRIPPER_FORCE_LIMIT_VALUES):
        set_max_force(stage, joint_name, limit)


# ============================== set_PD controller gains ========================================
def set_pd(stage, joint_name, kp, kd):
    prim = find_prim_by_name(stage, joint_name)
    drive_name = get_drive_name(prim)
    drive_api = UsdPhysics.DriveAPI.Get(prim, drive_name)
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(prim, drive_name)

    drive_api.CreateStiffnessAttr(float(kp), True)
    drive_api.CreateDampingAttr(float(kd), True)
    print(f"{joint_name}: {drive_name} stiffness = {kp}, damping = {kd}")

def apply_pd():
    for group_name, joint_names in PID_JOINT_MAP.items():
        kp_values = PID_VALUES[group_name]["Kp"]
        kd_values = PID_VALUES[group_name]["Kd"]
        if len(joint_names) != len(kp_values):
            raise RuntimeError(
                f"{group_name}: joint count {len(joint_names)} != Kp count {len(kp_values)}"
            )
        if len(joint_names) != len(kd_values):
            raise RuntimeError(
                f"{group_name}: joint count {len(joint_names)} != Kd count {len(kd_values)}"
            )
        for joint_name, kp, kd in zip(joint_names, kp_values, kd_values):
            set_pd(stage, joint_name, kp, kd)


def find_prim_by_name(stage, name):
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim
    raise RuntimeError(f"Cannot find prim: {name}")


def get_drive_name(prim):
    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "linear"
    if prim.IsA(UsdPhysics.RevoluteJoint):
        return "angular"
    raise RuntimeError(f"{prim.GetName()} is not a prismatic or revolute joint.")


def save_current_stage():
    if not USD_CONTEXT.save_stage():
        raise RuntimeError("Failed to save the current USD stage.")
    print("Saved current USD stage.")


def main():
    if WRITE_SYNC_SPEED_LIMITS:
        apply_speed_limits()
    if WRITE_GV_MAX_FORCE:
        apply_gv_max_force()
    if WRITE_ARM_MAX_FORCE:
        apply_max_force()
    if WRITE_GRIPPER_MAX_FORCE:
        apply_gripper_max_force()
    if WRITE_PD:
        apply_pd()

    if SAVE_STAGE:
        save_current_stage()

if __name__ == "__main__":
    main()
