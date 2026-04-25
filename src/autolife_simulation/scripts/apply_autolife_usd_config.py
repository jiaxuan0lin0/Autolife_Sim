import json
import omni.usd
from pxr import UsdPhysics
from pxr import PhysxSchema

# Run this script inside Isaac Sim after opening the target USD stage.

# ========================================================================================
# ========================== Selection for the config writing ============================
# ========================================================================================
WRITE_PD = False
WRITE_ARM_MAX_FORCE = False
WRITE_SYNC_SPEED_LIMITS = True
SAVE_STAGE = True
CONFIG = "/data/jiaxuanLin/autolife_ws/src/autolife_simulation/config/autolife.json"
USD_CONTEXT = omni.usd.get_context()

# =======================================================================
# ========================== Load the config ============================
# =======================================================================

with open(CONFIG, "r", encoding="utf-8") as f:
    cfg = json.load(f)

stage = USD_CONTEXT.get_stage()
if stage is None:
    raise RuntimeError("No USD stage is open. Open the target USD stage in Isaac Sim first.")


# =======================================================================
# =========================== Set Joint Mapping =========================
# =======================================================================

# set joint name mapping
SPEED_LIMIT_JOINT_MAP = {
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

PID_JOINT_MAP = {
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

PID_VALUES = {
    pid_name: {
        "Kp": cfg[pid_name]["Kp"],
        "Kd": cfg[pid_name]["Kd"],
    }
    for pid_name in PID_JOINT_MAP
}


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
    drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")

    drive_api.CreateMaxForceAttr(float(max_force), True)
    print(f"{joint_name}: maxForce = {max_force}")

def apply_max_force():
    for group_name, joint_names in FORCE_LIMIT_JOINT_MAP.items():
        limits = FORCE_LIMIT_VALUES[group_name]
        if len(joint_names) != len(limits):
            raise RuntimeError(
                f"{group_name}: joint count {len(joint_names)} != limit count {len(limits)}"
            )
        for joint_name, limit in zip(joint_names, limits):
            set_max_force(stage, joint_name, limit)


# ============================== set_PD controller gains ========================================
def set_pd(stage, joint_name, kp, kd):
    prim = find_prim_by_name(stage, joint_name)
    drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")

    drive_api.CreateStiffnessAttr(float(kp), True)
    drive_api.CreateDampingAttr(float(kd), True)
    print(f"{joint_name}: stiffness = {kp}, damping = {kd}")

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


def save_current_stage():
    if not USD_CONTEXT.save_stage():
        raise RuntimeError("Failed to save the current USD stage.")
    print("Saved current USD stage.")


def main():
    if WRITE_SYNC_SPEED_LIMITS:
        apply_speed_limits()
    if WRITE_ARM_MAX_FORCE:
        apply_max_force()
    if WRITE_PD:
        apply_pd()

    if SAVE_STAGE:
        save_current_stage()

if __name__ == "__main__":
    main()
