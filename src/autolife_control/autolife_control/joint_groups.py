"""Joint groups and default ROS topic names for Autolife control."""

BASE_JOINTS = (
    "Joint_Ground_Vehicle_X",
    "Joint_Ground_Vehicle_Y",
    "Joint_Ground_Vehicle_Z",
)

TORSO_JOINTS = (
    "Joint_Ankle",
    "Joint_Knee",
    "Joint_Waist_Pitch",
    "Joint_Waist_Yaw",
)

HEAD_JOINTS = (
    "Joint_Neck_Roll",
    "Joint_Neck_Pitch",
    "Joint_Neck_Yaw",
)

LEFT_ARM_JOINTS = (
    "Joint_Left_Shoulder_Inner",
    "Joint_Left_Shoulder_Outer",
    "Joint_Left_UpperArm",
    "Joint_Left_Elbow",
    "Joint_Left_Forearm",
    "Joint_Left_Wrist_Upper",
    "Joint_Left_Wrist_Lower",
)

RIGHT_ARM_JOINTS = (
    "Joint_Right_Shoulder_Inner",
    "Joint_Right_Shoulder_Outer",
    "Joint_Right_UpperArm",
    "Joint_Right_Elbow",
    "Joint_Right_Forearm",
    "Joint_Right_Wrist_Upper",
    "Joint_Right_Wrist_Lower",
)

ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

GRIPPER_JOINTS = (
    "Joint_Left_Gripper",
    "Joint_Right_Gripper",
)

CONTROLLABLE_JOINTS = BASE_JOINTS + TORSO_JOINTS + HEAD_JOINTS + ARM_JOINTS + GRIPPER_JOINTS

JOINT_STATES_TOPIC = "/joint_states"
MUX_OUTPUT_TOPIC = "/autolife/joint_command"

COMMAND_TOPICS = {
    "whole_body": "/autolife/command/whole_body",
    "arm": "/autolife/command/arm",
    "head": "/autolife/command/head",
    "torso": "/autolife/command/torso",
    "base": "/autolife/command/base",
    "gripper": "/autolife/command/gripper",
}

SOURCE_JOINTS = {
    "whole_body": CONTROLLABLE_JOINTS,
    "arm": ARM_JOINTS,
    "head": HEAD_JOINTS,
    "torso": TORSO_JOINTS,
    "base": BASE_JOINTS,
    "gripper": GRIPPER_JOINTS,
}

SOURCE_PRIORITY = (
    "whole_body",
    "arm",
    "head",
    "torso",
    "base",
    "gripper",
)
