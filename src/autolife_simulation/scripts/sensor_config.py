"""ROS2 sensor bridge naming and output configuration for Autolife."""

SENSOR_TOPIC_ROOT = "/autolife/sensors"

CAMERA_OUTPUTS = {
    "camera_head_forehead": ("rgb", "depth", "depth_pcl", "camera_info"),
}

DEFAULT_CAMERA_OUTPUTS = ("rgb", "camera_info")

LIDAR_OUTPUTS = {
    "lidar_front": ("point_cloud",),
    "lidar_back": ("point_cloud",),
}

DEFAULT_LIDAR_OUTPUTS = ("point_cloud",)

SENSOR_NAME_RULES = (
    ("Link_Camera_Head_Forehead", "camera_head_forehead"),
    ("Link_Camera_Head_Left_Eye", "camera_head_left_eye"),
    ("Link_Camera_Head_Right_Eye", "camera_head_right_eye"),
    ("Link_Camera_Head_Back", "camera_head_back"),
    ("Link_Camera_Gripper_Left", "camera_gripper_left"),
    ("Link_Camera_Gripper_Right", "camera_gripper_right"),
    ("Link_Lidar_Front", "lidar_front"),
    ("Link_Lidar_Back", "lidar_back"),
    ("Link_IMU", "imu"),
)

CAMERA_TOPIC_SUFFIXES = {
    "rgb": "rgb",
    "depth": "depth",
    "depth_pcl": "points",
    "camera_info": "camera_info",
}

LIDAR_TOPIC_SUFFIXES = {
    "laser_scan": "scan",
    "point_cloud": "points",
}


def sensor_role_from_path(prim_path: str, fallback_prefix: str) -> str:
    for path_token, role in SENSOR_NAME_RULES:
        if path_token in prim_path:
            return role
    return f"{fallback_prefix}_{sanitize_name(prim_path.rsplit('/', 1)[-1])}"


def camera_outputs_for_role(role: str):
    return CAMERA_OUTPUTS.get(role, DEFAULT_CAMERA_OUTPUTS)


def include_camera_prim(role: str, prim_path: str) -> bool:
    if role == "camera_head_forehead" and prim_path.rsplit("/", 1)[-1] == "DepthCamera":
        return False
    return True


def lidar_outputs_for_role(role: str):
    return LIDAR_OUTPUTS.get(role, DEFAULT_LIDAR_OUTPUTS)


def camera_topic(sensor_name: str, output_type: str) -> str:
    suffix = CAMERA_TOPIC_SUFFIXES[output_type]
    return f"{SENSOR_TOPIC_ROOT}/{sensor_name}/{suffix}"


def lidar_topic(sensor_name: str, output_type: str) -> str:
    suffix = LIDAR_TOPIC_SUFFIXES[output_type]
    return f"{SENSOR_TOPIC_ROOT}/{sensor_name}/{suffix}"


def imu_topic(sensor_name: str) -> str:
    return f"{SENSOR_TOPIC_ROOT}/{sensor_name}"


def frame_id_for_sensor(sensor_name: str) -> str:
    return sensor_name


def sanitize_name(value: str) -> str:
    result = []
    previous_was_separator = False
    for char in value:
        if char.isalnum():
            result.append(char.lower())
            previous_was_separator = False
        elif not previous_was_separator:
            result.append("_")
            previous_was_separator = True

    name = "".join(result).strip("_")
    return name or "sensor"
