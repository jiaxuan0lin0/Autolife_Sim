"""ROS 2 sensor naming and output configuration for Autolife."""

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

SENSOR_FRAME_IDS = {
    "lidar_front": "Link_Lidar_Front",
    "lidar_back": "Link_Lidar_Back",
    "imu": "Link_IMU",
}

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

CAMERA_MESSAGE_TYPES = {
    "rgb": "sensor_msgs/msg/Image",
    "depth": "sensor_msgs/msg/Image",
    "depth_pcl": "sensor_msgs/msg/PointCloud2",
    "camera_info": "sensor_msgs/msg/CameraInfo",
}

LIDAR_MESSAGE_TYPES = {
    "laser_scan": "sensor_msgs/msg/LaserScan",
    "point_cloud": "sensor_msgs/msg/PointCloud2",
}

IMU_MESSAGE_TYPE = "sensor_msgs/msg/Imu"
TF_TOPIC = "/tf"
TF_MESSAGE_TYPE = "tf2_msgs/msg/TFMessage"


def sensor_role_from_path(prim_path: str, fallback_prefix: str) -> str:
    """Return a stable sensor role inferred from a USD prim path."""
    for path_token, role in SENSOR_NAME_RULES:
        if path_token in prim_path:
            return role
    return f"{fallback_prefix}_{sanitize_name(prim_path.rsplit('/', 1)[-1])}"


def camera_outputs_for_role(role: str):
    """Return camera bridge outputs for a sensor role."""
    return CAMERA_OUTPUTS.get(role, DEFAULT_CAMERA_OUTPUTS)


def include_camera_prim(role: str, prim_path: str) -> bool:
    """Return whether a camera prim should get a ROS bridge output."""
    if role == "camera_head_forehead" and prim_path.rsplit("/", 1)[-1] == "DepthCamera":
        return False
    return True


def lidar_outputs_for_role(role: str):
    """Return LiDAR bridge outputs for a sensor role."""
    return LIDAR_OUTPUTS.get(role, DEFAULT_LIDAR_OUTPUTS)


def camera_topic(sensor_name: str, output_type: str) -> str:
    """Return the ROS topic for a camera output."""
    suffix = CAMERA_TOPIC_SUFFIXES[output_type]
    return f"{SENSOR_TOPIC_ROOT}/{sensor_name}/{suffix}"


def camera_message_type(output_type: str) -> str:
    """Return the ROS message type for a camera output."""
    return CAMERA_MESSAGE_TYPES[output_type]


def lidar_topic(sensor_name: str, output_type: str) -> str:
    """Return the ROS topic for a LiDAR output."""
    suffix = LIDAR_TOPIC_SUFFIXES[output_type]
    return f"{SENSOR_TOPIC_ROOT}/{sensor_name}/{suffix}"


def lidar_message_type(output_type: str) -> str:
    """Return the ROS message type for a LiDAR output."""
    return LIDAR_MESSAGE_TYPES[output_type]


def imu_topic(sensor_name: str) -> str:
    """Return the ROS topic for an IMU output."""
    return f"{SENSOR_TOPIC_ROOT}/{sensor_name}"


def frame_id_for_sensor(sensor_name: str) -> str:
    """Return the ROS frame id for a sensor output."""
    return SENSOR_FRAME_IDS.get(sensor_name, sensor_name)


def camera_optical_frame_id(sensor_name: str) -> str:
    """Return the stable optical frame id for a camera output."""
    return f"{sensor_name}_optical_frame"


def sanitize_name(value: str) -> str:
    """Convert an arbitrary USD token to a ROS-friendly lower-case name."""
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
