# autolife_sensors

Sensor naming, runtime validation, and RViz configuration for the Autolife
Isaac Sim / OmniGibson setup.

This package does not publish sensor data by itself. Sensor publishers are
created by the Isaac Sim ActionGraph in `autolife_simulation`. This package
defines the expected naming rules, validates live ROS 2 sensor topics, and
provides an RViz preset for inspection.

## Scope

`autolife_sensors` provides:

- Stable sensor names and ROS 2 topic rules.
- Message type and frame ID conventions.
- Runtime sensor checks based on the simulation manifest.
- RViz display configuration for cameras, point clouds, LiDARs, and TF.

The main files are:

| File | Purpose |
| --- | --- |
| `autolife_sensors/sensor_config.py` | Shared naming, topic, type, and frame rules |
| `test/sensor_test_runner.py` | Runtime ROS 2 sensor validator |
| `test/sensor_test_config.yaml` | Sensor test thresholds and options |
| `rviz/autolife_sensors.rviz` | RViz preset |

## Sensor Topic Root

All Autolife sensor topics use:

```text
/autolife/sensors
```

Sensor role names are derived from USD prim paths and normalized in
`sensor_config.py`.

## Sensor Names

Current stable sensor names:

| Sensor name | Source link / role |
| --- | --- |
| `camera_head_forehead` | Forehead Realsense camera |
| `camera_head_left_eye` | Left eye camera |
| `camera_head_right_eye` | Right eye camera |
| `camera_head_back` | Head back camera |
| `camera_gripper_left` | Left gripper camera |
| `camera_gripper_right` | Right gripper camera |
| `lidar_front` | Front LiDAR |
| `lidar_back` | Back LiDAR |
| `imu` | IMU |

## Published Outputs

### Cameras

The forehead camera publishes RGB, depth, point cloud, and camera info:

| Topic | Type |
| --- | --- |
| `/autolife/sensors/camera_head_forehead/rgb` | `sensor_msgs/msg/Image` |
| `/autolife/sensors/camera_head_forehead/depth` | `sensor_msgs/msg/Image` |
| `/autolife/sensors/camera_head_forehead/points` | `sensor_msgs/msg/PointCloud2` |
| `/autolife/sensors/camera_head_forehead/camera_info` | `sensor_msgs/msg/CameraInfo` |

Other cameras publish RGB and camera info:

| Sensor | Topics |
| --- | --- |
| `camera_head_left_eye` | `rgb`, `camera_info` |
| `camera_head_right_eye` | `rgb`, `camera_info` |
| `camera_head_back` | `rgb`, `camera_info` |
| `camera_gripper_left` | `rgb`, `camera_info` |
| `camera_gripper_right` | `rgb`, `camera_info` |

Example:

```text
/autolife/sensors/camera_head_left_eye/rgb
/autolife/sensors/camera_head_left_eye/camera_info
```

### LiDAR

Front and back LiDARs publish point clouds:

| Topic | Type |
| --- | --- |
| `/autolife/sensors/lidar_front/points` | `sensor_msgs/msg/PointCloud2` |
| `/autolife/sensors/lidar_back/points` | `sensor_msgs/msg/PointCloud2` |

### IMU

The IMU publishes:

| Topic | Type |
| --- | --- |
| `/autolife/sensors/imu` | `sensor_msgs/msg/Imu` |

### TF

Transforms are published on:

```text
/tf
```

with type:

```text
tf2_msgs/msg/TFMessage
```

## Frame IDs

Camera messages use optical frames:

```text
camera_head_forehead_optical_frame
camera_head_left_eye_optical_frame
camera_head_right_eye_optical_frame
camera_head_back_optical_frame
camera_gripper_left_optical_frame
camera_gripper_right_optical_frame
```

LiDAR and IMU messages use their robot link frames:

```text
Link_Lidar_Front
Link_Lidar_Back
Link_IMU
```

The RViz preset uses:

```text
World
```

as the fixed frame.

## Runtime Manifest

The simulation writes a sensor manifest after creating the sensor ActionGraph.
Default path:

```text
.cache/runtime/autolife_sensor_manifest.json
```

The manifest is generated from the actual USD stage and contains:

- graph path
- robot prim path
- discovered sensors
- sensor USD prim paths
- ROS 2 topics
- ROS 2 message types
- expected frame IDs
- TF targets

The runtime test reads this manifest instead of hard-coding a separate expected
topic list. The manifest is for validation and debugging; normal ROS 2 sensor
publishing does not require a consumer to read it.

The manifest path is controlled by the simulation loader:

```bash
python3 src/autolife_simulation/scripts/start_autolife_simulation.py \
  --scene-model Wainscott_0_int \
  --sensor-manifest-path .cache/runtime/autolife_sensor_manifest.json \
  --steps -1
```

## Runtime Sensor Test

Start the simulation with ROS 2 sensors enabled first. From another terminal:

```bash
cd "$(git rev-parse --show-toplevel)"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/autolife_sensors/test/sensor_test_runner.py
```

Print expected topics from the manifest without subscribing:

```bash
python3 src/autolife_sensors/test/sensor_test_runner.py --dry-run
```

Use a custom manifest path:

```bash
python3 src/autolife_sensors/test/sensor_test_runner.py \
  --manifest .cache/runtime/autolife_sensor_manifest.json
```

Use a custom test config:

```bash
python3 src/autolife_sensors/test/sensor_test_runner.py \
  --config src/autolife_sensors/test/sensor_test_config.yaml
```

The test checks:

- expected topics are present
- message types match the manifest
- each topic receives data
- image messages have nonzero size and nonempty data
- camera info messages have nonzero size and finite intrinsics
- point clouds contain data and `x`, `y`, `z` fields
- IMU values are finite
- message `header.frame_id` matches the manifest
- `/tf` contains the expected sensor frames

## Test Configuration

Default config:

```text
src/autolife_sensors/test/sensor_test_config.yaml
```

Important fields:

| Field | Meaning |
| --- | --- |
| `global.manifest_path` | Manifest path used when `--manifest` is not provided |
| `global.topic_timeout` | Time allowed for expected topics to appear |
| `global.sample_duration` | Time spent collecting sample messages |
| `global.min_messages` | Minimum messages required per topic |
| `global.require_tf` | Whether expected frame IDs must appear in `/tf` |
| `image.require_nonempty_data` | Require image data bytes |
| `camera_info.require_finite_intrinsics` | Require finite camera matrices |
| `pointcloud.require_xyz_fields` | Require `x`, `y`, `z` fields |
| `imu.require_finite_values` | Require finite IMU numeric values |

## RViz

Build and source the workspace:

```bash
cd "$(git rev-parse --show-toplevel)"
colcon build --packages-select autolife_sensors --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Start RViz with the preset:

```bash
rviz2 -d "$(ros2 pkg prefix autolife_sensors)/share/autolife_sensors/rviz/autolife_sensors.rviz"
```

The preset includes:

- Grid
- TF
- Forehead RGB
- Left eye RGB
- Right eye RGB
- Head back RGB
- Left gripper RGB
- Right gripper RGB
- Forehead depth
- Forehead point cloud
- Front LiDAR point cloud
- Back LiDAR point cloud

RViz may print:

```text
Stereo is NOT SUPPORTED
```

This is an RViz/OpenGL capability message and does not indicate that the sensor
topics are broken.

## Quick Checks

List Autolife sensor topics:

```bash
ros2 topic list | grep /autolife/sensors
```

Check one RGB image:

```bash
ros2 topic echo /autolife/sensors/camera_head_forehead/rgb --once
```

Check the forehead point cloud type:

```bash
ros2 topic type /autolife/sensors/camera_head_forehead/points
```

Check TF frames:

```bash
ros2 topic echo /tf --once
```

Inspect the manifest:

```bash
python3 -m json.tool .cache/runtime/autolife_sensor_manifest.json | head -100
```
