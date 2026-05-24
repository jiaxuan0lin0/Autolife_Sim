# autolife_simulation

Isaac Sim / OmniGibson integration package for loading BEHAVIOR-1K scenes with
the Autolife robot, ROS 2 joint bridge, and ROS 2 sensor bridge.

This package is the simulation assembly layer. It does not implement high-level
controllers and it does not validate sensor output. Those responsibilities live
in `autolife_control` and `autolife_sensors`.

## Scope

`autolife_simulation` provides:

- BEHAVIOR-1K / OmniGibson scene loading.
- Autolife robot USD insertion.
- Autolife sensor overlay copying from the original robot world USD.
- Simulation-side drive gain, force limit, and speed limit application.
- Isaac Sim ActionGraph creation for `/joint_states` and `/autolife/joint_command`.
- Isaac Sim ActionGraph creation for cameras, LiDARs, IMU, and TF.
- Runtime sensor manifest generation for downstream sensor tests.

## Main Entry

The main script is:

```text
src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py
```

Run it from an Isaac Sim / OmniGibson Python environment. In the current setup,
that usually means activating the `behavior` conda environment after Isaac Sim
and ROS 2 paths have been injected.

```bash
cd /data/jiaxuanLin/autolife_ws
source /opt/ros/jazzy/setup.bash
conda activate behavior

python3 src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py \
  --scene-model Wainscott_0_int \
  --steps -1
```

`--steps -1` keeps the simulation running until `Ctrl+C`.

For an Isaac Sim `python.sh` workflow, use the same script path:

```bash
source /opt/ros/jazzy/setup.bash

/data/jiaxuanLin/isaacsim/python.sh \
  /data/jiaxuanLin/autolife_ws/src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py \
  --scene-model Wainscott_0_int \
  --steps -1
```

## Created Stage Content

The loader creates or updates these main stage paths:

| Stage path | Purpose |
| --- | --- |
| `/World/autolife` | Autolife robot USD instance |
| `/World/autolife/root_joint` | Articulation root used by the ROS 2 bridge |
| `/ActionGraph` | Joint state publisher and joint command subscriber |
| `/AutolifeSensorGraph` | Sensor, TF, and render product publishers |

## ROS 2 Interfaces

The joint bridge publishes:

| Topic | Message | Direction |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Isaac Sim to ROS 2 |
| `/autolife/joint_command` | `sensor_msgs/msg/JointState` | ROS 2 to Isaac Sim |

The sensor bridge publishes:

| Topic group | Content |
| --- | --- |
| `/tf` | Robot, sensor, and camera optical transforms |
| `/autolife/sensors/.../rgb` | Camera RGB images |
| `/autolife/sensors/.../camera_info` | Camera intrinsics |
| `/autolife/sensors/camera_head_forehead/depth` | Forehead depth image |
| `/autolife/sensors/camera_head_forehead/points` | Forehead point cloud |
| `/autolife/sensors/.../scan` | LiDAR scan outputs |
| `/autolife/sensors/.../points` | LiDAR point cloud outputs |
| `/autolife/sensors/imu` | IMU data |

Exact sensor topics and frame IDs are written to the runtime manifest:

```text
/tmp/autolife_sensor_manifest.json
```

The manifest path can be changed with `--sensor-manifest-path`.

## Important Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `--omnigibson-root` | `/data/jiaxuanLin/BEHAVIOR-1K/OmniGibson` | Local OmniGibson source tree |
| `--scene-model` | `Wainscott_0_int` | BEHAVIOR scene model |
| `--robot-usd` | `src/asset/usd/autolife/autolife.usd` | Autolife robot USD |
| `--autolife-config` | `config/autolife.json` | Motor and limit configuration source |
| `--sensor-overlay-usd` | `src/asset/usd/world.usd` | Source USD for authored sensor prims |
| `--realsense-usd` | `src/asset/usd/realsense/d435i/realsense_d435i.usd` | Realsense asset reference |
| `--robot-prim-path` | `/World/autolife` | Target robot prim path |
| `--graph-path` | `/ActionGraph` | Joint bridge ActionGraph path |
| `--sensor-graph-path` | `/AutolifeSensorGraph` | Sensor bridge ActionGraph path |
| `--ros2-joint-command-topic` | `/autolife/joint_command` | Topic consumed by Isaac articulation controller |
| `--sensor-manifest-path` | `/tmp/autolife_sensor_manifest.json` | Runtime sensor manifest path |
| `--camera-width` | `640` | Camera render product width |
| `--camera-height` | `480` | Camera render product height |
| `--sensor-frame-skip` | `0` | Frames skipped between sensor messages |
| `--steps` | `-1` | Number of simulation steps; `-1` means run until interrupted |

## Optional Modes

### Headless

Run without the Isaac Sim viewport:

```bash
python3 src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py \
  --scene-model Wainscott_0_int \
  --headless \
  --steps -1
```

### Scene-only Check

Disable ROS 2 bridge creation:

```bash
python3 src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py \
  --scene-model Wainscott_0_int \
  --no-ros2-bridge \
  --steps -1
```

### Disable Sensor Bridge

Keep joint control but skip ROS 2 sensor publishers:

```bash
python3 src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py \
  --scene-model Wainscott_0_int \
  --no-ros2-sensors \
  --steps -1
```

### Quick Load

`--quick-load` loads only BEHAVIOR structure categories for faster smoke tests.
It is not the normal mode for validating full scene interaction because the
object-rich scene content is intentionally skipped.

```bash
python3 src/autolife_simulation/scripts/load_behavior_scene_with_autolife.py \
  --scene-model Wainscott_0_int \
  --quick-load \
  --steps 0
```

Use full loading, without `--quick-load`, for normal GUI inspection, object
interaction, controller testing, and sensor testing.

## Asset Dependencies

The Autolife robot USD assets are stored in this workspace under:

```text
src/asset/usd
```

BEHAVIOR-1K scene assets are external. They must be installed through the
BEHAVIOR / OmniGibson workflow and made available to OmniGibson. The default
OmniGibson source path is:

```text
/data/jiaxuanLin/BEHAVIOR-1K/OmniGibson
```

Use `--omnigibson-root` if your checkout is elsewhere.

## Drive Configuration

After importing the robot USD, the loader applies simulation-side drive values:

- PD stiffness and damping.
- Ground vehicle and gripper max force limits.
- Joint speed limits loaded from `config/autolife.json`.

This is done at runtime on the opened stage. The source robot USD is not
rewritten by the loader.

Disable this step with:

```bash
--no-apply-drive-config
```

## Sensor Overlay

The robot USD and the original Autolife `world.usd` are separated. The loader
copies the authored sensor prims from:

```text
src/asset/usd/world.usd
```

onto the robot instance at:

```text
/World/autolife
```

The copied sensor overlay includes cameras, the forehead Realsense setup, IMU,
and front/back LiDAR prims.

Disable this step with:

```bash
--no-sensor-overlay
```

## Utility Scripts

| Script | Purpose |
| --- | --- |
| `joint_state_bridge.py` | Helper that creates `/joint_states` and `/autolife/joint_command` ActionGraph nodes |
| `sensor_bridge.py` | Helper that creates camera, LiDAR, IMU, TF, and manifest output |
| `inspect_autolife_usd.py` | Inspect the Autolife USD for articulation, joints, drives, collisions, and sensors |
| `create_autolife_action_graph.py` | Standalone joint bridge graph creation helper for an already-open stage |
| `apply_autolife_usd_config.py` | Isaac GUI-stage utility for writing drive values into an open USD stage |

The active full-scene workflow is `load_behavior_scene_with_autolife.py`.

## Verification

After the simulation starts, check ROS 2 topics from another terminal:

```bash
cd /data/jiaxuanLin/autolife_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list | grep -E 'joint_states|autolife|tf'
```

Check that joint state publication is active:

```bash
ros2 topic echo /joint_states --once
```

Check the sensor manifest:

```bash
python3 -m json.tool /tmp/autolife_sensor_manifest.json | head -80
```

Then start controllers from `autolife_control` or run sensor validation from
`autolife_sensors`.

## Troubleshooting

### ROS 2 bridge environment error

The loader checks that the ROS 2 bridge environment is configured before Isaac
Sim starts. Source ROS 2 first:

```bash
source /opt/ros/jazzy/setup.bash
```

If using Isaac Sim's bundled Jazzy bridge libraries, set:

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

and ensure the Isaac Sim bridge library path is present in `LD_LIBRARY_PATH`.

### Full scene is missing interactive objects

Check whether the script was launched with `--quick-load`. Full BEHAVIOR object
loading requires running without `--quick-load`.

### No `/joint_states`

Check that:

1. The script was not launched with `--no-ros2-bridge`.
2. `/ActionGraph` exists in the stage.
3. `/World/autolife/root_joint` exists.
4. The ROS 2 bridge extension loaded successfully.

### `/autolife/joint_command` exists but the robot does not move

Check that:

1. `autolife_control` is publishing `/autolife/joint_command`.
2. The graph path is `/ActionGraph`.
3. The articulation controller robot path is `/World/autolife/root_joint`.
4. The command joint names match the names in `/joint_states`.

### Sensor topics are missing

Check that:

1. The script was not launched with `--no-ros2-sensors`.
2. The sensor overlay was copied successfully.
3. `/AutolifeSensorGraph` exists.
4. `/tmp/autolife_sensor_manifest.json` was written.

### Sensor TF is missing in RViz

Check `/tf` and the manifest frame IDs:

```bash
ros2 topic echo /tf --once
python3 -m json.tool /tmp/autolife_sensor_manifest.json | grep frame_id
```
