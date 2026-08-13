# autolife_simulation

`autolife_simulation` connects the Autolife USD robot and Isaac Sim to the ROS
2 packages in this repository. It supports direct USD loading and OmniGibson
through a single launcher.

For complete installation and system startup instructions, see the repository
[README](../../README.md).

## Start the Simulator

```bash
source scripts/activate_autolife_sim.sh
python src/autolife_simulation/scripts/start_autolife_simulation.py \
  --scene-model molmospace_scene \
  --steps -1
```

The launcher resolves repository files from its own location, so the repository
does not need to be installed at a fixed absolute path.

## Scene Selection

Set `--scene-model` to one of the following values:

| Value | Source |
| --- | --- |
| `molmospace_scene` | MolmoSpaces configuration in `config/molmospace_scene.json` |
| `desk` | Local configuration in `config/desk.json` |
| `Wainscott_0_int` or another model name | BEHAVIOR-1K through OmniGibson |

Before using `molmospace_scene` for the first time, install its assets:

```bash
python src/autolife_simulation/scripts/download_molmospace_scene.py
```

Use another model by changing only the launcher argument:

```bash
python src/autolife_simulation/scripts/start_autolife_simulation.py \
  --scene-model desk \
  --steps -1
```

## Isaac Sim Stage

The launcher adds these prims to the selected scene:

| Prim | Purpose |
| --- | --- |
| `/World/autolife` | Autolife robot articulation |
| `/ActionGraph` | Joint-state publisher and joint-command subscriber |
| `/AutolifeSensorGraph` | Camera, LiDAR, IMU, and TF publishers |

Robot drive values and sensor definitions are copied from repository assets at
startup. Source robot, sensor, and scene USD files are not modified.

## ROS 2 Interfaces

The joint bridge publishes `/joint_states` and subscribes to
`/autolife/joint_command`. Sensor outputs are published under
`/autolife/sensors`; topic and frame conventions are documented in
[autolife_sensors](../autolife_sensors/README.md).

Run the simulator and ROS 2 in separate terminals:

```text
Simulator: source scripts/activate_autolife_sim.sh
ROS 2:     source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

The simulator uses Python 3.11, while system ROS 2 Jazzy uses Python 3.12.
Communication between the two terminals uses DDS.

## Common Options

| Option | Purpose |
| --- | --- |
| `--scene-model NAME` | Select the scene |
| `--steps -1` | Run until interrupted |
| `--headless` | Run without the GUI viewport |
| `--robot-position X Y Z` | Override the robot spawn position |
| `--robot-orientation-wxyz W X Y Z` | Override robot orientation |
| `--no-ros2-bridge` | Disable the joint bridge |
| `--no-ros2-sensors` | Disable sensor publishers |
| `--no-apply-drive-config` | Keep drive values authored in the robot USD |
| `--no-sensor-overlay` | Skip repository sensor overlays |
| `--molmospace-config PATH` | Select a direct-USD configuration |
| `--molmospace-asset-root PATH` | Override the MolmoSpaces asset directory |

Run the launcher with `--help` for the complete argument list.

## Configuration and Utilities

| Path | Purpose |
| --- | --- |
| `config/autolife.json` | Autolife control and drive configuration |
| `config/molmospace_scene.json` | MolmoSpaces asset, spawn, physics, and interaction settings |
| `config/desk.json` | Local desk scene settings |
| `scripts/download_molmospace_scene.py` | Install configured MolmoSpaces assets |
| `scripts/inspect_autolife_usd.py` | Inspect articulation, joints, drives, and collisions |
| `scripts/apply_autolife_usd_config.py` | Apply drive values inside Isaac Sim |
| `scripts/sensor_bridge.py` | Build the ROS 2 sensor ActionGraph |
