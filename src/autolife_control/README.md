# autolife_control

ROS 2 control nodes for the Autolife Isaac Sim / OmniGibson simulation setup.

This package converts ROS 2 command interfaces into Autolife joint targets and
publishes a single command stream for the simulator-side articulation adapter.
It is a simulation control package, not a `ros2_control` hardware interface.

## Control Flow

```text
External ROS 2 commands
        |
        v
base / head / torso / arm / gripper / whole-body controllers
        |
        v
/autolife/command/*
        |
        v
joint_command_mux
        |
        v
/autolife/joint_command
        |
        v
Isaac Sim ActionGraph / articulation command adapter
```

All controller nodes read `/joint_states`. The mux waits until all controllable
joints have been seen before publishing `/autolife/joint_command`.

## Launch

Build and source the workspace first:

```bash
cd /data/jiaxuanLin/autolife_ws
colcon build --packages-select autolife_control --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Start the full control layer:

```bash
ros2 launch autolife_control controllers.launch.py
```

The launch file starts:

- `joint_command_mux`
- `base_controller`
- `torso_controller`
- `arm_controller`
- `gripper_controller`
- `head_controller`
- `whole_body_controller`

## Public Interfaces

### Input Commands

| Interface | Message / Action | Consumer |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `base_controller` |
| `/head/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | `head_controller` |
| `/torso/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | `torso_controller` |
| `/left_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `arm_controller` |
| `/right_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `arm_controller` |
| `/left_gripper/gripper_command` | `control_msgs/action/GripperCommand` | `gripper_controller` |
| `/right_gripper/gripper_command` | `control_msgs/action/GripperCommand` | `gripper_controller` |
| `/whole_body_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `whole_body_controller` |

### Internal Command Topics

| Topic | Message | Producer |
| --- | --- | --- |
| `/autolife/command/base` | `sensor_msgs/msg/JointState` | `base_controller` |
| `/autolife/command/head` | `sensor_msgs/msg/JointState` | `head_controller` |
| `/autolife/command/torso` | `sensor_msgs/msg/JointState` | `torso_controller` |
| `/autolife/command/arm` | `sensor_msgs/msg/JointState` | `arm_controller` |
| `/autolife/command/gripper` | `sensor_msgs/msg/JointState` | `gripper_controller` |
| `/autolife/command/whole_body` | `sensor_msgs/msg/JointState` | `whole_body_controller` |

### Simulator Command Output

| Topic | Message | Producer |
| --- | --- | --- |
| `/autolife/joint_command` | `sensor_msgs/msg/JointState` | `joint_command_mux` |

## Joint Command Mux

`joint_command_mux` merges the internal command topics into one joint command
stream. Each source is restricted to its own joint whitelist, defined in
`autolife_control/joint_groups.py`.

Source priority:

```text
whole_body > arm > head > torso > base > gripper
```

If two active sources command the same joint, the higher-priority source wins.
Commands also have source-specific timeouts. When a source command expires, the
mux holds the last target position for that joint and clears its velocity.

Default topics and joint groups live in:

```text
autolife_control/joint_groups.py
```

## Controllers

### Base

`base_controller` integrates `/cmd_vel` into the simulated planar base joints:

- `Joint_Ground_Vehicle_X`
- `Joint_Ground_Vehicle_Y`
- `Joint_Ground_Vehicle_Z`

It also publishes `/odom`. TF publication is available through the `publish_tf`
parameter and is disabled by default.

### Head and Torso

`head_controller` and `torso_controller` accept `JointTrajectory` messages and
sample trajectories with cubic Hermite interpolation.

Head command topic:

```text
/head/joint_trajectory
```

Torso command topic:

```text
/torso/joint_trajectory
```

### Arms

`arm_controller` exposes separate left and right
`FollowJointTrajectory` action servers:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

Each action validates joint names, waypoint timing, position arrays, velocity
arrays, and finite numeric values before execution.

### Grippers

`gripper_controller` exposes one `GripperCommand` action per gripper:

```text
/left_gripper/gripper_command
/right_gripper/gripper_command
```

The command range defaults to:

```text
open:   1.0
closed: 0.0
```

The mapping can be changed with node parameters:

- `command_open_position`
- `command_closed_position`
- `joint_open_position`
- `joint_closed_position`

### Whole Body

`whole_body_controller` exposes:

```text
/whole_body_controller/follow_joint_trajectory
```

It accepts any joint listed in `CONTROLLABLE_JOINTS` from
`autolife_control/joint_groups.py`.

Supported execution modes:

| Mode | Behavior |
| --- | --- |
| `position_goal` | Publish the final trajectory point as the active whole-body target. |
| `trajectory` | Sample the trajectory over time before publishing commands. |

Default:

```text
execution_mode: position_goal
```

## Runtime Test

The runtime test sends real ROS 2 commands to a live simulation and checks that
the corresponding `/joint_states` values change.

Start the simulator and controllers first, then run:

```bash
cd /data/jiaxuanLin/autolife_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/autolife_control/test/controller_test_runner.py
```

Interactive mode:

```bash
python3 src/autolife_control/test/controller_test_runner.py --interactive
```

Run selected controllers:

```bash
python3 src/autolife_control/test/controller_test_runner.py --controllers head,left_arm,whole_body
```

Print the test plan without publishing commands:

```bash
python3 src/autolife_control/test/controller_test_runner.py --dry-run
```

The test configuration is:

```text
src/autolife_control/test/controller_test_config.yaml
```

The config controls:

- which controllers are enabled
- absolute or delta targets
- per-controller target positions and deltas
- reset behavior
- tolerances and timeouts

## Quick Checks

Check that the controllers are running:

```bash
ros2 node list | grep controller
```

Check that simulation state is available:

```bash
ros2 topic echo /joint_states --once
```

Check the mux output:

```bash
ros2 topic echo /autolife/joint_command --once
```

List action servers:

```bash
ros2 action list
```

## Troubleshooting

### No motion

Check these in order:

1. `/joint_states` exists and contains Autolife joint names.
2. The relevant controller node is running.
3. The controller publishes to `/autolife/command/*`.
4. `joint_command_mux` publishes `/autolife/joint_command`.
5. The Isaac Sim ActionGraph subscribes to `/autolife/joint_command`.

### Action goal rejected

Most rejections are caused by one of these:

- joint name not in the controller whitelist
- duplicate joint name
- empty trajectory
- non-increasing waypoint times
- missing position values
- non-finite position or velocity value

### Action goal times out

Check that the target joints are present in `/joint_states` and that Isaac Sim
is applying `/autolife/joint_command`.

### Mux conflict warning

Two sources are commanding the same joint during overlapping timeout windows.
The mux keeps the higher-priority source according to the priority order above.

### Gripper does not reach goal

Check the command-to-joint mapping parameters and confirm that
`Joint_Left_Gripper` and `Joint_Right_Gripper` are present in `/joint_states`.
