# autolife_planning_bridge

ROS 2 action bridge for
[Autolife-Planning](https://github.com/AdaCompNUS/Autolife-Planning). It reads
the simulated robot state, invokes the native planner, time-parameterizes the
result, and sends trajectories to `autolife_control`.

Autolife-Planning is optional and is not installed into the `autolife_sim`
simulator environment because its native extensions use the system ROS Python
ABI.

## Interfaces

The bridge exposes:

```text
/autolife_planning/joint_control
/autolife_planning/pose_control
/autolife_planning/trajectory_execution
```

It subscribes to `/joint_states` and executes through:

```text
/whole_body_controller/follow_joint_trajectory
```

## Setup

Install Autolife-Planning by following its upstream instructions. A
repository-local checkout may be placed at `.deps/Autolife-Planning`:

```bash
git clone https://github.com/AdaCompNUS/Autolife-Planning.git \
  .deps/Autolife-Planning
```

Activate the environment in which Autolife-Planning was built, then source ROS
2 and this workspace:

```bash
conda activate autolife-planning
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

The launch file derives the planner Python executable and site-packages from
the active environment. Non-standard installations can override discovery with:

```bash
export AUTOLIFE_PLANNING_ROOT=/path/to/Autolife-Planning
export AUTOLIFE_PLANNING_PYTHON=/path/to/planner/python
export AUTOLIFE_PLANNING_PYTHON_SITE=/path/to/site-packages
```

## Launch

Start the simulator and controllers first. Then run:

```bash
ros2 launch autolife_planning_bridge planning.launch.py
```

Launch arguments with the same names are also available:

```bash
ros2 launch autolife_planning_bridge planning.launch.py \
  planner_root:=.deps/Autolife-Planning
```

## Runtime Test

```bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py --interactive
```

Run selected planning modes:

```bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py \
  --mode head torso left_arm
```

Run whole-body planning for selected parts:

```bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py \
  --mode whole_body --whole-body-parts base torso arm
```

Test targets and reset behavior are configured in
`test/planning_test_config.yaml`. The default suite covers base, head, torso,
left arm, right arm, and whole-body planning. `left_arm_pose` is an optional
PoseControl IK check.
