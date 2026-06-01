# autolife_planning_bridge

ROS 2 bridge for the standalone
[AdaCompNUS/Autolife-Planning](https://github.com/AdaCompNUS/Autolife-Planning)
Python/C++ library.

The bridge exposes high-level planning actions:

```text
/autolife_planning/joint_control
/autolife_planning/pose_control
/autolife_planning/trajectory_execution
```

Reference:

- Autolife-Planning: https://github.com/AdaCompNUS/Autolife-Planning

It reads `/joint_states`, maps the current robot state into the 24-DOF
Autolife-Planning model, plans and time-parameterizes a joint trajectory, then
executes it through:

```text
/whole_body_controller/follow_joint_trajectory
```

Use the existing `autolife-planning` conda environment when running this node:

```bash
source /home/sutai/home/etc/profile.d/conda.sh
conda activate autolife-planning
source /opt/ros/jazzy/setup.bash
source /data/jiaxuanLin/autolife_ws/install/setup.bash
```

Launch:

```bash
ros2 launch autolife_planning_bridge planning.launch.py
```

The launch file prepends these defaults to `PYTHONPATH` so that the planner uses
the conda Pinocchio/Pink stack instead of the ROS-distributed Pinocchio module:

```text
/data/jiaxuanLin/Autolife-Planning
/home/sutai/home/envs/autolife-planning/lib/python3.12/site-packages
```

## Runtime test

The runtime test mirrors `autolife_control/test/controller_test_runner.py`: it
connects to a live ROS graph, runs planning entries such as `base`, `head`,
`torso`, `left_arm`, `right_arm`, and `whole_body`, executes the configured
planned trajectories through the whole-body controller, and checks
`/joint_states`. The default reset pose is `planner_home`, matching
Autolife-Planning's native raised-arm HOME posture. Set `global.reset_pose:
controller_zero` in `test/planning_test_config.yaml` if you want the controller
runtime test's arms-down zero posture.

Start the sim, controllers, and planning bridge first.

Terminal 1: launch the controller stack after the simulator is publishing
`/joint_states`:

```bash
cd /data/jiaxuanLin/autolife_ws
source /home/sutai/home/etc/profile.d/conda.sh
conda activate autolife-planning
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autolife_control controllers.launch.py
```

Terminal 2: launch the planning bridge:

```bash
cd /data/jiaxuanLin/autolife_ws
source /home/sutai/home/etc/profile.d/conda.sh
conda activate autolife-planning
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch autolife_planning_bridge planning.launch.py
```

The launch file runs `planning_server` through the conda Python at
`/home/sutai/home/envs/autolife-planning/bin/python3`, so the compiled
Autolife-Planning extensions and their conda dynamic libraries resolve in the
same environment they were built in.

Terminal 3: run the interactive planning test:

```bash
cd /data/jiaxuanLin/autolife_ws
source /home/sutai/home/etc/profile.d/conda.sh
conda activate autolife-planning
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py --interactive
```

Run a subset:

```bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py \
  --mode head torso left_arm
```

Run a single entry without the start/end reset:

```bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py \
  --no-reset --mode left_arm_pose
```

Choose which robot parts are included in the `whole_body` mode:

```bash
python3 src/autolife_planning_bridge/test/planning_test_runner.py \
  --mode whole_body --whole-body-parts base torso arm
```

The mode entries live under `mode:` in `test/planning_test_config.yaml`.
The default runtime suite covers:

```text
base          joint planning for the mobile base
head          neck joint planning
torso         ankle/knee/waist planning for the torso module
left_arm      left-arm joint plan-only, then trajectory_execution of the cached plan
right_arm     right-arm joint plan and execute
whole_body    whole-body plan-and-execute for all configured parts by default
```

`whole_body` uses `planner_home` as its reference pose, so it does not compound
targets left over from earlier interactive entries. The arm test targets move
outward from HOME instead of folding inward toward the torso.

`left_arm_pose` is available as an optional PoseControl IK smoke test. Use
`--interactive` if you want the runner to wait for Enter before each entry.
Without `--interactive`, the selected entries run immediately.
