#!/usr/bin/env python3
"""Runtime checks for the Autolife planning bridge against a live ROS graph."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from autolife_planning_bridge.dependency_paths import ensure_autolife_planning_importable


try:
    ensure_autolife_planning_importable()
except RuntimeError as exc:
    raise SystemExit(str(exc)) from exc

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from autolife_planning_msgs.action import (
        JointControl,
        PoseControl,
        TrajectoryExecution,
    )
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ModuleNotFoundError as exc:
    if exc.name in {"rclpy", "autolife_planning_msgs"}:
        raise SystemExit(
            "Missing ROS Python package "
            f"{exc.name!r}. Build and source the workspace first:\n\n"
            "  cd /data/jiaxuanLin/autolife_ws\n"
            "  source /home/sutai/home/etc/profile.d/conda.sh\n"
            "  conda activate autolife-planning\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  colcon build --symlink-install --packages-select "
            "autolife_planning_msgs autolife_planning_bridge\n"
            "  source install/setup.bash\n"
            "  python3 src/autolife_planning_bridge/test/planning_test_runner.py "
            "--interactive"
        ) from exc
    raise

from autolife_planning_bridge.geometry import matrix_to_quat_xyzw
from autolife_planning_bridge.joint_mapping import (
    CONTROL_JOINTS,
    PLANNING_JOINTS,
    chain_factory_args,
    chain_joint_names,
    extract_positions,
    planning_to_control_name,
    resolve_chain_key,
)


DEFAULT_CONFIG_PATH = Path(__file__).with_name("planning_test_config.yaml")


class PlanningRuntimeTest(Node):
    def __init__(self, config):
        super().__init__("autolife_planning_runtime_test")
        self.config = config
        self.global_config = config.get("global", {})
        self.mode_configs = _mode_configs(config)
        self.joint_states_topic = self.global_config.get("joint_states_topic", "/joint_states")

        self.positions = {}
        self.velocities = {}
        self.lock = threading.Lock()
        self.joint_state_event = threading.Event()
        self.cached_trajectories = {}
        self.cached_targets = {}
        self._motion_monitor_joints = None
        self._motion_monitor_samples = []

        self.create_subscription(JointState, self.joint_states_topic, self._joint_states_cb, 20)
        self.follow_clients = {}
        self.joint_client = ActionClient(
            self,
            JointControl,
            self.global_config.get("joint_control_action", "/autolife_planning/joint_control"),
        )
        self.pose_client = ActionClient(
            self,
            PoseControl,
            self.global_config.get("pose_control_action", "/autolife_planning/pose_control"),
        )
        self.execution_client = ActionClient(
            self,
            TrajectoryExecution,
            self.global_config.get(
                "trajectory_execution_action",
                "/autolife_planning/trajectory_execution",
            ),
        )

    def run(self, selected_mode_names, interactive=False):
        if not self._wait_for_joint_states(CONTROL_JOINTS):
            self._print_fail("init", f"missing required joints on {self.joint_states_topic}")
            return 1

        if not self._preflight_action_servers(selected_mode_names):
            return 1

        if bool(self.global_config.get("reset_at_start", True)):
            if not self._reset_to_home("script start"):
                return 1

        results = []
        reset_at_end = bool(self.global_config.get("reset_at_end", True))
        for index, name in enumerate(selected_mode_names):
            mode_config = self.mode_configs[name]
            if not mode_config.get("enabled", True):
                self._print_skip(name, "disabled in config")
                continue
            if interactive:
                input(f"\nPress Enter to run planning entry: {name} ")

            try:
                ok = self._run_mode(name, mode_config)
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001 - runtime test should continue.
                ok = False
                self._print_fail(name, str(exc))
            results.append((name, ok))

            pause = float(self.global_config.get("pause_between_steps", 0.0))
            if pause > 0.0 and not (index == len(selected_mode_names) - 1 and reset_at_end):
                time.sleep(pause)

        end_reset_ok = True
        if reset_at_end:
            end_reset_ok = self._reset_to_home("script end")

        failed = [name for name, ok in results if not ok]
        if failed:
            print(f"\nFAILED planning entries: {', '.join(failed)}")
            return 1
        if not end_reset_ok:
            return 1

        print("\nAll enabled planning runtime checks passed.")
        return 0

    def _run_mode(self, name, mode_config):
        mode_type = mode_config.get("type", name)
        if mode_type == "joint":
            ok = self._run_joint_control(
                name,
                mode_config,
                plan_only=bool(mode_config.get("plan_only", True)),
            )
            pose_config = mode_config.get("pose_check", {})
            if ok and pose_config.get("enabled", False):
                ok = self._run_pose_control(
                    name,
                    pose_config,
                    plan_only=bool(pose_config.get("plan_only", True)),
                )
            return ok
        if mode_type == "pose":
            return self._run_pose_control(
                name,
                mode_config,
                plan_only=bool(mode_config.get("plan_only", True)),
            )
        raise ValueError(f"unknown planning entry type '{mode_type}' for '{name}'")

    def _preflight_action_servers(self, selected_mode_names):
        timeout = float(self.global_config.get("action_server_timeout", 10.0))
        clients = {}
        if bool(self.global_config.get("reset_at_start", True)) or bool(
            self.global_config.get("reset_at_end", True)
        ):
            action = self.global_config.get(
                "reset_action",
                "/whole_body_controller/follow_joint_trajectory",
            )
            clients[action] = self._follow_client(action)

        for name in selected_mode_names:
            mode_config = self.mode_configs[name]
            mode_type = mode_config.get("type", name)
            if mode_type == "joint":
                clients[self.joint_client._action_name] = self.joint_client
                if bool(mode_config.get("execute_trajectory", False)):
                    clients[self.execution_client._action_name] = self.execution_client
                if mode_config.get("pose_check", {}).get("enabled", False):
                    clients[self.pose_client._action_name] = self.pose_client
            elif mode_type == "pose":
                clients[self.pose_client._action_name] = self.pose_client

        for action_name, client in clients.items():
            if not client.wait_for_server(timeout_sec=timeout):
                hint = ""
                if action_name == self.global_config.get(
                    "reset_action",
                    "/whole_body_controller/follow_joint_trajectory",
                ):
                    hint = " Run: ros2 launch autolife_control controllers.launch.py"
                self._print_fail(
                    "init",
                    f"action server unavailable: {action_name}. "
                    "Start the controllers and planning bridge before running this mode."
                    f"{hint}",
                )
                return False
        return True

    def _run_joint_control(self, name, test_config, plan_only):
        targets = self._resolve_delta_targets(test_config)
        parts = test_config.get("parts")
        parts_suffix = f" parts={','.join(parts)}" if parts else ""
        print(
            f"\n[RUN] {name} action={self.joint_client._action_name} "
            f"mode={'plan_only' if plan_only else 'plan_execute'} "
            f"targets={_format_targets(targets)}{parts_suffix}"
        )
        goal = JointControl.Goal()
        goal.group = str(test_config.get("group", "auto"))
        goal.joint_names = list(targets.keys())
        goal.positions = [float(v) for v in targets.values()]
        goal.plan_only = bool(plan_only)
        goal.planner_name = str(test_config.get("planner_name", ""))
        goal.time_limit = float(test_config.get("time_limit", 0.0))
        goal.velocity_scaling = float(test_config.get("velocity_scaling", 0.0))
        goal.acceleration_scaling = float(test_config.get("acceleration_scaling", 0.0))
        goal.sample_dt = float(test_config.get("sample_dt", 0.0))

        result = self._send_action(self.joint_client, goal, name)
        ok = bool(result.success)
        if ok and len(result.trajectory.points) < int(test_config.get("min_trajectory_points", 1)):
            ok = False
            self._print_fail(name, "returned trajectory has too few points")

        if ok:
            self.cached_trajectories[name] = result.trajectory
            self.cached_targets[name] = targets
            if plan_only and bool(test_config.get("execute_trajectory", False)):
                ok = self._execute_trajectory(name, result.trajectory, targets, source=name)
            elif not plan_only:
                ok = self._wait_for_targets(name, targets)
        self._print_result(name, ok, result.message)
        return ok

    def _execute_trajectory(self, name, trajectory, targets, source):
        print(f"\n[RUN] {name} action={self.execution_client._action_name} source={source}")
        goal = TrajectoryExecution.Goal()
        goal.trajectory = trajectory
        result = self._send_action(self.execution_client, goal, name)
        ok = bool(result.success)
        if ok:
            ok = self._wait_for_targets(name, targets)
        self._print_result(name, ok, result.message)
        return ok

    def _run_pose_control(self, name, test_config, plan_only):
        target_pose = self._make_pose_target_from_current_fk(test_config)
        print(
            f"\n[RUN] {name} action={self.pose_client._action_name} "
            f"chain={test_config.get('chain', 'left_arm')} plan_only={plan_only}"
        )
        goal = PoseControl.Goal()
        goal.target_pose = target_pose
        goal.chain = str(test_config.get("chain", "left_arm"))
        goal.side = str(test_config.get("side", ""))
        goal.ik_backend = str(test_config.get("ik_backend", ""))
        goal.group = str(test_config.get("group", "auto"))
        goal.plan_only = bool(plan_only)
        goal.planner_name = str(test_config.get("planner_name", ""))
        goal.time_limit = float(test_config.get("time_limit", 0.0))
        goal.velocity_scaling = float(test_config.get("velocity_scaling", 0.0))
        goal.acceleration_scaling = float(test_config.get("acceleration_scaling", 0.0))
        goal.sample_dt = float(test_config.get("sample_dt", 0.0))

        result = self._send_action(self.pose_client, goal, name)
        ok = bool(result.success)
        if ok and len(result.trajectory.points) < int(test_config.get("min_trajectory_points", 1)):
            ok = False
            self._print_fail(name, "returned trajectory has too few points")
        self._print_result(name, ok, result.message)
        return ok

    def _make_pose_target_from_current_fk(self, test_config):
        from autolife_planning.kinematics import create_ik_solver

        chain_key = resolve_chain_key(
            str(test_config.get("chain", "left_arm")),
            str(test_config.get("side", "")),
        )
        chain_name, side = chain_factory_args(chain_key)
        joint_names = chain_joint_names(chain_key)
        current = self._snapshot(CONTROL_JOINTS)
        planning_config = self._control_snapshot_to_planning_config(current)
        seed = np.asarray(
            extract_positions(planning_config, joint_names),
            dtype=np.float64,
        )
        fk_seed = self._pose_fk_seed(seed, joint_names, test_config)
        backend = str(test_config.get("ik_backend", "trac_ik") or "trac_ik")
        solver = create_ik_solver(chain_name, side=side, backend=backend)
        pose = solver.fk(fk_seed)
        if "fk_joint_deltas" in test_config:
            target_position = pose.position
        else:
            delta = np.asarray(
                test_config.get("current_fk_delta_xyz", [0.02, 0.0, 0.0]),
                dtype=float,
            )
            target_position = pose.position + delta
        quat = matrix_to_quat_xyzw(pose.rotation)

        msg = PoseStamped()
        msg.header.frame_id = str(test_config.get("planning_frame", "World"))
        msg.pose.position.x = float(target_position[0])
        msg.pose.position.y = float(target_position[1])
        msg.pose.position.z = float(target_position[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def _pose_fk_seed(self, seed, joint_names, test_config):
        deltas = test_config.get("fk_joint_deltas")
        if not deltas:
            return seed

        target = np.array(seed, dtype=np.float64, copy=True)
        index_by_name = {joint: index for index, joint in enumerate(joint_names)}
        for joint, delta in deltas.items():
            planning_joint = joint
            if planning_joint not in index_by_name:
                raise ValueError(
                    f"fk_joint_deltas contains joint {joint!r}, "
                    f"but chain only includes {list(joint_names)}"
                )
            target[index_by_name[planning_joint]] += float(delta)
        return target

    def _reset_to_home(self, label):
        reset_pose = str(self.global_config.get("reset_pose", "controller_zero"))
        try:
            targets = self._reset_targets(reset_pose)
        except Exception as exc:  # noqa: BLE001
            self._print_fail("reset", str(exc))
            return False

        print(f"\n[RESET] {label}: {len(targets)} planning joints -> {reset_pose}")
        duration = float(self.global_config.get("reset_duration", 4.0))
        action = self.global_config.get(
            "reset_action",
            "/whole_body_controller/follow_joint_trajectory",
        )
        monitor_reset = bool(self.global_config.get("monitor_reset", True))
        if monitor_reset:
            self._begin_motion_monitor(targets.keys())
        try:
            if not self._send_follow_trajectory(action, targets, duration):
                return False
            return self._wait_for_targets(
                "reset",
                targets,
                tolerance=float(self.global_config.get("reset_tolerance", 0.05)),
                timeout=float(self.global_config.get("reset_timeout", 8.0)),
            )
        finally:
            if monitor_reset:
                samples = self._end_motion_monitor()
                self._print_reset_monitor(label, targets, samples)

    def _reset_targets(self, reset_pose):
        if reset_pose == "controller_zero":
            values = np.zeros(len(PLANNING_JOINTS), dtype=np.float64)
        elif reset_pose == "planner_home":
            try:
                from autolife_planning.autolife import HOME_JOINTS
            except Exception as exc:  # noqa: BLE001
                raise RuntimeError(f"cannot import HOME_JOINTS: {exc}") from exc
            values = np.asarray(HOME_JOINTS, dtype=np.float64)
        else:
            raise ValueError(
                "reset_pose must be 'controller_zero' or 'planner_home', "
                f"got {reset_pose!r}"
            )
        return {
            planning_to_control_name(joint): float(value)
            for joint, value in zip(PLANNING_JOINTS, values)
        }

    def _send_follow_trajectory(self, action_name, targets, duration):
        client = self._follow_client(action_name)
        if not client.wait_for_server(
            timeout_sec=float(self.global_config.get("action_server_timeout", 10.0))
        ):
            self._print_fail("reset", f"action server unavailable: {action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_trajectory(targets, duration)
        result = self._send_action(client, goal, "reset")
        return int(result.error_code) == FollowJointTrajectory.Result.SUCCESSFUL

    def _follow_client(self, action_name):
        client = self.follow_clients.get(action_name)
        if client is None:
            client = ActionClient(self, FollowJointTrajectory, action_name)
            self.follow_clients[action_name] = client
        return client

    def _send_action(self, client, goal, label):
        if not client.wait_for_server(
            timeout_sec=float(self.global_config.get("action_server_timeout", 10.0))
        ):
            raise RuntimeError(f"action server unavailable: {client._action_name}")
        send_future = client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, label)
        if not goal_handle.accepted:
            raise RuntimeError(f"{label} goal rejected by {client._action_name}")
        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_future(result_future, label)
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label} action finished with status {wrapped_result.status} "
                f"({goal_status_name(wrapped_result.status)})"
                f"{format_result_detail(wrapped_result.result)}"
            )
        return wrapped_result.result

    def _wait_future(self, future, label):
        timeout = float(self.global_config.get("action_result_timeout", 15.0))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        raise TimeoutError(f"{label} timed out after {timeout:.1f}s")

    def _make_trajectory(self, targets, duration):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(targets.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in targets.values()]
        point.velocities = [0.0 for _ in targets]
        point.time_from_start = _duration(duration)
        trajectory.points.append(point)
        return trajectory

    def _resolve_delta_targets(self, test_config):
        deltas = self._resolve_deltas(test_config)
        reference_pose = str(test_config.get("reference_pose", "current"))
        if reference_pose in ("", "current"):
            reference = self._snapshot(deltas.keys())
        elif reference_pose in ("controller_zero", "planner_home"):
            reset_targets = self._reset_targets(reference_pose)
            reference = {
                joint: reset_targets[joint]
                for joint in deltas
                if joint in reset_targets
            }
            missing = [joint for joint in deltas if joint not in reference]
            if missing:
                raise ValueError(
                    f"reference_pose {reference_pose!r} does not include joints: {missing}"
                )
        else:
            raise ValueError(
                "reference_pose must be 'current', 'controller_zero', or "
                f"'planner_home', got {reference_pose!r}"
            )
        return {joint: reference[joint] + float(delta) for joint, delta in deltas.items()}

    def _resolve_deltas(self, test_config):
        deltas = dict(test_config.get("deltas", {}))
        deltas_by_part = test_config.get("deltas_by_part")
        if not deltas_by_part:
            return deltas

        part_deltas = {}
        parts = test_config.get("parts", list(deltas_by_part))
        for part in parts:
            if part not in deltas_by_part:
                raise ValueError(
                    f"unknown whole_body part {part!r}; available: {sorted(deltas_by_part)}"
                )
            for joint, delta in deltas_by_part[part].items():
                if joint in part_deltas:
                    raise ValueError(
                        f"whole_body part selection assigns {joint!r} more than once"
                    )
                part_deltas[joint] = float(delta)
        part_deltas.update(deltas)
        return part_deltas

    def _control_snapshot_to_planning_config(self, control_positions):
        values = []
        for planning_joint in PLANNING_JOINTS:
            control_joint = planning_to_control_name(planning_joint)
            values.append(float(control_positions[control_joint]))
        return np.asarray(values, dtype=np.float64)

    def _wait_for_joint_states(self, joint_names):
        deadline = time.monotonic() + float(self.global_config.get("initial_state_timeout", 10.0))
        while time.monotonic() < deadline:
            snapshot = self._snapshot(joint_names, missing_ok=True)
            if all(joint in snapshot for joint in joint_names):
                return True
            self.joint_state_event.wait(0.05)
        return False

    def _wait_for_targets(self, label, targets, tolerance=None, timeout=None):
        tolerance = float(tolerance if tolerance is not None else self.global_config.get("target_tolerance", 0.05))
        timeout = float(timeout if timeout is not None else self.global_config.get("target_timeout", 6.0))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self._snapshot(targets.keys(), missing_ok=True)
            if len(snapshot) == len(targets):
                errors = {
                    joint: float(target) - snapshot[joint]
                    for joint, target in targets.items()
                }
                if max(abs(error) for error in errors.values()) <= tolerance:
                    print(f"  [OK] {label} reached target tolerance={tolerance:.4f}")
                    return True
            time.sleep(0.05)
        snapshot = self._snapshot(targets.keys(), missing_ok=True)
        errors = {
            joint: float(target) - snapshot.get(joint, float("nan"))
            for joint, target in targets.items()
        }
        worst_joint, worst_error = max(
            errors.items(),
            key=lambda item: -1.0 if math.isnan(item[1]) else abs(item[1]),
        )
        self._print_fail(
            label,
            f"target timeout; max_error={worst_error:.4f} at {worst_joint}",
        )
        return False

    def _snapshot(self, joint_names, missing_ok=False):
        with self.lock:
            snapshot = {
                joint: self.positions[joint]
                for joint in joint_names
                if joint in self.positions
            }
        if not missing_ok:
            missing = [joint for joint in joint_names if joint not in snapshot]
            if missing:
                raise RuntimeError(f"missing joint states for: {missing}")
        return snapshot

    def _joint_states_cb(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if math.isfinite(position):
                    self.positions[name] = float(position)
            for name, velocity in zip(msg.name, msg.velocity):
                if math.isfinite(velocity):
                    self.velocities[name] = float(velocity)
            if self._motion_monitor_joints is not None:
                sample = {
                    joint: self.positions[joint]
                    for joint in self._motion_monitor_joints
                    if joint in self.positions
                }
                if sample:
                    self._motion_monitor_samples.append((time.monotonic(), sample))
        self.joint_state_event.set()

    def _begin_motion_monitor(self, joint_names):
        with self.lock:
            self._motion_monitor_joints = list(joint_names)
            self._motion_monitor_samples = []

    def _end_motion_monitor(self):
        with self.lock:
            samples = list(self._motion_monitor_samples)
            self._motion_monitor_joints = None
            self._motion_monitor_samples = []
        return samples

    def _print_reset_monitor(self, label, targets, samples):
        if len(samples) < 2:
            print(f"  [MONITOR] reset {label}: insufficient position samples")
            return

        threshold = float(self.global_config.get("reset_overshoot_report_threshold", 0.003))
        min_motion = float(self.global_config.get("reset_overshoot_min_motion", 0.01))
        top_n = int(self.global_config.get("reset_overshoot_top_n", 5))
        start_time = samples[0][0]
        start = samples[0][1]
        final = samples[-1][1]
        overshoots = []
        for joint, target in targets.items():
            if joint not in start or joint not in final:
                continue
            travel = float(target) - start[joint]
            if abs(travel) < min_motion:
                continue
            if travel > 0.0:
                peak_time, peak = max(
                    ((time_stamp, sample[joint]) for time_stamp, sample in samples if joint in sample),
                    key=lambda item: item[1],
                )
                overshoot = peak - float(target)
            else:
                peak_time, peak = min(
                    ((time_stamp, sample[joint]) for time_stamp, sample in samples if joint in sample),
                    key=lambda item: item[1],
                )
                overshoot = float(target) - peak
            if overshoot > threshold:
                overshoots.append(
                    (
                        joint,
                        overshoot,
                        float(target) - final[joint],
                        peak_time - start_time,
                    )
                )

        if not overshoots:
            print(
                f"  [MONITOR] reset {label}: no overshoot above {threshold:.4f}"
            )
            return

        overshoots.sort(key=lambda item: item[1], reverse=True)
        print(
            f"  [MONITOR] reset {label}: overshoot above {threshold:.4f} "
            f"({min(len(overshoots), top_n)}/{len(overshoots)} shown)"
        )
        for joint, overshoot, final_error, peak_time in overshoots[:top_n]:
            print(
                f"    {joint}: overshoot={overshoot:.4f}, "
                f"final_error={final_error:.4f}, t={peak_time:.2f}s"
            )

    def _print_result(self, name, ok, message=""):
        status = "PASS" if ok else "FAIL"
        suffix = f" - {message}" if message else ""
        print(f"  [{status}] {name}{suffix}")

    def _print_fail(self, name, message):
        print(f"  [FAIL] {name}: {message}")

    def _print_skip(self, name, message):
        print(f"  [SKIP] {name}: {message}")


def _duration(seconds):
    sec = int(math.floor(float(seconds)))
    nanosec = int(round((float(seconds) - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def _format_targets(targets):
    return ", ".join(f"{joint}={value:.3f}" for joint, value in targets.items())


def goal_status_name(status):
    names = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return names.get(status, "UNRECOGNIZED")


def format_result_detail(result):
    details = []
    if hasattr(result, "error_code"):
        details.append(f"error_code={result.error_code}")
    if hasattr(result, "error_string") and result.error_string:
        details.append(f"error_string={result.error_string!r}")
    if hasattr(result, "message") and result.message:
        details.append(f"message={result.message!r}")
    if not details:
        return ""
    return "; " + "; ".join(details)


def _load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _mode_configs(config):
    mode = config.get("mode")
    if not isinstance(mode, dict):
        raise ValueError("config missing mode section")
    return mode


def _split_names(values):
    names = []
    for value in values or []:
        names.extend(name.strip() for name in str(value).split(",") if name.strip())
    return names


def _expand_whole_body_parts(parts):
    expanded = []
    for part in parts:
        if part in ("arm", "arms"):
            expanded.extend(["left_arm", "right_arm"])
        else:
            expanded.append(part)
    return expanded


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument(
        "--mode",
        nargs="+",
        default=None,
        help="Planning mode to run. Accepts one or more names after --mode.",
    )
    parser.add_argument(
        "--whole-body-parts",
        nargs="+",
        default=None,
        help=(
            "Parts included when running whole_body. Defaults to all configured "
            "whole-body parts."
        ),
    )
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Disable start/end HOME resets for pure plan-only checks.",
    )
    args = parser.parse_args(argv)

    config = _load_config(Path(args.config))
    if args.no_reset:
        config.setdefault("global", {})["reset_at_start"] = False
        config.setdefault("global", {})["reset_at_end"] = False

    mode_configs = _mode_configs(config)
    selected = _split_names(args.mode)
    selected = selected or [
        name for name, value in mode_configs.items() if value.get("enabled", True)
    ]
    unknown = [name for name in selected if name not in mode_configs]
    if unknown:
        raise SystemExit(f"unknown mode: {unknown}; available: {sorted(mode_configs)}")

    whole_body_parts = _expand_whole_body_parts(_split_names(args.whole_body_parts))
    if whole_body_parts:
        if "whole_body" not in selected:
            raise SystemExit("--whole-body-parts is only valid with --mode whole_body")
        if "whole_body" not in mode_configs:
            raise SystemExit("mode 'whole_body' is missing from the config")
        mode_configs["whole_body"]["parts"] = whole_body_parts

    rclpy.init()
    node = PlanningRuntimeTest(config)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        return node.run(selected, interactive=args.interactive)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
