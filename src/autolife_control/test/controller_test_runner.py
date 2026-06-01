#!/usr/bin/env python3
"""Run runtime controller checks against a live Autolife ROS 2 session."""

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import yaml

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from autolife_control.joint_groups import (
    ARM_JOINTS,
    BASE_JOINTS,
    COMMAND_TOPICS,
    CONTROLLABLE_JOINTS,
    GRIPPER_JOINTS,
    HEAD_JOINTS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    TORSO_JOINTS,
)


DEFAULT_CONFIG_PATH = Path(__file__).with_name("controller_test_config.yaml")
FOLLOW_JOINT_RESULT_SUCCESS = FollowJointTrajectory.Result.SUCCESSFUL


class ControllerRuntimeTest(Node):
    """Exercise the controller stack from ROS inputs to observed joint states."""

    def __init__(self, config):
        """Create subscriptions, action clients, and publishers for the checks."""
        super().__init__("controller_runtime_test")
        self.config = config
        self.global_config = config.get("global", {})
        self.joint_states_topic = self.global_config.get(
            "joint_states_topic", "/joint_states"
        )
        self.mux_output_topic = self.global_config.get(
            "mux_output_topic", "/autolife/joint_command"
        )

        self.positions = {}
        self.velocities = {}
        self.topic_counts = {}
        self.latest_messages = {}
        self.lock = threading.Lock()
        self.joint_state_event = threading.Event()
        self._controller_test_publishers = {}
        self.follow_clients = {}
        self.gripper_clients = {}
        self._controller_test_subscriptions = []

        self._controller_test_subscriptions.append(
            self.create_subscription(
                JointState,
                self.joint_states_topic,
                self._joint_states_cb,
                20,
            )
        )
        self._watch_joint_state_topic(self.mux_output_topic)
        for topic in COMMAND_TOPICS.values():
            self._watch_joint_state_topic(topic)

    def run(self, selected_controllers, interactive=False):
        """Run the selected controller checks and return a process status code."""
        if not self._wait_for_joint_states(CONTROLLABLE_JOINTS):
            self._print_fail(
                "init",
                f"missing required joints on {self.joint_states_topic}",
            )
            return 1

        if bool(self.global_config.get("reset_at_start", True)):
            if not self._reset_all_to_zero("script start"):
                return 1

        results = []
        reset_at_end = bool(self.global_config.get("reset_at_end", True))
        for index, name in enumerate(selected_controllers):
            controller_config = self.config["controllers"][name]
            if not controller_config.get("enabled", True):
                self._print_skip(name, "disabled in config")
                continue

            if interactive:
                input(f"\nPress Enter to run controller test: {name} ")

            try:
                if bool(self.global_config.get("reset_before_each", False)):
                    reset_ok = self._reset_all_to_zero(f"before {name}")
                else:
                    reset_ok = True
                ok = reset_ok and self._run_controller(name, controller_config)
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001 - runtime tool should continue.
                ok = False
                self._print_fail(name, str(exc))

            results.append((name, ok))
            pause = float(self.global_config.get("pause_between_steps", 0.0))
            is_last_controller = index == len(selected_controllers) - 1
            if pause > 0.0 and not (is_last_controller and reset_at_end):
                time.sleep(pause)

        end_reset_ok = True
        if reset_at_end:
            end_reset_ok = self._reset_all_to_zero("script end")

        failed = [name for name, ok in results if not ok]
        if failed:
            print(f"\nFAILED controllers: {', '.join(failed)}")
            return 1
        if not end_reset_ok:
            return 1

        print("\nAll enabled controller runtime checks passed.")
        return 0

    def _run_controller(self, name, controller_config):
        if name == "base":
            return self._run_base(controller_config)
        if name == "head":
            return self._run_topic_trajectory(name, controller_config, HEAD_JOINTS)
        if name == "torso":
            return self._run_topic_trajectory(name, controller_config, TORSO_JOINTS)
        if name == "left_arm":
            return self._run_action_trajectory(name, controller_config, LEFT_ARM_JOINTS)
        if name == "right_arm":
            return self._run_action_trajectory(
                name, controller_config, RIGHT_ARM_JOINTS
            )
        if name == "whole_body":
            if controller_config.get("command_mode", "action") == "direct_topic":
                topic = controller_config.get("topic", COMMAND_TOPICS["whole_body"])
                return self._run_direct_joint_command(
                    name,
                    controller_config,
                    CONTROLLABLE_JOINTS,
                    topic,
                )
            return self._run_action_trajectory(
                name, controller_config, CONTROLLABLE_JOINTS
            )
        if name == "gripper":
            return self._run_gripper(controller_config)
        raise ValueError(f"unknown controller '{name}'")

    def _reset_all_to_zero(self, label):
        reset_action = self.global_config.get(
            "reset_action",
            self.config["controllers"]["whole_body"].get(
                "action",
                "/whole_body_controller/follow_joint_trajectory",
            ),
        )
        duration = float(self.global_config.get("reset_duration", 1.5))
        timeout = float(self.global_config.get("reset_timeout", 4.0))
        tolerance = float(self.global_config.get("reset_tolerance", 0.03))
        if duration < 0.0:
            raise ValueError("reset_duration must be non-negative")
        if timeout < 0.0:
            raise ValueError("reset_timeout must be non-negative")

        targets = {joint: 0.0 for joint in CONTROLLABLE_JOINTS}
        print(
            f"\n[RESET] {label}: "
            f"{len(targets)} joints -> 0.0 via {reset_action}"
        )

        action_ok = self._send_follow_trajectory(reset_action, targets, duration)
        state_ok = self._wait_for_targets("reset", targets, tolerance, timeout)
        self._publish_zero_controller_targets(1.0)
        if action_ok and state_ok:
            print(f"  [RESET] reached zero tolerance={tolerance:.4f}")
            return True

        max_joint, max_error = self._max_target_error(targets)
        self._print_fail(
            "reset",
            f"zero reset timed out at {label}; "
            f"max_error={max_error:.4f} at {max_joint}",
        )
        return False

    def _publish_zero_controller_targets(self, duration):
        head_topic = self.config["controllers"]["head"].get("topic", "/head/joint_trajectory")
        torso_topic = self.config["controllers"]["torso"].get("topic", "/torso/joint_trajectory")
        cmd_vel_topic = self.config["controllers"]["base"].get("cmd_vel_topic", "/cmd_vel")

        head_targets = {joint: 0.0 for joint in HEAD_JOINTS}
        torso_targets = {joint: 0.0 for joint in TORSO_JOINTS}
        self._publisher(head_topic, JointTrajectory).publish(
            self._make_trajectory(head_targets, duration)
        )
        self._publisher(torso_topic, JointTrajectory).publish(
            self._make_trajectory(torso_targets, duration)
        )
        self._publisher(cmd_vel_topic, Twist).publish(Twist())

    def _run_base(self, controller_config):
        duration = float(controller_config.get("duration", 1.0))
        topic = controller_config.get("cmd_vel_topic", "/cmd_vel")
        command = controller_config.get("command", {})
        before = self._snapshot(BASE_JOINTS)

        print(
            "\n[RUN] base "
            f"cmd_vel=({command.get('linear_x', 0.0):.3f}, "
            f"{command.get('linear_y', 0.0):.3f}, "
            f"{command.get('angular_z', 0.0):.3f}) duration={duration:.2f}s"
        )
        twist = Twist()
        twist.linear.x = float(command.get("linear_x", 0.0))
        twist.linear.y = float(command.get("linear_y", 0.0))
        twist.angular.z = float(command.get("angular_z", 0.0))

        publisher = self._publisher(topic, Twist)
        self._wait_for_subscribers(publisher, topic)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            publisher.publish(twist)
            time.sleep(0.05)
        publisher.publish(Twist())

        ok = self._wait_for_motion("base", BASE_JOINTS, before)
        self._print_result("base", ok)
        return ok

    def _run_topic_trajectory(self, name, controller_config, allowed_joints):
        targets = self._resolve_targets(controller_config, allowed_joints)
        duration = float(controller_config.get("duration", 2.0))
        topic = controller_config["topic"]
        before = self._snapshot(targets.keys())

        print(
            f"\n[RUN] {name} topic={topic} mode={controller_config.get('mode', 'delta')} "
            f"targets={_format_targets(targets)}"
        )
        publisher = self._publisher(topic, JointTrajectory)
        self._wait_for_subscribers(publisher, topic)
        trajectory = self._make_trajectory(targets, duration)
        start_time = time.monotonic()
        self._publish_repeated(publisher, trajectory)

        ok = self._wait_for_motion(name, targets.keys(), before)
        self._sleep_remaining(start_time, duration)
        if self._should_restore(controller_config, ok):
            restore_targets = {joint: before[joint] for joint in targets}
            self._restore_topic_trajectory(name, topic, restore_targets)

        self._print_result(name, ok)
        return ok

    def _run_action_trajectory(self, name, controller_config, allowed_joints):
        targets = self._resolve_targets(controller_config, allowed_joints)
        duration = float(controller_config.get("duration", 2.0))
        action_name = controller_config["action"]
        before = self._snapshot(targets.keys())

        print(
            f"\n[RUN] {name} action={action_name} "
            f"mode={controller_config.get('mode', 'delta')} "
            f"targets={_format_targets(targets)}"
        )
        action_ok = self._send_follow_trajectory(action_name, targets, duration)
        motion_ok = self._wait_for_motion(name, targets.keys(), before)
        ok = action_ok and motion_ok

        if self._should_restore(controller_config, ok):
            restore_targets = {joint: before[joint] for joint in targets}
            self._restore_action_trajectory(name, action_name, restore_targets)

        self._print_result(name, ok)
        return ok

    def _run_direct_joint_command(self, name, controller_config, allowed_joints, topic):
        targets = self._resolve_targets(controller_config, allowed_joints)
        duration = float(controller_config.get("duration", 2.0))
        publish_rate_hz = float(controller_config.get("publish_rate_hz", 50.0))
        target_tolerance = float(controller_config.get("target_tolerance", 0.05))
        target_timeout = float(controller_config.get("target_timeout", 4.0))
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        before = self._snapshot(targets.keys())
        publisher = self._publisher(topic, JointState)
        self._wait_for_subscribers(publisher, topic)
        command = self._make_joint_state_command(targets)
        period = 1.0 / publish_rate_hz
        deadline = time.monotonic() + duration

        print(
            f"\n[RUN] {name} topic={topic} mode={controller_config.get('mode', 'delta')} "
            f"targets={_format_targets(targets)}"
        )
        moved = False
        while time.monotonic() < deadline:
            publisher.publish(command)
            if not moved and self._joints_moved(targets.keys(), before):
                moved = True
            time.sleep(period)

        if moved:
            after = self._snapshot(targets.keys(), require_all=False)
            max_delta = max(
                abs(after[joint] - before[joint])
                for joint in targets
                if joint in after and joint in before
            )
            print(f"  [MOTION] {name} max_delta={max_delta:.5f}")
        else:
            self._print_fail(name, "motion below threshold")

        target_ok = self._wait_for_targets(name, targets, target_tolerance, target_timeout)
        ok = moved and target_ok
        self._print_result(name, ok)
        return ok

    def _run_gripper(self, controller_config):
        sequence = controller_config.get("sequence", [])
        if not sequence:
            raise ValueError("gripper config requires a non-empty sequence")

        print(f"\n[RUN] gripper sequence={len(sequence)}")
        ok = True
        for step in sequence:
            step_name = step.get("name", "step")
            print(
                f"  [RUN] gripper/{step_name} "
                f"left={step.get('left')} right={step.get('right')}"
            )
            if "left" in step:
                position = float(step["left"])
                command_ok = self._send_gripper_command(
                    controller_config["left_action"],
                    position,
                    controller_config,
                )
                target = self._gripper_command_to_joint_position(
                    position,
                    controller_config,
                )
                state_ok = self._wait_for_joint_target(
                    "gripper/left",
                    "Joint_Left_Gripper",
                    target,
                    float(controller_config.get("joint_tolerance", 0.03)),
                    float(controller_config.get("action_result_timeout", 6.0)),
                )
                ok = command_ok and state_ok and ok
            if "right" in step:
                position = float(step["right"])
                command_ok = self._send_gripper_command(
                    controller_config["right_action"],
                    position,
                    controller_config,
                )
                target = self._gripper_command_to_joint_position(
                    position,
                    controller_config,
                )
                state_ok = self._wait_for_joint_target(
                    "gripper/right",
                    "Joint_Right_Gripper",
                    target,
                    float(controller_config.get("joint_tolerance", 0.03)),
                    float(controller_config.get("action_result_timeout", 6.0)),
                )
                ok = command_ok and state_ok and ok

        self._print_result("gripper", ok)
        return ok

    def _restore_topic_trajectory(self, name, topic, targets):
        duration = float(self.global_config.get("restore_duration", 1.5))
        print(f"  [RESTORE] {name} targets={_format_targets(targets)}")
        trajectory = self._make_trajectory(targets, duration)
        self._publish_repeated(self._publisher(topic, JointTrajectory), trajectory)
        time.sleep(duration)

    def _restore_action_trajectory(self, name, action_name, targets):
        duration = float(self.global_config.get("restore_duration", 1.5))
        print(f"  [RESTORE] {name} targets={_format_targets(targets)}")
        self._send_follow_trajectory(action_name, targets, duration)

    def _resolve_targets(self, controller_config, allowed_joints):
        mode = controller_config.get("mode", "delta")
        allowed = set(allowed_joints)
        if mode == "delta":
            values = controller_config.get("deltas", {})
        elif mode == "absolute":
            values = controller_config.get("positions", {})
        else:
            raise ValueError(f"unsupported mode '{mode}', expected delta or absolute")

        if not values:
            raise ValueError(f"{mode} mode requires non-empty target values")

        invalid = sorted(joint for joint in values if joint not in allowed)
        if invalid:
            raise ValueError(f"invalid joints for controller: {invalid}")

        self._wait_for_joint_states(values.keys())
        current = self._snapshot(values.keys())
        targets = {}
        for joint, value in values.items():
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"non-finite target for {joint}: {value}")
            targets[joint] = current[joint] + value if mode == "delta" else value
        return targets

    def _send_follow_trajectory(self, action_name, targets, duration):
        client = self.follow_clients.get(action_name)
        if client is None:
            client = ActionClient(self, FollowJointTrajectory, action_name)
            self.follow_clients[action_name] = client

        timeout = float(self.global_config.get("action_server_timeout", 5.0))
        if not client.wait_for_server(timeout_sec=timeout):
            self._print_fail(action_name, "action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_trajectory(targets, duration)
        goal_future = client.send_goal_async(goal)
        if not self._wait_future(goal_future, timeout):
            self._print_fail(action_name, "goal send timed out")
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._print_fail(action_name, "goal rejected")
            return False

        result_timeout = float(
            self.global_config.get("action_result_timeout", duration + 5.0)
        )
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, result_timeout):
            self._print_fail(action_name, "result timed out")
            return False

        result = result_future.result().result
        if result.error_code != FOLLOW_JOINT_RESULT_SUCCESS:
            self._print_fail(action_name, result.error_string or "action failed")
            return False
        return True

    def _send_gripper_command(self, action_name, position, controller_config):
        client = self.gripper_clients.get(action_name)
        if client is None:
            client = ActionClient(self, GripperCommand, action_name)
            self.gripper_clients[action_name] = client

        timeout = float(self.global_config.get("action_server_timeout", 5.0))
        if not client.wait_for_server(timeout_sec=timeout):
            self._print_fail(action_name, "action server not available")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = float(controller_config.get("max_effort", 0.0))
        goal_future = client.send_goal_async(goal)
        if not self._wait_future(goal_future, timeout):
            self._print_fail(action_name, "goal send timed out")
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._print_fail(action_name, "goal rejected")
            return False

        result_timeout = float(
            controller_config.get(
                "action_result_timeout",
                self.global_config.get("action_result_timeout", 6.0),
            )
        )
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, result_timeout):
            self._print_fail(action_name, "result timed out")
            return False

        response = result_future.result()
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self._print_fail(action_name, f"action status={response.status}")
            return False
        if not result.reached_goal:
            print(
                f"  [WARN] {action_name}: action result reached_goal=false; "
                "checking /joint_states instead"
            )
        return True

    def _gripper_command_to_joint_position(self, command_position, controller_config):
        command_open = float(controller_config.get("command_open_position", 1.0))
        command_closed = float(controller_config.get("command_closed_position", 0.0))
        joint_open = float(controller_config.get("joint_open_position", 1.0))
        joint_closed = float(controller_config.get("joint_closed_position", 0.0))
        command_range = command_closed - command_open
        if abs(command_range) < 1e-9:
            raise ValueError(
                "gripper command_open_position and command_closed_position must differ"
            )
        ratio = (command_position - command_open) / command_range
        return joint_open + ratio * (joint_closed - joint_open)

    def _wait_for_joint_target(self, label, joint, target, tolerance, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = self._snapshot([joint], require_all=False).get(joint)
            if current is not None:
                error = abs(target - current)
                if error <= tolerance:
                    print(f"  [STATE] {label} {joint}={current:.4f} target={target:.4f}")
                    return True
            time.sleep(0.05)

        current = self._snapshot([joint], require_all=False).get(joint)
        if current is None:
            self._print_fail(label, f"missing joint state for {joint}")
        else:
            self._print_fail(
                label,
                f"{joint}={current:.4f}, target={target:.4f}, tolerance={tolerance:.4f}",
            )
        return False

    def _wait_for_targets(self, label, targets, tolerance, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            max_joint, max_error = self._max_target_error(targets)
            if max_error <= tolerance:
                print(
                    f"  [STATE] {label} reached targets "
                    f"max_error={max_error:.4f} at {max_joint}"
                )
                return True
            time.sleep(0.05)

        max_joint, max_error = self._max_target_error(targets)
        self._print_fail(
            label,
            f"target error too high: max_error={max_error:.4f} "
            f"at {max_joint}, tolerance={tolerance:.4f}",
        )
        return False

    def _wait_for_joint_states(self, required_joints, timeout=None):
        if timeout is None:
            timeout = float(self.global_config.get("initial_state_timeout", 10.0))
        required = list(required_joints)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                missing = [joint for joint in required if joint not in self.positions]
            if not missing:
                return True
            self.joint_state_event.wait(0.05)
            self.joint_state_event.clear()
        return False

    def _wait_for_motion(self, name, joints, before):
        timeout = float(self.global_config.get("motion_timeout", 5.0))
        min_delta = float(self.global_config.get("min_motion_delta", 0.003))
        deadline = time.monotonic() + timeout
        joints = list(joints)
        while time.monotonic() < deadline:
            after = self._snapshot(joints, require_all=False)
            if not after:
                time.sleep(0.05)
                continue
            max_delta = max(
                abs(after[joint] - before[joint])
                for joint in joints
                if joint in after and joint in before
            )
            if max_delta >= min_delta:
                print(f"  [MOTION] {name} max_delta={max_delta:.5f}")
                return True
            time.sleep(0.05)

        after = self._snapshot(joints, require_all=False)
        max_delta = 0.0
        if after:
            max_delta = max(
                abs(after[joint] - before[joint])
                for joint in joints
                if joint in after and joint in before
            )
        self._print_fail(
            name,
            f"motion below threshold: max_delta={max_delta:.5f}, "
            f"threshold={min_delta:.5f}",
        )
        return False

    def _joints_moved(self, joints, before):
        min_delta = float(self.global_config.get("min_motion_delta", 0.003))
        after = self._snapshot(joints, require_all=False)
        if not after:
            return False
        max_delta = max(
            abs(after[joint] - before[joint])
            for joint in joints
            if joint in after and joint in before
        )
        return max_delta >= min_delta

    def _snapshot(self, joints, require_all=True):
        joints = list(joints)
        with self.lock:
            snapshot = {
                joint: self.positions[joint]
                for joint in joints
                if joint in self.positions
            }
        if require_all and len(snapshot) != len(joints):
            missing = [joint for joint in joints if joint not in snapshot]
            raise RuntimeError(f"missing joint states for: {missing}")
        return snapshot

    def _make_trajectory(self, targets, duration):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(targets.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(targets[joint]) for joint in trajectory.joint_names]
        point.velocities = [0.0 for _ in trajectory.joint_names]
        point.time_from_start = _duration_msg(duration)
        trajectory.points = [point]
        return trajectory

    def _make_joint_state_command(self, targets):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets.keys())
        msg.position = [float(targets[joint]) for joint in msg.name]
        msg.velocity = [0.0 for _ in msg.name]
        return msg

    def _joints_at_targets(self, targets, tolerance):
        _, max_error = self._max_target_error(targets)
        return max_error <= tolerance

    def _max_target_error(self, targets):
        current = self._snapshot(targets.keys(), require_all=False)
        max_joint = None
        max_error = float("inf")
        if current:
            max_joint = next(iter(targets))
            max_error = -1.0
            for joint, target in targets.items():
                if joint not in current:
                    continue
                error = abs(float(target) - current[joint])
                if error > max_error:
                    max_joint = joint
                    max_error = error
        return max_joint or "unknown", max_error

    def _publisher(self, topic, msg_type):
        key = (topic, msg_type)
        publisher = self._controller_test_publishers.get(key)
        if publisher is None:
            publisher = self.create_publisher(msg_type, topic, 10)
            self._controller_test_publishers[key] = publisher
        return publisher

    def _publish_repeated(self, publisher, message):
        repeats = int(self.global_config.get("publish_repeats", 1))
        period = float(self.global_config.get("publish_repeat_period", 0.1))
        for _ in range(max(1, repeats)):
            publisher.publish(message)
            time.sleep(period)

    def _wait_for_subscribers(self, publisher, topic):
        timeout = float(self.global_config.get("topic_subscriber_timeout", 5.0))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        print(f"  [WARN] no subscribers detected for {topic}; publishing anyway")
        return False

    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return future.done()

    def _should_restore(self, controller_config, test_passed):
        restore_after_each = bool(
            controller_config.get(
                "restore_after_each",
                self.global_config.get("restore_after_each", True),
            )
        )
        if not restore_after_each:
            return False
        if test_passed:
            return True
        return bool(
            controller_config.get(
                "restore_on_failure",
                self.global_config.get("restore_on_failure", True),
            )
        )

    def _sleep_remaining(self, start_time, duration):
        remaining = float(duration) - (time.monotonic() - start_time)
        if remaining > 0.0:
            time.sleep(remaining)

    def _watch_joint_state_topic(self, topic):
        self.topic_counts[topic] = 0
        self._controller_test_subscriptions.append(
            self.create_subscription(
                JointState,
                topic,
                lambda msg, watched_topic=topic: self._topic_cb(watched_topic, msg),
                20,
            )
        )

    def _topic_cb(self, topic, msg):
        with self.lock:
            self.topic_counts[topic] = self.topic_counts.get(topic, 0) + 1
            self.latest_messages[topic] = msg

    def _joint_states_cb(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if math.isfinite(position):
                    self.positions[name] = position
            for name, velocity in zip(msg.name, msg.velocity):
                if math.isfinite(velocity):
                    self.velocities[name] = velocity
        self.joint_state_event.set()

    def _print_result(self, name, ok):
        if ok:
            print(f"[PASS] {name}")
        else:
            print(f"[FAIL] {name}")

    def _print_fail(self, name, reason):
        print(f"  [FAIL] {name}: {reason}")

    def _print_skip(self, name, reason):
        print(f"[SKIP] {name}: {reason}")


def main(argv=None):
    """Run the controller runtime test command line entry point."""
    args = _parse_args(argv)
    config = _load_config(args.config)
    selected = _select_controllers(config, args.controllers)

    if args.dry_run:
        _print_plan(config, selected)
        return 0

    rclpy.init(args=None)
    node = ControllerRuntimeTest(config)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        return node.run(selected, interactive=args.interactive)
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run controller runtime checks against a live Autolife setup."
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to controller test yaml config.",
    )
    parser.add_argument(
        "--controllers",
        default="",
        help="Comma-separated controller names to run; default runs all enabled.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Wait for Enter before each controller check.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the selected controller checks without publishing commands.",
    )
    return parser.parse_args(argv)


def _load_config(path):
    path = Path(path).expanduser()
    with path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError(f"invalid config: {path}")
    if "controllers" not in config:
        raise ValueError(f"config missing controllers section: {path}")
    return config


def _select_controllers(config, controller_arg):
    controllers = config.get("controllers", {})
    if not controller_arg:
        return [name for name, item in controllers.items() if item.get("enabled", True)]

    selected = [name.strip() for name in controller_arg.split(",") if name.strip()]
    invalid = [name for name in selected if name not in controllers]
    if invalid:
        raise ValueError(f"unknown controllers requested: {invalid}")
    return selected


def _print_plan(config, selected):
    print("Controller runtime test plan:")
    for name in selected:
        item = config["controllers"][name]
        mode = item.get("mode", "n/a")
        command_mode = item.get("command_mode", "default")
        enabled = item.get("enabled", True)
        print(f"  - {name}: enabled={enabled} mode={mode} command_mode={command_mode}")


def _duration_msg(seconds):
    seconds = float(seconds)
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    return Duration(sec=sec, nanosec=nanosec)


def _format_targets(targets):
    return "{" + ", ".join(f"{joint}: {value:.4f}" for joint, value in targets.items()) + "}"


if __name__ == "__main__":
    raise SystemExit(main())
