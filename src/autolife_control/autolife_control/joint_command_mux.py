import math
from functools import partial

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from autolife_control.joint_groups import (
    COMMAND_TOPICS,
    CONTROLLABLE_JOINTS,
    JOINT_STATES_TOPIC,
    MUX_OUTPUT_TOPIC,
    SOURCE_JOINTS,
    SOURCE_PRIORITY,
)


DEFAULT_TIMEOUTS = {
    "whole_body": 0.2,
    "base": 0.2,
    "head": 0.5,
    "torso": 0.5,
    "arm": 2.0,
    "gripper": 2.0,
}


class JointCommandMux(Node):
    def __init__(self):
        super().__init__("joint_command_mux")

        self.joint_states_topic = self.declare_parameter("joint_states_topic", JOINT_STATES_TOPIC).value
        self.output_topic = self.declare_parameter("output_topic", MUX_OUTPUT_TOPIC).value
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 100.0).value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.source_topics = {
            source: self.declare_parameter(f"{source}_topic", COMMAND_TOPICS[source]).value
            for source in SOURCE_PRIORITY
        }
        self.source_timeouts = {
            source: float(self.declare_parameter(f"{source}_timeout", DEFAULT_TIMEOUTS[source]).value)
            for source in SOURCE_PRIORITY
        }
        self.allowed_joints = {source: set(SOURCE_JOINTS[source]) for source in SOURCE_PRIORITY}

        self.current_positions = {}
        self.current_velocities = {}
        self.target_positions = {}
        self.target_velocities = {}
        self.initialized = False

        self.latest_commands = {source: {} for source in SOURCE_PRIORITY}
        self.last_owner = {}

        self.warned_invalid_joints = set()
        self.warned_length_errors = set()
        self.warned_conflicts = set()
        self.reported_stale_joints = set()

        self.create_subscription(JointState, self.joint_states_topic, self.joint_states_cb, 10)
        for source, topic in self.source_topics.items():
            self.create_subscription(JointState, topic, partial(self.command_cb, source), 10)

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self.publish_command)

        self.get_logger().info(
            f"Joint command mux publishing {len(CONTROLLABLE_JOINTS)} joints to {self.output_topic} "
            f"at {publish_rate_hz:.1f} Hz"
        )

    def joint_states_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in CONTROLLABLE_JOINTS and math.isfinite(position):
                self.current_positions[name] = position
        for name, velocity in zip(msg.name, msg.velocity):
            if name in CONTROLLABLE_JOINTS and math.isfinite(velocity):
                self.current_velocities[name] = velocity

        if self.initialized:
            return

        missing = [joint for joint in CONTROLLABLE_JOINTS if joint not in self.current_positions]
        if missing:
            return

        self.target_positions = {joint: self.current_positions[joint] for joint in CONTROLLABLE_JOINTS}
        self.target_velocities = {joint: 0.0 for joint in CONTROLLABLE_JOINTS}
        self.initialized = True
        self.get_logger().info("Initialized mux targets from /joint_states")

    def command_cb(self, source, msg):
        if not self._valid_command_lengths(source, msg):
            return

        now = self.get_clock().now()
        allowed = self.allowed_joints[source]
        command = {}

        has_positions = bool(msg.position)
        has_velocities = bool(msg.velocity)
        if not has_positions and not has_velocities:
            self._warn_once(
                self.warned_length_errors,
                (source, "empty_command"),
                f"Dropping {source} command without position or velocity arrays",
            )
            return

        for index, joint_name in enumerate(msg.name):
            if joint_name not in allowed:
                self._warn_once(
                    self.warned_invalid_joints,
                    (source, joint_name),
                    f"Dropping joint '{joint_name}' from {source}; it is not in that source whitelist",
                )
                continue

            position = msg.position[index] if has_positions else None
            velocity = msg.velocity[index] if has_velocities else 0.0

            if position is not None and not math.isfinite(position):
                continue
            if velocity is not None and not math.isfinite(velocity):
                continue

            command[joint_name] = {
                "position": position,
                "velocity": velocity,
                "time": now,
            }

        if command:
            self.latest_commands[source].update(command)
            for joint_name in command:
                self.reported_stale_joints.discard((source, joint_name))

    def publish_command(self):
        if not self.initialized:
            return

        now = self.get_clock().now()
        claimed_joints = {}

        for source in SOURCE_PRIORITY:
            for joint_name, command in self.latest_commands[source].items():
                if self._joint_command_is_stale(source, joint_name, command, now):
                    continue

                previous_source = claimed_joints.get(joint_name)
                if previous_source is not None:
                    self._warn_once(
                        self.warned_conflicts,
                        (joint_name, previous_source, source),
                        f"Joint '{joint_name}' is commanded by both {previous_source} and {source}; "
                        f"using {previous_source}",
                    )
                    continue

                position = command["position"]
                velocity = command["velocity"]
                if position is not None:
                    self.target_positions[joint_name] = position
                self.target_velocities[joint_name] = 0.0 if velocity is None else velocity
                self.last_owner[joint_name] = source
                claimed_joints[joint_name] = source

        out = JointState()
        out.header.stamp = now.to_msg()
        out.name = list(CONTROLLABLE_JOINTS)
        out.position = [self.target_positions[joint] for joint in CONTROLLABLE_JOINTS]
        out.velocity = [self.target_velocities[joint] for joint in CONTROLLABLE_JOINTS]
        self.publisher.publish(out)

    def _valid_command_lengths(self, source, msg):
        expected = len(msg.name)
        checks = (
            ("position", msg.position),
            ("velocity", msg.velocity),
            ("effort", msg.effort),
        )
        for field_name, values in checks:
            if values and len(values) != expected:
                self._warn_once(
                    self.warned_length_errors,
                    (source, field_name),
                    f"Dropping {source} command: {field_name} length {len(values)} "
                    f"does not match name length {expected}",
                )
                return False
        return True

    def _joint_command_is_stale(self, source, joint_name, command, now):
        timeout = self.source_timeouts[source]
        if timeout < 0.0:
            return False
        age = (now - command["time"]).nanoseconds * 1e-9
        if age <= timeout:
            return False

        stale_key = (source, joint_name)
        if stale_key not in self.reported_stale_joints:
            self.get_logger().warn(
                f"{source} command for '{joint_name}' timed out; holding last target with zero velocity"
            )
            self.reported_stale_joints.add(stale_key)

        if self.last_owner.get(joint_name) == source:
            self.target_velocities[joint_name] = 0.0
        return True

    def _warn_once(self, warned_set, key, message):
        if key in warned_set:
            return
        warned_set.add(key)
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
