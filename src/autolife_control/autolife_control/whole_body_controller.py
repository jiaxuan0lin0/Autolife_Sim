import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from autolife_control.joint_groups import COMMAND_TOPICS, CONTROLLABLE_JOINTS, JOINT_STATES_TOPIC
from autolife_control.utils import auto_compute_velocities, cubic_hermite


class WholeBodyController(Node):
    def __init__(self):
        super().__init__("whole_body_controller")
        self.callback_group = ReentrantCallbackGroup()

        self.joint_states_topic = self.declare_parameter("joint_states_topic", JOINT_STATES_TOPIC).value
        self.joint_command_topic = self.declare_parameter(
            "joint_command_topic", COMMAND_TOPICS["whole_body"]
        ).value
        self.action_name = self.declare_parameter(
            "action_name", "/whole_body_controller/follow_joint_trajectory"
        ).value
        self.execution_mode = self.declare_parameter("execution_mode", "position_goal").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 100.0).value)
        self.publish_velocity_commands = bool(
            self.declare_parameter("publish_velocity_commands", False).value
        )
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.03).value)
        self.goal_timeout = float(self.declare_parameter("goal_timeout", 5.0).value)
        self.initial_state_timeout = float(self.declare_parameter("initial_state_timeout", 3.0).value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.execution_mode not in ("position_goal", "trajectory"):
            raise ValueError("execution_mode must be 'position_goal' or 'trajectory'")

        self.current_positions = {joint: 0.0 for joint in CONTROLLABLE_JOINTS}
        self.current_velocities = {joint: 0.0 for joint in CONTROLLABLE_JOINTS}
        self.seen_positions = set()
        self.state_lock = threading.Lock()
        self.active_goal_lock = threading.Lock()

        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_cb,
            10,
            callback_group=self.callback_group,
        )
        self.joint_cmd_pub = self.create_publisher(JointState, self.joint_command_topic, 10)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            self.execute_trajectory,
            callback_group=self.callback_group,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
        )

        self.get_logger().info(
            f"Whole-body controller action {self.action_name}; publishing to {self.joint_command_topic}"
        )

    def joint_state_cb(self, msg):
        with self.state_lock:
            for name, position in zip(msg.name, msg.position):
                if name in self.current_positions and math.isfinite(position):
                    self.current_positions[name] = position
                    self.seen_positions.add(name)
            for name, velocity in zip(msg.name, msg.velocity):
                if name in self.current_velocities and math.isfinite(velocity):
                    self.current_velocities[name] = velocity

    def execute_trajectory(self, goal_handle):
        if not self.active_goal_lock.acquire(blocking=False):
            return self.reject_goal(
                goal_handle,
                FollowJointTrajectory.Result.INVALID_GOAL,
                "Another whole-body trajectory is already active",
            )

        try:
            return self._execute_trajectory(goal_handle)
        finally:
            self.active_goal_lock.release()

    def _execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        valid, error_code, error_string = self.validate_trajectory(trajectory)
        if not valid:
            return self.reject_goal(goal_handle, error_code, error_string)

        joint_names = list(trajectory.joint_names)
        state_ready, missing = self.wait_for_joint_states(joint_names)
        if not state_ready:
            return self.reject_goal(
                goal_handle,
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"Missing /joint_states for joints before execution: {missing}",
            )

        traj_data = self.build_trajectory_data(trajectory)
        if self.execution_mode == "position_goal":
            return self.execute_position_goal(goal_handle, joint_names, traj_data)

        period = 1.0 / self.publish_rate_hz
        start_time = time.monotonic()
        final_command = None

        while True:
            if goal_handle.is_cancel_requested:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory canceled"
                goal_handle.canceled()
                return result

            elapsed = time.monotonic() - start_time
            positions, velocities = self.sample_trajectory(traj_data, elapsed)
            final_command = self.publish_command(joint_names, positions, velocities)
            self.publish_feedback(goal_handle, joint_names, positions, velocities)

            if elapsed >= traj_data["times"][-1]:
                break

            time.sleep(period)

        return self.wait_for_goal(goal_handle, joint_names, final_command, period)

    def execute_position_goal(self, goal_handle, joint_names, traj_data):
        period = 1.0 / self.publish_rate_hz
        final_positions = [
            traj_data["positions"][joint][-1]
            for joint in joint_names
        ]
        final_velocities = [0.0 for _ in joint_names]
        final_command = self.publish_command(joint_names, final_positions, final_velocities)
        self.publish_feedback(goal_handle, joint_names, final_positions, final_velocities)
        return self.wait_for_goal(goal_handle, joint_names, final_command, period)

    def validate_trajectory(self, trajectory):
        joint_names = list(trajectory.joint_names)
        if not joint_names:
            return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory has no joint names"
        if len(set(joint_names)) != len(joint_names):
            return False, FollowJointTrajectory.Result.INVALID_JOINTS, "Trajectory has duplicate joint names"

        invalid_joints = [joint for joint in joint_names if joint not in CONTROLLABLE_JOINTS]
        if invalid_joints:
            return (
                False,
                FollowJointTrajectory.Result.INVALID_JOINTS,
                f"Trajectory contains invalid whole-body joints: {invalid_joints}",
            )

        if not trajectory.points:
            return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory has no points"

        previous_time = None
        for point in trajectory.points:
            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if not math.isfinite(target_time):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory waypoint time is non-finite"
            if target_time < 0.0 or (previous_time is not None and target_time <= previous_time):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory waypoint times must increase"
            previous_time = target_time

            if len(point.positions) < len(joint_names):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory point has missing positions"
            if point.velocities and len(point.velocities) < len(joint_names):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory point has incomplete velocities"

            values = list(point.positions[:len(joint_names)])
            if point.velocities:
                values += list(point.velocities[:len(joint_names)])
            if not all(math.isfinite(value) for value in values):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory contains non-finite values"

        return True, FollowJointTrajectory.Result.SUCCESSFUL, ""

    def build_trajectory_data(self, trajectory):
        joint_names = list(trajectory.joint_names)
        times = [
            point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            for point in trajectory.points
        ]
        positions = {}
        velocities = {}

        for index, joint_name in enumerate(joint_names):
            joint_positions = [point.positions[index] for point in trajectory.points]
            positions[joint_name] = joint_positions

            has_all_velocities = all(len(point.velocities) > index for point in trajectory.points)
            if has_all_velocities:
                velocities[joint_name] = [point.velocities[index] for point in trajectory.points]
            else:
                velocities[joint_name] = auto_compute_velocities(joint_positions, times)

        return {
            "joint_names": joint_names,
            "times": times,
            "positions": positions,
            "velocities": velocities,
            "start_positions": self.get_actual_position_map(joint_names),
        }

    def sample_trajectory(self, traj_data, elapsed):
        joint_names = traj_data["joint_names"]
        times = traj_data["times"]
        positions_by_joint = traj_data["positions"]
        velocities_by_joint = traj_data["velocities"]

        if elapsed >= times[-1]:
            positions = [positions_by_joint[joint][-1] for joint in joint_names]
            velocities = [0.0 for _ in joint_names]
            return positions, velocities

        if elapsed <= times[0]:
            if times[0] <= 0.0:
                positions = [positions_by_joint[joint][0] for joint in joint_names]
                velocities = [velocities_by_joint[joint][0] for joint in joint_names]
                return positions, velocities

            sampled_positions = []
            sampled_velocities = []
            start_positions = traj_data["start_positions"]
            for joint in joint_names:
                pos, vel = cubic_hermite(
                    elapsed,
                    0.0,
                    times[0],
                    start_positions[joint],
                    positions_by_joint[joint][0],
                    0.0,
                    velocities_by_joint[joint][0],
                )
                sampled_positions.append(pos)
                sampled_velocities.append(vel)
            return sampled_positions, sampled_velocities

        segment = 1
        for index in range(1, len(times)):
            if elapsed < times[index]:
                segment = index
                break

        sampled_positions = []
        sampled_velocities = []
        for joint in joint_names:
            pos, vel = cubic_hermite(
                elapsed,
                times[segment - 1],
                times[segment],
                positions_by_joint[joint][segment - 1],
                positions_by_joint[joint][segment],
                velocities_by_joint[joint][segment - 1],
                velocities_by_joint[joint][segment],
            )
            sampled_positions.append(pos)
            sampled_velocities.append(vel)

        return sampled_positions, sampled_velocities

    def wait_for_joint_states(self, joint_names):
        deadline = time.monotonic() + self.initial_state_timeout
        while time.monotonic() < deadline:
            with self.state_lock:
                missing = [joint for joint in joint_names if joint not in self.seen_positions]
            if not missing:
                return True, []
            time.sleep(0.02)

        with self.state_lock:
            missing = [joint for joint in joint_names if joint not in self.seen_positions]
        return False, missing

    def wait_for_goal(self, goal_handle, joint_names, final_command, period):
        desired = list(final_command.position)
        feedback = FollowJointTrajectory.Feedback()
        deadline = time.monotonic() + self.goal_timeout

        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory canceled"
                goal_handle.canceled()
                return result

            actual = self.get_actual_positions(joint_names)
            errors = [desired_value - actual_value for desired_value, actual_value in zip(desired, actual)]

            feedback.joint_names = joint_names
            feedback.desired.positions = desired
            feedback.desired.velocities = [0.0 for _ in joint_names]
            feedback.actual.positions = actual
            feedback.error.positions = errors
            goal_handle.publish_feedback(feedback)

            if errors and max(abs(error) for error in errors) <= self.goal_tolerance:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ""
                goal_handle.succeed()
                return result

            final_command.header.stamp = self.get_clock().now().to_msg()
            self.joint_cmd_pub.publish(final_command)
            time.sleep(min(period, 0.05))

        actual = self.get_actual_positions(joint_names)
        errors = [desired_value - actual_value for desired_value, actual_value in zip(desired, actual)]
        return self.reject_goal(
            goal_handle,
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            f"Whole-body goal tolerance violated; final errors: {dict(zip(joint_names, errors))}",
        )

    def publish_command(self, joint_names, positions, velocities):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(joint_names)
        cmd.position = list(positions)
        if self.publish_velocity_commands:
            cmd.velocity = list(velocities)
        self.joint_cmd_pub.publish(cmd)
        return cmd

    def publish_feedback(self, goal_handle, joint_names, desired_positions, desired_velocities):
        actual_positions = self.get_actual_positions(joint_names)
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(joint_names)
        feedback.desired.positions = list(desired_positions)
        feedback.desired.velocities = list(desired_velocities)
        feedback.actual.positions = actual_positions
        feedback.error.positions = [
            desired - actual for desired, actual in zip(desired_positions, actual_positions)
        ]
        goal_handle.publish_feedback(feedback)

    def get_actual_positions(self, joint_names):
        with self.state_lock:
            return [self.current_positions.get(joint, 0.0) for joint in joint_names]

    def get_actual_position_map(self, joint_names):
        with self.state_lock:
            return {joint: self.current_positions[joint] for joint in joint_names}

    def reject_goal(self, goal_handle, error_code, error_string):
        self.get_logger().warn(error_string)
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        goal_handle.abort()
        return result


def spin_node(node):
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        try:
            executor.remove_node(node)
        except RCLError:
            pass
        executor.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyController()
    try:
        spin_node(node)
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
