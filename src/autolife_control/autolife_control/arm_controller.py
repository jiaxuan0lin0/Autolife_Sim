import math
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

LEFT_ARM_JOINTS = [
    "Joint_Left_Shoulder_Inner",
    "Joint_Left_Shoulder_Outer",
    "Joint_Left_UpperArm",
    "Joint_Left_Elbow",
    "Joint_Left_Forearm",
    "Joint_Left_Wrist_Upper",
    "Joint_Left_Wrist_Lower",
]

RIGHT_ARM_JOINTS = [
    "Joint_Right_Shoulder_Inner",
    "Joint_Right_Shoulder_Outer",
    "Joint_Right_UpperArm",
    "Joint_Right_Elbow",
    "Joint_Right_Forearm",
    "Joint_Right_Wrist_Upper",
    "Joint_Right_Wrist_Lower",
]

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.callback_group = ReentrantCallbackGroup()
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.03).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 5.0).value

        ALL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

        # current state
        self.current_positions = {j:0.0 for j in ALL_ARM_JOINTS}
        self.current_velocities = {j:0.0 for j in ALL_ARM_JOINTS}
        self.seen_positions = set()

        # subscribe & publish
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10,
            callback_group=self.callback_group,
        )
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)

        # left arm action server
        self._left_action = ActionServer(
            self, FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            lambda goal: self.execute_trajectory(goal, LEFT_ARM_JOINTS),
            callback_group=self.callback_group,
        )

        # right arm action server
        self._right_action = ActionServer(
            self, FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            lambda goal: self.execute_trajectory(goal, RIGHT_ARM_JOINTS),
            callback_group=self.callback_group,
        )

    def joint_state_cb(self,msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = position
                self.seen_positions.add(name)
        for name, velocity in zip(msg.name, msg.velocity):
            if name in self.current_velocities:
                self.current_velocities[name] = velocity

    def reject_goal(self, goal_handle, error_code, error_string):
        self.get_logger().warn(error_string)
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        goal_handle.abort()
        return result

    def validate_trajectory(self, trajectory, allowed_joints):
        joint_names = list(trajectory.joint_names)
        if not joint_names:
            return False, FollowJointTrajectory.Result.INVALID_GOAL, 'Trajectory has no joint names'
        if len(set(joint_names)) != len(joint_names):
            return False, FollowJointTrajectory.Result.INVALID_JOINTS, 'Trajectory has duplicate joint names'

        invalid_joints = [joint for joint in joint_names if joint not in allowed_joints]
        if invalid_joints:
            return (
                False,
                FollowJointTrajectory.Result.INVALID_JOINTS,
                f'Trajectory contains invalid arm joints: {invalid_joints}',
            )

        if not trajectory.points:
            return False, FollowJointTrajectory.Result.INVALID_GOAL, 'Trajectory has no points'

        previous_time = None
        for point in trajectory.points:
            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if target_time < 0.0 or (previous_time is not None and target_time <= previous_time):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, 'Trajectory waypoint times must increase'
            previous_time = target_time

            if len(point.positions) < len(joint_names):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, 'Trajectory point has missing positions'
            if point.velocities and len(point.velocities) < len(joint_names):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, 'Trajectory point has incomplete velocities'

            values = list(point.positions[:len(joint_names)])
            if point.velocities:
                values += list(point.velocities[:len(joint_names)])
            if not all(math.isfinite(value) for value in values):
                return False, FollowJointTrajectory.Result.INVALID_GOAL, 'Trajectory contains non-finite values'

        return True, FollowJointTrajectory.Result.SUCCESSFUL, ''

    def execute_trajectory(self, goal_handle, joints):
        trajectory = goal_handle.request.trajectory
        valid, error_code, error_string = self.validate_trajectory(trajectory, joints)
        if not valid:
            return self.reject_goal(goal_handle, error_code, error_string)

        start_time = time.monotonic()
        feedback = FollowJointTrajectory.Feedback()
        joint_names = list(trajectory.joint_names)
        last_command = None

        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'Trajectory canceled'
                return result

            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9

            # wait tiem to execute
            while True:
                elapsed = time.monotonic() - start_time
                if elapsed >= target_time:
                    break
                time.sleep(0.01)
            
            # send execute command
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = joint_names
            cmd.position = list(point.positions[:len(joint_names)])
            if point.velocities:
                cmd.velocity = list(point.velocities[:len(joint_names)])
            self.joint_cmd_pub.publish(cmd)
            last_command = cmd

            # send feedback
            feedback.joint_names = joint_names
            feedback.desired.positions = list(cmd.position)
            feedback.actual.positions = [self.current_positions.get(j, 0.0) for j in joint_names]
            feedback.error.positions = [
                d - a for d, a in zip(feedback.desired.positions, feedback.actual.positions)
            ]
            goal_handle.publish_feedback(feedback)

        deadline = time.monotonic() + float(self.goal_timeout)
        desired = list(last_command.position) if last_command is not None else []
        while time.monotonic() < deadline:
            missing_joints = [joint for joint in joint_names if joint not in self.seen_positions]
            actual = [self.current_positions.get(joint, 0.0) for joint in joint_names]
            errors = [desired_value - actual_value for desired_value, actual_value in zip(desired, actual)]

            feedback.joint_names = joint_names
            feedback.desired.positions = desired
            feedback.actual.positions = actual
            feedback.error.positions = errors
            goal_handle.publish_feedback(feedback)

            if not missing_joints and errors and max(abs(error) for error in errors) <= self.goal_tolerance:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ''
                goal_handle.succeed()
                return result

            if last_command is not None:
                last_command.header.stamp = self.get_clock().now().to_msg()
                self.joint_cmd_pub.publish(last_command)
            time.sleep(0.05)

        actual = [self.current_positions.get(joint, 0.0) for joint in joint_names]
        errors = [desired_value - actual_value for desired_value, actual_value in zip(desired, actual)]
        error_string = f'Arm goal tolerance violated; final errors: {dict(zip(joint_names, errors))}'
        if any(joint not in self.seen_positions for joint in joint_names):
            missing = [joint for joint in joint_names if joint not in self.seen_positions]
            error_string = f'Arm goal failed because joint states were not received for: {missing}'
        return self.reject_goal(
            goal_handle,
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            error_string,
        )


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
    node = ArmController()
    try:
        spin_node(node)
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
