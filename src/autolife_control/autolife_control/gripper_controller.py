import math
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

GRIPPER_JOINTS = ("Joint_Left_Gripper", "Joint_Right_Gripper")

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.callback_group = ReentrantCallbackGroup()
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.01).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 5.0).value
        self.command_open_position = self.declare_parameter('command_open_position', 1.0).value
        self.command_closed_position = self.declare_parameter('command_closed_position', 0.0).value
        self.joint_open_position = self.declare_parameter('joint_open_position', 1.0).value
        self.joint_closed_position = self.declare_parameter('joint_closed_position', 0.0).value

        self.current_positions = {}
        self.seen_positions = set()

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10,
            callback_group=self.callback_group,
        )
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)

        self._left_action = ActionServer(
            self, 
            GripperCommand,
            '/left_gripper/gripper_command',
            lambda goal: self.execute_gripper(goal, 'Joint_Left_Gripper'),
            callback_group=self.callback_group,
        )

        self._right_action = ActionServer(
            self,
            GripperCommand,
            '/right_gripper/gripper_command',
            lambda goal: self.execute_gripper(goal, 'Joint_Right_Gripper'),
            callback_group=self.callback_group,
        )

    def joint_state_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in GRIPPER_JOINTS:
                self.current_positions[name] = position
                self.seen_positions.add(name)

    def command_to_joint_position(self, command_position):
        command_range = self.command_closed_position - self.command_open_position
        if abs(command_range) < 1e-9:
            raise ValueError('command_open_position and command_closed_position must be different')

        ratio = (command_position - self.command_open_position) / command_range
        return self.joint_open_position + ratio * (self.joint_closed_position - self.joint_open_position)

    def is_valid_command_position(self, command_position):
        lower = min(self.command_open_position, self.command_closed_position)
        upper = max(self.command_open_position, self.command_closed_position)
        return lower <= command_position <= upper

    def joint_to_command_position(self, joint_position):
        joint_range = self.joint_closed_position - self.joint_open_position
        if abs(joint_range) < 1e-9:
            return self.command_open_position

        ratio = (joint_position - self.joint_open_position) / joint_range
        ratio = max(0.0, min(1.0, ratio))
        return self.command_open_position + ratio * (
            self.command_closed_position - self.command_open_position
        )

    def execute_gripper(self, goal_handle, joint_name):
        command_pos = goal_handle.request.command.position
        self.get_logger().info(f'{joint_name}: moving to command {command_pos}')

        # 1. get the target position
        if not math.isfinite(command_pos):
            result = GripperCommand.Result()
            result.position = self.joint_to_command_position(
                self.current_positions.get(joint_name, self.joint_open_position)
            )
            result.reached_goal = False
            result.stalled = True
            goal_handle.abort()
            self.get_logger().warn(f'{joint_name}: rejected non-finite target')
            return result

        if not self.is_valid_command_position(command_pos):
            lower = min(self.command_open_position, self.command_closed_position)
            upper = max(self.command_open_position, self.command_closed_position)
            result = GripperCommand.Result()
            result.position = self.joint_to_command_position(
                self.current_positions.get(joint_name, self.joint_open_position)
            )
            result.reached_goal = False
            result.stalled = True
            goal_handle.abort()
            self.get_logger().warn(
                f'{joint_name}: rejected target {command_pos}; expected '
                f'{lower}..{upper}'
            )
            return result

        target_pos = self.command_to_joint_position(command_pos)

        # 2.send joint_command
        cmd = JointState()
        cmd.name = [joint_name]
        cmd.position = [target_pos]

        # 3.Loop util the goal is reached while publishing feedback
        feedback = GripperCommand.Feedback()
        tolerance = float(self.goal_tolerance)
        timeout = float(self.goal_timeout)
        start_time = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                result = GripperCommand.Result()
                result.position = self.joint_to_command_position(
                    self.current_positions.get(joint_name, self.joint_open_position)
                )
                result.reached_goal = False
                result.stalled = False
                goal_handle.canceled()
                return result

            current = self.current_positions.get(joint_name, 0.0)
            error = abs(target_pos - current)

            # send feedback
            feedback.position = self.joint_to_command_position(current)
            feedback.stalled = False
            feedback.reached_goal = error < tolerance
            goal_handle.publish_feedback(feedback)

            # reach goal
            if joint_name in self.seen_positions and error < tolerance:
                break

            # time out 
            elapsed = time.monotonic() - start_time
            if elapsed > timeout:
                feedback.stalled = True
                break
            
            # send command continuely
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.joint_cmd_pub.publish(cmd)
            time.sleep(0.05)

        # return the result 
        result = GripperCommand.Result()
        current = self.current_positions.get(joint_name, 0.0)
        result.position = self.joint_to_command_position(current)
        result.reached_goal = abs(target_pos - current) < tolerance
        result.stalled = not result.reached_goal

        if result.reached_goal:
            goal_handle.succeed()
            self.get_logger().info(f'{joint_name}: reached target')
        else:
            goal_handle.abort()
            if joint_name not in self.seen_positions:
                self.get_logger().warn(f'{joint_name}: failed because joint state was not received')
            else:
                self.get_logger().warn(f'{joint_name}: failed to reach target')
        
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
    node = GripperController()
    try:
        spin_node(node)
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
