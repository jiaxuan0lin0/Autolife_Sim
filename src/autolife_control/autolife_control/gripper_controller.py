import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self.current_positions = {}

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)

        self._left_acion = ActionServer(
            self, 
            GripperCommand,
            '/left_gripper/gripper_command',
            lambda goal: self.execute_gripper(goal, 'Joint_Left_Gripper')
        )

        self._right_action = ActionServer(
            self,
            GripperCommand,
            '/right_gripper/gripper_command',
            lambda goal: self.execute_gripper(goal, 'Joint_Right_Gripper')
        )

    def joint_state_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        self.current_positions.update(pos)

    def execute_gripper(self, goal_handle, joint_name):
        self.get_logger().info(f'{joint_name}: moving to {goal_handle.request.command.position}')

        # 1. get the target position
        target_pos = goal_handle.request.command.position

        # 2.send joint_command
        cmd = JointState()
        cmd.name = [joint_name]
        cmd.position = [target_pos]

        # 3.Loop util the goal is reached while publishing feedback
        feedback = GripperCommand.Feedback()
        tolerance = 0.01 # goal tolerance to see whether reach the goal()
        timeout = 5.0 
        start_time = self.get_clock().now()

        while True:
            current = self.current_positions.get(joint_name, 0.0)
            error = abs(target_pos - current)

            # send feedback
            feedback.position = current
            feedback.reached_goal = error < tolerance
            goal_handle.publish_feedback(feedback)

            # reach goal
            if error < tolerance:
                break

            # time out 
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                feedback.stalled = True
                break
            
            # send command continuely
            self.joint_cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        # return the result 
        result = GripperCommand.Result()
        result.position = self.current_positions.get(joint_name, 0.0)
        result.reached_goal = abs(target_pos - result.position) < tolerance
        result.stalled = not result.reached_goal

        if result.reached_goal:
            goal_handle.succeed()
            self.get_logger().info(f'{joint_name}: reached target')
        else:
            goal_handle.abort()
            self.get_logger().warn(f'{joint_name}: failed to reach target')
        
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
