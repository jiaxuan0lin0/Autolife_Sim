import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

class GripperController(Node):
    def __inti__(self):
        super().__init__('gripper_controller')

        self.current_positions = {}

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_publisher(JointState, '/joint_commond', 10)

        self._left_acion = ActionServer(
            self, GripperCommand,
            '/left_gripper/gripper_command',
            
        )