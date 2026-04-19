import rclpy
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

        ALL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

        # current state
        self.current_positions = {j:0.0 for j in ALL_ARM_JOINTS}
        self.current_velocities = {j:0.0 for j in ALL_ARM_JOINTS}

        # subscribe & publish
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)

        # left arm action server
        self._left_action = ActionServer(
            self, FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            lambda goal: self.execute_trajectory(goal, LEFT_ARM_JOINTS)
        )

        # right arm action server
        self._right_action = ActionServer(
            self, FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            lambda goal: self.execute_trajectory(goal, RIGHT_ARM_JOINTS)
        )

    def joint_state_cb(self,msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity))
        self.current_positions.update(pos)
        self.current_velocities.update(vel)

    def execute_trajectory(self, goal_handle, joints):
        trajectory = goal_handle.request.trajectory
        start_time = self.get_clock().now()
        feedback = FollowJointTrajectory.Feedback()

        for point in trajectory.points:
            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9

            # wait tiem to execute
            while True:
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed >= target_time:
                    break
                rclpy.spin_once(self, timeout_sec=0.01)
            
            # send execute command
            cmd = JointState()
            cmd.name = list(trajectory.joint_names)
            cmd.position = list(point.positions)
            if point.velocities:
                cmd.velocity = list(point.velocities)
            self.joint_cmd_pub.publish(cmd)

            # send feedback
            feedback.joint_names = list(trajectory.joint_names)
            feedback.desired.positions = list(point.positions)
            feedback.actual.positions = [self.current_positions.get(j, 0.0) for j in trajectory.joint_names]
            feedback.error.positions = [
                d - a for d, a in zip(feedback.desired.positions, feedback.actual.positions)
            ]
            goal_handle.publish_feedback(feedback)
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


