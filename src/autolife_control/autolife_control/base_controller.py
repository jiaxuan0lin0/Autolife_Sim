import math 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')

        # current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # target positon
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        # current cmd_vel (body frame)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self.dt = 1.0 / 30.0  # control frequency, set to 30 Hz

        # subscribe
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # publish
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # timer
        self.create_timer(self.dt, self.control_loop)

    # ================== 1.get the current pose =====================
    def joint_states_cb(self, msg):
        """
        get the X/Y/Yaw of the base from /joint_states
        """
        pos = dict(zip(msg.name, msg.position))
        self.current_x = pos['Joint_Ground_Vehicle_X']
        self.current_y = pos['Joint_Ground_Vehicle_Y']
        self.current_yaw = pos['Joint_Ground_Vehicle_Z']

    # ================== 2.cmd_vel velocity control ===================
    def cmd_vel_cb(self, msg):
        """
        Integrate robot_frame velocity into target positions in the world frame
        """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # store for velocity feedforward
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz

        # get the midpoint of yaw to improve the accuracy
        mid_yaw = self.current_yaw + (wz * self.dt) / 2

        self.target_x = self.current_x + (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * self.dt
        self.target_y = self.current_y + (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * self.dt
        self.target_yaw = self.current_yaw + wz * self.dt
        self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))

    # ===================== 3.control loop =======================
    def control_loop(self):
        # convert body velocity to world velocity for feedforward
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        world_vx = self.cmd_vx * cos_yaw - self.cmd_vy * sin_yaw
        world_vy = self.cmd_vx * sin_yaw + self.cmd_vy * cos_yaw

        # publish the joint command (position + velocity)
        cmd = JointState()
        cmd.name = ['Joint_Ground_Vehicle_X', 'Joint_Ground_Vehicle_Y', 'Joint_Ground_Vehicle_Z']
        cmd.position = [self.target_x, self.target_y, self.target_yaw]
        cmd.velocity = [world_vx, world_vy, self.cmd_wz]
        self.joint_cmd_pub.publish(cmd)

        # publish the odom
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        odom.twist.twist.linear.x = self.cmd_vx
        odom.twist.twist.linear.y = self.cmd_vy
        odom.twist.twist.angular.z = self.cmd_wz
        self.odom_pub.publish(odom)

        # tf transformation: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

        
