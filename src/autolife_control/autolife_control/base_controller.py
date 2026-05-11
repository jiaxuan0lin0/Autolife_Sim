import math 
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

BASE_JOINTS = (
    "Joint_Ground_Vehicle_X",
    "Joint_Ground_Vehicle_Y",
    "Joint_Ground_Vehicle_Z",
)

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.cmd_vel_timeout = self.declare_parameter('cmd_vel_timeout', 0.5).value
        self.max_control_dt = self.declare_parameter('max_control_dt', 0.1).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'Link_Ground_Vehicle').value
        self.publish_tf = self.declare_parameter('publish_tf', False).value

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
        self.last_cmd_vel_time = None
        self.last_control_time = None
        self.target_initialized = False
        self.dt = 1.0 / 30.0  # control frequency, set to 30 Hz

        # subscribe
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # publish
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # timer
        self.create_timer(self.dt, self.control_loop)

    # ================== 1.get the current pose =====================
    def joint_states_cb(self, msg):
        """
        get the X/Y/Yaw of the base from /joint_states
        """
        pos = dict(zip(msg.name, msg.position))

        if not all(name in pos for name in BASE_JOINTS):
            return

        self.current_x = pos['Joint_Ground_Vehicle_X']
        self.current_y = pos['Joint_Ground_Vehicle_Y']
        self.current_yaw = pos['Joint_Ground_Vehicle_Z']

        if not self.target_initialized:
            self.sync_target_to_current()
            self.target_initialized = True

    def sync_target_to_current(self):
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.target_yaw = self.current_yaw

    def has_active_cmd_vel(self, now):
        if self.last_cmd_vel_time is None:
            return False
        return (now - self.last_cmd_vel_time).nanoseconds * 1e-9 <= self.cmd_vel_timeout

    # ================== 2.cmd_vel velocity control ===================
    def cmd_vel_cb(self, msg):
        """
        Store the latest velocity command. Integration happens in control_loop().
        """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.get_logger().warn('Ignoring non-finite /cmd_vel command')
            return

        # store for velocity feedforward
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz
        self.last_cmd_vel_time = self.get_clock().now()


    # ===================== 3.control loop =======================
    def control_loop(self):
        if not self.target_initialized:
            return

        now = self.get_clock().now()
        if self.last_control_time is None:
            self.last_control_time = now
            return

        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        if dt <= 0.0:
            return
        dt = min(dt, self.max_control_dt)

        if not self.has_active_cmd_vel(now):
            vx = 0.0
            vy = 0.0
            wz = 0.0
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0
            self.sync_target_to_current()
        else:
            vx = self.cmd_vx
            vy = self.cmd_vy
            wz = self.cmd_wz

        mid_yaw = self.target_yaw + 0.5 * wz * dt
        cos_mid_yaw = math.cos(mid_yaw)
        sin_mid_yaw = math.sin(mid_yaw)
        world_vx = vx * cos_mid_yaw - vy * sin_mid_yaw
        world_vy = vx * sin_mid_yaw + vy * cos_mid_yaw

        self.target_x += world_vx * dt
        self.target_y += world_vy * dt
        self.target_yaw += wz * dt

        # publish the joint command (position + velocity)
        cmd = JointState()
        cmd.name = ['Joint_Ground_Vehicle_X', 'Joint_Ground_Vehicle_Y', 'Joint_Ground_Vehicle_Z']
        cmd.position = [self.target_x, self.target_y, self.target_yaw]
        cmd.velocity = [world_vx, world_vy, wz]
        self.joint_cmd_pub.publish(cmd)

        # publish the odom
        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
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

        
