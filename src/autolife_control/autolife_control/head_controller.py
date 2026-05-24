import math
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from autolife_control.joint_groups import COMMAND_TOPICS, HEAD_JOINTS, JOINT_STATES_TOPIC
from autolife_control.utils import cubic_hermite, auto_compute_velocities

class HeadController(Node):
    def __init__(self):
        super().__init__('head_controller')
        self.joint_states_topic = self.declare_parameter('joint_states_topic', JOINT_STATES_TOPIC).value
        self.joint_command_topic = self.declare_parameter(
            'joint_command_topic', COMMAND_TOPICS['head']
        ).value
        self.control_rate_hz = float(self.declare_parameter('control_rate_hz', 100.0).value)
        if self.control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self.publish_velocity_commands = bool(
            self.declare_parameter('publish_velocity_commands', False).value
        )

        # current state
        self.current_positions = {j: 0.0 for j in HEAD_JOINTS}
        self.current_velocities = {j: 0.0 for j in HEAD_JOINTS}
        self.seen_positions = set()
        self.state_initialized = False

        # target command
        self.target_positions = {j: 0.0 for j in HEAD_JOINTS}
        self.target_velocities = {j: 0.0 for j in HEAD_JOINTS}

        # trajectory execution state
        self.trajectory = None
        self.traj_start_time = None
        self.traj_start_positions = {}
        self.traj_times = []
        self.traj_positions = {}
        self.traj_velocities = {}

        # subscribe
        self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(JointTrajectory, '/head/joint_trajectory',
                                 self.trajectory_cb, 10)

        # publish
        self.joint_cmd_pub = self.create_publisher(JointState, self.joint_command_topic, 10)

        self.dt = 1.0 / self.control_rate_hz
        self.create_timer(self.dt, self.control_loop)

    def joint_state_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        for j in HEAD_JOINTS:
            if j in pos:
                self.current_positions[j] = pos[j]
                self.seen_positions.add(j)
            if j in vel:
                self.current_velocities[j] = vel[j]

        if not self.state_initialized and len(self.seen_positions) == len(HEAD_JOINTS):
            for j in HEAD_JOINTS:
                self.target_positions[j] = self.current_positions[j]
                self.target_velocities[j] = 0.0
            self.state_initialized = True
            self.get_logger().info('Initialized targets from current joint states')

    def trajectory_cb(self, msg):
        if len(msg.points) == 0:
            self.get_logger().warn('Ignoring empty head trajectory')
            return

        if not self.state_initialized:
            self.get_logger().warn('Ignoring head trajectory until /joint_states initializes targets')
            return

        joint_names = list(msg.joint_names)
        if not joint_names:
            self.get_logger().warn('Ignoring head trajectory without joint names')
            return

        if len(set(joint_names)) != len(joint_names):
            self.get_logger().warn(f'Ignoring head trajectory with duplicate joints: {joint_names}')
            return

        invalid_joints = [jn for jn in joint_names if jn not in HEAD_JOINTS]
        if invalid_joints:
            self.get_logger().warn(f'Ignoring head trajectory with invalid joints: {invalid_joints}')
            return

        traj_times = []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
            if t < 0.0 or (traj_times and t <= traj_times[-1]):
                self.get_logger().warn('Ignoring head trajectory with non-increasing waypoint times')
                return
            if len(pt.positions) < len(joint_names):
                self.get_logger().warn('Ignoring head trajectory with missing point positions')
                return
            if pt.velocities and len(pt.velocities) < len(joint_names):
                self.get_logger().warn('Ignoring head trajectory with incomplete point velocities')
                return
            values = list(pt.positions[:len(joint_names)])
            if pt.velocities:
                values += list(pt.velocities[:len(joint_names)])
            if not all(math.isfinite(v) for v in values):
                self.get_logger().warn('Ignoring head trajectory with non-finite values')
                return
            traj_times.append(t)

        traj_positions = {}
        traj_velocities = {}

        for idx, jn in enumerate(joint_names):
            pos_list = [pt.positions[idx] for pt in msg.points]
            traj_positions[jn] = pos_list

            has_vel = all(len(pt.velocities) > idx for pt in msg.points)
            if has_vel:
                vel_list = [pt.velocities[idx] for pt in msg.points]
            else:
                vel_list = auto_compute_velocities(pos_list, traj_times)
            traj_velocities[jn] = vel_list

        self.traj_times = traj_times
        self.traj_positions = traj_positions
        self.traj_velocities = traj_velocities
        self.traj_start_positions = {
            jn: self.current_positions[jn]
            for jn in joint_names
        }
        self.trajectory = msg
        self.traj_start_time = self.get_clock().now()
        self.get_logger().info(
            f'Trajectory received: {len(msg.points)} points, '
            f'joints: {joint_names}, '
            f'duration: {self.traj_times[-1]:.2f}s'
        )

    def control_loop(self):
        if not self.state_initialized:
            return

        if self.trajectory is not None:
            elapsed = (self.get_clock().now() - self.traj_start_time).nanoseconds / 1e9
            times = self.traj_times

            if elapsed >= times[-1]:
                for jn in self.traj_positions:
                    self.target_positions[jn] = self.traj_positions[jn][-1]
                    self.target_velocities[jn] = 0.0
                self.get_logger().info('Trajectory completed')
                self.trajectory = None
            else:
                seg = 0
                for i in range(1, len(times)):
                    if elapsed < times[i]:
                        seg = i
                        break

                for jn in self.traj_positions:
                    positions = self.traj_positions[jn]
                    velocities = self.traj_velocities[jn]

                    if seg == 0:
                        p0 = self.traj_start_positions.get(jn, self.current_positions[jn])
                        p1 = positions[0]
                        v0 = 0.0
                        v1 = velocities[0]
                        pos, vel = cubic_hermite(elapsed, 0.0, times[0], p0, p1, v0, v1)
                    else:
                        p0 = positions[seg - 1]
                        p1 = positions[seg]
                        v0 = velocities[seg - 1]
                        v1 = velocities[seg]
                        pos, vel = cubic_hermite(elapsed, times[seg - 1], times[seg],
                                                 p0, p1, v0, v1)

                    self.target_positions[jn] = pos
                    self.target_velocities[jn] = vel

        # publish joint command
        cmd = JointState()
        cmd.name = list(HEAD_JOINTS)
        cmd.position = [self.target_positions[j] for j in HEAD_JOINTS]
        if self.publish_velocity_commands:
            cmd.velocity = [self.target_velocities[j] for j in HEAD_JOINTS]
        self.joint_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
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
