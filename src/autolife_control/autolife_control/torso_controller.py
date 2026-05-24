import math
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from autolife_control.joint_groups import COMMAND_TOPICS, JOINT_STATES_TOPIC, TORSO_JOINTS
from autolife_control.utils import cubic_hermite, auto_compute_velocities

class TorsoController(Node):
    def __init__(self):
        super().__init__('torso_controller')
        self.joint_states_topic = self.declare_parameter('joint_states_topic', JOINT_STATES_TOPIC).value
        self.joint_command_topic = self.declare_parameter(
            'joint_command_topic', COMMAND_TOPICS['torso']
        ).value

        # current state
        self.current_positions = {j: 0.0 for j in TORSO_JOINTS}
        self.current_velocities = {j: 0.0 for j in TORSO_JOINTS}
        self.seen_positions = set()
        self.state_initialized = False

        # target command
        self.target_positions = {j: 0.0 for j in TORSO_JOINTS}
        self.target_velocities = {j: 0.0 for j in TORSO_JOINTS}

        # trajectory execution state
        self.trajectory = None
        self.traj_start_time = None
        self.traj_times = []        # time of each waypoint (seconds)
        self.traj_positions = {}    # {joint_name: [pos at each waypoint]}
        self.traj_velocities = {}   # {joint_name: [vel at each waypoint]}

        # subscribe
        self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(JointTrajectory, '/torso/joint_trajectory',
                                 self.trajectory_cb, 10)

        # publish
        self.joint_cmd_pub = self.create_publisher(JointState, self.joint_command_topic, 10)

        # control loop at 30 Hz
        self.dt = 1.0 / 30.0
        self.create_timer(self.dt, self.control_loop)

    # ================== 1. read current joint state ==================
    def joint_state_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        for j in TORSO_JOINTS:
            if j in pos:
                self.current_positions[j] = pos[j]
                self.seen_positions.add(j)
            if j in vel:
                self.current_velocities[j] = vel[j]

        if not self.state_initialized and len(self.seen_positions) == len(TORSO_JOINTS):
            for j in TORSO_JOINTS:
                self.target_positions[j] = self.current_positions[j]
                self.target_velocities[j] = 0.0
            self.state_initialized = True
            self.get_logger().info('Initialized targets from current joint states')

    # ================== 2. receive trajectory ========================
    def trajectory_cb(self, msg):
        if len(msg.points) == 0:
            self.get_logger().warn('Ignoring empty torso trajectory')
            return

        if not self.state_initialized:
            self.get_logger().warn('Ignoring torso trajectory until /joint_states initializes targets')
            return

        joint_names = list(msg.joint_names)
        if not joint_names:
            self.get_logger().warn('Ignoring torso trajectory without joint names')
            return

        if len(set(joint_names)) != len(joint_names):
            self.get_logger().warn(f'Ignoring torso trajectory with duplicate joints: {joint_names}')
            return

        invalid_joints = [jn for jn in joint_names if jn not in TORSO_JOINTS]
        if invalid_joints:
            self.get_logger().warn(f'Ignoring torso trajectory with invalid joints: {invalid_joints}')
            return

        # extract times for each waypoint
        traj_times = []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
            if t < 0.0 or (traj_times and t <= traj_times[-1]):
                self.get_logger().warn('Ignoring torso trajectory with non-increasing waypoint times')
                return
            if len(pt.positions) < len(joint_names):
                self.get_logger().warn('Ignoring torso trajectory with missing point positions')
                return
            if pt.velocities and len(pt.velocities) < len(joint_names):
                self.get_logger().warn('Ignoring torso trajectory with incomplete point velocities')
                return
            values = list(pt.positions[:len(joint_names)])
            if pt.velocities:
                values += list(pt.velocities[:len(joint_names)])
            if not all(math.isfinite(v) for v in values):
                self.get_logger().warn('Ignoring torso trajectory with non-finite values')
                return
            traj_times.append(t)

        # extract positions per joint
        traj_positions = {}
        traj_velocities = {}

        for idx, jn in enumerate(joint_names):
            # positions
            pos_list = [pt.positions[idx] for pt in msg.points]
            traj_positions[jn] = pos_list

            # velocities: use provided or auto-compute
            has_vel = all(len(pt.velocities) > idx for pt in msg.points)
            if has_vel:
                vel_list = [pt.velocities[idx] for pt in msg.points]
            else:
                vel_list = auto_compute_velocities(pos_list, traj_times)
            traj_velocities[jn] = vel_list

        self.traj_times = traj_times
        self.traj_positions = traj_positions
        self.traj_velocities = traj_velocities
        self.trajectory = msg
        self.traj_start_time = self.get_clock().now()
        self.get_logger().info(
            f'Trajectory received: {len(msg.points)} points, '
            f'joints: {joint_names}, '
            f'duration: {self.traj_times[-1]:.2f}s'
        )

    # ================== 3. control loop ==============================
    def control_loop(self):
        if not self.state_initialized:
            return

        if self.trajectory is not None:
            elapsed = (self.get_clock().now() - self.traj_start_time).nanoseconds / 1e9
            times = self.traj_times

            if elapsed >= times[-1]:
                # trajectory finished: hold last position, velocity = 0
                for jn in self.traj_positions:
                    self.target_positions[jn] = self.traj_positions[jn][-1]
                    self.target_velocities[jn] = 0.0
                self.get_logger().info('Trajectory completed')
                self.trajectory = None
            else:
                # find which segment the robot is in
                seg = 0
                for i in range(1, len(times)):
                    if elapsed < times[i]:
                        seg = i
                        break

                for jn in self.traj_positions:
                    positions = self.traj_positions[jn]
                    velocities = self.traj_velocities[jn]

                    if seg == 0:
                        # before first waypoint: interpolate from current to first
                        p0 = self.current_positions[jn]
                        p1 = positions[0]
                        v0 = 0.0
                        v1 = velocities[0]
                        pos, vel = cubic_hermite(elapsed, 0.0, times[0], p0, p1, v0, v1)
                    else:
                        # between waypoints seg-1 and seg
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
        cmd.name = list(TORSO_JOINTS)
        cmd.position = [self.target_positions[j] for j in TORSO_JOINTS]
        cmd.velocity = [self.target_velocities[j] for j in TORSO_JOINTS]
        self.joint_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TorsoController()
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
