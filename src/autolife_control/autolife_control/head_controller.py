import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from autolife_control.utils import cubic_hermite, auto_compute_velocities

HEAD_JOINTS = [
    "Joint_Neck_Roll",
    "Joint_Neck_Pitch",
    "Joint_Neck_Yaw",
]


class HeadController(Node):
    def __init__(self):
        super().__init__('head_controller')

        # current state
        self.current_positions = {j: 0.0 for j in HEAD_JOINTS}
        self.current_velocities = {j: 0.0 for j in HEAD_JOINTS}

        # target command
        self.target_positions = {j: 0.0 for j in HEAD_JOINTS}
        self.target_velocities = {j: 0.0 for j in HEAD_JOINTS}

        # trajectory execution state
        self.trajectory = None
        self.traj_start_time = None
        self.traj_times = []
        self.traj_positions = {}
        self.traj_velocities = {}

        # subscribe
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(JointTrajectory, '/head/joint_trajectory',
                                 self.trajectory_cb, 10)

        # publish
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_command', 10)

        # control loop at 30 Hz
        self.dt = 1.0 / 30.0
        self.create_timer(self.dt, self.control_loop)

    def joint_state_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        for j in HEAD_JOINTS:
            if j in pos:
                self.current_positions[j] = pos[j]
            if j in vel:
                self.current_velocities[j] = vel[j]

    def trajectory_cb(self, msg):
        if len(msg.points) == 0:
            return

        joint_names = msg.joint_names

        self.traj_times = []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9
            self.traj_times.append(t)

        self.traj_positions = {}
        self.traj_velocities = {}

        for idx, jn in enumerate(joint_names):
            if jn not in self.target_positions:
                continue

            pos_list = [pt.positions[idx] for pt in msg.points]
            self.traj_positions[jn] = pos_list

            has_vel = all(len(pt.velocities) > idx for pt in msg.points)
            if has_vel:
                vel_list = [pt.velocities[idx] for pt in msg.points]
            else:
                vel_list = auto_compute_velocities(pos_list, self.traj_times)
            self.traj_velocities[jn] = vel_list

        self.trajectory = msg
        self.traj_start_time = self.get_clock().now()
        self.get_logger().info(
            f'Trajectory received: {len(msg.points)} points, '
            f'joints: {joint_names}, '
            f'duration: {self.traj_times[-1]:.2f}s'
        )

    def control_loop(self):
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
                        p0 = self.current_positions[jn]
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
        cmd.velocity = [self.target_velocities[j] for j in HEAD_JOINTS]
        self.joint_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
