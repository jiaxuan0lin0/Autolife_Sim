import argparse
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointDriveResponseTester(Node):
    def __init__(self, joint_name):
        super().__init__("autolife_joint_drive_response_test")
        self.joint_name = joint_name
        self.last_position = None
        self.last_velocity = None
        self.publisher = self.create_publisher(JointState, "/joint_command", 10)
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, msg):
        for index, name in enumerate(msg.name):
            if name != self.joint_name:
                continue
            if index < len(msg.position):
                self.last_position = float(msg.position[index])
            if index < len(msg.velocity):
                self.last_velocity = float(msg.velocity[index])
            return

    def wait_for_state(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_position is not None:
                return self.last_position
        return None

    def publish_target(self, target, duration, rate_hz):
        period = 1.0 / rate_hz
        deadline = time.monotonic() + duration
        samples = []
        command = JointState()
        command.name = [self.joint_name]
        command.position = [float(target)]
        command.velocity = [0.0]

        while time.monotonic() < deadline:
            command.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.last_position is not None:
                samples.append((time.monotonic(), self.last_position, self.last_velocity))
            time.sleep(period)

        return samples


def compute_target(start, args):
    if args.target is not None:
        return args.target
    return start + args.step


def settling_time(samples, target, tolerance):
    if not samples:
        return None
    for index, (sample_time, position, _velocity) in enumerate(samples):
        remaining = samples[index:]
        if all(abs(item[1] - target) <= tolerance for item in remaining):
            return sample_time - samples[0][0]
    return None


def summarize(joint_name, samples, start, target, tolerance):
    if not samples:
        return False, "No /joint_states samples received."

    positions = [item[1] for item in samples]
    if not all(math.isfinite(value) for value in positions):
        return False, f"{joint_name}: FAIL, joint state became NaN or Inf."

    direction = 1.0 if target >= start else -1.0
    peak = max((position - target) * direction for position in positions)
    overshoot = max(0.0, peak)
    final = positions[-1]
    final_error = final - target
    settle = settling_time(samples, target, tolerance)

    last_start = samples[-1][0] - 1.0
    tail = [position for sample_time, position, _velocity in samples if sample_time >= last_start]
    tail_std = statistics.pstdev(tail) if len(tail) > 1 else 0.0
    tail_p2p = (max(tail) - min(tail)) if tail else 0.0
    velocities = [
        abs(velocity)
        for sample_time, _position, velocity in samples
        if sample_time >= last_start and velocity is not None and math.isfinite(velocity)
    ]
    tail_max_velocity = max(velocities) if velocities else 0.0

    passed = (
        abs(final_error) <= tolerance
        and overshoot <= tolerance
        and tail_p2p <= tolerance
    )
    lines = [
        f"Joint drive response: {joint_name}",
        f"  start={start:.5f}, target={target:.5f}, final={final:.5f}",
        f"  final_error={final_error:.5f}, overshoot={overshoot:.5f}, tolerance={tolerance:.5f}",
        f"  settle_time={'not settled' if settle is None else f'{settle:.3f}s'}",
        f"  last_1s_std={tail_std:.5f}, last_1s_peak_to_peak={tail_p2p:.5f}, "
        f"last_1s_max_velocity={tail_max_velocity:.5f}",
        f"  result: {'PASS' if passed else 'FAIL'}",
    ]
    return passed, "\n".join(lines)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("joint")
    parser.add_argument("--target", type=float)
    parser.add_argument("--step", type=float, default=0.35)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--wait-timeout", type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = JointDriveResponseTester(args.joint)
    try:
        start = node.wait_for_state(args.wait_timeout)
        if start is None:
            print("FAIL: /joint_states was not received. Press Play in Isaac Sim first.")
            return 1

        target = compute_target(start, args)
        samples = node.publish_target(target, args.duration, args.rate)
        passed, summary = summarize(args.joint, samples, start, target, args.tolerance)
        print(summary)
        return 0 if passed else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
