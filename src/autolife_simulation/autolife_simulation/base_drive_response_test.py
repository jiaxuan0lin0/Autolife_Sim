import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


BASE_JOINTS = [
    "Joint_Ground_Vehicle_X",
    "Joint_Ground_Vehicle_Y",
    "Joint_Ground_Vehicle_Z",
]


class BaseDriveResponseTester(Node):
    def __init__(self):
        super().__init__("autolife_base_drive_response_test")
        self.last_positions = None
        self.publisher = self.create_publisher(JointState, "/joint_command", 10)
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in BASE_JOINTS):
            self.last_positions = {name: positions[name] for name in BASE_JOINTS}

    def wait_for_state(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_positions is not None:
                return dict(self.last_positions)
        return None

    def publish_targets(self, targets, duration, rate_hz):
        period = 1.0 / rate_hz
        deadline = time.time() + duration
        samples = []
        command = JointState()
        command.name = list(targets.keys())
        command.position = [float(targets[name]) for name in command.name]

        while time.time() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.last_positions is not None:
                samples.append((time.time(), dict(self.last_positions)))
            time.sleep(period)

        return samples


def build_targets(args):
    return {
        "Joint_Ground_Vehicle_X": args.x,
        "Joint_Ground_Vehicle_Y": args.y,
        "Joint_Ground_Vehicle_Z": args.z,
    }


def summarize(samples, targets, tolerance):
    if not samples:
        return False, "No /joint_states samples received."

    start = samples[0][1]
    final = samples[-1][1]
    final_values = [final[name] for name in BASE_JOINTS]
    if not all(math.isfinite(value) for value in final_values):
        lines = [
            "Base drive response:",
            f"  target: X={targets['Joint_Ground_Vehicle_X']:.4f}, "
            f"Y={targets['Joint_Ground_Vehicle_Y']:.4f}, "
            f"Z={targets['Joint_Ground_Vehicle_Z']:.4f}",
            f"  start:  X={start['Joint_Ground_Vehicle_X']:.4f}, "
            f"Y={start['Joint_Ground_Vehicle_Y']:.4f}, "
            f"Z={start['Joint_Ground_Vehicle_Z']:.4f}",
            f"  final:  X={final['Joint_Ground_Vehicle_X']}, "
            f"Y={final['Joint_Ground_Vehicle_Y']}, "
            f"Z={final['Joint_Ground_Vehicle_Z']}",
            "  result: FAIL (joint state became NaN or Inf)",
        ]
        return False, "\n".join(lines)

    errors = {name: final[name] - targets[name] for name in BASE_JOINTS}
    max_abs_error = max(abs(value) for value in errors.values())
    passed = max_abs_error <= tolerance

    lines = [
        "Base drive response:",
        f"  target: X={targets['Joint_Ground_Vehicle_X']:.4f}, "
        f"Y={targets['Joint_Ground_Vehicle_Y']:.4f}, "
        f"Z={targets['Joint_Ground_Vehicle_Z']:.4f}",
        f"  start:  X={start['Joint_Ground_Vehicle_X']:.4f}, "
        f"Y={start['Joint_Ground_Vehicle_Y']:.4f}, "
        f"Z={start['Joint_Ground_Vehicle_Z']:.4f}",
        f"  final:  X={final['Joint_Ground_Vehicle_X']:.4f}, "
        f"Y={final['Joint_Ground_Vehicle_Y']:.4f}, "
        f"Z={final['Joint_Ground_Vehicle_Z']:.4f}",
        f"  error:  X={errors['Joint_Ground_Vehicle_X']:.4f}, "
        f"Y={errors['Joint_Ground_Vehicle_Y']:.4f}, "
        f"Z={errors['Joint_Ground_Vehicle_Z']:.4f}",
        f"  max_abs_error={max_abs_error:.4f}, tolerance={tolerance:.4f}",
        f"  result: {'PASS' if passed else 'FAIL'}",
    ]
    return passed, "\n".join(lines)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.5)
    parser.add_argument("--z", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    targets = build_targets(args)

    rclpy.init()
    node = BaseDriveResponseTester()
    try:
        initial = node.wait_for_state(args.wait_timeout)
        if initial is None:
            print("FAIL: /joint_states was not received. Press Play in Isaac Sim first.")
            return 1

        samples = node.publish_targets(targets, args.duration, args.rate)
        passed, summary = summarize(samples, targets, args.tolerance)
        print(summary)
        return 0 if passed else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
