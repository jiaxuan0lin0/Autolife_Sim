#!/usr/bin/env python3
"""Run runtime checks against live Autolife ROS 2 sensor topics."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import yaml

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from autolife_sensors.sensor_config import TF_MESSAGE_TYPE, TF_TOPIC


DEFAULT_CONFIG_PATH = Path(__file__).with_name("sensor_test_config.yaml")

MESSAGE_TYPES = {
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "sensor_msgs/msg/Imu": Imu,
    "tf2_msgs/msg/TFMessage": TFMessage,
}


class SensorRuntimeTest(Node):
    """Validate sensor bridge topics against a runtime sensor manifest."""

    def __init__(self, config, manifest):
        """Create the checker node and prepare runtime state."""
        super().__init__("sensor_runtime_test")
        self.config = config
        self.global_config = config.get("global", {})
        self.manifest = manifest
        self.topic_stats = {}
        self._sensor_test_subscriptions = []
        self.tf_frames = set()

    def run(self):
        """Run topic, message, and TF checks."""
        expected_outputs = _expected_outputs(self.manifest)
        if not expected_outputs:
            print("[FAIL] manifest has no expected sensor outputs")
            return 1

        print(
            f"[MANIFEST] sensors={len(self.manifest.get('sensors', []))} "
            f"outputs={len(expected_outputs)} "
            f"graph={self.manifest.get('graph_path', '<unknown>')}"
        )

        if not self._wait_for_topics(expected_outputs):
            return 1

        self._subscribe_expected_outputs(expected_outputs)
        if bool(self.global_config.get("require_tf", False)):
            self._subscribe_tf()

        sample_duration = float(self.global_config.get("sample_duration", 5.0))
        if sample_duration <= 0.0:
            raise ValueError("sample_duration must be positive")

        print(f"[SAMPLE] collecting sensor messages for {sample_duration:.1f}s")
        deadline = time.monotonic() + sample_duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        ok = self._validate_outputs(expected_outputs)
        if bool(self.global_config.get("require_tf", False)):
            ok = self._validate_tf(expected_outputs) and ok

        if ok:
            print("\nAll expected sensor runtime checks passed.")
            return 0
        return 1

    def _wait_for_topics(self, expected_outputs):
        topic_timeout = float(self.global_config.get("topic_timeout", 10.0))
        if topic_timeout < 0.0:
            raise ValueError("topic_timeout must be non-negative")

        deadline = time.monotonic() + topic_timeout
        missing = expected_outputs
        type_errors = []
        while time.monotonic() <= deadline:
            topic_types = self._topic_types()
            missing = [
                output for output in expected_outputs
                if output["topic"] not in topic_types
            ]
            type_errors = self._type_errors(expected_outputs, topic_types)
            if not missing and not type_errors:
                print(f"[DISCOVER] found {len(expected_outputs)} expected sensor topics")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        for output in missing:
            print(
                "[FAIL] missing topic "
                f"{output['topic']} source={output.get('prim_path', '<unknown>')}"
            )
        for output, actual_types in type_errors:
            print(
                "[FAIL] topic type mismatch "
                f"{output['topic']} expected={output['type']} actual={actual_types}"
            )
        return False

    def _subscribe_expected_outputs(self, expected_outputs):
        for output in expected_outputs:
            msg_type = MESSAGE_TYPES.get(output["type"])
            if msg_type is None:
                raise ValueError(f"unsupported message type: {output['type']}")
            self.topic_stats[output["topic"]] = {
                "count": 0,
                "first_time": None,
                "last_time": None,
                "last_msg": None,
            }
            self._sensor_test_subscriptions.append(
                self.create_subscription(
                    msg_type,
                    output["topic"],
                    lambda msg, topic=output["topic"]: self._sensor_msg_cb(topic, msg),
                    10,
                )
            )

    def _subscribe_tf(self):
        self._sensor_test_subscriptions.append(
            self.create_subscription(TFMessage, TF_TOPIC, self._tf_msg_cb, 50)
        )

    def _sensor_msg_cb(self, topic, msg):
        now = time.monotonic()
        stats = self.topic_stats[topic]
        if stats["count"] == 0:
            stats["first_time"] = now
        stats["count"] += 1
        stats["last_time"] = now
        stats["last_msg"] = msg

    def _tf_msg_cb(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id:
                self.tf_frames.add(transform.child_frame_id)

    def _validate_outputs(self, expected_outputs):
        min_messages = int(self.global_config.get("min_messages", 1))
        ok = True
        for output in expected_outputs:
            stats = self.topic_stats[output["topic"]]
            msg = stats["last_msg"]
            if stats["count"] < min_messages or msg is None:
                print(
                    f"[FAIL] {output['topic']} no data "
                    f"count={stats['count']} required={min_messages}"
                )
                ok = False
                continue

            errors = self._message_errors(output, msg)
            if errors:
                print(f"[FAIL] {output['topic']} {'; '.join(errors)}")
                ok = False
                continue

            print(self._pass_line(output, stats, msg))
        return ok

    def _validate_tf(self, expected_outputs):
        expected_frames = {
            output["frame_id"]
            for output in expected_outputs
            if output.get("frame_id")
        }
        missing = sorted(expected_frames - self.tf_frames)
        if not missing:
            print(f"[PASS] {TF_TOPIC} frames={len(expected_frames)}")
            return True

        print(f"[FAIL] {TF_TOPIC} missing frames: {', '.join(missing)}")
        return False

    def _message_errors(self, output, msg):
        expected_frame = output.get("frame_id")
        errors = []
        if expected_frame and getattr(msg, "header", None) is not None:
            actual_frame = msg.header.frame_id
            if actual_frame != expected_frame:
                errors.append(
                    f"frame_id expected={expected_frame} actual={actual_frame}"
                )

        if isinstance(msg, Image):
            errors.extend(self._image_errors(msg))
        elif isinstance(msg, CameraInfo):
            errors.extend(self._camera_info_errors(msg))
        elif isinstance(msg, PointCloud2):
            errors.extend(self._pointcloud_errors(msg))
        elif isinstance(msg, Imu):
            errors.extend(self._imu_errors(msg))
        return errors

    def _image_errors(self, msg):
        cfg = self.config.get("image", {})
        errors = []
        if cfg.get("require_nonzero_size", True) and (msg.width <= 0 or msg.height <= 0):
            errors.append(f"invalid image size {msg.width}x{msg.height}")
        if cfg.get("require_nonempty_data", True) and not msg.data:
            errors.append("empty image data")
        if not msg.encoding:
            errors.append("empty image encoding")
        return errors

    def _camera_info_errors(self, msg):
        cfg = self.config.get("camera_info", {})
        errors = []
        if cfg.get("require_nonzero_size", True) and (msg.width <= 0 or msg.height <= 0):
            errors.append(f"invalid camera_info size {msg.width}x{msg.height}")
        if cfg.get("require_finite_intrinsics", True):
            values = list(msg.k) + list(msg.r) + list(msg.p)
            if not all(math.isfinite(value) for value in values):
                errors.append("camera intrinsics contain non-finite values")
        return errors

    def _pointcloud_errors(self, msg):
        cfg = self.config.get("pointcloud", {})
        errors = []
        if cfg.get("require_nonempty_data", True) and not msg.data:
            errors.append("empty pointcloud data")
        if cfg.get("require_xyz_fields", True):
            field_names = {field.name for field in msg.fields}
            missing = {"x", "y", "z"} - field_names
            if missing:
                errors.append(f"missing pointcloud fields {sorted(missing)}")
        return errors

    def _imu_errors(self, msg):
        if not self.config.get("imu", {}).get("require_finite_values", True):
            return []

        values = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        if all(math.isfinite(value) for value in values):
            return []
        return ["imu contains non-finite values"]

    def _pass_line(self, output, stats, msg):
        hz = _hz(stats)
        frame_id = getattr(getattr(msg, "header", None), "frame_id", "")
        detail = f"type={output['type']} hz={hz:.2f} frame={frame_id}"
        if isinstance(msg, Image):
            detail += f" size={msg.width}x{msg.height} encoding={msg.encoding}"
        elif isinstance(msg, CameraInfo):
            detail += f" size={msg.width}x{msg.height}"
        elif isinstance(msg, PointCloud2):
            fields = ",".join(field.name for field in msg.fields)
            detail += f" size={msg.width}x{msg.height} fields={fields}"
        return f"[PASS] {output['topic']} {detail}"

    def _topic_types(self):
        return {
            name: types
            for name, types in self.get_topic_names_and_types()
        }

    def _type_errors(self, expected_outputs, topic_types):
        errors = []
        for output in expected_outputs:
            actual_types = topic_types.get(output["topic"])
            if actual_types is not None and output["type"] not in actual_types:
                errors.append((output, actual_types))
        return errors


def load_config(path):
    """Load a YAML sensor test configuration."""
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def load_manifest(path):
    """Load a runtime sensor manifest emitted by the Isaac sensor bridge."""
    manifest_path = Path(path).expanduser()
    if not manifest_path.exists():
        raise FileNotFoundError(
            f"sensor manifest does not exist: {manifest_path}. "
            "Start the simulation with ROS2 sensors enabled first."
        )
    with manifest_path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Validate live Autolife ROS 2 sensor topics."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="Override manifest path from the YAML config.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print expected topics from the manifest without subscribing.",
    )
    return parser.parse_args()


def main():
    """Run the sensor runtime test."""
    args = parse_args()
    config = load_config(args.config)
    manifest_path = args.manifest or config.get("global", {}).get(
        "manifest_path", "/tmp/autolife_sensor_manifest.json"
    )
    try:
        manifest = load_manifest(manifest_path)
    except FileNotFoundError as exc:
        print(f"[FAIL] {exc}")
        return 1

    if args.dry_run:
        for output in _expected_outputs(manifest):
            print(f"{output['topic']} [{output['type']}] frame={output['frame_id']}")
        return 0

    rclpy.init()
    node = SensorRuntimeTest(config, manifest)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _expected_outputs(manifest):
    outputs = []
    for sensor in manifest.get("sensors", []):
        for output in sensor.get("outputs", []):
            item = dict(output)
            item["sensor_name"] = sensor.get("name", "")
            item["sensor_kind"] = sensor.get("kind", "")
            item["prim_path"] = sensor.get("prim_path", "")
            outputs.append(item)
    return outputs


def _hz(stats):
    if stats["count"] <= 1:
        return 0.0
    elapsed = stats["last_time"] - stats["first_time"]
    if elapsed <= 0.0:
        return 0.0
    return (stats["count"] - 1) / elapsed


if __name__ == "__main__":
    raise SystemExit(main())
