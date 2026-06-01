"""Small geometry helpers for pose and point transforms."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped


def normalize_quat_xyzw(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm <= 0.0:
        raise ValueError("Quaternion has zero norm")
    return quat / norm


def quat_multiply_xyzw(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = normalize_quat_xyzw(left)
    rx, ry, rz, rw = normalize_quat_xyzw(right)
    return normalize_quat_xyzw(
        np.array(
            [
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz,
            ],
            dtype=np.float64,
        )
    )


def quat_to_matrix_xyzw(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(quat)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat_xyzw(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalize_quat_xyzw(np.array([x, y, z, w], dtype=np.float64))


def pose_to_arrays(pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quat = np.array(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        dtype=np.float64,
    )
    return position, normalize_quat_xyzw(quat)


def transform_to_arrays(transform: TransformStamped) -> tuple[np.ndarray, np.ndarray]:
    t = transform.transform.translation
    q = transform.transform.rotation
    translation = np.array([t.x, t.y, t.z], dtype=np.float64)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    return translation, normalize_quat_xyzw(quat)


def transform_pose(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    translation, transform_quat = transform_to_arrays(transform)
    position, pose_quat = pose_to_arrays(pose.pose)
    rotated = quat_to_matrix_xyzw(transform_quat) @ position
    output_quat = quat_multiply_xyzw(transform_quat, pose_quat)

    out = PoseStamped()
    out.header.stamp = transform.header.stamp
    out.header.frame_id = transform.header.frame_id
    out.pose.position.x = float(rotated[0] + translation[0])
    out.pose.position.y = float(rotated[1] + translation[1])
    out.pose.position.z = float(rotated[2] + translation[2])
    out.pose.orientation.x = float(output_quat[0])
    out.pose.orientation.y = float(output_quat[1])
    out.pose.orientation.z = float(output_quat[2])
    out.pose.orientation.w = float(output_quat[3])
    return out


def transform_points(points: np.ndarray, transform: TransformStamped) -> np.ndarray:
    translation, quat = transform_to_arrays(transform)
    rotation = quat_to_matrix_xyzw(quat)
    pts = np.asarray(points, dtype=np.float32)
    return (pts @ rotation.T + translation).astype(np.float32)
