"""Quaternion (xyzw) and :class:`geometry_msgs.msg.Pose` helpers.

Scalar-last **(x, y, z, w)** matches ROS / ``geometry_msgs`` and
:mod:`scipy.spatial.transform.Rotation` conventions.
"""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose
from typing import Any


def euler_rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Intrinsic **ZYX**（先 yaw、再 pitch、再 roll）欧拉角（弧度）→ 单位四元数 **(w, x, y, z)**（标量在前）。"""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (float(w), float(x), float(y), float(z))


def euler_rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """同 :func:`euler_rpy_to_quat_wxyz`，输出 **(x, y, z, w)** 供 ``geometry_msgs.Quaternion`` 使用。"""
    w, x, y, z = euler_rpy_to_quat_wxyz(roll, pitch, yaw)
    return (x, y, z, w)


def quat_multiply(
    q1: tuple[float, float, float, float], q2: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    arr = np.array(q, dtype=np.float64)
    n = np.linalg.norm(arr)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    arr /= n
    return (float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))


def rotate_vector_by_quat(
    vec: tuple[float, float, float], quat_xyzw: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    """Apply rotation **R(q)** to ``vec`` (body/tool → world if ``q`` is world-from-body)."""
    q = quat_normalize(quat_xyzw)
    v_quat = (vec[0], vec[1], vec[2], 0.0)
    qc = quat_conjugate(q)
    t = quat_multiply(quat_multiply(q, v_quat), qc)
    return (t[0], t[1], t[2])


def rotate_vector_by_quat_inverse(
    vec: tuple[float, float, float], quat_xyzw: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    q_inv = quat_conjugate(quat_normalize(quat_xyzw))
    v_quat = (vec[0], vec[1], vec[2], 0.0)
    rotated = quat_multiply(quat_multiply(q_inv, v_quat), quat_conjugate(q_inv))
    return (rotated[0], rotated[1], rotated[2])


def pose_from_tuple(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


def check_pose_arrival(
    label: str,
    current_pose: Pose | None,
    target_pose: Pose | None,
    pose_threshold: float,
    orient_threshold: float,
) -> dict[str, Any]:
    """Cartesian pose arrival check (position Euclidean + quaternion angle).

    Same algorithm as the historical ``ArmHandler.check_arrival`` path: normalize
    both quaternions, use absolute dot product for angle (handles q/-q), and
    require both position and orientation thresholds.
    """
    arrived = False
    pos_dist = float("inf")
    orient_dist = float("inf")
    orient_angle_deg = float("inf")
    total_dist = float("inf")
    status_msg = None

    if target_pose is not None and current_pose is not None:
        pos_dist = (
            (current_pose.position.x - target_pose.position.x) ** 2
            + (current_pose.position.y - target_pose.position.y) ** 2
            + (current_pose.position.z - target_pose.position.z) ** 2
        ) ** 0.5

        cqx = current_pose.orientation.x
        cqy = current_pose.orientation.y
        cqz = current_pose.orientation.z
        cqw = current_pose.orientation.w
        current_norm = math.sqrt(cqx * cqx + cqy * cqy + cqz * cqz + cqw * cqw)
        if current_norm > 1e-12:
            cqx /= current_norm
            cqy /= current_norm
            cqz /= current_norm
            cqw /= current_norm
        else:
            cqx, cqy, cqz, cqw = 0.0, 0.0, 0.0, 1.0

        tqx = target_pose.orientation.x
        tqy = target_pose.orientation.y
        tqz = target_pose.orientation.z
        tqw = target_pose.orientation.w
        target_norm = math.sqrt(tqx * tqx + tqy * tqy + tqz * tqz + tqw * tqw)
        if target_norm > 1e-12:
            tqx /= target_norm
            tqy /= target_norm
            tqz /= target_norm
            tqw /= target_norm
        else:
            tqx, tqy, tqz, tqw = 0.0, 0.0, 0.0, 1.0

        dot_product = cqw * tqw + cqx * tqx + cqy * tqy + cqz * tqz
        dot_product = max(-1.0, min(1.0, dot_product))
        orient_dist = 1.0 - abs(dot_product)
        orient_angle_deg = math.degrees(2.0 * math.acos(abs(dot_product)))

        total_dist = pos_dist + orient_dist * 0.1
        arrived = pos_dist < pose_threshold and orient_angle_deg < orient_threshold
        status_msg = f"{label}已到达目标位置" if arrived else f"{label}未到达目标位置"

    return {
        "arrived": arrived,
        "distance": total_dist,
        "position_distance": pos_dist,
        "orientation_distance": orient_dist,
        "orientation_angle_deg": orient_angle_deg,
        "status_message": status_msg,
    }
