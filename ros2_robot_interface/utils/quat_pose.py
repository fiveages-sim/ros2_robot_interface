"""Quaternion (xyzw) and :class:`geometry_msgs.msg.Pose` helpers.

Scalar-last **(x, y, z, w)** matches ROS / ``geometry_msgs`` and
:mod:`scipy.spatial.transform.Rotation` conventions.
"""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose


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
