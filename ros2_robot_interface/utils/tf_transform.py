"""Generic ROS TF / TransformStamped to NumPy conversion helpers."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from geometry_msgs.msg import TransformStamped

from .quat_pose import quat_normalize


def quat_xyzw_to_rotation_matrix(quat_xyzw: Sequence[float]) -> np.ndarray:
    """Unit quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    x, y, z, w = quat_normalize(
        (float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3]))
    )
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def transform_stamped_to_homogeneous(transform: TransformStamped) -> np.ndarray:
    """Build a 4x4 matrix mapping points from child_frame to header.frame_id."""
    rotation = quat_xyzw_to_rotation_matrix(
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
    )
    translation = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ],
        dtype=float,
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix
