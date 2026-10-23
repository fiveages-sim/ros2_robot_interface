"""Ground-plane field-of-view estimation utilities."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable

import numpy as np
from geometry_msgs.msg import TransformStamped

from .box_fov_estimator import (
    DEFAULT_HEAD_CAMERA_INTRINSICS,
    CameraIntrinsics,
    Transform3D,
    transform_stamped_to_transform3d,
)


class GroundFovEstimatorError(RuntimeError):
    """Raised when ground FOV estimation inputs are invalid."""


class GroundIntersectionStatus(str, Enum):
    """Ground-plane intersection classification for an image boundary ray."""

    INTERSECTS = "intersects"
    PARALLEL = "parallel"
    BEHIND_CAMERA = "behind_camera"


@dataclass(frozen=True)
class GroundRayIntersection:
    """One image-corner ray and its intersection with a ground plane."""

    pixel_uv: tuple[float, float]
    direction_camera: tuple[float, float, float]
    direction_base: tuple[float, float, float]
    distance_along_ray: float | None
    point_base: tuple[float, float, float] | None
    status: GroundIntersectionStatus


@dataclass(frozen=True)
class GroundFovEstimate:
    """Camera image footprint on a ground plane expressed in base_footprint."""

    ground_z: float
    camera_origin_base: tuple[float, float, float]
    polygon_base: tuple[tuple[float, float, float], ...]
    center_intersection: GroundRayIntersection
    intersections: tuple[GroundRayIntersection, ...]
    all_corners_intersect_ground: bool
    camera_frame_id: str
    source_frame_id: str


def _validate_intrinsics(intrinsics: CameraIntrinsics) -> CameraIntrinsics:
    values = (
        float(intrinsics.fx),
        float(intrinsics.fy),
        float(intrinsics.cx),
        float(intrinsics.cy),
        float(intrinsics.width),
        float(intrinsics.height),
    )
    if not all(np.isfinite(value) for value in values):
        raise GroundFovEstimatorError("camera intrinsics contain NaN or infinite values")
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise GroundFovEstimatorError("camera intrinsics fx and fy must be > 0")
    if intrinsics.width <= 0 or intrinsics.height <= 0:
        raise GroundFovEstimatorError("camera intrinsics width and height must be > 0")
    return intrinsics


def _validate_ground_z(ground_z: float) -> float:
    value = float(ground_z)
    if not np.isfinite(value):
        raise GroundFovEstimatorError("ground_z must be finite")
    return value


def _as_transform_parts(
    camera_T_base: Transform3D | np.ndarray,
) -> tuple[np.ndarray, np.ndarray, str, str]:
    if isinstance(camera_T_base, Transform3D):
        rotation = np.asarray(camera_T_base.rotation, dtype=float)
        translation = np.asarray(camera_T_base.translation, dtype=float)
        target_frame_id = camera_T_base.target_frame_id
        source_frame_id = camera_T_base.source_frame_id
    else:
        matrix = np.asarray(camera_T_base, dtype=float)
        if matrix.shape != (4, 4):
            raise GroundFovEstimatorError(f"camera_T_base matrix must have shape (4, 4), got {matrix.shape}")
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]
        target_frame_id = "camera_optical_frame"
        source_frame_id = "base_footprint"

    if rotation.shape != (3, 3):
        raise GroundFovEstimatorError(f"rotation must have shape (3, 3), got {rotation.shape}")
    if translation.shape != (3,):
        raise GroundFovEstimatorError(f"translation must have shape (3,), got {translation.shape}")
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise GroundFovEstimatorError("camera_T_base contains NaN or infinite values")
    return rotation, translation, target_frame_id, source_frame_id


def _image_corner_pixels(intrinsics: CameraIntrinsics) -> tuple[tuple[float, float], ...]:
    width = float(intrinsics.width)
    height = float(intrinsics.height)
    return (
        (0.0, 0.0),
        (width - 1.0, 0.0),
        (width - 1.0, height - 1.0),
        (0.0, height - 1.0),
    )


def _pixel_to_camera_ray(pixel_uv: tuple[float, float], intrinsics: CameraIntrinsics) -> np.ndarray:
    u, v = pixel_uv
    ray = np.array(
        [
            (float(u) - float(intrinsics.cx)) / float(intrinsics.fx),
            (float(v) - float(intrinsics.cy)) / float(intrinsics.fy),
            1.0,
        ],
        dtype=float,
    )
    norm = np.linalg.norm(ray)
    if norm <= 0.0 or not np.isfinite(norm):
        raise GroundFovEstimatorError(f"cannot normalize camera ray for pixel {pixel_uv}")
    return ray / norm


def _tuple3(values: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = tuple(values)
    return (float(x), float(y), float(z))


def _intersect_ground(
    pixel_uv: tuple[float, float],
    direction_camera: np.ndarray,
    direction_base: np.ndarray,
    origin_base: np.ndarray,
    ground_z: float,
) -> GroundRayIntersection:
    dz = float(direction_base[2])
    if abs(dz) < 1e-9:
        return GroundRayIntersection(
            pixel_uv=(float(pixel_uv[0]), float(pixel_uv[1])),
            direction_camera=_tuple3(direction_camera),
            direction_base=_tuple3(direction_base),
            distance_along_ray=None,
            point_base=None,
            status=GroundIntersectionStatus.PARALLEL,
        )

    distance = (float(ground_z) - float(origin_base[2])) / dz
    if distance < 0.0:
        return GroundRayIntersection(
            pixel_uv=(float(pixel_uv[0]), float(pixel_uv[1])),
            direction_camera=_tuple3(direction_camera),
            direction_base=_tuple3(direction_base),
            distance_along_ray=float(distance),
            point_base=None,
            status=GroundIntersectionStatus.BEHIND_CAMERA,
        )

    point = origin_base + distance * direction_base
    point[2] = float(ground_z)
    return GroundRayIntersection(
        pixel_uv=(float(pixel_uv[0]), float(pixel_uv[1])),
        direction_camera=_tuple3(direction_camera),
        direction_base=_tuple3(direction_base),
        distance_along_ray=float(distance),
        point_base=_tuple3(point),
        status=GroundIntersectionStatus.INTERSECTS,
    )


def estimate_ground_fov(
    camera_T_base: Transform3D | np.ndarray,
    intrinsics: CameraIntrinsics | None = None,
    *,
    ground_z: float = 0.0,
) -> GroundFovEstimate:
    """Estimate camera image footprint on a ground plane in base_footprint.

    Args:
        camera_T_base: Transform from base_footprint to OpenCV camera optical frame.
            Pass either Transform3D or a 4x4 homogeneous matrix. When using TF,
            prefer :func:`estimate_ground_fov_from_transform_stamped`.
        intrinsics: Pinhole camera intrinsics. Defaults to
            :data:`DEFAULT_HEAD_CAMERA_INTRINSICS` when omitted.
        ground_z: Ground-plane z coordinate in the source/base frame. Defaults
            to 0.0 because base_footprint ground is z=0.
    """
    intr = _validate_intrinsics(intrinsics or DEFAULT_HEAD_CAMERA_INTRINSICS)
    ground = _validate_ground_z(ground_z)
    rotation_camera_base, translation_camera_base, target_frame_id, source_frame_id = _as_transform_parts(camera_T_base)

    rotation_base_camera = rotation_camera_base.T
    origin_base = -rotation_base_camera @ translation_camera_base

    intersections: list[GroundRayIntersection] = []
    polygon_points: list[tuple[float, float, float]] = []
    for pixel_uv in _image_corner_pixels(intr):
        direction_camera = _pixel_to_camera_ray(pixel_uv, intr)
        direction_base = rotation_base_camera @ direction_camera
        intersection = _intersect_ground(pixel_uv, direction_camera, direction_base, origin_base, ground)
        intersections.append(intersection)
        if intersection.point_base is not None:
            polygon_points.append(intersection.point_base)

    center_pixel_uv = (float(intr.cx), float(intr.cy))
    center_direction_camera = _pixel_to_camera_ray(center_pixel_uv, intr)
    center_direction_base = rotation_base_camera @ center_direction_camera
    center_intersection = _intersect_ground(
        center_pixel_uv,
        center_direction_camera,
        center_direction_base,
        origin_base,
        ground,
    )

    return GroundFovEstimate(
        ground_z=ground,
        camera_origin_base=_tuple3(origin_base),
        polygon_base=tuple(polygon_points),
        center_intersection=center_intersection,
        intersections=tuple(intersections),
        all_corners_intersect_ground=all(
            intersection.status is GroundIntersectionStatus.INTERSECTS for intersection in intersections
        ),
        camera_frame_id=target_frame_id,
        source_frame_id=source_frame_id,
    )


def estimate_ground_fov_from_transform_stamped(
    transform: TransformStamped,
    intrinsics: CameraIntrinsics | None = None,
    *,
    ground_z: float = 0.0,
) -> GroundFovEstimate:
    """Estimate ground FOV from an existing ``lookup_transform`` result.

    Expected TF pair: ``lookup_transform(HEAD_CAMERA_FRAME, DEFAULT_BASE_FRAME)``.
    """
    return estimate_ground_fov(
        transform_stamped_to_transform3d(transform),
        intrinsics,
        ground_z=ground_z,
    )
