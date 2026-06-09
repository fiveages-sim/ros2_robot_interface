"""Box field-of-view and image-center estimation utilities."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import sqrt
from typing import Iterable, Sequence

import numpy as np
from geometry_msgs.msg import TransformStamped

from ..utils.tf_transform import transform_stamped_to_homogeneous

DEFAULT_BASE_FRAME = "base_footprint"
HEAD_CAMERA_FRAME = "head_camera"
DEFAULT_IMAGE_WIDTH = 1280
DEFAULT_IMAGE_HEIGHT = 720


class BoxFovEstimatorError(RuntimeError):
    """Raised when box FOV estimation inputs are invalid."""


class BoxVisibilityStatus(str, Enum):
    """Visibility classification for a box projection."""

    NOT_VISIBLE = "not_visible"
    PARTIAL = "partial"
    FULL = "full"


class BoxCenterStatus(str, Enum):
    """Image-center classification for the projected box center."""

    NOT_PROJECTABLE = "not_projectable"
    OUT_OF_IMAGE = "out_of_image"
    OFF_CENTER = "off_center"
    CENTERED = "centered"


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


# FiveAges head Orbbec intrinsics; matches waist_sim config/config_grasp_fun.json.
DEFAULT_HEAD_CAMERA_INTRINSICS = CameraIntrinsics(
    fx=611.864502,
    fy=611.478271,
    cx=638.961182,
    cy=363.642456,
    width=DEFAULT_IMAGE_WIDTH,
    height=DEFAULT_IMAGE_HEIGHT,
)


def default_head_camera_intrinsics() -> CameraIntrinsics:
    """Return the default head-camera intrinsics for this robot platform."""
    return DEFAULT_HEAD_CAMERA_INTRINSICS


@dataclass(frozen=True)
class Transform3D:
    """Rigid transform represented as target_R_source and target_t_source."""

    rotation: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    translation: tuple[float, float, float]
    target_frame_id: str = "camera_optical_frame"
    source_frame_id: str = "base_footprint"


@dataclass(frozen=True)
class ProjectedPoint:
    """One 3D point and its image projection."""

    xyz_camera: tuple[float, float, float]
    uv: tuple[float, float] | None
    in_front: bool
    in_image: bool


@dataclass(frozen=True)
class BoxFovEstimate:
    """Result of projecting a box into a camera image."""

    visibility_status: BoxVisibilityStatus
    center_status: BoxCenterStatus
    box_any_corner_in_image: bool
    box_all_corners_in_image: bool
    box_center_in_image: bool
    box_centered: bool
    center_score: float | None
    center_offset_norm: tuple[float, float] | None
    center_offset_px: tuple[float, float] | None
    box_center_base: tuple[float, float, float]
    box_center_camera: tuple[float, float, float]
    box_center_uv: tuple[float, float] | None
    corners: tuple[ProjectedPoint, ...]
    centered_threshold: float
    camera_frame_id: str
    source_frame_id: str


def estimate_box_fov(
    box_corners_base: Sequence[Sequence[float]],
    camera_T_base: Transform3D | np.ndarray,
    intrinsics: CameraIntrinsics | None = None,
    *,
    centered_threshold: float = 0.15,
) -> BoxFovEstimate:
    """Estimate box visibility and projected-center alignment.

    Args:
        box_corners_base: Eight box corner points in meters, expressed in base_footprint.
        camera_T_base: Transform from base_footprint to OpenCV camera optical frame.
            Pass either Transform3D or a 4x4 homogeneous matrix. When using TF,
            prefer :func:`estimate_box_fov_from_transform_stamped`.
        intrinsics: Pinhole camera intrinsics. Defaults to
            :data:`DEFAULT_HEAD_CAMERA_INTRINSICS` when omitted.
        centered_threshold: Maximum normalized center distance for CENTERED.
    """
    corners_base = _validate_corners(box_corners_base)
    intr = _validate_intrinsics(intrinsics or DEFAULT_HEAD_CAMERA_INTRINSICS)
    rotation, translation, target_frame_id, source_frame_id = _as_transform_parts(camera_T_base)
    threshold = _validate_centered_threshold(centered_threshold)

    corners_camera = _transform_points(corners_base, rotation, translation)
    projected_corners = tuple(_project_point(point, intr) for point in corners_camera)
    corner_visible = [point.in_front and point.in_image for point in projected_corners]
    any_corner = any(corner_visible)
    all_corners = all(corner_visible)

    if all_corners:
        visibility_status = BoxVisibilityStatus.FULL
    elif any_corner:
        visibility_status = BoxVisibilityStatus.PARTIAL
    else:
        visibility_status = BoxVisibilityStatus.NOT_VISIBLE

    center_base_np = np.mean(corners_base, axis=0)
    center_camera_np = _transform_points(center_base_np.reshape(1, 3), rotation, translation)[0]
    center_projection = _project_point(center_camera_np, intr)
    center_uv = center_projection.uv
    center_in_image = center_projection.in_front and center_projection.in_image

    center_score: float | None = None
    center_offset_norm: tuple[float, float] | None = None
    center_offset_px: tuple[float, float] | None = None
    box_centered = False

    if not center_projection.in_front:
        center_status = BoxCenterStatus.NOT_PROJECTABLE
    elif center_uv is None:
        center_status = BoxCenterStatus.NOT_PROJECTABLE
    else:
        du_px = float(center_uv[0] - intr.cx)
        dv_px = float(center_uv[1] - intr.cy)
        du_norm = du_px / float(intr.width)
        dv_norm = dv_px / float(intr.height)
        center_score = float(sqrt(du_norm * du_norm + dv_norm * dv_norm))
        center_offset_norm = (float(du_norm), float(dv_norm))
        center_offset_px = (du_px, dv_px)
        if not center_projection.in_image:
            center_status = BoxCenterStatus.OUT_OF_IMAGE
        elif center_score <= threshold:
            center_status = BoxCenterStatus.CENTERED
            box_centered = True
        else:
            center_status = BoxCenterStatus.OFF_CENTER

    return BoxFovEstimate(
        visibility_status=visibility_status,
        center_status=center_status,
        box_any_corner_in_image=bool(any_corner),
        box_all_corners_in_image=bool(all_corners),
        box_center_in_image=bool(center_in_image),
        box_centered=bool(box_centered),
        center_score=center_score,
        center_offset_norm=center_offset_norm,
        center_offset_px=center_offset_px,
        box_center_base=_tuple3(center_base_np),
        box_center_camera=_tuple3(center_camera_np),
        box_center_uv=center_uv,
        corners=projected_corners,
        centered_threshold=threshold,
        camera_frame_id=target_frame_id,
        source_frame_id=source_frame_id,
    )


def _validate_corners(box_corners_base: Sequence[Sequence[float]]) -> np.ndarray:
    arr = np.asarray(box_corners_base, dtype=float)
    if arr.shape != (8, 3):
        raise BoxFovEstimatorError(f"box_corners_base must have shape (8, 3), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise BoxFovEstimatorError("box_corners_base contains NaN or infinite values")
    return arr


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
        raise BoxFovEstimatorError("camera intrinsics contain NaN or infinite values")
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise BoxFovEstimatorError("camera intrinsics fx and fy must be > 0")
    if intrinsics.width <= 0 or intrinsics.height <= 0:
        raise BoxFovEstimatorError("camera intrinsics width and height must be > 0")
    return intrinsics


def _validate_centered_threshold(centered_threshold: float) -> float:
    threshold = float(centered_threshold)
    if not np.isfinite(threshold):
        raise BoxFovEstimatorError("centered_threshold must be finite")
    if threshold < 0.0:
        raise BoxFovEstimatorError("centered_threshold must be >= 0")
    return threshold


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
            raise BoxFovEstimatorError(f"camera_T_base matrix must have shape (4, 4), got {matrix.shape}")
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]
        target_frame_id = "camera_optical_frame"
        source_frame_id = "base_footprint"

    if rotation.shape != (3, 3):
        raise BoxFovEstimatorError(f"rotation must have shape (3, 3), got {rotation.shape}")
    if translation.shape != (3,):
        raise BoxFovEstimatorError(f"translation must have shape (3,), got {translation.shape}")
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise BoxFovEstimatorError("camera_T_base contains NaN or infinite values")
    return rotation, translation, target_frame_id, source_frame_id


def _transform_points(points_source: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (rotation @ points_source.T).T + translation.reshape(1, 3)


def _project_point(point_camera: Iterable[float], intrinsics: CameraIntrinsics) -> ProjectedPoint:
    point = np.asarray(tuple(point_camera), dtype=float).reshape(3)
    z = float(point[2])
    in_front = z > 1e-6
    uv: tuple[float, float] | None = None
    in_image = False
    if in_front:
        u = float(intrinsics.fx * point[0] / z + intrinsics.cx)
        v = float(intrinsics.fy * point[1] / z + intrinsics.cy)
        uv = (u, v)
        in_image = 0.0 <= u < float(intrinsics.width) and 0.0 <= v < float(intrinsics.height)
    return ProjectedPoint(
        xyz_camera=_tuple3(point),
        uv=uv,
        in_front=bool(in_front),
        in_image=bool(in_image),
    )


def _tuple3(values: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = tuple(values)
    return (float(x), float(y), float(z))


def transform_stamped_to_transform3d(transform: TransformStamped) -> Transform3D:
    """Convert a ``lookup_transform`` result into :class:`Transform3D`."""
    matrix = transform_stamped_to_homogeneous(transform)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    return Transform3D(
        rotation=tuple(tuple(float(value) for value in row) for row in rotation),
        translation=(float(translation[0]), float(translation[1]), float(translation[2])),
        target_frame_id=str(transform.header.frame_id),
        source_frame_id=str(transform.child_frame_id),
    )


def estimate_box_fov_from_transform_stamped(
    box_corners_base: Sequence[Sequence[float]],
    transform: TransformStamped,
    intrinsics: CameraIntrinsics | None = None,
    *,
    centered_threshold: float = 0.15,
) -> BoxFovEstimate:
    """Estimate box FOV from an existing ``lookup_transform`` result.

    Expected TF pair: ``lookup_transform(HEAD_CAMERA_FRAME, DEFAULT_BASE_FRAME)``.
    """
    return estimate_box_fov(
        box_corners_base,
        transform_stamped_to_transform3d(transform),
        intrinsics,
        centered_threshold=centered_threshold,
    )
