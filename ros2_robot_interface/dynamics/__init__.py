"""Dynamics and model-based utilities for ros2_robot_interface."""

from .box_fov_estimator import (
    DEFAULT_BASE_FRAME,
    DEFAULT_HEAD_CAMERA_INTRINSICS,
    HEAD_CAMERA_FRAME,
    BoxCenterStatus,
    BoxFovEstimate,
    BoxFovEstimatorError,
    BoxVisibilityStatus,
    CameraIntrinsics,
    ProjectedPoint,
    Transform3D,
    default_head_camera_intrinsics,
    estimate_box_fov,
    estimate_box_fov_from_transform_stamped,
    transform_stamped_to_transform3d,
)
from .ground_fov_estimator import (
    GroundFovEstimate,
    GroundFovEstimatorError,
    GroundIntersectionStatus,
    GroundRayIntersection,
    estimate_ground_fov,
    estimate_ground_fov_from_transform_stamped,
)

# Pinocchio / CoM is imported lazily: ROS distro Pinocchio is built against
# NumPy 1.x and can abort the process under NumPy 2.x.
_COM_EXPORTS = frozenset(
    {
        "ComEstimator",
        "ComEstimate",
        "ComEstimatorError",
        "FrameDiagnostics",
        "SupportMargins",
        "SupportRectangle",
        "SupportStatus",
        "evaluate_support_margins",
    }
)

__all__ = [
    "DEFAULT_BASE_FRAME",
    "DEFAULT_HEAD_CAMERA_INTRINSICS",
    "HEAD_CAMERA_FRAME",
    "BoxCenterStatus",
    "BoxFovEstimate",
    "BoxFovEstimatorError",
    "BoxVisibilityStatus",
    "CameraIntrinsics",
    "GroundFovEstimate",
    "GroundFovEstimatorError",
    "GroundIntersectionStatus",
    "GroundRayIntersection",
    "default_head_camera_intrinsics",
    "ComEstimator",
    "ComEstimate",
    "ComEstimatorError",
    "FrameDiagnostics",
    "ProjectedPoint",
    "SupportMargins",
    "SupportRectangle",
    "SupportStatus",
    "Transform3D",
    "estimate_box_fov",
    "estimate_box_fov_from_transform_stamped",
    "estimate_ground_fov",
    "estimate_ground_fov_from_transform_stamped",
    "transform_stamped_to_transform3d",
    "evaluate_support_margins",
]


def __getattr__(name: str):
    if name in _COM_EXPORTS:
        try:
            from . import com_estimator as _com_estimator

            return getattr(_com_estimator, name)
        except AttributeError as exc:
            raise ImportError(f"Lazy import of {name!r} from {__name__} failed") from exc
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
