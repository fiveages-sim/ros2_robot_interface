"""
ROS 2 Robot Interface

A standalone Python package for communicating with ROS 2 robots through topics.
This package is independent of LeRobot and can be used in any ROS 2 environment.
"""

from .config import ControlType, ROS2RobotInterfaceConfig
from .constants import (
    FSM_COMPLIANCE,
    FSM_HOLD,
    FSM_HOME,
    FSM_MOVEJ,
    FSM_OCS2,
    is_body_joint_name,
    is_head_joint_name,
)
from .utils.exceptions import (
    ROS2InterfaceError,
    ROS2NotConnectedError,
    ROS2AlreadyConnectedError,
)

__version__ = "0.1.0"

# Heavy modules are imported lazily so config-only CLI tools (e.g. ros2-stack
# launch) do not load rclpy or Pinocchio. ROS distro Pinocchio is built against
# NumPy 1.x and can abort the process under NumPy 2.x.
_LAZY_EXPORTS = {
    "ROS2RobotInterface": ".ros_interface",
    "BoxFovEstimate": ".dynamics",
    "BoxFovEstimatorError": ".dynamics",
    "CameraIntrinsics": ".dynamics",
    "ComEstimate": ".dynamics",
    "ComEstimator": ".dynamics",
    "ComEstimatorError": ".dynamics",
    "Transform3D": ".dynamics",
    "estimate_box_fov": ".dynamics",
}

__all__ = [
    "ROS2RobotInterface",
    "ROS2RobotInterfaceConfig",
    "ControlType",
    "FSM_HOME",
    "FSM_HOLD",
    "FSM_OCS2",
    "FSM_MOVEJ",
    "FSM_COMPLIANCE",
    "is_body_joint_name",
    "is_head_joint_name",
    "ROS2InterfaceError",
    "ROS2NotConnectedError",
    "ROS2AlreadyConnectedError",
    "BoxFovEstimate",
    "BoxFovEstimatorError",
    "CameraIntrinsics",
    "ComEstimate",
    "ComEstimator",
    "ComEstimatorError",
    "Transform3D",
    "estimate_box_fov",
]


def __getattr__(name: str):
    module_name = _LAZY_EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module

    try:
        module = import_module(module_name, __name__)
        return getattr(module, name)
    except AttributeError as exc:
        # Do not let submodule AttributeError look like a missing package export.
        raise ImportError(f"Lazy import of {name!r} from {__name__} failed") from exc
