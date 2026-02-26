"""
ROS 2 Robot Interface

A standalone Python package for communicating with ROS 2 robots through topics.
This package is independent of LeRobot and can be used in any ROS 2 environment.
"""

from .config import ControlType, ROS2RobotInterfaceConfig
from .constants import FSM_HOME, FSM_HOLD, FSM_OCS2, FSM_MOVEJ
from .utils.exceptions import (
    ROS2InterfaceError,
    ROS2NotConnectedError,
    ROS2AlreadyConnectedError,
)
from .ros_interface import ROS2RobotInterface

__version__ = "0.1.0"

__all__ = [
    "ROS2RobotInterface",
    "ROS2RobotInterfaceConfig",
    "ControlType",
    "FSM_HOME",
    "FSM_HOLD",
    "FSM_OCS2",
    "FSM_MOVEJ",
    "ROS2InterfaceError",
    "ROS2NotConnectedError",
    "ROS2AlreadyConnectedError",
]

