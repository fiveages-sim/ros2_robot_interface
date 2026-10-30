"""Utility modules for ROS2 Robot Interface."""

from .discovery import (
    discover_robot_description_topics,
    discover_topics,
    discover_topics_with_types,
    list_nodes,
    list_node_parameters,
    set_node_parameters,
)
from .exceptions import (
    ROS2InterfaceError,
    ROS2NotConnectedError,
    ROS2AlreadyConnectedError,
)
from .quat_pose import (
    check_pose_arrival,
    pose_from_tuple,
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    rotate_vector_by_quat_inverse,
)
from .topic_namespace import (
    apply_prefix_to_config_paths,
    list_robot_description_topics,
    relativize_config_topics_for_namespace,
    resolve_core_topic,
    scope_names,
    to_node_relative_topic,
    topic_prefix_from_robot_description,
)

__all__ = [
    "discover_topics",
    "discover_topics_with_types",
    "discover_robot_description_topics",
    "list_nodes",
    "list_node_parameters",
    "set_node_parameters",
    "apply_prefix_to_config_paths",
    "list_robot_description_topics",
    "relativize_config_topics_for_namespace",
    "resolve_core_topic",
    "scope_names",
    "to_node_relative_topic",
    "topic_prefix_from_robot_description",
    "ROS2InterfaceError",
    "ROS2NotConnectedError",
    "ROS2AlreadyConnectedError",
    "check_pose_arrival",
    "pose_from_tuple",
    "quat_conjugate",
    "quat_multiply",
    "quat_normalize",
    "rotate_vector_by_quat_inverse",
]
