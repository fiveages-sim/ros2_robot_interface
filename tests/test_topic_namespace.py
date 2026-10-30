"""Unit tests for topic_prefix helpers (no ROS graph required)."""

from __future__ import annotations

from dataclasses import dataclass

from ros2_robot_interface.utils.topic_namespace import (
    apply_prefix_to_config_paths,
    controller_topic,
    list_robot_description_topics,
    relativize_config_topics_for_namespace,
    resolve_core_topic,
    scope_names,
    to_node_relative_topic,
    topic_prefix_from_robot_description,
)


def test_topic_prefix_from_robot_description():
    assert topic_prefix_from_robot_description("/robot_description") == ""
    assert topic_prefix_from_robot_description("/cr5/robot_description") == "/cr5"
    assert topic_prefix_from_robot_description("/cell/cr5/robot_description") == "/cell/cr5"


def test_scope_names_strips_prefix_without_mixing_namespaces():
    names = [
        "/cr5/robot_description",
        "/cr5/joint_states",
        "/cr5/right_target",
        "/cr5/ocs2_arm_controller/target_joint_position",
        "/arm/robot_description",
        "/arm/right_target",
        "/joint_states",
        "/right_target",
    ]
    scoped = scope_names(names, "/cr5")
    assert scoped == [
        "/robot_description",
        "/joint_states",
        "/right_target",
        "/ocs2_arm_controller/target_joint_position",
    ]
    assert "/arm/right_target" not in scoped
    assert "arm/right_target" not in scoped


def test_scope_names_empty_prefix_is_identity():
    names = ["/robot_description", "/cr5/joint_states"]
    assert scope_names(names, "") == names


def test_apply_prefix_to_config_paths_rewrites_existing_prefixed_topics():
    @dataclass
    class _Cfg:
        right_end_effector_target_topic: str | None = "/right_target"
        left_arm_joint_controller_topic: str | None = "/ocs2_arm_controller/target_joint_position"
        end_effector_pose_topic: str = "/left_current_pose"
        joint_states_topic: str = "/joint_states"
        robot_description_topic: str = "/robot_description"
        unrelated: str = "relative_name"

    cfg = _Cfg()
    available = {
        "/cr5/right_target",
        "/cr5/ocs2_arm_controller/target_joint_position",
        "/cr5/joint_states",
        "/cr5/robot_description",
        # left_current_pose only at global root — must not rewrite
        "/left_current_pose",
    }
    apply_prefix_to_config_paths(cfg, "/cr5", available)
    assert cfg.right_end_effector_target_topic == "/cr5/right_target"
    assert cfg.left_arm_joint_controller_topic == "/cr5/ocs2_arm_controller/target_joint_position"
    assert cfg.end_effector_pose_topic == "/left_current_pose"
    # Core topics are resolved separately — must not be rewritten here.
    assert cfg.joint_states_topic == "/joint_states"
    assert cfg.robot_description_topic == "/robot_description"
    assert cfg.unrelated == "relative_name"


def test_apply_prefix_matches_topic_prefix_via_children():
    @dataclass
    class _Cfg:
        left_hand_tactile_topic_prefix: str | None = "/o7_hand/left/tactile"

    cfg = _Cfg()
    available = {
        "/cr5/o7_hand/left/tactile/thumb",
        "/cr5/o7_hand/left/tactile/index",
    }
    apply_prefix_to_config_paths(cfg, "/cr5", available)
    assert cfg.left_hand_tactile_topic_prefix == "/cr5/o7_hand/left/tactile"


def test_joint_states_falls_back_to_global():
    available = {"/cr5/robot_description", "/joint_states"}
    assert resolve_core_topic("/joint_states", "/cr5", available) == "/joint_states"
    available_scoped = {"/cr5/robot_description", "/cr5/joint_states", "/joint_states"}
    assert resolve_core_topic("/joint_states", "/cr5", available_scoped) == "/cr5/joint_states"


def test_joint_states_no_global_fallback_when_namespaced():
    """Node in robot namespace must not latch onto root /joint_states."""
    available = {"/g1d_01/robot_description", "/joint_states"}
    scoped = resolve_core_topic(
        "/joint_states",
        "/g1d_01",
        available,
        allow_global_fallback=False,
    )
    assert scoped == "/g1d_01/joint_states"
    assert to_node_relative_topic(scoped, "g1d_01") == "joint_states"


def test_resolve_does_not_double_prefix_already_scoped_joint_states():
    """apply_prefix may rewrite joint_states to /{ns}/joint_states before resolve."""
    available = {
        "/g1d_01/robot_description",
        "/g1d_01/joint_states",
        "/g1d_01/g1d_01/joint_states",  # stale double-scoped sub must not win
    }
    # Already-prefixed base name (as after apply_prefix_to_config_paths).
    resolved = resolve_core_topic(
        "/g1d_01/joint_states",
        "/g1d_01",
        available,
        allow_global_fallback=False,
    )
    assert resolved == "/g1d_01/joint_states"
    assert to_node_relative_topic(resolved, "g1d_01") == "joint_states"

    # Pathological double-scoped input still collapses.
    resolved_double = resolve_core_topic(
        "/g1d_01/g1d_01/joint_states",
        "/g1d_01",
        available,
        allow_global_fallback=False,
    )
    assert resolved_double == "/g1d_01/joint_states"


def test_to_node_relative_topic():
    assert to_node_relative_topic("/g1d_01/joint_states", "g1d_01") == "joint_states"
    assert to_node_relative_topic("/g1d_01/fsm_state", "/g1d_01") == "fsm_state"
    assert to_node_relative_topic("/joint_states", "g1d_01") == "/joint_states"
    assert to_node_relative_topic("joint_states", "g1d_01") == "joint_states"
    assert to_node_relative_topic("/g1d_01/joint_states", "") == "/g1d_01/joint_states"
    # Collapse double-scoped absolute / relative leftovers.
    assert to_node_relative_topic("/g1d_01/g1d_01/joint_states", "g1d_01") == "joint_states"
    assert to_node_relative_topic("g1d_01/joint_states", "g1d_01") == "joint_states"


def test_controller_topic_respects_namespace_and_prefix():
    assert (
        controller_topic("left_hand_controller", "target_command")
        == "/left_hand_controller/target_command"
    )
    assert (
        controller_topic(
            "left_hand_controller",
            "target_command",
            topic_prefix="/g1d",
        )
        == "/g1d/left_hand_controller/target_command"
    )
    # Node already in namespace → relative name (resolves to /g1d/...).
    assert (
        controller_topic(
            "left_hand_controller",
            "target_command",
            namespace="g1d",
            topic_prefix="/g1d",
        )
        == "left_hand_controller/target_command"
    )


def test_relativize_config_topics_for_namespace():
    @dataclass
    class _Cfg:
        joint_states_topic: str = "/g1d_01/joint_states"
        robot_description_topic: str = "/g1d_01/robot_description"
        right_target: str = "/g1d_01/right_target"
        global_clock: str = "/clock"
        relative_name: str = "already_relative"

    cfg = _Cfg()
    relativize_config_topics_for_namespace(cfg, "g1d_01")
    assert cfg.joint_states_topic == "joint_states"
    assert cfg.robot_description_topic == "robot_description"
    assert cfg.right_target == "right_target"
    assert cfg.global_clock == "/clock"
    assert cfg.relative_name == "already_relative"


def test_robot_description_list_filters_by_type_and_suffix():
    topics_and_types = [
        ("/robot_description", ["std_msgs/msg/String"]),
        ("/cr5/robot_description", ["std_msgs/msg/String"]),
        ("/other_description", ["std_msgs/msg/String"]),
        ("/fake/robot_description", ["std_msgs/msg/Int32"]),
        ("/arm/joint_states", ["sensor_msgs/msg/JointState"]),
    ]
    assert list_robot_description_topics(topics_and_types) == [
        "/cr5/robot_description",
        "/robot_description",
    ]
