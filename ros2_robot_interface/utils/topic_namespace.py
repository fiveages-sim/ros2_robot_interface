"""Helpers for ROS 2 topic namespace / topic_prefix scoping.

Pure functions with no rclpy dependency so they can be unit-tested offline.
"""

from __future__ import annotations

from dataclasses import fields
from typing import Any, Iterable, List, Optional, Sequence, Set

ROBOT_DESCRIPTION_SUFFIX = "/robot_description"

# Topics that may live at the graph root even when the robot stack uses a prefix.
_GLOBAL_FALLBACK_TOPICS = frozenset(
    {
        "/joint_states",
        "/fsm_state",
        "/fsm_command",
        "/mode_command",
    }
)


def topic_prefix_from_robot_description(topic: str) -> str:
    """Derive topic_prefix from a robot_description topic name.

    Examples:
        ``/robot_description`` -> ``""``
        ``/cr5/robot_description`` -> ``"/cr5"``
        ``/cell/cr5/robot_description`` -> ``"/cell/cr5"``
    """
    if not topic.endswith(ROBOT_DESCRIPTION_SUFFIX):
        raise ValueError(
            f"Expected topic ending with {ROBOT_DESCRIPTION_SUFFIX!r}, got {topic!r}"
        )
    return topic[: -len(ROBOT_DESCRIPTION_SUFFIX)]


def robot_description_topic_for_prefix(prefix: str) -> str:
    """Return the robot_description topic for a topic_prefix."""
    if not prefix:
        return ROBOT_DESCRIPTION_SUFFIX
    return f"{prefix}{ROBOT_DESCRIPTION_SUFFIX}"


def scope_names(names: Sequence[str], prefix: str) -> List[str]:
    """Keep names under ``prefix`` and strip that prefix for absolute-path matching.

    With an empty prefix, returns a shallow copy of ``names``.
    Does not mix in names from other namespaces.
    """
    if not prefix:
        return list(names)

    scoped: List[str] = []
    child_marker = prefix + "/"
    for name in names:
        if name.startswith(child_marker):
            scoped.append(name[len(prefix) :])
    return scoped


def prefixed_name(name: str, prefix: str) -> str:
    """Join ``prefix`` with an absolute topic/action ``name`` (``name`` starts with ``/``)."""
    if not prefix:
        return name
    if not name.startswith("/"):
        return f"{prefix}/{name}"
    return f"{prefix}{name}"


def controller_topic(
    controller_name: str,
    suffix: str,
    *,
    namespace: str = "",
    topic_prefix: str = "",
) -> str:
    """Build a controller child topic under an optional robot namespace/prefix.

    Examples (``controller_name='left_hand_controller'``, ``suffix='target_command'``):
        namespace ``g1d`` → relative ``left_hand_controller/target_command``
            (node in ``g1d`` resolves to ``/g1d/left_hand_controller/target_command``)
        topic_prefix ``/g1d`` only → absolute ``/g1d/left_hand_controller/target_command``
        neither → absolute ``/left_hand_controller/target_command``
    """
    controller_name = (controller_name or "").strip("/")
    suffix = (suffix or "").strip("/")
    if not controller_name:
        raise ValueError("controller_name is required")
    leaf = f"{controller_name}/{suffix}" if suffix else controller_name

    ns = (namespace or "").strip("/")
    if ns:
        # Node already lives in the robot namespace — keep relative.
        return leaf

    prefix = (topic_prefix or "").rstrip("/")
    if prefix:
        if not prefix.startswith("/"):
            prefix = f"/{prefix}"
        return f"{prefix}/{leaf}"
    return f"/{leaf}"


def strip_topic_prefix(name: str, prefix: str) -> str:
    """Strip one or more leading ``prefix`` segments from an absolute topic name.

    Examples:
        ``("/g1d_01/joint_states", "/g1d_01")`` -> ``"/joint_states"``
        ``("/g1d_01/g1d_01/joint_states", "/g1d_01")`` -> ``"/joint_states"``
        ``("/joint_states", "/g1d_01")`` -> ``"/joint_states"``
    """
    if not prefix or not name:
        return name
    p = prefix if prefix.startswith("/") else f"/{prefix}"
    p = p.rstrip("/")
    while name == p or name.startswith(p + "/"):
        if name == p:
            return "/"
        name = name[len(p) :]
    return name


def path_exists_in_graph(path: str, available: Iterable[str]) -> bool:
    """True if ``path`` is listed, or any graph name is under ``path/`` (prefix topics)."""
    available_set = available if isinstance(available, set) else set(available)
    if path in available_set:
        return True
    child = path + "/"
    return any(item.startswith(child) for item in available_set)


def resolve_scoped_or_global(
    base_name: str,
    prefix: str,
    available: Iterable[str],
    *,
    allow_global_fallback: bool,
) -> str:
    """Prefer ``prefix + base_name`` when present; optionally fall back to global ``base_name``.

    When neither exists yet, still return the scoped name so subscriptions are created
    under the selected robot namespace.

    ``base_name`` may already include ``prefix`` (e.g. after ``apply_prefix_to_config_paths``);
    leading prefix segments are stripped first so we never produce ``/{ns}/{ns}/...``.
    """
    if not prefix:
        return base_name

    available_set = available if isinstance(available, set) else set(available)
    unscoped = strip_topic_prefix(base_name, prefix)
    scoped = prefixed_name(unscoped, prefix)
    if path_exists_in_graph(scoped, available_set):
        return scoped
    if allow_global_fallback and path_exists_in_graph(unscoped, available_set):
        return unscoped
    return scoped


def resolve_core_topic(
    base_name: str,
    prefix: str,
    available: Iterable[str],
    *,
    allow_global_fallback: bool | None = None,
) -> str:
    """Resolve joint_states / fsm / mode topics with optional global fallback.

    When ``allow_global_fallback`` is omitted, uses the default allow-list
    (``_GLOBAL_FALLBACK_TOPICS``). Pass ``False`` when the interface node already
    lives in the robot namespace — falling back to a global ``/joint_states``
    would mix multi-robot graphs.
    """
    unscoped_for_allow = strip_topic_prefix(base_name, prefix) if prefix else base_name
    if allow_global_fallback is None:
        allow_global_fallback = unscoped_for_allow in _GLOBAL_FALLBACK_TOPICS
    return resolve_scoped_or_global(
        base_name,
        prefix,
        available,
        allow_global_fallback=allow_global_fallback,
    )


def to_node_relative_topic(topic: str, namespace: str) -> str:
    """Rewrite ``/{namespace}/foo`` to relative ``foo`` when the node is in that namespace.

    Repeated ``/{namespace}/`` prefixes are stripped (guards double-scoping). Absolute
    topics outside the namespace are left unchanged (e.g. a true global ``/joint_states``).
    Empty ``namespace`` leaves ``topic`` unchanged.
    """
    ns = (namespace or "").strip("/")
    if not ns or not topic:
        return topic

    if topic.startswith("/"):
        abs_topic = topic
        prefix = f"/{ns}/"
        stripped_any = False
        while abs_topic.startswith(prefix):
            abs_topic = abs_topic[len(prefix) :]
            stripped_any = True
            if not abs_topic.startswith("/"):
                # Relative remainder; keep stripping ns/ if double-scoped.
                while abs_topic == ns or abs_topic.startswith(ns + "/"):
                    abs_topic = "" if abs_topic == ns else abs_topic[len(ns) + 1 :]
                return abs_topic
        return abs_topic if stripped_any else topic

    # Already relative: strip leading ns/ segments.
    rel = topic
    while rel == ns or rel.startswith(ns + "/"):
        rel = "" if rel == ns else rel[len(ns) + 1 :]
    return rel


def relativize_config_topics_for_namespace(config: Any, namespace: str) -> None:
    """Rewrite absolute ``/{namespace}/...`` string fields on ``config`` to relative names.

    Leaves absolute topics outside the namespace unchanged. No-op when
    ``namespace`` is empty.
    """
    ns = (namespace or "").strip("/")
    if not ns:
        return
    for field in fields(config):
        value = getattr(config, field.name)
        if isinstance(value, str) and value.startswith("/"):
            setattr(config, field.name, to_node_relative_topic(value, ns))


# Handled by resolve_core_topic / robot_description helpers — do not rewrite here,
# otherwise connect() can double-prefix (e.g. /g1d_01/g1d_01/joint_states).
_APPLY_PREFIX_SKIP_FIELDS = frozenset(
    {
        "joint_states_topic",
        "robot_description_topic",
        "topic_prefix",
        "namespace",
        "node_name",
    }
)


def apply_prefix_to_config_paths(
    config: Any,
    prefix: str,
    available: Iterable[str],
) -> None:
    """Rewrite absolute topic/action string fields on ``config`` when the prefixed path exists.

    Only mutates string fields whose current value starts with ``/``.
    Fields that are ``None`` or relative names are left unchanged.
    Core topics (``joint_states_topic``, ``robot_description_topic``) are skipped —
    ``connect()`` resolves those via ``resolve_core_topic`` to avoid double-prefixing.
    """
    if not prefix:
        return

    available_set: Set[str] = available if isinstance(available, set) else set(available)
    for field in fields(config):
        if field.name in _APPLY_PREFIX_SKIP_FIELDS:
            continue
        value = getattr(config, field.name)
        if not isinstance(value, str) or not value.startswith("/"):
            continue
        candidate = prefixed_name(value, prefix)
        if path_exists_in_graph(candidate, available_set):
            setattr(config, field.name, candidate)


def is_robot_description_topic(name: str, types: Optional[Sequence[str]] = None) -> bool:
    """True if ``name`` looks like a robot_description topic (optionally check String type)."""
    if not name.endswith(ROBOT_DESCRIPTION_SUFFIX):
        return False
    if types is None:
        return True
    return any(
        t in ("std_msgs/msg/String", "std_msgs/String")
        for t in types
    )


def list_robot_description_topics(
    topics_and_types: Sequence[tuple[str, Sequence[str]]],
) -> List[str]:
    """Return sorted robot_description topic names from a get_topic_names_and_types result."""
    found = [
        name
        for name, types in topics_and_types
        if is_robot_description_topic(name, types)
    ]
    return sorted(found)
