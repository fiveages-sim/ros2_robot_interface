"""Shared ROS2 robot interface constants."""

# FSM state command values used by controller interfaces.
FSM_HOME = 1
FSM_HOLD = 2
FSM_OCS2 = 3
FSM_MOVEJ = 4
FSM_COMPLIANCE = 5


def is_body_joint_name(joint_name: str) -> bool:
    """Whether a joint belongs to the body/waist group.

    Aligned with arms_controller_common StateMoveJ ``isJointInPrefixGroup(..., "body")``:
    ``lift_joint`` / ``*_lift_joint``, ``body*``, and Galbot-style ``leg_*``.
    """
    name = joint_name.lower()
    return (
        "body" in name
        or name.startswith("leg_")
        or name == "lift_joint"
        or name.endswith("_lift_joint")
    )


def is_head_joint_name(joint_name: str) -> bool:
    """Whether a joint belongs to the head group."""
    return "head" in joint_name.lower()

