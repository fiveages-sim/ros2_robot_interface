"""Motion generation for ros2_robot_interface.

Provides Pose-native builders for common motion sequences (pick, place, handover)
and a configurable executor that supports multiple ROS2 motion interfaces
(unstamped, stamped, dual-arm stamped).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any, Callable

from geometry_msgs.msg import Pose

if TYPE_CHECKING:
    from .ros_interface import ROS2RobotInterface

logger = logging.getLogger(__name__)

DirectionVec = tuple[float, float, float]

PICK_STAGE_SUFFIXES: tuple[str, ...] = ("Approach", "CloseIn", "Grasp", "Retreat")
PLACE_STAGE_SUFFIXES: tuple[str, ...] = ("Place", "Release", "PostReleaseRetreat")
HANDOVER_STAGE_SUFFIXES: tuple[str, ...] = ("SyncMove", "ReceiverGrasp", "SourceRelease")
CARRY_STAGE_SUFFIXES: tuple[str, ...] = ("Approach", "Forward", "CloseIn", "Grasp", "Lift", "Retreat")

_GRASP_DIRECTION_TO_VEC: dict[str, DirectionVec] = {
    "top": (0.0, 0.0, 1.0),
    "front": (-1.0, 0.0, 0.0),
    "back": (1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
}


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class ArmSide(Enum):
    LEFT = "left"
    RIGHT = "right"


class SendMode(Enum):
    """Which ROS2 transport to use when publishing arm targets."""
    UNSTAMPED = "unstamped"
    STAMPED = "stamped"
    DUAL_ARM_STAMPED = "dual_stamped"


class GripperMode(Enum):
    """Which gripper command interface to use."""
    JOINT_POSITION = "joint_position"
    TARGET_COMMAND = "target_command"


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class ArmTarget:
    """Arm-agnostic target: a Pose plus a gripper value."""
    pose: Pose
    gripper: float


ArmStage = tuple[str, ArmTarget]
"""(stage_name, arm_target) — arm-agnostic; left/right assigned later."""


@dataclass
class StageTarget:
    """A single execution stage with optional left/right arm targets."""
    name: str
    left: ArmTarget | None = None
    right: ArmTarget | None = None

    def to_action_dict(
        self,
        left_ee_prefix: str = "left_ee",
        left_gripper_key: str = "left_gripper.pos",
        right_ee_prefix: str = "right_ee",
        right_gripper_key: str = "right_gripper.pos",
    ) -> dict[str, float]:
        """Flatten to a ``{key: float}`` dict (for recording / LeRobot compatibility)."""
        d: dict[str, float] = {}
        if self.left:
            d.update(_arm_target_to_flat_dict(self.left, left_ee_prefix, left_gripper_key))
        if self.right:
            d.update(_arm_target_to_flat_dict(self.right, right_ee_prefix, right_gripper_key))
        return d


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _arm_target_to_flat_dict(
    target: ArmTarget, ee_prefix: str, gripper_key: str,
) -> dict[str, float]:
    return {
        f"{ee_prefix}.pos.x": target.pose.position.x,
        f"{ee_prefix}.pos.y": target.pose.position.y,
        f"{ee_prefix}.pos.z": target.pose.position.z,
        f"{ee_prefix}.quat.x": target.pose.orientation.x,
        f"{ee_prefix}.quat.y": target.pose.orientation.y,
        f"{ee_prefix}.quat.z": target.pose.orientation.z,
        f"{ee_prefix}.quat.w": target.pose.orientation.w,
        gripper_key: target.gripper,
    }


def _stage_name(prefix: str, index: int, suffix: str) -> str:
    return f"{prefix}-{index}-{suffix}"


def _resolve_grasp_direction_vec(
    *,
    grasp_direction: str,
    grasp_direction_vector: DirectionVec | None,
) -> DirectionVec:
    if grasp_direction_vector is not None:
        vx, vy, vz = (
            float(grasp_direction_vector[0]),
            float(grasp_direction_vector[1]),
            float(grasp_direction_vector[2]),
        )
    else:
        key = grasp_direction.lower()
        if key not in _GRASP_DIRECTION_TO_VEC:
            raise ValueError(
                f"Unsupported grasp_direction: {grasp_direction}. "
                f"Expected one of: {', '.join(sorted(_GRASP_DIRECTION_TO_VEC))}"
            )
        vx, vy, vz = _GRASP_DIRECTION_TO_VEC[key]

    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1e-8:
        raise ValueError("grasp_direction_vector norm is too small")
    return (vx / norm, vy / norm, vz / norm)


def _make_pose_from_target(
    target_pose: Pose,
    *,
    offset: float,
    direction_vec: DirectionVec,
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    dx, dy, dz = direction_vec
    pose.position.x = target_pose.position.x + dx * offset
    pose.position.y = target_pose.position.y + dy * offset
    pose.position.z = target_pose.position.z + dz * offset
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


def _make_pose(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


# ---------------------------------------------------------------------------
# Sequence builders (arm-agnostic → list[ArmStage])
# ---------------------------------------------------------------------------

def build_single_arm_pick_sequence(
    *,
    target_pose: Pose,
    approach_clearance: float,
    grasp_clearance: float,
    grasp_orientation: tuple[float, float, float, float],
    grasp_direction: str = "top",
    grasp_direction_vector: DirectionVec | None = None,
    grasp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    retreat_direction_extra: float = 0.0,
    retreat_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Pickup",
) -> list[ArmStage]:
    direction_vec = _resolve_grasp_direction_vec(
        grasp_direction=grasp_direction,
        grasp_direction_vector=grasp_direction_vector,
    )
    approach_pose = _make_pose_from_target(
        target_pose, offset=approach_clearance,
        direction_vec=direction_vec, orientation=grasp_orientation,
    )
    close_in_pose = _make_pose_from_target(
        target_pose, offset=grasp_clearance,
        direction_vec=direction_vec, orientation=grasp_orientation,
    )
    close_in_pose.position.x += grasp_offset[0]
    close_in_pose.position.y += grasp_offset[1]
    close_in_pose.position.z += grasp_offset[2]
    retreat_pose = _make_pose_from_target(
        target_pose, offset=approach_clearance + retreat_direction_extra,
        direction_vec=direction_vec, orientation=grasp_orientation,
    )
    retreat_pose.position.x += retreat_offset[0]
    retreat_pose.position.y += retreat_offset[1]
    retreat_pose.position.z += retreat_offset[2]

    return [
        (
            _stage_name(stage_prefix, 1, PICK_STAGE_SUFFIXES[0]),
            ArmTarget(pose=approach_pose, gripper=gripper_open),
        ),
        (
            _stage_name(stage_prefix, 2, PICK_STAGE_SUFFIXES[1]),
            ArmTarget(pose=close_in_pose, gripper=gripper_open),
        ),
        (
            _stage_name(stage_prefix, 3, PICK_STAGE_SUFFIXES[2]),
            ArmTarget(pose=close_in_pose, gripper=gripper_closed),
        ),
        (
            _stage_name(stage_prefix, 4, PICK_STAGE_SUFFIXES[3]),
            ArmTarget(pose=retreat_pose, gripper=gripper_closed),
        ),
    ]


def build_single_arm_place_sequence(
    *,
    place_position: tuple[float, float, float],
    place_orientation: tuple[float, float, float, float],
    post_release_retract_offset: tuple[float, float, float],
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Place",
    start_index: int = 1,
) -> list[ArmStage]:
    place_pose = _make_pose(place_position, place_orientation)
    retract_pose = _make_pose(
        (
            place_position[0] + post_release_retract_offset[0],
            place_position[1] + post_release_retract_offset[1],
            place_position[2] + post_release_retract_offset[2],
        ),
        place_orientation,
    )
    return [
        (
            _stage_name(stage_prefix, start_index, PLACE_STAGE_SUFFIXES[0]),
            ArmTarget(pose=place_pose, gripper=gripper_closed),
        ),
        (
            _stage_name(stage_prefix, start_index + 1, PLACE_STAGE_SUFFIXES[1]),
            ArmTarget(pose=place_pose, gripper=gripper_open),
        ),
        (
            _stage_name(stage_prefix, start_index + 2, PLACE_STAGE_SUFFIXES[2]),
            ArmTarget(pose=retract_pose, gripper=gripper_open),
        ),
    ]


def build_single_arm_return_home_sequence(
    *,
    home_pose: Pose,
    gripper: float,
    stage_name: str = "Return-1-ReturnHome",
) -> list[ArmStage]:
    return [(stage_name, ArmTarget(pose=home_pose, gripper=gripper))]


# ---------------------------------------------------------------------------
# Arm assignment / composition (ArmStage → StageTarget)
# ---------------------------------------------------------------------------

def assign_to_arm(
    sequence: list[ArmStage],
    side: ArmSide,
) -> list[StageTarget]:
    """Assign an arm-agnostic sequence to a specific arm side."""
    result: list[StageTarget] = []
    for name, target in sequence:
        if side == ArmSide.LEFT:
            result.append(StageTarget(name=name, left=target))
        else:
            result.append(StageTarget(name=name, right=target))
    return result


def compose_bimanual_synchronized_sequence(
    left_sequence: list[ArmStage],
    right_sequence: list[ArmStage],
) -> list[StageTarget]:
    """Merge two arm-agnostic sequences into a synchronised bimanual sequence."""
    if len(left_sequence) != len(right_sequence):
        raise ValueError(
            "Left/right sequences must have the same number of stages "
            "for synchronized composition"
        )
    merged: list[StageTarget] = []
    for idx, ((left_name, left_target), (right_name, right_target)) in enumerate(
        zip(left_sequence, right_sequence), start=1,
    ):
        stage_label = f"{idx:02d}-{left_name}|{right_name}"
        merged.append(StageTarget(name=stage_label, left=left_target, right=right_target))
    return merged


def build_handover_sequence(
    *,
    source_handover_pose: Pose,
    receiver_handover_pose: Pose,
    source_arm: ArmSide,
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Handover",
) -> list[StageTarget]:
    """Build a bimanual handover sequence (inherently two-armed)."""
    receiver_open = ArmTarget(pose=receiver_handover_pose, gripper=gripper_open)
    receiver_closed = ArmTarget(pose=receiver_handover_pose, gripper=gripper_closed)
    source_closed = ArmTarget(pose=source_handover_pose, gripper=gripper_closed)
    source_open = ArmTarget(pose=source_handover_pose, gripper=gripper_open)

    def _target(idx: int, suffix: str, src: ArmTarget, rcv: ArmTarget) -> StageTarget:
        name = _stage_name(stage_prefix, idx, suffix)
        if source_arm == ArmSide.LEFT:
            return StageTarget(name=name, left=src, right=rcv)
        return StageTarget(name=name, left=rcv, right=src)

    return [
        _target(1, HANDOVER_STAGE_SUFFIXES[0], source_closed, receiver_open),
        _target(2, HANDOVER_STAGE_SUFFIXES[1], source_closed, receiver_closed),
        _target(3, HANDOVER_STAGE_SUFFIXES[2], source_open, receiver_closed),
    ]


def build_bimanual_carry_sequence(
    *,
    object_center: Pose,
    lateral_offset: float,
    approach_offset: tuple[float, float, float],
    lateral_clearance: float = 0.0,
    grasp_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    left_orientation: tuple[float, float, float, float],
    right_orientation: tuple[float, float, float, float],
    lift_offset: tuple[float, float, float],
    retreat_offset: tuple[float, float, float],
    gripper_open: float,
    gripper_closed: float,
    stage_prefix: str = "Carry",
) -> list[StageTarget]:
    """Build a bimanual symmetric carry sequence.

    Both arms approach, close in, grasp, lift and retreat synchronously.
    Left/right positions are mirrored about the object centre along Y.
    """
    cx = object_center.position.x
    cy = object_center.position.y
    cz = object_center.position.z

    def _lr_poses(
        x: float, y_half: float, z: float,
    ) -> tuple[Pose, Pose]:
        left = Pose()
        left.position.x, left.position.y, left.position.z = x, cy + y_half, z
        left.orientation.x, left.orientation.y = left_orientation[0], left_orientation[1]
        left.orientation.z, left.orientation.w = left_orientation[2], left_orientation[3]
        right = Pose()
        right.position.x, right.position.y, right.position.z = x, cy - y_half, z
        right.orientation.x, right.orientation.y = right_orientation[0], right_orientation[1]
        right.orientation.z, right.orientation.w = right_orientation[2], right_orientation[3]
        return left, right

    grasp_x = cx + grasp_offset[0]
    grasp_y_half = lateral_offset + grasp_offset[1]
    grasp_z = cz + grasp_offset[2]

    # 1-Approach: far back, arms spread wide
    approach_l, approach_r = _lr_poses(
        grasp_x + approach_offset[0],
        grasp_y_half + lateral_clearance + approach_offset[1],
        grasp_z + approach_offset[2],
    )
    # 2-Forward: move to object X, but still spread wide
    forward_l, forward_r = _lr_poses(
        grasp_x,
        grasp_y_half + lateral_clearance,
        grasp_z,
    )
    # 3-CloseIn: narrow to grasp position
    closein_l, closein_r = _lr_poses(grasp_x, grasp_y_half, grasp_z)
    # 4-Grasp: same position, close grippers
    # 5-Lift
    lift_l, lift_r = _lr_poses(
        grasp_x + lift_offset[0],
        grasp_y_half + lift_offset[1],
        grasp_z + lift_offset[2],
    )
    # 6-Retreat
    retreat_l, retreat_r = _lr_poses(
        grasp_x + lift_offset[0] + retreat_offset[0],
        grasp_y_half + lift_offset[1] + retreat_offset[1],
        grasp_z + lift_offset[2] + retreat_offset[2],
    )

    return [
        StageTarget(
            name=_stage_name(stage_prefix, 1, CARRY_STAGE_SUFFIXES[0]),
            left=ArmTarget(pose=approach_l, gripper=gripper_open),
            right=ArmTarget(pose=approach_r, gripper=gripper_open),
        ),
        StageTarget(
            name=_stage_name(stage_prefix, 2, CARRY_STAGE_SUFFIXES[1]),
            left=ArmTarget(pose=forward_l, gripper=gripper_open),
            right=ArmTarget(pose=forward_r, gripper=gripper_open),
        ),
        StageTarget(
            name=_stage_name(stage_prefix, 3, CARRY_STAGE_SUFFIXES[2]),
            left=ArmTarget(pose=closein_l, gripper=gripper_open),
            right=ArmTarget(pose=closein_r, gripper=gripper_open),
        ),
        StageTarget(
            name=_stage_name(stage_prefix, 4, CARRY_STAGE_SUFFIXES[3]),
            left=ArmTarget(pose=closein_l, gripper=gripper_closed),
            right=ArmTarget(pose=closein_r, gripper=gripper_closed),
        ),
        StageTarget(
            name=_stage_name(stage_prefix, 5, CARRY_STAGE_SUFFIXES[4]),
            left=ArmTarget(pose=lift_l, gripper=gripper_closed),
            right=ArmTarget(pose=lift_r, gripper=gripper_closed),
        ),
        StageTarget(
            name=_stage_name(stage_prefix, 6, CARRY_STAGE_SUFFIXES[5]),
            left=ArmTarget(pose=retreat_l, gripper=gripper_closed),
            right=ArmTarget(pose=retreat_r, gripper=gripper_closed),
        ),
    ]


# ---------------------------------------------------------------------------
# Executor helpers
# ---------------------------------------------------------------------------

def _send_arm_targets(
    interface: ROS2RobotInterface,
    stage: StageTarget,
    send_mode: SendMode,
    frame_id: str,
) -> None:
    if send_mode == SendMode.DUAL_ARM_STAMPED and stage.left and stage.right:
        interface.send_dual_arm_target_stamped(
            stage.left.pose, stage.right.pose, frame_id,
        )
        return

    use_stamped = send_mode in (SendMode.STAMPED, SendMode.DUAL_ARM_STAMPED)
    if stage.left:
        if use_stamped:
            interface.left_arm_handler.send_target_stamped(frame_id, stage.left.pose)
        else:
            interface.left_arm_handler.send_target(stage.left.pose)
    if stage.right and interface.right_arm_handler:
        if use_stamped:
            interface.right_arm_handler.send_target_stamped(frame_id, stage.right.pose)
        else:
            interface.right_arm_handler.send_target(stage.right.pose)


def _send_gripper_commands(
    interface: ROS2RobotInterface,
    stage: StageTarget,
    gripper_mode: GripperMode,
) -> None:
    if stage.left and interface.left_gripper_handler:
        if gripper_mode == GripperMode.TARGET_COMMAND:
            interface.left_gripper_handler.send_target_command(int(stage.left.gripper))
        else:
            interface.send_gripper_command(stage.left.gripper)
    if stage.right and interface.right_gripper_handler:
        if gripper_mode == GripperMode.TARGET_COMMAND:
            interface.right_gripper_handler.send_target_command(int(stage.right.gripper))
        else:
            interface.send_right_gripper_command(stage.right.gripper)


# ---------------------------------------------------------------------------
# Main executor
# ---------------------------------------------------------------------------

def execute_stage_sequence(
    *,
    interface: ROS2RobotInterface,
    sequence: list[StageTarget],
    send_mode: SendMode = SendMode.UNSTAMPED,
    frame_id: str = "arm_base",
    gripper_mode: GripperMode = GripperMode.TARGET_COMMAND,
    arrival_timeout: float,
    arrival_poll: float,
    time_now_fn: Callable[[], float],
    sleep_fn: Callable[[float], None],
    gripper_action_wait: float,
    left_arrival_guard_stage: str | None = None,
    warn_prefix: str = "Stage timeout",
    on_stage_start: Callable[[str, StageTarget], None] | None = None,
    on_stage_poll: Callable[[str, str, dict[str, Any] | None, float], None] | None = None,
    on_stage_end: Callable[[str, dict[str, Any], dict[str, Any]], None] | None = None,
) -> None:
    """Execute a sequence of :class:`StageTarget` stages.

    Which arms are waited on is determined automatically from which
    :class:`ArmTarget` slots are populated in each stage — no need for
    ``wait_both_arms`` / ``single_arm_part`` parameters.
    """
    for stage in sequence:
        logger.info("[Stage] %s", stage.name)
        if on_stage_start is not None:
            on_stage_start(stage.name, stage)

        _send_arm_targets(interface, stage, send_mode, frame_id)
        _send_gripper_commands(interface, stage, gripper_mode)

        arrive_results: dict[str, dict[str, Any]] = {}

        if stage.left:
            arrive_results["left_arm"] = interface.wait_until_arrive(
                part="left_arm",
                timeout=arrival_timeout,
                poll_period=arrival_poll,
                time_now_fn=time_now_fn,
                sleep_fn=sleep_fn,
                on_poll=(
                    (lambda _r, _e, _sn=stage.name: on_stage_poll(_sn, "left_arm", _r, _e))
                    if on_stage_poll is not None
                    else None
                ),
            )
        if stage.right:
            arrive_results["right_arm"] = interface.wait_until_arrive(
                part="right_arm",
                timeout=arrival_timeout,
                poll_period=arrival_poll,
                time_now_fn=time_now_fn,
                sleep_fn=sleep_fn,
                on_poll=(
                    (lambda _r, _e, _sn=stage.name: on_stage_poll(_sn, "right_arm", _r, _e))
                    if on_stage_poll is not None
                    else None
                ),
            )

        if "Grasp" in stage.name or "Release" in stage.name:
            sleep_fn(gripper_action_wait)

        not_arrived = {
            part: r for part, r in arrive_results.items()
            if not r.get("arrived", False)
        }
        if not_arrived:
            parts: list[str] = []
            for part, r in not_arrived.items():
                inner = r.get("result") or {}
                pos_d = inner.get("position_distance")
                ori_d = inner.get("orientation_distance")
                elapsed = r.get("elapsed")
                if isinstance(pos_d, float) and isinstance(ori_d, float):
                    parts.append(
                        f"{part}(pos_dist={pos_d:.4f}, ori_dist={ori_d:.4f}, elapsed={elapsed:.2f}s)"
                    )
                else:
                    parts.append(f"{part}(no_feedback, elapsed={elapsed:.2f}s)" if elapsed else f"{part}(no_feedback)")
            logger.warning("%s [%s]: %s", warn_prefix, stage.name, ", ".join(parts))

        left_arrive = arrive_results.get("left_arm", {"arrived": True})
        right_arrive = arrive_results.get("right_arm", {"arrived": True})

        if (
            left_arrival_guard_stage
            and stage.name == left_arrival_guard_stage
            and not left_arrive.get("arrived", False)
        ):
            raise RuntimeError(
                "Left arm did not arrive at guarded stage; skip subsequent grasp."
            )
        if on_stage_end is not None:
            on_stage_end(stage.name, left_arrive, right_arrive)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

__all__ = [
    "ArmSide",
    "ArmStage",
    "ArmTarget",
    "DirectionVec",
    "GripperMode",
    "SendMode",
    "StageTarget",
    "CARRY_STAGE_SUFFIXES",
    "HANDOVER_STAGE_SUFFIXES",
    "PICK_STAGE_SUFFIXES",
    "PLACE_STAGE_SUFFIXES",
    "assign_to_arm",
    "build_bimanual_carry_sequence",
    "build_handover_sequence",
    "build_single_arm_pick_sequence",
    "build_single_arm_place_sequence",
    "build_single_arm_return_home_sequence",
    "compose_bimanual_synchronized_sequence",
    "execute_stage_sequence",
]
