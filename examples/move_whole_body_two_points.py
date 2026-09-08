"""Run one whole-body pass through a selected dual-arm pose from measured home.

The home values supplied by the user are in degrees. The chosen Cartesian pose
is already expressed in ``base_link`` as ``[x, y, z, qx, qy, qz, qw]``; no
frame conversion is performed at runtime. Before the pose runs, the body is
switched to ``BODY_CUSTOM_LOCK``; a single pose (``pose1`` by default) runs and
the robot returns to the measured home posture before it unless ``--skip-home``
is set.

Run::

    python examples/move_whole_body_two_points.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Sequence

from geometry_msgs.msg import Point, Pose, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


# body joint 限位（度），来源 fa-w2-components/xacro/components/body.xacro：
#   body_joint1 [-77.5,  +32.5]    body_joint2 [-152.41, +91.96]    body_joint3 [-100, +100]
HOME_BODY_DEG = [
    -75.00,    # body_joint1（限位 [-77.5, +32.5]）
    -145.00,   # body_joint2（限位 [-152.41, +91.96]）
    -60.00,    # body_joint3（限位 [-100, +100]）
    0.00,      # body_joint4（限位 [-180, +180]）
]
HOME_LEFT_ARM_DEG = [-25.32, 54.17, 54.19, -124.08, -48.84, -20.11, -8.99]
HOME_RIGHT_ARM_DEG = [25.32, 54.17, -54.19, -124.08, 48.84, -20.11, 8.99]

# Values are already in base_link (measured from the old arm_base conversion);
# no frame conversion is performed at runtime anymore.
POSES = (
    (
        "pose1",
        [0.744108, 0.290164, 0.472944,
         0.688362, -0.042152, 0.723706, 0.025126],
        [0.733129, -0.278338, 0.462215,
         -0.688356, -0.042154, -0.723711, 0.025126],
    ),
)
COMMAND_FRAME = "base_link"
BODY_LOCK_COMMAND = "BODY_CUSTOM_LOCK"
JOINT_TOLERANCE_RAD = math.radians(2.0)
POSE_TOLERANCE_M = 0.005
POLL_PERIOD_SEC = 0.1


def degrees_to_radians(values: Sequence[float]) -> list[float]:
    return [math.radians(value) for value in values]


def make_pose(values: Sequence[float]) -> Pose:
    if len(values) != 7:
        raise ValueError(f"pose must contain 7 values, got {len(values)}")
    x, y, z, qx, qy, qz, qw = map(float, values)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError("pose quaternion must be non-zero")
    return Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=qx / norm, y=qy / norm, z=qz / norm, w=qw / norm),
    )


def format_pose(pose: Pose) -> str:
    p = pose.position
    q = pose.orientation
    return (
        f"pos=({p.x:.6f}, {p.y:.6f}, {p.z:.6f}) "
        f"quat=({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})"
    )


def wait_for_both_arms(
    interface: ROS2RobotInterface,
    *,
    timeout: float,
    pose_tolerance: float,
) -> bool:
    deadline = time.monotonic() + timeout
    last_left = last_right = None
    while time.monotonic() <= deadline:
        last_left = interface.left_arm_handler.check_arrival(
            pose_threshold=pose_tolerance
        )
        last_right = interface.right_arm_handler.check_arrival(
            pose_threshold=pose_tolerance
        )
        if last_left.get("arrived") and last_right.get("arrived"):
            return True
        time.sleep(POLL_PERIOD_SEC)
    print(f"  timeout: left={last_left}, right={last_right}", file=sys.stderr)
    return False


def move_home(interface: ROS2RobotInterface, timeout: float) -> bool:
    body = degrees_to_radians(HOME_BODY_DEG)
    left = degrees_to_radians(HOME_LEFT_ARM_DEG)
    right = degrees_to_radians(HOME_RIGHT_ARM_DEG)

    print("[home] sending measured body and dual-arm joint targets")
    interface.send_coordinated_joint_positions(
        body_positions=body,
        left_arm_positions=left,
        right_arm_positions=right,
    )
    result = interface.wait_until_joint_arrive(
        body_target_positions=body,
        left_target_positions=left,
        right_target_positions=right,
        timeout=timeout,
        poll_period=POLL_PERIOD_SEC,
        joint_tolerance=JOINT_TOLERANCE_RAD,
    )
    if not result["arrived"]:
        print(f"[home] timeout: {result}", file=sys.stderr)
        return False
    print(f"[home] arrived in {result['elapsed']:.2f}s")
    return True


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            interface.send_fsm_command(2)  # HOLD
    except Exception as exc:
        print(f"[cleanup] failed to switch to HOLD: {exc}", file=sys.stderr)
    finally:
        if interface.is_connected:
            interface.disconnect()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-home",
        action="store_true",
        help="do not return home at the beginning of the run",
    )
    parser.add_argument("--return-home", action="store_true", help="return home after the selected pose")
    parser.add_argument(
        "--pose",
        choices=[label for label, _, _ in POSES],
        default="pose1",
        help="which single pose to run (default: pose1)",
    )
    parser.add_argument("--home-timeout", type=float, default=15.0)
    parser.add_argument("--pose-timeout", type=float, default=15.0)
    parser.add_argument("--movel-duration", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if min(args.home_timeout, args.pose_timeout, args.movel_duration) <= 0:
        raise ValueError("timeouts and movel duration must be positive")

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    try:
        time.sleep(1.0)
        if interface.right_arm_handler is None:
            raise RuntimeError("this program requires a dual-arm robot")

        poses_by_label = {label: (left, right) for label, left, right in POSES}
        left_values, right_values = poses_by_label[args.pose]
        step = args.pose
        print(f"\n[run] pose={args.pose}")
        if not args.skip_home and not move_home(interface, args.home_timeout):
            return 1

        print(f"[mode] switching body to custom lock ({BODY_LOCK_COMMAND})")
        interface.send_mode_command(BODY_LOCK_COMMAND)

        left_target = make_pose(left_values)
        right_target = make_pose(right_values)
        print(f"[{step}] sending whole-body target in {COMMAND_FRAME}:")
        print(f"  left : {format_pose(left_target)}")
        print(f"  right: {format_pose(right_target)}")
        interface.send_dual_arm_target_stamped(
            left_pose=left_target,
            right_pose=right_target,
            frame_id=COMMAND_FRAME,
            movel_duration=args.movel_duration,
        )
        if not wait_for_both_arms(
            interface,
            timeout=args.pose_timeout,
            pose_tolerance=POSE_TOLERANCE_M,
        ):
            print(f"[{step}] failed to arrive; stopping sequence", file=sys.stderr)
            return 1
        print(f"[{step}] arrived")

        if args.return_home and not move_home(interface, args.home_timeout):
            return 1
        return 0
    finally:
        cleanup(interface)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted; switching to HOLD.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
