"""
Check whether arm_handler.check_arrival() honors pose_threshold.

The script sends a sequence of targets with progressively stricter position
thresholds. As soon as check_arrival() returns True for the current threshold,
it prints the returned distances and sends the next target.
"""

import argparse
import inspect
import sys
import time
from typing import Iterable

from geometry_msgs.msg import Point, Pose, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


CHECK_INTERVAL = 0.1
MAX_WAIT_TIME = 20.0
ORIENT_THRESHOLD_DEG = 5.0

# (pose_threshold_m, z_offset_from_start_m)
THRESHOLD_STEPS = [
    (0.2, 0.2),
    (0.2, -0.2),
    (0.05, 0.2),
]


def create_pose(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def print_result(prefix: str, result: dict, pose_threshold: float) -> None:
    print(
        f"{prefix} arrived={result.get('arrived')}  "
        f"pose_threshold={pose_threshold:.3f}m  "
        f"position_distance={result.get('position_distance', float('inf')):.4f}m  "
        f"orientation_angle_deg={result.get('orientation_angle_deg', float('inf')):.4f}  "
        f"orientation_distance={result.get('orientation_distance', float('inf')):.6f}",
        flush=True,
    )


def wait_until_arrived(handler, pose_threshold: float, orient_threshold: float) -> bool:
    start = time.time()
    checks = 0
    final_result = None

    while True:
        result = handler.check_arrival(
            pose_threshold=pose_threshold,
            orient_threshold=orient_threshold,
        )
        final_result = result
        checks += 1

        if result.get("arrived"):
            print_result(f"  TRUE after {time.time() - start:.2f}s / {checks} checks:", result, pose_threshold)
            return True

        if checks == 1 or checks % 10 == 0:
            print_result(f"  poll {checks:03d}:", result, pose_threshold)

        if time.time() - start >= MAX_WAIT_TIME:
            print_result(f"  TIMEOUT after {MAX_WAIT_TIME:.1f}s:", final_result, pose_threshold)
            return False

        time.sleep(CHECK_INTERVAL)


def parse_threshold_steps(raw_steps: Iterable[str]) -> list[tuple[float, float]]:
    steps = []
    for raw_step in raw_steps:
        threshold_text, offset_text = raw_step.split(":", 1)
        steps.append((float(threshold_text), float(offset_text)))
    return steps


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send target ladder and print when check_arrival() becomes true."
    )
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument("--frame-id", default=None, help="Override target frame_id; defaults to handler frame_id or world.")
    parser.add_argument(
        "--step",
        action="append",
        default=[],
        help="Override threshold ladder item as THRESHOLD:Z_OFFSET, e.g. --step 0.5:0.08",
    )
    args = parser.parse_args()

    threshold_steps = parse_threshold_steps(args.step) if args.step else THRESHOLD_STEPS

    print("\n" + "=" * 78)
    print(" " * 16 + "check_arrival pose_threshold ladder test")
    print("=" * 78)

    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)

    try:
        print("[1] Connecting to ROS 2...")
        interface.connect()
        time.sleep(2.0)

        handler = interface.left_arm_handler if args.arm == "left" else interface.right_arm_handler
        if handler is None:
            print(f"ERROR: {args.arm} arm handler is not available.")
            return 1

        print(f"[2] Using {handler.label}")
        print(f"    check_arrival source: {inspect.getsourcefile(handler.check_arrival)}")
        print(f"    config.pose_position_threshold: {interface.config.pose_position_threshold}")
        print(f"    config.pose_orientation_threshold: {interface.config.pose_orientation_threshold}")

        start_pose = handler.get_pose()
        if start_pose is None:
            print("ERROR: current arm pose is unavailable. Check current pose topic.")
            return 1

        frame_id = args.frame_id or handler.get_frame_id() or "world"
        print(f"[3] Target frame_id: {frame_id}")
        print(
            "    start position: "
            f"({start_pose.position.x:.4f}, {start_pose.position.y:.4f}, {start_pose.position.z:.4f})"
        )

        base_x = start_pose.position.x
        base_y = start_pose.position.y
        base_z = start_pose.position.z
        qx = start_pose.orientation.x
        qy = start_pose.orientation.y
        qz = start_pose.orientation.z
        qw = start_pose.orientation.w

        for index, (pose_threshold, z_offset) in enumerate(threshold_steps, start=1):
            target_pose = create_pose(base_x, base_y, base_z + z_offset, qx, qy, qz, qw)
            print("\n" + "-" * 78)
            print(
                f"[step {index}] send target z_offset={z_offset:+.3f}m, "
                f"pose_threshold={pose_threshold:.3f}m"
            )
            handler.send_target_stamped(frame_id, target_pose)
            print("  target published; polling check_arrival()...")

            arrived = wait_until_arrived(handler, pose_threshold, ORIENT_THRESHOLD_DEG)
            if not arrived:
                print("  stopping because this threshold never returned true.")
                return 2

        print("\nAll threshold steps returned true. If the TRUE distances track each threshold, the argument is effective.")
        return 0
    finally:
        try:
            interface.disconnect()
            print("\nDisconnected.")
        except Exception:
            pass


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(1)
