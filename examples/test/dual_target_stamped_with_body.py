"""
Test send_dual_arm_target_stamped() with body target.

This script sends [left, right, body] together to /dual_target/stamped.
Body frame_id is explicitly set to "base_footprint".
"""

import sys
import time
import math

from geometry_msgs.msg import Point, Pose, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


MAX_WAIT_TIME = 10.0
CHECK_INTERVAL = 0.5
POSE_THRESHOLD = 0.005


def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if q_norm > 1e-12:
        qx /= q_norm
        qy /= q_norm
        qz /= q_norm
        qw /= q_norm
    else:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def wait_for_arrival(interface, max_wait_time=5.0, check_interval=0.5, pose_threshold=0.003):
    start_time = time.time()
    while time.time() - start_time < max_wait_time:
        left_result = interface.left_arm_handler.check_arrival(pose_threshold=pose_threshold)
        right_result = interface.right_arm_handler.check_arrival(pose_threshold=pose_threshold)
        if left_result["arrived"] and right_result["arrived"]:
            print(f"  Arrived in {time.time() - start_time:.2f}s")
            return True
        time.sleep(check_interval)

    print("  Timeout waiting for dual-arm arrival")
    return False


def _quat_angle_error_deg(target: Quaternion, current: Quaternion) -> float:
    """Return shortest-angle quaternion error in degrees."""
    dot = (
        target.x * current.x
        + target.y * current.y
        + target.z * current.z
        + target.w * current.w
    )
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def print_final_error(interface, left_target_pose: Pose, right_target_pose: Pose, label: str = "Final error"):
    left_current_pose = interface.left_arm_handler.get_pose()
    right_current_pose = interface.right_arm_handler.get_pose()
    if not left_current_pose or not right_current_pose:
        print(f"  [{label}] cannot read current arm poses")
        return

    ldx = left_target_pose.position.x - left_current_pose.position.x
    ldy = left_target_pose.position.y - left_current_pose.position.y
    ldz = left_target_pose.position.z - left_current_pose.position.z
    rdx = right_target_pose.position.x - right_current_pose.position.x
    rdy = right_target_pose.position.y - right_current_pose.position.y
    rdz = right_target_pose.position.z - right_current_pose.position.z

    lpos = math.sqrt(ldx * ldx + ldy * ldy + ldz * ldz)
    rpos = math.sqrt(rdx * rdx + rdy * rdy + rdz * rdz)
    lrot = _quat_angle_error_deg(left_target_pose.orientation, left_current_pose.orientation)
    rrot = _quat_angle_error_deg(right_target_pose.orientation, right_current_pose.orientation)

    print(f"  [{label}]")
    print(f"    Left  pos_err={lpos:.4f}m (dx={ldx:+.4f}, dy={ldy:+.4f}, dz={ldz:+.4f}), rot_err={lrot:.2f}deg")
    print(f"    Right pos_err={rpos:.4f}m (dx={rdx:+.4f}, dy={rdy:+.4f}, dz={rdz:+.4f}), rot_err={rrot:.2f}deg")


def main():
    print("=" * 70)
    print("Dual Target Stamped + Body Test")
    print("=" * 70)

    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)

    try:
        print("[1] Connecting...")
        interface.connect()
        time.sleep(2.0)
        print("  Connected")

        is_dual_arm = interface.config.right_end_effector_pose_topic is not None
        if not is_dual_arm:
            print("  Error: dual-arm mode is required")
            return 1

        left_current_pose = interface.left_arm_handler.get_pose()
        right_current_pose = interface.right_arm_handler.get_pose()
        if not left_current_pose or not right_current_pose:
            print("  Error: cannot read current arm poses")
            return 1

        arm_frame_id = interface.left_arm_handler.get_frame_id()
        if arm_frame_id is None:
            arm_frame_id = interface.right_arm_handler.get_frame_id()
        if arm_frame_id is None:
            arm_frame_id = "arm_base"
            print("  Warn: arm frame_id unavailable, fallback to arm_base")

        print(f"[2] arm_frame_id={arm_frame_id}, body_frame_id=base_footprint")

        left_target_pose = create_pose(
            x=0.85,
            y=0.5,
            z=0.65,
            qx=0.0707,
            qy=0.0,
            qz=0.0707,
            qw=0.0,
        )
        right_target_pose = create_pose(
            x=0.85,
            y=-0.5,
            z=0.65,
            qx=0.0707,
            qy=0.0,
            qz=0.0707,
            qw=0.0,
        )

        # Body target in base_footprint.
        # body_target_pose = create_pose(
        #     x=0.25,
        #     y=0.0,
        #     z=0.75,
        #     qx=0.0,
        #     qy=0.288,
        #     qz=0.0,
        #     qw=0.958,
        # )

        print("[3] Sending [left, right, body] to /dual_target/stamped ...")
        interface.send_dual_arm_target_stamped(
            left_pose=left_target_pose,
            right_pose=right_target_pose,
            frame_id=arm_frame_id,
            # body_pose=body_target_pose,
            # body_frame_id="base_footprint",
            body_mode="body_free"
        )
        print("  Sent")

        print(f"[4] Waiting arrival (max {MAX_WAIT_TIME}s)...")
        wait_for_arrival(interface, MAX_WAIT_TIME, CHECK_INTERVAL, POSE_THRESHOLD)
        print_final_error(interface, left_target_pose, right_target_pose, label="Step 1")

        left_target_pose = create_pose(
            x=0.7,
            y=0.4,
            z=0.9,
            qx=0.707,
            qy=0.0,
            qz=0.707,
            qw=0.0,
        )
        right_target_pose = create_pose(
            x=0.7,
            y=-0.4,
            z=0.9,
            qx=0.707,
            qy=0.0,
            qz=0.707,
            qw=0.0,
        )

        # Body target in base_footprint.
        body_target_pose = create_pose(
            x=0.0,
            y=0.0,
            z=0.85,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
        )

        print("[3] Sending [left, right, body] to /dual_target/stamped ...")
        interface.send_dual_arm_target_stamped(
            left_pose=left_target_pose,
            right_pose=right_target_pose,
            frame_id=arm_frame_id,
            body_pose=body_target_pose,
            body_frame_id="base_footprint",
        )
        print("  Sent")

        print(f"[4] Waiting arrival (max {MAX_WAIT_TIME}s)...")
        wait_for_arrival(interface, MAX_WAIT_TIME, CHECK_INTERVAL, POSE_THRESHOLD)
        print_final_error(interface, left_target_pose, right_target_pose, label="Step 2")

        # print("[5] Switching back to HOLD...")
        # interface.send_fsm_command(2)
        # time.sleep(1.0)
        return 0
    finally:
        print("[6] Disconnecting...")
        interface.disconnect()
        print("  Done")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(1)
    except Exception as exc:
        print(f"\nTest failed: {exc}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
