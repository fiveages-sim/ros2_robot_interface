"""
测试双臂末端笛卡尔目标与 body 关节目标连续发布。

覆盖以下接口：
- send_dual_arm_target_stamped(left_pose, right_pose, frame_id="arm_base")
- send_body_joint_positions([j1, j2, j3, j4])
"""

import sys
import time

from geometry_msgs.msg import Point, Pose, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


# ============================================================================
# 测试参数（按需修改）
# ============================================================================
INITIAL_ARM_FRAME_ID = "arm_base"
TARGET_ARM_FRAME_ID = "base_link"
STEP_WAIT = 6.0

# 初始左右臂目标位姿（arm_base）
INITIAL_LEFT_POSE = {
    "position": (0.520, 0.36, -0.02),
    "orientation": (-0.697472, 0.116332, -0.701219, -0.091063),
}
INITIAL_RIGHT_POSE = {
    "position": (0.520, -0.36, -0.02),
    "orientation": (0.687865, 0.163835, 0.695101, -0.129747),
}
INITIAL_BODY_POSITIONS = [-0.6, -1.2, -0.6, -3.141592653589793]

# 左右臂目标位姿（base_link）
LEFT_TARGET_POSE = {
    "position": (0.636043, 0.426844, -0.115453),
    "orientation": (0.49616, -0.116139, 0.859226, 0.0454699),
}
RIGHT_TARGET_POSE = {
    "position": (0.669485, -0.27671, -0.116767),
    "orientation": (0.48127, 0.2151, 0.843821, -0.100383),
}

BODY_TARGET_POSITIONS = [-1.3526, -2.4199, -1.6997, -3.1416]


def print_pose(label, pose) -> None:
    print(f"  {label}:")
    print(
        f"    position: ({pose.position.x:+.4f}, {pose.position.y:+.4f}, "
        f"{pose.position.z:+.4f})"
    )
    print(
        f"    orientation: ({pose.orientation.x:+.4f}, {pose.orientation.y:+.4f}, "
        f"{pose.orientation.z:+.4f}, {pose.orientation.w:+.4f})"
    )


def create_pose(pose_config) -> Pose:
    position = pose_config["position"]
    orientation = pose_config["orientation"]
    pose = Pose()
    pose.position = Point(x=position[0], y=position[1], z=position[2])
    pose.orientation = Quaternion(
        x=orientation[0],
        y=orientation[1],
        z=orientation[2],
        w=orientation[3],
    )
    return pose


def send_arm_and_body_targets(interface, label, left_pose, right_pose, arm_frame_id, body_positions) -> None:
    print(f"  {label}: frame_id={arm_frame_id}")
    print_pose("左臂位姿", left_pose)
    print_pose("右臂位姿", right_pose)
    print(f"  body 目标: {[round(pos, 6) for pos in body_positions]}")
    interface.send_body_joint_positions(body_positions)
    print("        ✓ send_body_joint_positions() 已返回")
    # time.sleep(4.0)
    interface.send_dual_arm_target_stamped(left_pose, right_pose, frame_id=arm_frame_id)
    print("        ✓ send_dual_arm_target_stamped() 已返回")
    print(f"        ✓ 两个 publish 调用之间无 sleep，等待执行 {STEP_WAIT:.1f}s")
    time.sleep(STEP_WAIT)


def main() -> int:
    print("\n" + "=" * 70)
    print(" " * 7 + "Dual Arm Target + Body Joint Positions Test")
    print("=" * 70 + "\n")

    print("[1] 创建配置并初始化接口...")
    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)

    print("[2] 连接到 ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功\n")
    except Exception as exc:
        print(f"    ✗ 连接失败: {exc}\n")
        return 1

    print("[3] 等待初始数据（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据就绪\n")

    print("[4] 检查双臂和 body 控制 topic 是否可用...")
    if interface.right_arm_handler is None or interface.config.right_end_effector_target_topic is None:
        print("    ✗ 未检测到双臂笛卡尔目标配置")
        interface.disconnect()
        return 1
    if interface.body_joint_controller_pub is None:
        print("    ✗ 未启用 body_joint_controller_pub")
        print("      请确认存在 /body_joint_controller/target_joint_position")
        interface.disconnect()
        return 1
    print("    ✓ 双臂笛卡尔目标与 body 关节控制功能已启用")
    print(f"    ✓ body target topic: {interface.config.body_joint_controller_topic}\n")

    if interface.is_wbc:
        print("    ⚠ 当前检测为 WBC 模式；本测试主要用于分体控制模式")
    else:
        print("    ✓ 当前不是 WBC 模式，适合验证分体控制连续发布\n")

    print("[5] 设置初始位姿和目标位姿...")
    initial_left_pose = create_pose(INITIAL_LEFT_POSE)
    initial_right_pose = create_pose(INITIAL_RIGHT_POSE)
    left_target_pose = create_pose(LEFT_TARGET_POSE)
    right_target_pose = create_pose(RIGHT_TARGET_POSE)
    print_pose("左臂初始位姿", initial_left_pose)
    print_pose("右臂初始位姿", initial_right_pose)
    print_pose("左臂目标位姿", left_target_pose)
    print_pose("右臂目标位姿", right_target_pose)
    print()

    print("[6] 设置 body 初始关节目标和最终关节目标...")
    initial_body_positions = [float(pos) for pos in INITIAL_BODY_POSITIONS]
    body_target_positions = [float(pos) for pos in BODY_TARGET_POSITIONS]
    print(f"    body 初始目标: {[round(pos, 6) for pos in initial_body_positions]}")
    print(f"    body 最终目标: {[round(pos, 6) for pos in body_target_positions]}\n")

    # 将状态切至适合手臂笛卡尔控制的流程：HOME -> HOLD -> OCS2
    print("[7] 切换到 HOME 状态...")
    interface.send_fsm_command(1)
    time.sleep(5.0)
    print("    ✓ HOME 完成\n")

    print("[8] 切换到 HOLD 状态...")
    interface.send_fsm_command(2)
    time.sleep(1.0)
    print("    ✓ HOLD 完成\n")

    print("[9] 切换到 OCS2 状态...")
    interface.send_fsm_command(3)
    time.sleep(1.0)
    print("    ✓ OCS2 完成\n")

    print("-" * 70)
    print("[10] 移动到初始姿态：先 send_dual_arm_target_stamped，再 send_body_joint_positions")
    print("-" * 70)
    send_arm_and_body_targets(
        interface,
        "初始姿态",
        initial_left_pose,
        initial_right_pose,
        INITIAL_ARM_FRAME_ID,
        initial_body_positions,
    )

    print("-" * 70)
    print("[11] 移动到最终姿态：先 send_dual_arm_target_stamped，再 send_body_joint_positions")
    print("-" * 70)
    send_arm_and_body_targets(
        interface,
        "最终姿态",
        left_target_pose,
        right_target_pose,
        TARGET_ARM_FRAME_ID,
        body_target_positions,
    )

    print("\n[12] 切回 HOLD 并断开连接...")
    try:
        interface.send_fsm_command(2)
        time.sleep(1.0)
    except Exception as exc:
        print(f"    ⚠ 切换 HOLD 失败: {exc}")

    interface.disconnect()
    print("    ✓ 已断开连接\n")

    print("=" * 70)
    print("测试完成")
    print("=" * 70 + "\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⚠ 用户中断测试")
        sys.exit(1)
    except Exception as exc:
        print(f"\n✗ 测试失败: {exc}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
