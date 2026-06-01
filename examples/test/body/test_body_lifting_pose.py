"""
测试 body 升降与姿态（x, z, phi）接口。

覆盖以下接口：
- send_waist_lifting_pose_relative(dx, dz, dphi)
- send_waist_lifting_pose_absolute(x, z, phi)
"""

import math
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main() -> int:
    print("\n" + "=" * 70)
    print(" " * 14 + "Body Lifting Pose (x, z, phi) Test")
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

    print("[4] 检查 body x/z/phi 控制 topic 是否可用...")
    if interface.config.waist_lifting_pose_relative_topic is None:
        print("    ✗ 未启用 waist_lifting_pose_relative_topic")
        interface.disconnect()
        return 1
    if interface.config.waist_lifting_pose_absolute_topic is None:
        print("    ✗ 未启用 waist_lifting_pose_absolute_topic")
        interface.disconnect()
        return 1
    print("    ✓ body x/z/phi 控制功能已启用\n")

    # 将状态切至可执行控制指令的流程：HOME -> HOLD -> MOVEJ
    print("[5] 切换到 HOME 状态...")
    interface.send_fsm_command(1)
    time.sleep(5.0)
    print("    ✓ HOME 完成\n")

    print("[6] 切换到 HOLD 状态...")
    interface.send_fsm_command(2)
    time.sleep(1.0)
    print("    ✓ HOLD 完成\n")

    print("[7] 切换到 MOVEJ 状态...")
    interface.send_fsm_command(4)
    time.sleep(1.0)
    print("    ✓ MOVEJ 完成\n")

    print("-" * 70)
    print("[8] 相对位姿测试：send_waist_lifting_pose_relative(dx, dz, dphi)")
    print("-" * 70)
    relative_steps = [
        (0.02, 0.02, math.radians(5.0), "前上+小角度正向"),
        (-0.02, -0.02, math.radians(-5.0), "回到初始附近"),
    ]
    for idx, (dx, dz, dphi, label) in enumerate(relative_steps, start=1):
        print(
            f"  [8.{idx}] {label}: dx={dx:+.3f}, dz={dz:+.3f}, dphi={math.degrees(dphi):+.1f}deg"
        )
        interface.send_waist_lifting_pose_relative(dx, dz, dphi)
        time.sleep(3.0)
        print("        ✓ 指令已发送并等待执行")

    print("-" * 70)
    print("[9] 绝对位姿测试：send_waist_lifting_pose_absolute(x, z, phi)")
    print("-" * 70)
    absolute_steps = [
        (0.02, 0.78, math.radians(8.0), "目标A"),
        (0.00, 0.75, 0.0, "目标B（回中）"),
    ]
    for idx, (x, z, phi, label) in enumerate(absolute_steps, start=1):
        print(
            f"  [9.{idx}] {label}: x={x:+.3f}, z={z:+.3f}, phi={math.degrees(phi):+.1f}deg"
        )
        interface.send_waist_lifting_pose_absolute(x, z, phi)
        time.sleep(3.0)
        print("        ✓ 指令已发送并等待执行")

    print("\n[10] 切回 HOLD 并断开连接...")
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
