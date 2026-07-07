"""
测试 body 升降与姿态（x, z, phi）action 接口。

覆盖以下接口：
- execute_waist_lifting_pose_relative_action(dx, dz, dphi)
- execute_waist_lifting_pose_absolute_action(x, z, phi)
"""

import math
import sys
import time
from typing import Any, Optional

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def print_action_feedback(feedback: Any) -> None:
    print(f"        progress={feedback.progress:.1%}")


def print_waist_pose_result(result: Optional[Any], label: str) -> bool:
    if result is None:
        print(f"        ✗ {label}: action 无结果（goal 被拒绝或超时）")
        return False

    print(
        f"        reachable={result.reachable}, success={result.success}, "
        f"error_code={result.error_code}"
    )
    print(
        f"        planned: x={result.planned_x:+.4f}, z={result.planned_z:+.4f}, "
        f"phi={math.degrees(result.planned_phi):+.2f}deg"
    )
    if result.message:
        print(f"        message: {result.message}")

    if not result.success:
        print(f"        ✗ {label}: action 执行失败")
        return False

    if not result.reachable:
        print(f"        ⚠ {label}: 目标被裁剪后执行完成")

    print(f"        ✓ {label}: action 执行完成")
    return True


def main() -> int:
    print("\n" + "=" * 70)
    print(" " * 10 + "Body Lifting Pose Action (x, z, phi) Test")
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

    print("[4] 检查腰部位姿 action 是否可用...")
    action_name = interface.config.waist_lifting_pose_action_name
    if action_name is None:
        print("    ✗ 未检测到 waist_lifting_pose_action_name")
        interface.disconnect()
        return 1
    print(f"    action: {action_name}")

    if not interface.wait_for_waist_lifting_pose_action_server(timeout=5.0):
        print("    ✗ waist lifting pose action server 不可用")
        interface.disconnect()
        return 1
    print("    ✓ 腰部位姿 action 已就绪\n")

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
    interface.send_fsm_command(3)
    time.sleep(1.0)
    print("    ✓ MOVEJ 完成\n")

    print("-" * 70)
    print("[8] 相对位姿测试：execute_waist_lifting_pose_relative_action(dx, dz, dphi)")
    print("-" * 70)
    relative_steps = [
        (0.02, 0.02, math.radians(5.0), "前上+小角度正向"),
        (-0.02, -0.02, math.radians(-5.0), "回到初始附近"),
    ]
    for idx, (dx, dz, dphi, label) in enumerate(relative_steps, start=1):
        print(
            f"  [8.{idx}] {label}: dx={dx:+.3f}, dz={dz:+.3f}, dphi={math.degrees(dphi):+.1f}deg"
        )
        result = interface.execute_waist_lifting_pose_relative_action(
            dx, dz, dphi,
            feedback_callback=print_action_feedback,
        )
        if not print_waist_pose_result(result, label):
            interface.disconnect()
            return 1

    print("-" * 70)
    print("[9] 绝对位姿测试：execute_waist_lifting_pose_absolute_action(x, z, phi)")
    print("-" * 70)
    absolute_steps = [
        (0.02, 0.78, math.radians(8.0), "目标A"),
        (0.00, 0.75, 0.0, "目标B（回中）"),
    ]
    for idx, (x, z, phi, label) in enumerate(absolute_steps, start=1):
        print(
            f"  [9.{idx}] {label}: x={x:+.3f}, z={z:+.3f}, phi={math.degrees(phi):+.1f}deg"
        )
        result = interface.execute_waist_lifting_pose_absolute_action(
            x, z, phi,
            feedback_callback=print_action_feedback,
        )
        if not print_waist_pose_result(result, label):
            interface.disconnect()
            return 1

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
