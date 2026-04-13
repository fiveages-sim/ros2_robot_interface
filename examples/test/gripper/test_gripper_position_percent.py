"""
夹爪百分比位置控制测试脚本
每3秒在 0.0 和 1.0 之间交替发送百分比目标位置
支持单臂和双臂机器人
"""

import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    """主测试函数"""
    print("\n" + "=" * 70)
    print(" " * 16 + "夹爪百分比位置控制测试")
    print("=" * 70 + "\n")

    # ========================================================================
    # 第一部分：初始化和连接
    # ========================================================================
    print("[1] 创建配置...")
    config = ROS2RobotInterfaceConfig()

    print("[2] 创建ROS2RobotInterface实例...")
    interface = ROS2RobotInterface(config)

    print("[3] 连接到ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功!\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1

    # 等待数据到达
    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")

    # 检测是否为双臂模式
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    print(f"[5] 检测到模式: {'双臂模式' if is_dual_arm else '单臂模式'}\n")

    # ========================================================================
    # 第二部分：检查可用夹爪及百分比话题
    # ========================================================================
    left_gripper_available = (
        interface.left_gripper_handler is not None
        and interface.left_gripper_handler.target_percent_pub is not None
    )
    right_gripper_available = (
        is_dual_arm
        and interface.right_gripper_handler is not None
        and interface.right_gripper_handler.target_percent_pub is not None
    )

    if not left_gripper_available and not right_gripper_available:
        print("⚠ 没有检测到支持 target_percent 话题的夹爪，退出测试")
        print("  请确认 /left_gripper_controller/target_percent 或")
        print("  /right_gripper_controller/target_percent 话题已发布")
        interface.disconnect()
        return 1

    print("-" * 70)
    print("开始夹爪百分比位置控制测试")
    print("-" * 70)
    if left_gripper_available:
        print(f"  左夹爪: 已启用  话题: {interface.left_gripper_handler.target_percent_topic}")
    if right_gripper_available:
        print(f"  右夹爪: 已启用  话题: {interface.right_gripper_handler.target_percent_topic}")
    print("  测试间隔: 3秒")
    print("  将依次发送 0.0 / 0.25 / 0.5 / 0.75 / 1.0 五个目标值")
    print("  按 Ctrl+C 退出测试\n")

    # 等待一下，让订阅器接收初始状态
    print("等待状态同步（1秒）...")
    time.sleep(1.0)
    print("✓ 开始测试\n")

    # ========================================================================
    # 第三部分：主循环 - 每3秒交替发送
    # ========================================================================
    i = 0
    targets = [0.0, 0.25, 0.5, 0.75, 1.0]
    try:
        while True:
            time.sleep(3.0)
            i += 1
            target_percent = targets[i % len(targets)]

            # 左夹爪
            if left_gripper_available:
                try:
                    interface.left_gripper_handler.send_position_percent(target_percent)
                    print(f"[{i:3d}s] [左夹爪] 发送百分比目标: {target_percent:.2f}")
                except Exception as e:
                    print(f"[{i:3d}s] [左夹爪] ✗ 发送失败: {e}")

            # 右夹爪（仅双臂模式）
            if right_gripper_available:
                try:
                    interface.right_gripper_handler.send_position_percent(target_percent)
                    print(f"[{i:3d}s] [右夹爪] 发送百分比目标: {target_percent:.2f}")
                except Exception as e:
                    print(f"[{i:3d}s] [右夹爪] ✗ 发送失败: {e}")

            print()  # 空行分隔

    except KeyboardInterrupt:
        print("\n\n用户中断测试")

    # ========================================================================
    # 第四部分：清理和退出
    # ========================================================================
    print("\n" + "=" * 70)
    print("测试完成")
    print("=" * 70)

    print("\n断开连接...")
    interface.disconnect()
    print("✓ 已断开连接")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
