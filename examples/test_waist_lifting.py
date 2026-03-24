"""
测试 send_waist_lifting_relative_position() send_waist_lifting_velocity_scale() send_waist_turning_velocity_scale()
"""

import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    """测试腰部升降相对位置模式"""
    
    print("\n" + "=" * 70)
    print(" " * 15 + "Waist Lifting Position Test")
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

    # ========================================================================
    # 第二部分：检查是否启用腰部控制
    # ========================================================================
    print("-" * 70)
    print("[5] 检查是否启用腰部控制")
    print("-" * 70)
    
    # 提取左臂和右臂关节位置
    waist_control_available = interface.config.waist_lifting_topic is not None
    
    if waist_control_available:
        print("腰部控制功能成功启用")
        print()

    else:
        print("⚠ 错误: 腰部控制功能未启用！\n")
        interface.disconnect()
        return 1
    
    # 切换到HOME状态（FSM循环的起始状态）
    print("-" * 70)
    print("[6] 切换到HOME状态（起始状态）")
    print("-" * 70)
    try:
        interface.send_fsm_command(1)  # 1 = HOME状态
        print("  ✓ FSM命令已发送: 切换到HOME状态")
        time.sleep(5.0)  # 等待状态转换完成
        print("  ✓ 状态转换完成\n")
    except Exception as e:
        print(f"  ⚠ 切换到HOME状态失败: {e}\n")
    
    # ========================================================================
    # 第三部分：腰部升降指令测试
    # ========================================================================
    # 切换到HOLD状态
    print("-" * 70)
    print("[7] 切换到HOLD状态")
    print("-" * 70)
    try:
        interface.send_fsm_command(2)  # 2 = HOLD状态
        print("  ✓ FSM命令已发送: 切换到HOLD状态")
        time.sleep(1.0)  # 等待状态转换完成
        print("  ✓ 状态转换完成\n")
    except Exception as e:
        print(f"  ⚠ 切换到HOLD状态失败: {e}\n")

    # 切换到MOVEJ状态
    print("-" * 70)
    print("[8] 切换到MOVEJ状态")
    print("-" * 70)
    try:
        interface.send_fsm_command(4)  # 4 = MOVEJ状态
        print("  ✓ FSM命令已发送: 切换到MOVEJ状态")
        time.sleep(1.0)  # 等待状态转换完成
        print("  ✓ 状态转换完成\n")
    except Exception as e:
        print(f"  ⚠ 切换到MOVEJ状态失败: {e}\n")

    # 发送腰部上升0.1m指令
    print("-" * 70)
    print("[9] 发送腰部上升0.1m指令")
    print("-" * 70)
    try:
        success = interface.set_node_parameters(
            full_node_name="/body_joint_controller",
            parameters={"waist_lifting_duration": float(10.0)},
        )
        if success:
            print("  ✓ 参数设置成功")
        else:
            print("  ⚠ 参数设置失败")
        print()
        interface.send_waist_lifting_relative_position(0.1)  # 发送相对当前位置上升0.1m
        print("  ✓ 命令已发送: 腰部开始上升")
        time.sleep(12.0)  # 等待腰部运动完成
        print("  ✓ 腰部到达目标位置\n")
    except Exception as e:
        print(f"  ⚠ 腰部上升控制失败: {e}\n")

    # 发送腰部下降0.1m指令
    print("-" * 70)
    print("[10] 发送腰部下降0.1m指令")
    print("-" * 70)
    try:
        success = interface.set_node_parameters(
            full_node_name="/body_joint_controller",
            parameters={"waist_lifting_duration": float(3.0)},
        )
        if success:
            print("  ✓ 参数设置成功")
        else:
            print("  ⚠ 参数设置失败")
        print()
        interface.send_waist_lifting_relative_position(-0.1)  # 发送相对当前位置下降0.1m
        print("  ✓ 命令已发送: 腰部开始下降")
        time.sleep(5.0)  # 等待腰部运动完成
        print("  ✓ 腰部到达目标位置\n")
    except Exception as e:
        print(f"  ⚠ 腰部下降控制失败: {e}\n")

    # 发送腰部以0.9倍最大升降速度上升，指令持续2s
    print("-" * 70)
    print("[11] 发送腰部0.9倍最大升降速度持续上升2s指令")
    print("-" * 70)
    try:
        interface.send_waist_lifting_velocity_scale(0.9)  # 发送设置上升速度为0.9倍最大速度
        print("  ✓ 命令已发送: 腰部开始持续上升")
        time.sleep(2.0)  # 等待腰部运动完成
        interface.send_waist_lifting_velocity_scale(0.0)  # 发送设置上升速度为0，停止升降运动
        print("  ✓ 腰部停止运动\n")
    except Exception as e:
        print(f"  ⚠ 腰部速度指令上升控制失败: {e}\n")

    # 发送腰部以0.9倍最大升降速度下降，指令持续2s
    print("-" * 70)
    print("[12] 发送腰部0.9倍最大升降速度持续下降2s指令")
    print("-" * 70)
    try:
        interface.send_waist_lifting_velocity_scale(-0.9)  # 发送设置下降速度为0.9倍最大速度
        print("  ✓ 命令已发送: 腰部开始持续下降")
        time.sleep(2.0)  # 等待腰部运动完成
        interface.send_waist_lifting_velocity_scale(0.0)  # 发送设置下降速度为0，停止升降运动
        print("  ✓ 腰部停止运动\n")
    except Exception as e:
        print(f"  ⚠ 腰部速度指令下降控制失败: {e}\n")

    # 发送腰部以0.8倍最大旋转速度正转（右转），指令持续5s
    print("-" * 70)
    print("[13] 发送腰部0.8倍最大旋转速度持续正转（右转）5s指令")
    print("-" * 70)
    try:
        interface.send_waist_turning_velocity_scale(0.8)  # 发送设置旋转速度为0.8倍最大速度正转
        print("  ✓ 命令已发送: 腰部开始持续正转（右转）")
        time.sleep(5.0)  # 等待腰部运动完成
        interface.send_waist_turning_velocity_scale(0.0)  # 发送设置旋转速度为0，停止旋转运动
        print("  ✓ 腰部停止运动\n")
    except Exception as e:
        print(f"  ⚠ 腰部速度指令正转（右转）控制失败: {e}\n")

    # 发送腰部以0.8倍最大旋转速度反转（左转），指令持续5s
    print("-" * 70)
    print("[14] 发送腰部0.8倍最大旋转速度持续反转（左转）5s指令")
    print("-" * 70)
    try:
        interface.send_waist_turning_velocity_scale(-0.8)  # 发送设置旋转速度为0.8倍最大速度反转
        print("  ✓ 命令已发送: 腰部开始持续反转（左转）")
        time.sleep(5.0)  # 等待腰部运动完成
        interface.send_waist_turning_velocity_scale(0.0)  # 发送设置旋转速度为0，停止旋转运动
        print("  ✓ 腰部停止运动\n")
    except Exception as e:
        print(f"  ⚠ 腰部速度指令反转（左转）控制失败: {e}\n")
    
    # ========================================================================
    # 第四部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[15] 测试完成，断开连接")
    print("=" * 70)
    
    # 切换回HOLD状态
    print("  → 切换回HOLD状态...")
    try:
        interface.send_fsm_command(2)  # HOLD
        time.sleep(1.0)
    except Exception as e:
        print(f"  ⚠ 切换状态失败: {e}")
    
    # 断开连接
    interface.disconnect()
    print("  ✓ 已断开连接\n")
    
    print("=" * 70)
    print("测试完成！")
    print("=" * 70 + "\n")
    
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

