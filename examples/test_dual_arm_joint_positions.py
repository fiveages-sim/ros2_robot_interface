"""
测试 send_dual_arm_joint_positions() 功能
同时控制双臂的所有关节位置
"""

import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    """测试双臂关节位置控制（统一 topic）"""
    
    print("\n" + "=" * 70)
    print(" " * 15 + "Dual-Arm Joint Positions Test")
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
    
    # 检查是否是双臂模式
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    if not is_dual_arm:
        print("⚠ 错误: 当前不是双臂模式！")
        print("   send_dual_arm_joint_positions() 需要双臂模式。\n")
        interface.disconnect()
        return 1
    
    # 检查统一 topic 是否配置
    if not interface.config.unified_arm_joint_controller_topic:
        print("⚠ 错误: 未检测到统一双臂关节控制器话题！")
        print("   请确保机器人正在运行并发布以下话题之一：")
        print("   - /ocs2_wbc_controller/target_joint_position")
        print("   - /ocs2_arm_controller/target_joint_position")
        print()
        interface.disconnect()
        return 1
    
    print("✓ 检测到统一双臂关节控制器话题:")
    print(f"  {interface.config.unified_arm_joint_controller_topic}\n")
    
    # ========================================================================
    # 第二部分：获取当前关节状态
    # ========================================================================
    print("-" * 70)
    print("[5] 获取当前关节状态")
    print("-" * 70)
    
    categorized_state = interface.get_joint_state(categorized=True)
    if not categorized_state:
        print("⚠ 错误: 无法获取关节状态！")
        print("   请确保机器人正在运行并发布关节状态数据。\n")
        interface.disconnect()
        return 1
    
    # 提取左臂和右臂关节位置
    left_arm_joints = categorized_state.get('left_arm', {}).get('positions', [])
    right_arm_joints = categorized_state.get('right_arm', {}).get('positions', [])
    
    if not left_arm_joints:
        print("⚠ 错误: 无法获取左臂关节位置！\n")
        interface.disconnect()
        return 1
    
    if not right_arm_joints:
        print("⚠ 错误: 无法获取右臂关节位置！\n")
        interface.disconnect()
        return 1
    
    if len(left_arm_joints) != len(right_arm_joints):
        print("⚠ 警告: 左臂和右臂关节数量不一致！")
        print(f"   左臂: {len(left_arm_joints)} 个关节")
        print(f"   右臂: {len(right_arm_joints)} 个关节\n")
    
    print(f"  左臂关节数: {len(left_arm_joints)}")
    print(f"  左臂关节位置: {[f'{p:.3f}' for p in left_arm_joints]}")
    print()
    print(f"  右臂关节数: {len(right_arm_joints)}")
    print(f"  右臂关节位置: {[f'{p:.3f}' for p in right_arm_joints]}")
    print()
    
    # 保存初始关节位置（用于后续测试）
    left_arm_initial_positions = list(left_arm_joints)
    right_arm_initial_positions = list(right_arm_joints)
    
    # ========================================================================
    # 第三部分：测试双臂关节控制
    # ========================================================================
    print("=" * 70)
    print("[6] 测试双臂关节位置控制")
    print("=" * 70)
    
    print("\n  测试说明:")
    print("  - 将每2秒将双臂最后一个关节弧度增加0.1")
    print("  - 使用 send_dual_arm_joint_positions() 同时控制双臂")
    print("  - 按 Ctrl+C 停止测试\n")
    
    step_count = 0
    interval = 2.0  # 2秒间隔
    increment = 0.1  # 每次增加0.1弧度
    
    # 创建当前位置的副本用于修改
    left_arm_current_positions = list(left_arm_initial_positions)
    right_arm_current_positions = list(right_arm_initial_positions)
    
    try:
        while True:
            step_count += 1
            
            # 更新最后一个关节位置
            left_arm_current_positions[-1] += increment
            right_arm_current_positions[-1] += increment
            
            print(f"[6.{step_count}] 发送双臂关节位置（增加 {step_count * increment:.1f} 弧度）")
            print("-" * 70)
            print(f"  左臂最后一个关节: {left_arm_current_positions[-1]:.3f} 弧度")
            print(f"  右臂最后一个关节: {right_arm_current_positions[-1]:.3f} 弧度")
            print(f"  左臂所有关节: {[f'{p:.3f}' for p in left_arm_current_positions]}")
            print(f"  右臂所有关节: {[f'{p:.3f}' for p in right_arm_current_positions]}")
            
            # 发送双臂关节位置（统一控制）
            try:
                interface.send_dual_arm_joint_positions(
                    left_arm_current_positions,
                    right_arm_current_positions
                )
                print(f"  ✓ 双臂关节位置已发送（统一 topic）")
            except Exception as e:
                print(f"  ✗ 发送失败: {e}")
                import traceback
                traceback.print_exc()
                break
            
            # 等待指定间隔时间
            print(f"  → 等待 {interval} 秒后发送下一个目标...")
            time.sleep(interval)
            
    except KeyboardInterrupt:
        print(f"\n\n  ⚠ 用户中断测试（已执行 {step_count} 次）")
    
    # ========================================================================
    # 第四部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[7] 测试完成，断开连接")
    print("=" * 70)
    
    # 切换回HOLD状态
    print("  → 切换回HOLD状态...")
    try:
        interface.send_fsm_command(2)  # HOLD
        time.sleep(1.0)
        print("  ✓ 已切换到HOLD状态")
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

