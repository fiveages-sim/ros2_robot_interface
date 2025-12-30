"""
测试 send_left_arm_joint_positions() 和 send_right_arm_joint_positions() 功能
每2秒将双臂最后一个关节弧度增加0.1
"""

import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    """测试手臂关节位置控制（MoveJ模式）"""
    
    print("\n" + "=" * 70)
    print(" " * 15 + "Arm Joint MoveJ Test")
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
        print("⚠ 警告: 当前不是双臂模式，将只控制左臂\n")
    
    # 检查手臂关节控制器话题是否配置
    if not interface.config.left_arm_joint_controller_topic:
        print("⚠ 错误: 未检测到手臂关节控制器话题！")
        print("   请确保机器人正在运行并发布 MoveJ 模式的话题。\n")
        interface.disconnect()
        return 1
    
    print("✓ 检测到手臂关节控制器话题:")
    print(f"  左臂: {interface.config.left_arm_joint_controller_topic}")
    if interface.config.right_arm_joint_controller_topic:
        print(f"  右臂: {interface.config.right_arm_joint_controller_topic}")
    print()
    
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
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    
    if is_dual_arm:
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
        
        print(f"  左臂关节数: {len(left_arm_joints)}")
        print(f"  左臂关节位置: {[f'{p:.3f}' for p in left_arm_joints]}")
        print(f"  左臂最后一个关节: {left_arm_joints[-1]:.3f} 弧度")
        print()
        print(f"  右臂关节数: {len(right_arm_joints)}")
        print(f"  右臂关节位置: {[f'{p:.3f}' for p in right_arm_joints]}")
        print(f"  右臂最后一个关节: {right_arm_joints[-1]:.3f} 弧度")
        print()
        
        # 保存初始关节位置（用于后续更新）
        left_arm_current_positions = list(left_arm_joints)
        right_arm_current_positions = list(right_arm_joints)
    else:
        # 单臂模式
        arm_joints = categorized_state.get('arm', {}).get('positions', [])
        
        if not arm_joints:
            print("⚠ 错误: 无法获取手臂关节位置！\n")
            interface.disconnect()
            return 1
        
        print(f"  手臂关节数: {len(arm_joints)}")
        print(f"  手臂关节位置: {[f'{p:.3f}' for p in arm_joints]}")
        print(f"  最后一个关节: {arm_joints[-1]:.3f} 弧度")
        print()
        
        # 保存初始关节位置（用于后续更新）
        left_arm_current_positions = list(arm_joints)
        right_arm_current_positions = None
    
    # ========================================================================
    # 第三部分：每2秒增加最后一个关节0.1弧度
    # ========================================================================
    print("=" * 70)
    print("[6] 每2秒将双臂最后一个关节弧度增加0.1")
    print("=" * 70)
    
    print(f"\n  初始最后一个关节位置:")
    print(f"    左臂: {left_arm_current_positions[-1]:.3f} 弧度")
    if right_arm_current_positions:
        print(f"    右臂: {right_arm_current_positions[-1]:.3f} 弧度")
    print(f"  每次增加: 0.1 弧度")
    print(f"  间隔时间: 2秒")
    print(f"  按 Ctrl+C 停止测试\n")
    
    step_count = 0
    interval = 2.0  # 2秒间隔
    increment = 0.1  # 每次增加0.1弧度
    
    try:
        while True:
            step_count += 1
            
            # 更新最后一个关节位置
            left_arm_current_positions[-1] += increment
            if right_arm_current_positions:
                right_arm_current_positions[-1] += increment
            
            print(f"[6.{step_count}] 增加 {step_count * increment:.1f} 弧度")
            print("-" * 70)
            print(f"  左臂最后一个关节: {left_arm_current_positions[-1]:.3f} 弧度")
            if right_arm_current_positions:
                print(f"  右臂最后一个关节: {right_arm_current_positions[-1]:.3f} 弧度")
            
            # 发送左臂关节位置（会自动切换到MOVEJ状态）
            try:
                interface.send_left_arm_joint_positions(left_arm_current_positions)
                print(f"  ✓ 左臂关节位置已发送")
            except Exception as e:
                print(f"  ✗ 左臂发送失败: {e}")
                break
            
            # 发送右臂关节位置（双臂模式，会自动切换到MOVEJ状态）
            if right_arm_current_positions:
                try:
                    interface.send_right_arm_joint_positions(right_arm_current_positions)
                    print(f"  ✓ 右臂关节位置已发送")
                except Exception as e:
                    print(f"  ✗ 右臂发送失败: {e}")
                    # 继续发送左臂，不中断
            
            # 等待指定间隔时间
            print(f"  → 等待 {interval} 秒后发送下一个目标...")
            time.sleep(interval)
            
    except KeyboardInterrupt:
        print(f"\n\n  ⚠ 用户中断测试（已执行 {step_count} 次增加）")
    
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

