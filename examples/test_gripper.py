"""
夹爪功能测试脚本
每3秒自动切换左右夹爪的开关状态
支持单臂和双臂机器人
"""

import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig, ControlType


def main():
    """主测试函数"""
    print("\n" + "=" * 70)
    print(" " * 20 + "夹爪功能测试")
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
    # 第二部分：检查可用夹爪
    # ========================================================================
    left_gripper_available = interface.left_gripper_handler is not None
    right_gripper_available = is_dual_arm and interface.right_gripper_handler is not None
    
    if not left_gripper_available and not right_gripper_available:
        print("⚠ 没有可用的夹爪，退出测试")
        interface.disconnect()
        return 1
    
    print("-" * 70)
    print("开始夹爪开关控制测试")
    print("-" * 70)
    if left_gripper_available:
        print("  左夹爪: 已启用")
    if right_gripper_available:
        print("  右夹爪: 已启用")
    print("  切换间隔: 3秒")
    print("  将根据当前实际状态切换到相反状态")
    print("  按 Ctrl+C 退出测试\n")
    
    # 等待一下，让订阅器接收初始状态
    print("等待状态同步（1秒）...")
    time.sleep(1.0)
    print("✓ 开始测试\n")
    
    # ========================================================================
    # 第三部分：主循环 - 每3秒切换夹爪状态
    # ========================================================================
    i = 0
    try:
        while True:
            time.sleep(3.0)  # 每3秒循环一次
            i += 1
            
            # 左夹爪：读取当前状态并切换到相反状态
            if left_gripper_available:
                try:
                    # 读取当前实际状态（通过订阅器回调更新的状态）
                    current_state = interface.left_gripper_handler.is_open
                    # 切换到相反状态
                    target_value = 0 if current_state else 1
                    target_status = "打开" if target_value == 1 else "关闭"
                    current_status = "打开" if current_state else "关闭"
            
                    interface.left_gripper_handler.send_target_command(target_value)
                    print(f"[{i:3d}s] [左夹爪-开关] {current_status} → {target_status} (值: {current_state} → {target_value})")
            
                    # 获取当前位置并检查到达
                    joint_state = interface.get_joint_state(categorized=True)
                    if joint_state:
                        gripper_category = 'left_gripper' if is_dual_arm else 'gripper'
                        gripper_data = joint_state.get(gripper_category, {})
                        current_pos = gripper_data.get('positions', [None])[0] if gripper_data.get('positions') else None
            
                        if current_pos is not None:
                            # 直接使用 gripper_handler.check_arrival 方法
                            result = interface.left_gripper_handler.check_arrival(current_pos)
                            arrived = result.get('arrived', False)
            
                            # check_arrival 方法内部已经打印了详细信息，这里只打印简要结果
                            if arrived:
                                print(f"[{i:3d}s] [左夹爪-到达] ✓ 已到达目标位置")
                            else:
                                print(f"[{i:3d}s] [左夹爪-到达] ⚠ 未到达目标位置")
                        else:
                            print(f"[{i:3d}s] [左夹爪-到达] ⚠ 无法获取当前位置")
                except Exception as e:
                    print(f"[{i:3d}s] ✗ 控制左夹爪失败: {e}")
            
            # 右夹爪：读取当前状态并切换到相反状态（仅双臂模式）
            if right_gripper_available:
                try:
                    # 读取当前实际状态（通过订阅器回调更新的状态）
                    current_state = interface.right_gripper_handler.is_open
                    # 切换到相反状态
                    target_value = 0 if current_state else 1
                    target_status = "打开" if target_value == 1 else "关闭"
                    current_status = "打开" if current_state else "关闭"
                    
                    interface.right_gripper_handler.send_target_command(target_value)
                    print(f"[{i:3d}s] [右夹爪-开关] {current_status} → {target_status} (值: {current_state} → {target_value})")
                    
                    # 获取当前位置并检查到达
                    joint_state = interface.get_joint_state(categorized=True)
                    if joint_state:
                        right_gripper_data = joint_state.get('right_gripper', {})
                        current_pos = right_gripper_data.get('positions', [None])[0] if right_gripper_data.get('positions') else None
                        
                        if current_pos is not None:
                            # 直接使用 gripper_handler.check_arrival 方法
                            result = interface.right_gripper_handler.check_arrival(current_pos)
                            arrived = result.get('arrived', False)
                            
                            # check_arrival 方法内部已经打印了详细信息，这里只打印简要结果
                            if arrived:
                                print(f"[{i:3d}s] [右夹爪-到达] ✓ 已到达目标位置")
                            else:
                                print(f"[{i:3d}s] [右夹爪-到达] ⚠ 未到达目标位置")
                        else:
                            print(f"[{i:3d}s] [右夹爪-到达] ⚠ 无法获取当前位置")
                except Exception as e:
                    print(f"[{i:3d}s] ✗ 控制右夹爪失败: {e}")
            
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

