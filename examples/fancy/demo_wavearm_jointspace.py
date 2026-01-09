"""
W2 机器人 关节空间挥手测试脚本
测试新的 target_joint_trajectory 接口，使用多节点关节轨迹规划
"""

import time
import sys
import math

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    """测试W2机器人的关节空间多节点轨迹接口"""

    # ========================================================================
    # 配置参数
    # ========================================================================
    TRAJECTORY_EXECUTION_WAIT_TIME = 6.0  # 轨迹执行完成后的等待时间（秒）

    print("\n" + "=" * 70)
    print(" " * 15 + "W2 Robot Wave Arm Joint Space Demo")
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

    # 检查是否为双臂模式
    is_dual_arm = interface.config.right_end_effector_target_topic is not None
    if not is_dual_arm:
        print("    ✗ 错误: 此测试需要双臂模式，但未检测到右臂topic\n")
        interface.disconnect()
        return 1

    print("    ✓ 检测到双臂模式\n")

    # 等待数据到达
    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")

    # 硬编码 W2 机器人的关节名称（参考 fa w2）
    # W2 机器人每臂7个关节
    left_arm_joint_names = [
        "left_joint1", "left_joint2", "left_joint3", "left_joint4",
        "left_joint5", "left_joint6", "left_joint7"
    ]
    right_arm_joint_names = [
        "right_joint1", "right_joint2", "right_joint3", "right_joint4",
        "right_joint5", "right_joint6", "right_joint7"
    ]
    
    print(f"    ✓ 左臂关节: {len(left_arm_joint_names)} 个")
    print(f"      关节名称: {left_arm_joint_names}")
    print(f"    ✓ 右臂关节: {len(right_arm_joint_names)} 个")
    print(f"      关节名称: {right_arm_joint_names}\n")
    
    # 获取当前关节位置
    print("[5] 获取当前关节位置...")
    joint_state = interface.get_joint_state(categorized=False)
    if joint_state is None:
        print("    ✗ 无法获取关节状态\n")
        interface.disconnect()
        return 1
    
    # 从关节状态中提取当前位置
    all_joint_names = joint_state.get('names', [])
    all_joint_positions = joint_state.get('positions', [])
    
    # 创建名称到位置的映射
    joint_name_to_position = dict(zip(all_joint_names, all_joint_positions))
    
    # 提取左臂和右臂的当前位置
    left_arm_current_pos = [joint_name_to_position.get(name, 0.0) for name in left_arm_joint_names]
    right_arm_current_pos = [joint_name_to_position.get(name, 0.0) for name in right_arm_joint_names]
    
    print(f"    ✓ 已获取当前关节位置")
    print(f"      左臂位置: {[f'{p:.3f}' for p in left_arm_current_pos]}")
    print(f"      右臂位置: {[f'{p:.3f}' for p in right_arm_current_pos]}\n")

    # ========================================================================
    # 第二部分：切换到MOVEJ状态
    # ========================================================================
    print("-" * 70)
    print("[6] 切换到MOVEJ状态")
    print("-" * 70)
    try:
        print("  → 切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")
        
        # 先切换到HOME状态
        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)
        print("  ✓ 已切换到HOME状态")

        # 再切换到Hold状态
        print("  → 再切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")

        # 切换到MOVEJ状态
        # 注意：MOVEJ状态通常不需要特殊命令，在HOLD状态下发送轨迹会自动激活
        # 如果需要显式切换到MOVEJ，可能需要特定的FSM命令（根据实际实现调整）
        print("  → 准备MOVEJ状态...")
        interface.send_fsm_command(4)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已准备MOVEJ状态（将在发送轨迹时自动激活）\n")
    except Exception as e:
        print(f"  ✗ 切换到MOVEJ状态失败: {e}\n")
        interface.disconnect()
        return 1

    # ========================================================================
    # 第三部分：准备关节空间轨迹数据
    # ========================================================================
    print("-" * 70)
    print("[7] 准备关节空间轨迹数据")
    print("-" * 70)

    # 定义挥手轨迹的关节角度（绝对位置）
    # 每个waypoint是7个关节的绝对角度值（单位：弧度）
    
    # 左臂挥手轨迹的绝对位置
    # 关节顺序: [left_joint1, left_joint2, left_joint3, left_joint4, left_joint5, left_joint6, left_joint7]
    left_arm_wave_waypoints = [
        [2.0, -1.05, -2.35, -2.46, 2., 0.237, 0.0],  # 第一个目标点：稍微抬起
        [1.7, -0.99, -2.29, -1.98, 2., 0.38, 0.18],  # 第二个目标点：向左挥
        [1.2, -0.807, -1.45, -2.2, 2., 0.166, 0.0],  # 第三个目标点：向右挥
        [1.7, -0.99, -2.29, -1.98, 2., 0.38, 0.18],  # 第四个目标点：回到中间位置
    ]
    
    # 右臂挥手轨迹的绝对位置
    # 关节顺序: [right_joint1, right_joint2, right_joint3, right_joint4, right_joint5, right_joint6, right_joint7]
    right_arm_wave_waypoints = [
        [-2.0, -1.05, 2.35, -2.46, 1., -0.237, 0.0],  # 第一个目标点：稍微抬起
        [-1.7, -0.99, 2.29, -1.98, 1., -0.38, 0.18],  # 第二个目标点：向右挥
        [-1.2, -0.807, 1.45, -2.2, 1., -0.166, 0.0],  # 第三个目标点：向左挥
        [-1.7, -0.99, 2.29, -1.98, 1., -0.38, 0.18],  # 第四个目标点：回到中间位置
    ]
    
    # 右臂回到指定位置（在左臂挥手时使用）
    # 关节顺序: [right_joint1, right_joint2, right_joint3, right_joint4, right_joint5, right_joint6, right_joint7]
    right_arm_home_position = [-1.5435, -1.1615, 1.4302, -0.9121, 1.9784, 0.1420, 0.6302]
    
    # 创建左臂挥手+右臂回到指定位置的组合轨迹
    # 关节顺序: 先左臂7个关节，再右臂7个关节，共14个关节
    left_arm_wave_with_right_arm_home_waypoints = []
    for left_waypoint in left_arm_wave_waypoints:
        # 组合：左臂挥手轨迹 + 右臂固定位置
        combined_waypoint = left_waypoint + right_arm_home_position
        left_arm_wave_with_right_arm_home_waypoints.append(combined_waypoint)
    
    # 组合关节名称：左臂 + 右臂
    left_arm_wave_with_right_arm_home_joint_names = left_arm_joint_names + right_arm_joint_names

    print("  ✓ 轨迹数据已准备（使用绝对位置）")
    print(f"    左臂轨迹: {len(left_arm_wave_waypoints)} 个目标点")
    print(f"    右臂轨迹: {len(right_arm_wave_waypoints)} 个目标点")
    print(f"    左臂挥手+右臂回位轨迹: {len(left_arm_wave_with_right_arm_home_waypoints)} 个目标点")
    print(f"    右臂回位位置: {right_arm_home_position}")
    print(f"    注意: 当前位置将自动作为第一个点，然后执行上述绝对位置轨迹\n")

    # ========================================================================
    # 第四部分：测试关节空间轨迹（重复执行两次）
    # ========================================================================
    print("=" * 70)
    print("[8] 测试关节空间挥手轨迹（将重复执行2次）")
    print("=" * 70)

    NUM_REPETITIONS = 2
    for repetition in range(1, NUM_REPETITIONS + 1):
        print("-" * 70)
        if repetition == 1:
            print(f"[8.{repetition}] 第 {repetition} 次执行轨迹：右臂挥手")
        else:
            print(f"[8.{repetition}] 第 {repetition} 次执行轨迹：左臂挥手")
        print("-" * 70)

        try:
            if repetition == 1:
                # 第一次：右臂挥手
                print(f"  → 发送右臂关节轨迹（{len(right_arm_wave_waypoints)}个目标点）...")
                interface.send_joint_trajectory(
                    joint_names=right_arm_joint_names,
                    waypoints=right_arm_wave_waypoints
                )
                print("  ✓ 右臂轨迹已发送")
            else:
                # 第二次：左臂挥手，同时右臂回到指定位置
                print(f"  → 发送左臂挥手+右臂回位轨迹（{len(left_arm_wave_with_right_arm_home_waypoints)}个目标点）...")
                print(f"     左臂: 挥手动作")
                print(f"     右臂: 回到位置 {right_arm_home_position}")
                interface.send_joint_trajectory(
                    joint_names=left_arm_wave_with_right_arm_home_joint_names,
                    waypoints=left_arm_wave_with_right_arm_home_waypoints
                )
                print("  ✓ 左臂挥手+右臂回位轨迹已发送")

            print(f"  → 等待轨迹执行完成（约{TRAJECTORY_EXECUTION_WAIT_TIME}秒）...")
            time.sleep(TRAJECTORY_EXECUTION_WAIT_TIME)

        except Exception as e:
            print(f"  ✗ 第 {repetition} 次执行失败: {e}\n")
            import traceback
            traceback.print_exc()
            break

    print("=" * 70)
    print(f"  ✓ 所有 {NUM_REPETITIONS} 次轨迹执行完成")
    print("=" * 70 + "\n")

    # ========================================================================
    # 第五部分：回到HOME位置并切换到HOLD状态
    # ========================================================================
    print("-" * 70)
    print("[9] 回到HOME位置并切换到HOLD状态")
    print("-" * 70)
    try:
        # 先切换到HOLD状态
        print("  → 先切换到HOLD状态...")
        interface.send_fsm_command(2)  # 2 = HOLD状态
        time.sleep(0.1)
        print("  ✓ 已切换到HOLD状态")

        # 然后切换到HOME状态，让机器人回到HOME位置
        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)  # 等待回到HOME位置
        print("  ✓ 已切换到HOME状态，机器人已回到HOME位置")

        # 最后再切换到HOLD状态
        print("  → 再切换到HOLD状态...")
        interface.send_fsm_command(2)  # 2 = HOLD状态
        time.sleep(0.1)
        print("  ✓ 已切换到HOLD状态\n")
    except Exception as e:
        print(f"  ✗ 回到HOME/HOLD状态失败: {e}\n")

    # ========================================================================
    # 第六部分：完成和清理
    # ========================================================================
    print("=" * 70)
    print("[10] 测试完成")
    print("=" * 70)
    print("  ✓ 所有轨迹测试已完成")
    print("  → 等待3秒后断开连接...\n")
    time.sleep(3.0)

    try:
        interface.disconnect()
        print("  ✓ 已断开连接\n")
    except Exception as e:
        print(f"  ⚠ 断开连接时出错: {e}\n")

    print("=" * 70)
    print("测试结束")
    print("=" * 70 + "\n")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n未预期的错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

