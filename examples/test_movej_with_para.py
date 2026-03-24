#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arms_ros2_control_msgs.srv import JointTrajectory
from arms_ros2_control_msgs.msg import JointWaypoint
from rclpy.executors import SingleThreadedExecutor
import time
import sys
import math

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

def main():
    """测试机器人设置参数的movej单点和多点的service"""

    # ========================================================================
    # 第一部分：初始化和连接（保持不变）
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

    # 硬编码 W2 机器人的关节名称
    left_arm_joint_names = [
        "left_joint1", "left_joint2", "left_joint3", "left_joint4",
        "left_joint5", "left_joint6", "left_joint7"
    ]
    right_arm_joint_names = [
        "right_joint1", "right_joint2", "right_joint3", "right_joint4",
        "right_joint5", "right_joint6", "right_joint7"
    ]

    dual_arm_joint_names = left_arm_joint_names + right_arm_joint_names
    
    print(f"    ✓ 左臂关节: {len(left_arm_joint_names)} 个")
    print(f"    ✓ 右臂关节: {len(right_arm_joint_names)} 个")
    print(f"    ✓ 双臂关节总数: {len(dual_arm_joint_names)} 个\n")
    
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
    # 第二部分：切换到HOME状态作为起点
    # ========================================================================
    print("-" * 70)
    print("[6] 切换到HOME状态作为起点")
    print("-" * 70)
    try:
        print("  → 切换到HOLD状态...")
        interface.send_fsm_command(2)  # 2 = HOLD状态
        time.sleep(0.5)
        print("  ✓ 已切换到HOLD状态\n")

        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)  # 等待回家完成
        print("  ✓ 已回到HOME位置\n")
        
        print("  → 切换到HOLD状态...")
        interface.send_fsm_command(2)  # 2 = HOLD状态
        time.sleep(0.5)
        print("  ✓ 已切换到HOLD状态\n")

        print("  → 切换到movej状态...")
        interface.send_fsm_command(4)  # 4 = movej状态
        time.sleep(0.5)
        print("  ✓ 已切换到movej状态\n")
    except Exception as e:
        print(f"  ✗ 切换状态失败: {e}\n")
        interface.disconnect()
        return 1

    # ========================================================================
    # 第三部分：测试单点轨迹
    # ========================================================================
    print("-" * 70)
    print("[7] 测试关节单点轨迹服务")
    print("-" * 70)

    # 初始化ROS 2（如果还没初始化）
    if not rclpy.ok():
        rclpy.init()
    
    # 创建节点和客户端
    node = Node('joint_trajectory_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    client = node.create_client(JointTrajectory, '/ocs2_arm_controller/joint_trajectory_with_para')
    
    # 等待服务
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('服务不可用')
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        interface.disconnect()
        return 1
    
    # 创建单点请求
    req = JointTrajectory.Request()
    req.joint_names = dual_arm_joint_names
    
    wp = JointWaypoint()
    wp.position = [1.5, -0.3, 0.0, -0.5, -0.6, -0.2, 0.5,
                   -1.6, -1.0, 0.4, -0.6, 1.2, 0.0, -0.3]
    wp.time_mode = False
    wp.max_velocity = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5,
                       1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5]
    wp.max_acceleration = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0,
                           2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
    wp.max_jerk = [5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0,
                   5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0]
    req.waypoints = [wp]
    
    # 发送请求
    node.get_logger().info('发送运动指令...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result():
        res = future.result()
        if res.success:
            planned_duration = res.planned_duration
            node.get_logger().info(f'✓ 成功: {res.message}')
            node.get_logger().info(f'  规划时长: {planned_duration:.3f} 秒')
            
            node.get_logger().info(f'  等待 {planned_duration:.3f} 秒让运动完成...')
            time.sleep(planned_duration)
            node.get_logger().info('  ✓ 运动完成')
        else:
            node.get_logger().error(f'✗ 失败: {res.message}')
    else:
        node.get_logger().error('调用失败')
    
    # 清理单点测试的节点
    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown()
    
    # 等待稳定
    print("\n  → 等待机器人稳定...")
    time.sleep(1.0)
    
    # ========================================================================
    # 第四部分：回到HOLD
    # ========================================================================
    print("-" * 70)
    print("[8] 回到HOLD位置")
    print("-" * 70)
    try:
        print("  → 切换到HOLD状态...")
        interface.send_fsm_command(2)
        time.sleep(0.5)
        print("  ✓ 已切换到HOLD状态\n")
    except Exception as e:
        print(f"  ✗ 回到HOLD位置失败: {e}\n")
    
    # ========================================================================
    # 第五部分：多点轨迹测试（重新创建节点）
    # ========================================================================
    print("-" * 70)
    print("[9] 测试关节多点轨迹服务")
    print("-" * 70)
    try:
        print("  → 切换到movej状态...")
        interface.send_fsm_command(4)
        time.sleep(0.5)
        print("  ✓ 已切换到movej状态\n")
    except Exception as e:
        print(f"  ✗ 回到movej状态失败: {e}\n")
    
    # 重新创建节点和客户端
    node_multi = Node('joint_trajectory_test_client_multi')  # 使用不同的节点名
    executor_multi = SingleThreadedExecutor()
    executor_multi.add_node(node_multi)
    
    client_multi = node_multi.create_client(JointTrajectory, '/ocs2_arm_controller/joint_trajectory_with_para')
    
    # 等待服务
    if not client_multi.wait_for_service(timeout_sec=5.0):
        node_multi.get_logger().error('服务不可用')
        executor_multi.remove_node(node_multi)
        node_multi.destroy_node()
        executor_multi.shutdown()
        interface.disconnect()
        return 1
    
    # 创建多点轨迹请求
    req_multi = JointTrajectory.Request()
    req_multi.joint_names = dual_arm_joint_names
    
    # 定义多个目标位置
    wp1 = JointWaypoint()
    wp1.position = [0.5, 0.0, 0.7, 0.3, -0.2, 0.4, -0.1,
                    -1.0, -1.5, 0.9, -0.3, 0.7, -0.2, 0.4]
    wp1.time_mode = False
    # wp1.total_time=6.0
    wp1.max_velocity = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5,
                        1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5]
    wp1.max_acceleration = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0,
                            2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
    wp1.max_jerk = [5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0,
                    5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0]
    wp1.blend_ratio_percent = 0.2
    
    wp2 = JointWaypoint()
    wp2.position = [0.0, -0.3, 0.4, 0.3, 0.2, -0.1, 0.3,
                    -0.4, -0.8, 0.2, 0.0, 0.7, -0.5, 0.3]
    wp2.time_mode = True
    wp2.total_time = 5.0
    wp2.blend_ratio_percent = 0.3
    
    wp3 = JointWaypoint()
    wp3.position = [-0.5, 0.1, 0.0, 0.3, 0.6, 0.2, 0.3,
                    -0.9, -0.2, 0.3, 0.2, 0.7, -0.6, 0.1]
    wp3.time_mode = True
    wp3.total_time = 6.0
    wp3.max_velocity = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5,
                        1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5]
    wp3.max_acceleration = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0,
                            2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
    wp3.max_jerk = [5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0,
                    5.0, 5.0, 5.0, 8.0, 8.0, 8.0, 8.0]
    wp3.blend_ratio_percent = 0.0
    
    req_multi.waypoints = [wp1, wp2, wp3]
    
    # 发送请求
    node_multi.get_logger().info('发送运动指令...')
    future_multi = client_multi.call_async(req_multi)
    rclpy.spin_until_future_complete(node_multi, future_multi)
    
    if future_multi.result():
        res = future_multi.result()
        if res.success:
            planned_duration = res.planned_duration
            node_multi.get_logger().info(f'✓ 成功: {res.message}')
            node_multi.get_logger().info(f'  规划时长: {planned_duration:.3f} 秒')
            
            node_multi.get_logger().info(f'  等待 {planned_duration:.3f} 秒让运动完成...')
            time.sleep(planned_duration)
            node_multi.get_logger().info('  ✓ 运动完成')
        else:
            node_multi.get_logger().error(f'✗ 失败: {res.message}')
    else:
        node_multi.get_logger().error('调用失败')
    
    # 清理多点测试的节点
    executor_multi.remove_node(node_multi)
    node_multi.destroy_node()
    executor_multi.shutdown()
    
    # 等待稳定
    print("\n  → 等待机器人稳定...")
    time.sleep(1.0)
    
    # ========================================================================
    # 第六部分：回到HOLD
    # ========================================================================
    print("-" * 70)
    print("[10] 回到HOLD位置")
    print("-" * 70)
    try:
        print("  → 切换到HOLD状态...")
        interface.send_fsm_command(2)
        time.sleep(0.5)
        print("  ✓ 已切换到HOLD状态\n")
    except Exception as e:
        print(f"  ✗ 回到HOLD位置失败: {e}\n")
    
    # ========================================================================
    # 第七部分：完成和清理
    # ========================================================================
    print("=" * 70)
    print("[11] 测试完成")
    print("=" * 70)
    print("  ✓ 所有轨迹测试已完成")
    print("  → 等待2秒后断开连接...\n")
    time.sleep(2.0)

    try:
        interface.disconnect()
        print("  ✓ 已断开连接\n")
    except Exception as e:
        print(f"  ⚠ 断开连接时出错: {e}\n")
    
    print("=" * 70)
    print("测试结束")
    print("=" * 70 + "\n")

    return 0

if __name__ == '__main__':
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