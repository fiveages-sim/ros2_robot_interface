#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time
import sys
import math
import numpy as np

# 导入运动学服务
from arms_ros2_control_msgs.srv import KinematicsService
from geometry_msgs.msg import Pose, Point, Quaternion

# 如果您使用的是自定义的robot interface
try:
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
    HAS_ROBOT_INTERFACE = True
except ImportError:
    HAS_ROBOT_INTERFACE = False
    print("警告: 未找到 ros2_robot_interface，将只测试服务调用")


def create_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """创建Pose消息，支持欧拉角转四元数"""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    
    # 欧拉角转四元数
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    
    return pose


def pose_distance(pose1, pose2):
    """计算两个位姿之间的误差"""
    # 位置误差
    pos_error = math.sqrt(
        (pose1.position.x - pose2.position.x)**2 +
        (pose1.position.y - pose2.position.y)**2 +
        (pose1.position.z - pose2.position.z)**2
    )
    
    # 姿态误差（四元数点积）
    q1 = pose1.orientation
    q2 = pose2.orientation
    dot_product = abs(q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z)
    dot_product = min(1.0, max(-1.0, dot_product))
    rot_error = math.acos(2.0 * dot_product * dot_product - 1.0)
    
    return pos_error, rot_error


def verify_ik_with_fk(node, kinematics_client, arm_type, ik_solution, target_pose):
    """使用FK验证IK结果"""
    print(f"\n    → 验证IK结果（使用FK）:")
    print(f"      " + "-" * 60)
    
    # 创建FK请求验证IK结果
    fk_verify_request = KinematicsService.Request()
    fk_verify_request.operation_type = "fk"
    fk_verify_request.arm_type = arm_type
    fk_verify_request.joint_angles = ik_solution
    fk_verify_request.max_iterations = 100
    fk_verify_request.tolerance = 1e-8
    
    future = kinematics_client.call_async(fk_verify_request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() and future.result().success:
        response = future.result()
        if response.result_poses:
            actual_pose = response.result_poses[0]
            
            # 计算误差
            pos_error, rot_error = pose_distance(actual_pose, target_pose)
            
            print(f"      目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
            print(f"      实际位置: ({actual_pose.position.x:.4f}, {actual_pose.position.y:.4f}, {actual_pose.position.z:.4f})")
            print(f"      ─────────────────────────────────────────────────")
            print(f"      位置误差: {pos_error:.6f} m ({pos_error*1000:.3f} mm)")
            print(f"      姿态误差: {rot_error:.6f} rad ({math.degrees(rot_error):.3f}°)")
            
            # 判断是否成功
            if pos_error < 0.01 and rot_error < 0.1:
                print(f"      ✓ 验证成功！误差在可接受范围内")
                return True, pos_error, rot_error, actual_pose
            else:
                print(f"      ⚠ 验证警告：误差较大")
                return False, pos_error, rot_error, actual_pose
    else:
        print(f"      ✗ FK验证失败：无法计算实际位姿")
        return False, float('inf'), float('inf'), None


def test_ik_with_verification(node, kinematics_client, arm_type, initial_joints, 
                               target_pose, max_iterations=1000, tolerance=1e-6):
    """测试IK并进行FK验证"""
    
    print(f"\n    目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
    print(f"    目标姿态: w={target_pose.orientation.w:.4f}, x={target_pose.orientation.x:.4f}, "
          f"y={target_pose.orientation.y:.4f}, z={target_pose.orientation.z:.4f}")
    
    # 创建IK请求
    ik_request = KinematicsService.Request()
    ik_request.operation_type = "ik"
    ik_request.arm_type = arm_type
    ik_request.joint_angles = initial_joints
    ik_request.target_poses = [target_pose]
    ik_request.max_iterations = max_iterations
    ik_request.tolerance = tolerance
    
    start_time = time.time()
    future = kinematics_client.call_async(ik_request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    elapsed_time = time.time() - start_time
    
    if future.result():
        response = future.result()
        if response.success:
            print(f"    ✓ IK计算成功!")
            print(f"      消息: {response.message}")
            print(f"      服务端耗时: {response.computation_time_ms:.2f} ms")
            print(f"      客户端耗时: {elapsed_time*1000:.2f} ms")
            
            if response.result_joint_angles:
                joints = response.result_joint_angles
                joint_str = ", ".join([f"{j:.4f}" for j in joints])
                print(f"      IK解: [{joint_str}]")
                
                # 计算关节变化
                if len(initial_joints) == len(joints):
                    joint_diff = [abs(j - i) for j, i in zip(joints, initial_joints)]
                    max_diff = max(joint_diff)
                    mean_diff = sum(joint_diff) / len(joint_diff)
                    print(f"      关节变化: 最大={max_diff:.4f} rad, 平均={mean_diff:.4f} rad")
                
                # 使用FK验证IK结果
                success, pos_err, rot_err, _ = verify_ik_with_fk(
                    node, kinematics_client, arm_type, joints, target_pose
                )
                
                return True, joints, pos_err, rot_err
        else:
            print(f"    ✗ IK计算失败: {response.message}")
            return False, None, float('inf'), float('inf')
    else:
        print(f"    ✗ 服务调用超时")
        return False, None, float('inf'), float('inf')


def main():
    """测试运动学服务（FK和IK）"""
    
    # ========================================================================
    # 第一部分：初始化ROS 2
    # ========================================================================
    print("=" * 70)
    print("[1] 初始化ROS 2节点")
    print("=" * 70)
    
    if not rclpy.ok():
        rclpy.init()
    
    # 创建节点
    node = Node('kinematics_service_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    print("    ✓ 节点创建成功\n")
    
    # ========================================================================
    # 第二部分：连接机器人接口（如果需要获取真实关节状态）
    # ========================================================================
    interface = None
    if HAS_ROBOT_INTERFACE:
        print("[2] 初始化机器人接口")
        print("-" * 70)
        try:
            config = ROS2RobotInterfaceConfig()
            interface = ROS2RobotInterface(config)
            interface.connect()
            print("    ✓ 机器人接口连接成功\n")
            
            # 等待数据到达
            print("    → 等待数据到达（2秒）...")
            time.sleep(2.0)
            print("    ✓ 数据收集已开始\n")
            
            # 获取关节名称
            left_arm_joint_names = [
                "left_joint1", "left_joint2", "left_joint3", "left_joint4",
                "left_joint5", "left_joint6", "left_joint7"
            ]
            right_arm_joint_names = [
                "right_joint1", "right_joint2", "right_joint3", "right_joint4",
                "right_joint5", "right_joint6", "right_joint7"
            ]
            
            # 获取当前关节位置
            joint_state = interface.get_joint_state(categorized=False)
            if joint_state:
                all_joint_names = joint_state.get('names', [])
                all_joint_positions = joint_state.get('positions', [])
                joint_name_to_position = dict(zip(all_joint_names, all_joint_positions))
                
                left_current = [joint_name_to_position.get(name, 0.0) for name in left_arm_joint_names]
                right_current = [joint_name_to_position.get(name, 0.0) for name in right_arm_joint_names]
                
                print("    ✓ 当前关节位置:")
                print(f"      左臂: {[f'{p:.3f}' for p in left_current]}")
                print(f"      右臂: {[f'{p:.3f}' for p in right_current]}\n")
        except Exception as e:
            print(f"    ⚠ 机器人接口连接失败: {e}")
            print("    将只测试服务调用，不切换机器人状态\n")
            interface = None
    else:
        print("[2] 跳过机器人接口初始化（未安装）\n")
    
    # ========================================================================
    # 第三部分：创建运动学服务客户端
    # ========================================================================
    print("[3] 创建运动学服务客户端")
    print("-" * 70)
    
    # 创建服务客户端
    kinematics_client = node.create_client(KinematicsService, '/kinematics_service')
    
    # 等待服务
    if not kinematics_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('运动学服务不可用')
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if interface:
            interface.disconnect()
        return 1
    
    print("    ✓ 运动学服务连接成功\n")
    
    # 定义测试用的关节角度
    LEFT_TEST_JOINTS = [-0.3525227330, -0.7798290600, 0.8896949257, 
                        -1.8910405790, -2.7986485415, 0.4619120915, 0.8030297356]
    RIGHT_TEST_JOINTS = [0.0982891175, -0.8816540101, -0.5496532015, 
                         -1.8582004280, 2.8749939521, 0.3564035253, -1.0466912832]
    
    # ========================================================================
    # 第四部分：测试正运动学（FK）
    # ========================================================================
    print("=" * 70)
    print("[4] 测试正运动学 (FK)")
    print("=" * 70)
    
    left_fk_pose = None
    right_fk_pose = None
    
    # 4.1 测试左臂FK
    print("\n[4.1] 测试左臂FK")
    print("-" * 70)
    print(f"    关节角度: {[f'{j:.4f}' for j in LEFT_TEST_JOINTS]}")
    
    fk_request = KinematicsService.Request()
    fk_request.operation_type = "fk"
    fk_request.arm_type = "left"
    fk_request.joint_angles = LEFT_TEST_JOINTS
    fk_request.max_iterations = 100
    fk_request.tolerance = 1e-8
    
    future = kinematics_client.call_async(fk_request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result():
        response = future.result()
        if response.success:
            print(f"    ✓ FK计算成功!")
            print(f"      耗时: {response.computation_time_ms:.2f} ms")
            
            if response.result_poses:
                left_fk_pose = response.result_poses[0]
                print(f"      计算结果:")
                print(f"        位置: x={left_fk_pose.position.x:.4f}, y={left_fk_pose.position.y:.4f}, z={left_fk_pose.position.z:.4f}")
                print(f"        姿态: x={left_fk_pose.orientation.x:.4f}, y={left_fk_pose.orientation.y:.4f}, "
                      f"z={left_fk_pose.orientation.z:.4f}, w={left_fk_pose.orientation.w:.4f}")
        else:
            print(f"    ✗ FK计算失败: {response.message}")
    else:
        print(f"    ✗ 服务调用失败")
    
    # 4.2 测试右臂FK
    print("\n[4.2] 测试右臂FK")
    print("-" * 70)
    print(f"    关节角度: {[f'{j:.4f}' for j in RIGHT_TEST_JOINTS]}")
    
    fk_request = KinematicsService.Request()
    fk_request.operation_type = "fk"
    fk_request.arm_type = "right"
    fk_request.joint_angles = RIGHT_TEST_JOINTS
    fk_request.max_iterations = 100
    fk_request.tolerance = 1e-8
    
    future = kinematics_client.call_async(fk_request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result():
        response = future.result()
        if response.success:
            print(f"    ✓ FK计算成功!")
            print(f"      耗时: {response.computation_time_ms:.2f} ms")
            
            if response.result_poses:
                right_fk_pose = response.result_poses[0]
                print(f"      计算结果:")
                print(f"        位置: x={right_fk_pose.position.x:.4f}, y={right_fk_pose.position.y:.4f}, z={right_fk_pose.position.z:.4f}")
                print(f"        姿态: x={right_fk_pose.orientation.x:.4f}, y={right_fk_pose.orientation.y:.4f}, "
                      f"z={right_fk_pose.orientation.z:.4f}, w={right_fk_pose.orientation.w:.4f}")
        else:
            print(f"    ✗ FK计算失败: {response.message}")
    else:
        print(f"    ✗ 服务调用失败")
    
    # ========================================================================
    # 第五部分：测试逆运动学（IK）并进行FK验证
    # ========================================================================
    print("\n" + "=" * 70)
    print("[5] 测试逆运动学 (IK) 并进行FK验证")
    print("=" * 70)
    
    ik_results = []
    
    # 5.1 测试左臂IK（自洽性测试 - 目标就是FK计算出的位姿）
    if left_fk_pose:
        print("\n[5.1] 测试左臂IK - 自洽性测试（目标 = FK计算结果）")
        print("-" * 70)
        
        success, joints, pos_err, rot_err = test_ik_with_verification(
            node, kinematics_client, "left", LEFT_TEST_JOINTS, 
            left_fk_pose, max_iterations=1000, tolerance=1e-6
        )
        ik_results.append(("左臂自洽性", success, pos_err, rot_err))
    
    # 5.2 测试左臂IK（Z方向移动）
    if left_fk_pose:
        print("\n[5.2] 测试左臂IK - Z方向移动 -0.2m")
        print("-" * 70)
        
        import copy
        target_pose = Pose()
        target_pose.position.x = left_fk_pose.position.x
        target_pose.position.y = left_fk_pose.position.y
        target_pose.position.z = left_fk_pose.position.z - 0.2
        target_pose.orientation = left_fk_pose.orientation
        
        success, joints, pos_err, rot_err = test_ik_with_verification(
            node, kinematics_client, "left", LEFT_TEST_JOINTS,
            target_pose, max_iterations=1000, tolerance=1e-6
        )
        ik_results.append(("左臂Z移动", success, pos_err, rot_err))
    
    # 5.3 测试右臂IK（自洽性测试）
    if right_fk_pose:
        print("\n[5.3] 测试右臂IK - 自洽性测试（目标 = FK计算结果）")
        print("-" * 70)
        
        success, joints, pos_err, rot_err = test_ik_with_verification(
            node, kinematics_client, "right", RIGHT_TEST_JOINTS,
            right_fk_pose, max_iterations=1000, tolerance=1e-6
        )
        ik_results.append(("右臂自洽性", success, pos_err, rot_err))
    
    # 5.4 测试右臂IK（Z方向移动）
    if right_fk_pose:
        print("\n[5.4] 测试右臂IK - Z方向移动 -0.2m")
        print("-" * 70)
        
        target_pose = Pose()
        target_pose.position.x = right_fk_pose.position.x
        target_pose.position.y = right_fk_pose.position.y
        target_pose.position.z = right_fk_pose.position.z - 0.2
        target_pose.orientation = right_fk_pose.orientation
        
        success, joints, pos_err, rot_err = test_ik_with_verification(
            node, kinematics_client, "right", RIGHT_TEST_JOINTS,
            target_pose, max_iterations=1000, tolerance=1e-6
        )
        ik_results.append(("右臂Z移动", success, pos_err, rot_err))
    
    # ========================================================================
    # 第六部分：测试结果汇总
    # ========================================================================
    print("\n" + "=" * 70)
    print("[6] 测试结果汇总")
    print("=" * 70)
    
    print("\n  IK测试结果:")
    print("  " + "-" * 60)
    print(f"  {'测试名称':<20} {'成功':<8} {'位置误差(m)':<15} {'姿态误差(°)':<15}")
    print("  " + "-" * 60)
    
    for name, success, pos_err, rot_err in ik_results:
        status = "✓" if success else "✗"
        rot_err_deg = math.degrees(rot_err) if rot_err != float('inf') else float('inf')
        pos_str = f"{pos_err:.6f}" if pos_err != float('inf') else "N/A"
        rot_str = f"{rot_err_deg:.3f}" if rot_err_deg != float('inf') else "N/A"
        print(f"  {name:<20} {status:<8} {pos_str:<15} {rot_str:<15}")
    
    print("  " + "-" * 60)
    
    success_count = sum(1 for _, s, _, _ in ik_results if s)
    print(f"\n  总计: {success_count}/{len(ik_results)} 个测试通过")
    
    # ========================================================================
    # 第七部分：清理和退出
    # ========================================================================
    print("\n" + "=" * 70)
    print("[7] 清理和退出")
    print("=" * 70)
    
    # 关闭服务客户端
    kinematics_client.destroy()
    
    # 断开机器人接口
    if interface:
        try:
            interface.disconnect()
            print("  ✓ 机器人接口已断开")
        except Exception as e:
            print(f"  ⚠ 断开机器人接口时出错: {e}")
    
    # 关闭ROS节点
    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown()
    
    if rclpy.ok():
        rclpy.shutdown()
    
    print("\n" + "=" * 70)
    print("测试完成")
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