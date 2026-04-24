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


def quaternion_to_euler(q):
    """将四元数转换为欧拉角（弧度）"""
    # 计算欧拉角
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def test_zero_pose_comparison(node, kinematics_client, arm_type, solver_types):
    """测试零位时的正运动学"""
    
    zero_joints = [0.0] * 7
    print(f"\n    测试零位关节角: {zero_joints}")
    
    results = []
    
    for solver_type in solver_types:
        print(f"\n    → 使用求解器: {solver_type}")
        
        fk_request = KinematicsService.Request()
        fk_request.operation_type = "fk"
        fk_request.arm_type = arm_type
        fk_request.solver_type = solver_type
        fk_request.joint_angles = zero_joints
        fk_request.max_iterations = 100
        fk_request.tolerance = 1e-8
        
        start_time = time.time()
        future = kinematics_client.call_async(fk_request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        elapsed_time = (time.time() - start_time) * 1000
        
        if future.result() and future.result().success:
            response = future.result()
            if response.result_poses:
                pose = response.result_poses[0]
                
                # 转换为欧拉角
                roll, pitch, yaw = quaternion_to_euler(pose.orientation)
                
                results.append({
                    'solver': solver_type,
                    'success': True,
                    'pose': pose,
                    'position': (pose.position.x, pose.position.y, pose.position.z),
                    'quaternion': (pose.orientation.x, pose.orientation.y, 
                                   pose.orientation.z, pose.orientation.w),
                    'euler': (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
                    'computation_time': response.computation_time_ms,
                    'total_time': elapsed_time
                })
                
                print(f"      位置: x={pose.position.x:.6f}m ({pose.position.x*1000:.1f}mm), "
                      f"y={pose.position.y:.6f}m ({pose.position.y*1000:.1f}mm), "
                      f"z={pose.position.z:.6f}m ({pose.position.z*1000:.1f}mm)")
                print(f"      姿态四元数: x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, "
                      f"z={pose.orientation.z:.6f}, w={pose.orientation.w:.6f}")
                print(f"      姿态欧拉角: roll={math.degrees(roll):.2f}°, "
                      f"pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
                print(f"      耗时: {response.computation_time_ms:.2f} ms")
            else:
                print(f"      ✗ 未返回位姿结果")
                results.append({'solver': solver_type, 'success': False})
        else:
            print(f"      ✗ FK计算失败")
            results.append({'solver': solver_type, 'success': False})
    
    return results


def test_fk_comparison(node, kinematics_client, arm_type, joint_angles, solver_types):
    """对比测试不同求解器的正运动学结果"""
    
    print(f"\n    关节角度: {[f'{j:.4f}' for j in joint_angles]}")
    
    results = []
    
    for solver_type in solver_types:
        print(f"\n    → 使用求解器: {solver_type}")
        
        fk_request = KinematicsService.Request()
        fk_request.operation_type = "fk"
        fk_request.arm_type = arm_type
        fk_request.solver_type = solver_type
        fk_request.joint_angles = joint_angles
        fk_request.max_iterations = 100
        fk_request.tolerance = 1e-8
        
        start_time = time.time()
        future = kinematics_client.call_async(fk_request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        elapsed_time = (time.time() - start_time) * 1000
        
        if future.result() and future.result().success:
            response = future.result()
            if response.result_poses:
                pose = response.result_poses[0]
                
                results.append({
                    'solver': solver_type,
                    'success': True,
                    'pose': pose,
                    'position': (pose.position.x, pose.position.y, pose.position.z),
                    'quaternion': (pose.orientation.x, pose.orientation.y, 
                                   pose.orientation.z, pose.orientation.w),
                    'computation_time': response.computation_time_ms,
                    'total_time': elapsed_time
                })
                
                print(f"      位置: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}")
                print(f"      姿态: x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, "
                      f"z={pose.orientation.z:.6f}, w={pose.orientation.w:.6f}")
                print(f"      耗时: {response.computation_time_ms:.2f} ms (服务端)")
            else:
                print(f"      ✗ 未返回位姿结果")
                results.append({'solver': solver_type, 'success': False})
        else:
            print(f"      ✗ FK计算失败")
            results.append({'solver': solver_type, 'success': False})
    
    # 对比分析
    if len(results) >= 2 and results[0]['success'] and results[1]['success']:
        print(f"\n    → 对比分析:")
        print(f"      " + "-" * 60)
        
        # 计算位置差异
        pos1 = results[0]['position']
        pos2 = results[1]['position']
        pos_diff = math.sqrt(sum((p1 - p2)**2 for p1, p2 in zip(pos1, pos2)))
        
        # 计算姿态差异
        q1 = results[0]['quaternion']
        q2 = results[1]['quaternion']
        dot_product = abs(q1[3]*q2[3] + q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2])
        dot_product = min(1.0, max(-1.0, dot_product))
        rot_diff = math.acos(2.0 * dot_product * dot_product - 1.0)
        
        print(f"      两种求解器结果差异:")
        print(f"        位置差异: {pos_diff:.6f} m ({pos_diff*1000:.3f} mm)")
        print(f"        姿态差异: {rot_diff:.6f} rad ({math.degrees(rot_diff):.3f}°)")
        
        # 时间对比
        time1 = results[0]['computation_time']
        time2 = results[1]['computation_time']
        if time1 > 0 and time2 > 0:
            faster = results[0]['solver'] if time1 < time2 else results[1]['solver']
            print(f"        时间对比: {results[0]['solver']}={time1:.2f}ms, "
                  f"{results[1]['solver']}={time2:.2f}ms")
            print(f"        更快: {faster}")
        
        # 判断是否一致
        if pos_diff < 0.001 and rot_diff < 0.01:
            print(f"      ✓ 两种求解器结果一致")
        else:
            print(f"      ⚠ 两种求解器结果存在差异")
    
    return results


def verify_ik_with_fk(node, kinematics_client, arm_type, ik_solution, target_pose, solver_name=""):
    """使用FK验证IK结果"""
    prefix = f"      [{solver_name}] " if solver_name else "      "
    print(f"    → 验证IK结果（使用FK）:")
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
            
            print(f"      {prefix}目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
            print(f"      {prefix}实际位置: ({actual_pose.position.x:.4f}, {actual_pose.position.y:.4f}, {actual_pose.position.z:.4f})")
            print(f"      ─────────────────────────────────────────────────")
            print(f"      {prefix}位置误差: {pos_error:.6f} m ({pos_error*1000:.3f} mm)")
            print(f"      {prefix}姿态误差: {rot_error:.6f} rad ({math.degrees(rot_error):.3f}°)")
            
            # 判断是否成功
            if pos_error < 0.01 and rot_error < 0.1:
                print(f"      {prefix}✓ 验证成功！误差在可接受范围内")
                return True, pos_error, rot_error, actual_pose
            else:
                print(f"      {prefix}⚠ 验证警告：误差较大")
                return False, pos_error, rot_error, actual_pose
    else:
        print(f"      {prefix}✗ FK验证失败：无法计算实际位姿")
        return False, float('inf'), float('inf'), None


def test_ik_with_verification(node, kinematics_client, arm_type, initial_joints, 
                               target_pose, solver_type, max_iterations=1000, 
                               tolerance=1e-6, extra_params=None):
    """测试IK并进行FK验证，支持不同求解器"""
    
    print(f"\n    目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
    print(f"    目标姿态: w={target_pose.orientation.w:.4f}, x={target_pose.orientation.x:.4f}, "
          f"y={target_pose.orientation.y:.4f}, z={target_pose.orientation.z:.4f}")
    
    # 创建IK请求
    ik_request = KinematicsService.Request()
    ik_request.operation_type = "ik"
    ik_request.arm_type = arm_type
    ik_request.solver_type = solver_type
    ik_request.joint_angles = initial_joints
    ik_request.target_poses = [target_pose]
    ik_request.max_iterations = max_iterations
    ik_request.tolerance = tolerance
    
    # 设置SDK额外参数
    if extra_params:
        ik_request.dgr1 = extra_params.get('dgr1', 0.05)
        ik_request.dgr2 = extra_params.get('dgr2', 0.05)
        ik_request.dgr3 = extra_params.get('dgr3', 0.0)
        ik_request.dls_damping = extra_params.get('dls_damping', 0.01)
        ik_request.enable_random_restart = extra_params.get('random_restart', False)
    
    start_time = time.time()
    future = kinematics_client.call_async(ik_request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    elapsed_time = time.time() - start_time
    
    if future.result():
        response = future.result()
        if response.success:
            print(f"    ✓ IK计算成功! (求解器: {solver_type})")
            print(f"      消息: {response.message}")
            print(f"      服务端耗时: {response.computation_time_ms:.2f} ms")
            print(f"      客户端耗时: {elapsed_time*1000:.2f} ms")
            
            if hasattr(response, 'used_solver') and response.used_solver:
                print(f"      实际使用求解器: {response.used_solver}")
            if hasattr(response, 'iterations') and response.iterations > 0:
                print(f"      迭代次数: {response.iterations}")
            if hasattr(response, 'final_error') and response.final_error > 0:
                print(f"      最终误差: {response.final_error:.6f}")
            
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
                    node, kinematics_client, arm_type, joints, target_pose, solver_type
                )
                
                return True, joints, pos_err, rot_err, response.computation_time_ms
        else:
            print(f"    ✗ IK计算失败 (求解器: {solver_type}): {response.message}")
            return False, None, float('inf'), float('inf'), 0.0
    else:
        print(f"    ✗ 服务调用超时 (求解器: {solver_type})")
        return False, None, float('inf'), float('inf'), 0.0


def test_single_ik_comparison(node, kinematics_client, test_name, arm_type, 
                               initial_joints, target_pose, extra_params=None):
    """对比测试单个IK任务在不同求解器下的表现"""
    
    print("\n" + "=" * 70)
    print(f"对比测试: {test_name}")
    print("=" * 70)
    
    solvers = [
        ("SDK", {"dgr1": 0.05, "dgr2": 0.05}),
        ("BFGS", {"max_iterations": 1000, "tolerance": 1e-6}),
        ("DLS", {"dls_damping": 0.01}),
        ("AUTO", {})
    ]
    
    results = []
    
    for solver_type, params in solvers:
        print(f"\n{'='*60}")
        print(f"使用求解器: {solver_type}")
        print(f"{'='*60}")
        
        max_iter = params.get('max_iterations', 1000) if solver_type != "SDK" else 100
        tol = params.get('tolerance', 1e-6)
        
        success, joints, pos_err, rot_err, comp_time = test_ik_with_verification(
            node, kinematics_client, arm_type, initial_joints, target_pose,
            solver_type, max_iter, tol, extra_params
        )
        
        results.append({
            'solver': solver_type,
            'success': success,
            'pos_error': pos_err,
            'rot_error': rot_err,
            'computation_time': comp_time,
            'joints': joints
        })
    
    # 打印对比结果
    print("\n" + "=" * 70)
    print(f"对比结果汇总: {test_name}")
    print("=" * 70)
    print(f"\n{'求解器':<8} {'成功':<6} {'位置误差(mm)':<15} {'姿态误差(°)':<15} {'耗时(ms)':<12}")
    print("-" * 70)
    
    for r in results:
        status = "✓" if r['success'] else "✗"
        pos_mm = r['pos_error'] * 1000 if r['pos_error'] != float('inf') else float('inf')
        rot_deg = math.degrees(r['rot_error']) if r['rot_error'] != float('inf') else float('inf')
        
        pos_str = f"{pos_mm:.3f}" if pos_mm != float('inf') else "N/A"
        rot_str = f"{rot_deg:.3f}" if rot_deg != float('inf') else "N/A"
        time_str = f"{r['computation_time']:.2f}" if r['computation_time'] > 0 else "N/A"
        
        print(f"{r['solver']:<8} {status:<6} {pos_str:<15} {rot_str:<15} {time_str:<12}")
    
    print("-" * 70)
    
    # 找出最佳求解器
    successful = [r for r in results if r['success']]
    if successful:
        best_by_time = min(successful, key=lambda x: x['computation_time'])
        best_by_error = min(successful, key=lambda x: x['pos_error'])
        print(f"\n  最快求解器: {best_by_time['solver']} ({best_by_time['computation_time']:.2f} ms)")
        print(f"  最精确求解器: {best_by_error['solver']} (误差: {best_by_error['pos_error']*1000:.3f} mm)")
    
    return results


def main():
    """测试运动学服务（FK和IK），对比不同求解器"""
    
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
    
    # ========================================================================
    # 第四部分：零位测试
    # ========================================================================
    print("=" * 70)
    print("[4] 零位测试 (所有关节角度为0)")
    print("=" * 70)
    
    # 4.1 左臂零位测试
    print("\n[4.1] 左臂零位测试")
    print("-" * 70)
    
    zero_results_left = test_zero_pose_comparison(
        node, kinematics_client, "left", ["numerical", "SDK"]
    )
    
    # 4.2 右臂零位测试
    print("\n[4.2] 右臂零位测试")
    print("-" * 70)
    
    zero_results_right = test_zero_pose_comparison(
        node, kinematics_client, "right", ["numerical", "SDK"]
    )
    
    # 零位测试汇总
    print("\n" + "=" * 70)
    print("[4.3] 零位测试汇总")
    print("=" * 70)
    print(f"\n  {'手臂':<6} {'求解器':<12} {'位置(mm)':<40} {'耗时(ms)':<12}")
    print("-" * 70)
    
    for arm_name, results in [("左臂", zero_results_left), ("右臂", zero_results_right)]:
        for r in results:
            if r['success']:
                pos = r['position']
                euler = r['euler']
                pos_str = f"({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f})"
                euler_str = f"r={euler[0]:.1f}°, p={euler[1]:.1f}°, y={euler[2]:.1f}°"
                print(f"  {arm_name:<6} {r['solver']:<12} {pos_str:<30} {r['computation_time']:<12.2f}")
                print(f"                     {euler_str}")
    
    # 定义测试用的关节角度
    LEFT_TEST_JOINTS = [-0.3525227330, -0.7798290600, 0.8896949257, 
                        -1.8910405790, -2.7986485415, 0.4619120915, 0.8030297356]
    RIGHT_TEST_JOINTS = [0.0982891175, -0.8816540101, -0.5496532015, 
                         -1.8582004280, 2.8749939521, 0.3564035253, -1.0466912832]
    
    # ========================================================================
    # 第五部分：测试正运动学（FK）- 对比不同求解器
    # ========================================================================
    print("=" * 70)
    print("[5] 测试正运动学 (FK) - 对比数值解 vs SDK")
    print("=" * 70)
    
    left_fk_pose = None
    right_fk_pose = None
    fk_results = []
    
    # 5.1 测试左臂FK
    print("\n[5.1] 测试左臂FK - 对比数值解 vs SDK")
    print("-" * 70)
    
    results = test_fk_comparison(
        node, kinematics_client, "left", LEFT_TEST_JOINTS,
        ["numerical", "SDK"]
    )
    fk_results.append(("左臂", results))
    
    for r in results:
        if r['solver'] == "SDK" and r['success']:
            left_fk_pose = r['pose']
    
    # 5.2 测试右臂FK
    print("\n[5.2] 测试右臂FK - 对比数值解 vs SDK")
    print("-" * 70)
    
    results = test_fk_comparison(
        node, kinematics_client, "right", RIGHT_TEST_JOINTS,
        ["numerical", "SDK"]
    )
    fk_results.append(("右臂", results))
    
    for r in results:
        if r['solver'] == "SDK" and r['success']:
            right_fk_pose = r['pose']
    
    # 5.3 FK对比汇总
    print("\n" + "=" * 70)
    print("[5.3] 正运动学对比汇总")
    print("=" * 70)
    print(f"\n  {'手臂':<6} {'求解器':<12} {'位置(mm)':<30} {'耗时(ms)':<12}")
    print("-" * 70)
    
    for arm_name, results in fk_results:
        for r in results:
            if r['success']:
                pos = r['position']
                pos_str = f"({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f})"
                print(f"  {arm_name:<6} {r['solver']:<12} {pos_str:<30} {r['computation_time']:<12.2f}")
    
    # ========================================================================
    # 第六部分：对比测试不同求解器
    # ========================================================================
    print("\n" + "=" * 70)
    print("[6] 对比测试不同求解器 (SDK vs BFGS vs DLS vs AUTO)")
    print("=" * 70)
    
    all_comparison_results = []
    
    # 6.1 左臂自洽性测试
    if left_fk_pose:
        results = test_single_ik_comparison(
            node, kinematics_client,
            "左臂自洽性测试（目标 = SDK正解结果）",
            "left", LEFT_TEST_JOINTS, left_fk_pose,
            extra_params={'dgr1': 0.05, 'dgr2': 0.05}
        )
        all_comparison_results.append(("左臂自洽性", results))
    
    # 6.2 左臂Z方向移动测试
    if left_fk_pose:
        target_pose = Pose()
        target_pose.position.x = left_fk_pose.position.x
        target_pose.position.y = left_fk_pose.position.y
        target_pose.position.z = left_fk_pose.position.z - 0.2
        target_pose.orientation = left_fk_pose.orientation
        
        results = test_single_ik_comparison(
            node, kinematics_client,
            "左臂Z方向移动 -0.2m",
            "left", LEFT_TEST_JOINTS, target_pose,
            extra_params={'dgr1': 0.05, 'dgr2': 0.05}
        )
        all_comparison_results.append(("左臂Z移动", results))
    
    # 6.3 右臂自洽性测试
    if right_fk_pose:
        results = test_single_ik_comparison(
            node, kinematics_client,
            "右臂自洽性测试（目标 = SDK正解结果）",
            "right", RIGHT_TEST_JOINTS, right_fk_pose,
            extra_params={'dgr1': 0.05, 'dgr2': 0.05}
        )
        all_comparison_results.append(("右臂自洽性", results))
    
    # 6.4 右臂Z方向移动测试
    if right_fk_pose:
        target_pose = Pose()
        target_pose.position.x = right_fk_pose.position.x
        target_pose.position.y = right_fk_pose.position.y
        target_pose.position.z = right_fk_pose.position.z - 0.2
        target_pose.orientation = right_fk_pose.orientation
        
        results = test_single_ik_comparison(
            node, kinematics_client,
            "右臂Z方向移动 -0.2m",
            "right", RIGHT_TEST_JOINTS, target_pose,
            extra_params={'dgr1': 0.05, 'dgr2': 0.05}
        )
        all_comparison_results.append(("右臂Z移动", results))
    
    # ========================================================================
    # 第七部分：全局对比汇总
    # ========================================================================
    print("\n" + "=" * 70)
    print("[7] 全局求解器性能对比汇总")
    print("=" * 70)
    
    # 统计各求解器的表现
    solver_stats = {
        "SDK": {"success": 0, "total": 0, "total_time": 0, "total_pos_error": 0},
        "BFGS": {"success": 0, "total": 0, "total_time": 0, "total_pos_error": 0},
        "DLS": {"success": 0, "total": 0, "total_time": 0, "total_pos_error": 0},
        "AUTO": {"success": 0, "total": 0, "total_time": 0, "total_pos_error": 0}
    }
    
    for test_name, results in all_comparison_results:
        for r in results:
            solver = r['solver']
            solver_stats[solver]['total'] += 1
            if r['success']:
                solver_stats[solver]['success'] += 1
                if r['computation_time'] > 0:
                    solver_stats[solver]['total_time'] += r['computation_time']
                if r['pos_error'] != float('inf'):
                    solver_stats[solver]['total_pos_error'] += r['pos_error']
    
    print("\n  求解器性能统计:")
    print("  " + "-" * 70)
    print(f"  {'求解器':<8} {'成功率':<10} {'平均耗时(ms)':<15} {'平均位置误差(mm)':<20}")
    print("  " + "-" * 70)
    
    for solver, stats in solver_stats.items():
        if stats['total'] > 0:
            success_rate = stats['success'] / stats['total'] * 100
            avg_time = stats['total_time'] / stats['success'] if stats['success'] > 0 else 0
            avg_error = (stats['total_pos_error'] / stats['success'] * 1000) if stats['success'] > 0 else 0
            
            print(f"  {solver:<8} {success_rate:>6.1f}%      {avg_time:>10.2f}        {avg_error:>12.3f}")
    
    print("  " + "-" * 70)
    
    # 找出最佳求解器
    best_by_success = max(solver_stats.items(), key=lambda x: x[1]['success'] / max(x[1]['total'], 1))
    best_by_time = min([(s, d) for s, d in solver_stats.items() if d['success'] > 0], 
                       key=lambda x: x[1]['total_time'] / x[1]['success'])
    best_by_error = min([(s, d) for s, d in solver_stats.items() if d['success'] > 0], 
                        key=lambda x: x[1]['total_pos_error'] / x[1]['success'])
    
    print(f"\n  综合评价:")
    print(f"    最高成功率: {best_by_success[0]} ({best_by_success[1]['success']/max(best_by_success[1]['total'],1)*100:.1f}%)")
    print(f"    最快速度: {best_by_time[0]} ({best_by_time[1]['total_time']/best_by_time[1]['success']:.2f} ms)")
    print(f"    最高精度: {best_by_error[0]} ({best_by_error[1]['total_pos_error']/best_by_error[1]['success']*1000:.3f} mm)")
    
    # ========================================================================
    # 第八部分：清理和退出
    # ========================================================================
    print("\n" + "=" * 70)
    print("[8] 清理和退出")
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