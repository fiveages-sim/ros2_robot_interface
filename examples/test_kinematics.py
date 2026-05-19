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

# 预定义的测试关节角度
LEFT_TEST_JOINTS = [-0.3525227330, -0.7798290600, 0.8896949257, 
                    -1.8910405790, -2.7986485415, 0.4619120915, 0.8030297356]
RIGHT_TEST_JOINTS = [0.0982891175, -0.8816540101, -0.5496532015, 
                     -1.8582004280, 2.8749939521, 0.3564035253, -1.0466912832]


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


def get_fk_pose(node, kinematics_client, arm_type, joint_angles):
    """获取正运动学结果（用于生成测试目标）"""
    
    fk_request = KinematicsService.Request()
    fk_request.operation_type = "fk"
    fk_request.arm_type = arm_type
    fk_request.solver_type = "auto"
    fk_request.joint_angles = joint_angles
    fk_request.max_iterations = 100
    fk_request.tolerance = 1e-8
    
    future = kinematics_client.call_async(fk_request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    if future.result() and future.result().success and future.result().result_poses:
        return future.result().result_poses[0]
    else:
        return None


def verify_ik_with_fk(node, kinematics_client, arm_type, ik_solution, target_pose, solver_name=""):
    """使用FK验证IK结果"""
    prefix = f"      [{solver_name}] " if solver_name else "      "
    print(f"    → 验证IK结果（使用FK）:")
    print(f"      " + "-" * 60)
    
    # 创建FK请求验证IK结果
    fk_verify_request = KinematicsService.Request()
    fk_verify_request.operation_type = "fk"
    fk_verify_request.arm_type = arm_type
    fk_verify_request.solver_type = "auto"
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


def test_ik_solver(node, kinematics_client, arm_type, initial_joints, 
                   target_pose, solver_type, max_iterations=1000, 
                   tolerance=1e-6, extra_params=None):
    """测试单个IK求解器并进行FK验证"""
    
    print(f"\n  初始关节角度: {[f'{j:.4f}' for j in initial_joints]}")
    print(f"  目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
    
    # 创建IK请求
    ik_request = KinematicsService.Request()
    ik_request.operation_type = "ik"
    ik_request.arm_type = arm_type
    ik_request.solver_type = solver_type
    ik_request.joint_angles = initial_joints
    ik_request.target_poses = [target_pose]
    ik_request.max_iterations = max_iterations
    ik_request.tolerance = tolerance
    
    # 设置额外参数（DLS专用）
    if extra_params:
        ik_request.dls_damping = extra_params.get('dls_damping', 0.01)
        ik_request.enable_random_restart = extra_params.get('random_restart', False)
    
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
            print(f"    ✗ IK计算失败: {response.message}")
            return False, None, float('inf'), float('inf'), 0.0
    else:
        print(f"    ✗ 服务调用超时")
        return False, None, float('inf'), float('inf'), 0.0


def compare_ik_solvers(node, kinematics_client, test_name, arm_type, 
                       initial_joints, target_pose):
    """对比测试 IK 的两种求解器: BFGS 和 DLS"""
    
    print("\n" + "=" * 70)
    print(f"对比测试: {test_name}")
    print("=" * 70)
    
    # 定义两种求解器
    solvers = [
        ("BFGS", {"max_iterations": 1000, "tolerance": 1e-6}),
        ("DLS", {"dls_damping": 0.01})
    ]
    
    results = []
    
    for solver_type, params in solvers:
        print(f"\n{'='*60}")
        print(f"使用求解器: {solver_type}")
        print(f"{'='*60}")
        
        max_iter = params.get('max_iterations', 1000)
        tol = params.get('tolerance', 1e-6)
        extra_params = {'dls_damping': params.get('dls_damping', 0.01)} if solver_type == "DLS" else None
        
        success, joints, pos_err, rot_err, comp_time = test_ik_solver(
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
    """测试运动学服务 - 对比 BFGS 和 DLS 两种 IK 求解器"""
    
    # ========================================================================
    # 第一部分：初始化ROS 2
    # ========================================================================
    print("=" * 70)
    print("[1] 初始化ROS 2节点")
    print("=" * 70)
    
    if not rclpy.ok():
        rclpy.init()
    
    node = Node('kinematics_service_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    print("    ✓ 节点创建成功\n")
    
    # ========================================================================
    # 第二部分：使用预定义的测试关节角度
    # ========================================================================
    print("[2] 使用预定义的测试关节角度")
    print("-" * 70)
    
    print(f"左臂测试关节角度:")
    print(f"  {[f'{j:.4f}' for j in LEFT_TEST_JOINTS]}")
    print(f"\n右臂测试关节角度:")
    print(f"  {[f'{j:.4f}' for j in RIGHT_TEST_JOINTS]}\n")
    
    # ========================================================================
    # 第三部分：创建运动学服务客户端
    # ========================================================================
    print("[3] 创建运动学服务客户端")
    print("-" * 70)
    
    kinematics_client = node.create_client(KinematicsService, '/kinematics_service')
    
    if not kinematics_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('运动学服务不可用')
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        return 1
    
    print("    ✓ 运动学服务连接成功\n")
    
    # ========================================================================
    # 第四部分：获取正运动学结果作为测试目标
    # ========================================================================
    print("=" * 70)
    print("[4] 计算测试关节角度的正运动学结果")
    print("=" * 70)
    
    print("\n[4.1] 左臂正运动学")
    print("-" * 70)
    left_pose = get_fk_pose(node, kinematics_client, "left", LEFT_TEST_JOINTS)
    if left_pose:
        print(f"    左臂位姿:")
        print(f"      位置: x={left_pose.position.x:.4f}m, "
              f"y={left_pose.position.y:.4f}m, z={left_pose.position.z:.4f}m")
        roll, pitch, yaw = quaternion_to_euler(left_pose.orientation)
        print(f"      姿态: roll={math.degrees(roll):.2f}°, "
              f"pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
    else:
        print("    ✗ 左臂FK计算失败")
        return 1
    
    print("\n[4.2] 右臂正运动学")
    print("-" * 70)
    right_pose = get_fk_pose(node, kinematics_client, "right", RIGHT_TEST_JOINTS)
    if right_pose:
        print(f"    右臂位姿:")
        print(f"      位置: x={right_pose.position.x:.4f}m, "
              f"y={right_pose.position.y:.4f}m, z={right_pose.position.z:.4f}m")
        roll, pitch, yaw = quaternion_to_euler(right_pose.orientation)
        print(f"      姿态: roll={math.degrees(roll):.2f}°, "
              f"pitch={math.degrees(pitch):.2f}°, yaw={math.degrees(yaw):.2f}°")
    else:
        print("    ✗ 右臂FK计算失败")
        return 1
    
    # ========================================================================
    # 第五部分：测试自洽性（IK求解当前位姿，验证是否能回到原位置）
    # ========================================================================
    print("\n" + "=" * 70)
    print("[5] 自洽性测试 (IK求解当前位姿，验证是否能回到原位置)")
    print("=" * 70)
    
    print("\n[5.1] 左臂自洽性测试")
    print("-" * 70)
    compare_ik_solvers(
        node, kinematics_client,
        "左臂自洽性测试（目标 = 当前位姿）",
        "left", LEFT_TEST_JOINTS, left_pose
    )
    
    print("\n[5.2] 右臂自洽性测试")
    print("-" * 70)
    compare_ik_solvers(
        node, kinematics_client,
        "右臂自洽性测试（目标 = 当前位姿）",
        "right", RIGHT_TEST_JOINTS, right_pose
    )
    
    # ========================================================================
    # 第六部分：测试移动目标（Z方向移动 -0.1m）
    # ========================================================================
    print("\n" + "=" * 70)
    print("[6] 移动目标测试 (Z方向向下移动 0.1m)")
    print("=" * 70)
    
    # 创建左臂移动目标
    left_moved_pose = Pose()
    left_moved_pose.position.x = left_pose.position.x
    left_moved_pose.position.y = left_pose.position.y
    left_moved_pose.position.z = left_pose.position.z - 0.1
    left_moved_pose.orientation = left_pose.orientation
    
    print("\n[6.1] 左臂 Z-0.1m 移动测试")
    print("-" * 70)
    compare_ik_solvers(
        node, kinematics_client,
        "左臂 Z方向移动 -0.1m",
        "left", LEFT_TEST_JOINTS, left_moved_pose
    )
    
    # 创建右臂移动目标
    right_moved_pose = Pose()
    right_moved_pose.position.x = right_pose.position.x
    right_moved_pose.position.y = right_pose.position.y
    right_moved_pose.position.z = right_pose.position.z - 0.1
    right_moved_pose.orientation = right_pose.orientation
    
    print("\n[6.2] 右臂 Z-0.1m 移动测试")
    print("-" * 70)
    compare_ik_solvers(
        node, kinematics_client,
        "右臂 Z方向移动 -0.1m",
        "right", RIGHT_TEST_JOINTS, right_moved_pose
    )
    
    # ========================================================================
    # 第七部分：测试更大幅度的移动（X方向移动 +0.15m）
    # ========================================================================
    print("\n" + "=" * 70)
    print("[7] 大幅度移动测试 (X方向向前移动 0.15m)")
    print("=" * 70)
    
    # 创建左臂大幅度移动目标
    left_big_move_pose = Pose()
    left_big_move_pose.position.x = left_pose.position.x + 0.15
    left_big_move_pose.position.y = left_pose.position.y
    left_big_move_pose.position.z = left_pose.position.z
    left_big_move_pose.orientation = left_pose.orientation
    
    print("\n[7.1] 左臂 X+0.15m 移动测试")
    print("-" * 70)
    compare_ik_solvers(
        node, kinematics_client,
        "左臂 X方向移动 +0.15m",
        "left", LEFT_TEST_JOINTS, left_big_move_pose
    )
    
    # 创建右臂大幅度移动目标
    right_big_move_pose = Pose()
    right_big_move_pose.position.x = right_pose.position.x + 0.15
    right_big_move_pose.position.y = right_pose.position.y
    right_big_move_pose.position.z = right_pose.position.z
    right_big_move_pose.orientation = right_pose.orientation
    
    print("\n[7.2] 右臂 X+0.15m 移动测试")
    print("-" * 70)
    compare_ik_solvers(
        node, kinematics_client,
        "右臂 X方向移动 +0.15m",
        "right", RIGHT_TEST_JOINTS, right_big_move_pose
    )
    
    # ========================================================================
    # 第八部分：全局对比汇总
    # ========================================================================
    print("\n" + "=" * 70)
    print("[8] 全局求解器性能对比汇总")
    print("=" * 70)
    
    print("\n  BFGS 求解器特点:")
    print("    - 使用 BFGS 优化算法（拟牛顿法）")
    print("    - 适合复杂约束、冗余机械臂")
    print("    - 迭代次数较多但鲁棒性好")
    print("    - 需要计算目标函数的梯度和 Hessian 矩阵")
    
    print("\n  DLS 求解器特点:")
    print("    - 使用阻尼最小二乘法")
    print("    - 适合非冗余机械臂，速度快")
    print("    - 通过阻尼因子避免奇异点")
    print("    - 实时性较好")
    
    print("\n  建议:")
    print("    - 对于实时控制（如 MoveJ/MoveL），推荐使用 DLS")
    print("    - 对于高精度离线规划，推荐使用 BFGS")
    print("    - 可以在参数中配置 solver_type 自动选择")
    
    # ========================================================================
    # 第九部分：清理和退出
    # ========================================================================
    print("\n" + "=" * 70)
    print("[9] 清理和退出")
    print("=" * 70)
    
    kinematics_client.destroy()
    
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