"""
W2 机器人 倒酒动作测试脚本
基于 pick_by_registration.py 中的 pour_beer 方法实现
适用于 O6 灵巧手
"""

import time
import sys
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def euler_to_rotation_matrix(rx, ry, rz, axes='sxyz'):
    """
    将欧拉角转换为旋转矩阵（使用numpy实现）
    支持固定轴顺序 sxyz (X->Y->Z)
    
    参数:
        rx, ry, rz: 绕X, Y, Z轴的旋转角度（弧度）
        axes: 轴顺序，默认 'sxyz' 表示固定轴顺序 X->Y->Z
    
    返回:
        3x3旋转矩阵
    """
    # 固定轴顺序 sxyz: 先绕X轴旋转rx，再绕Y轴旋转ry，最后绕Z轴旋转rz
    # 旋转矩阵: R = Rz * Ry * Rx
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    
    # 计算旋转矩阵
    # Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    # Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    # Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    # R = Rz * Ry * Rx
    R = np.array([
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
        [-sy, cy * sx, cx * cy]
    ])
    
    return R


def rotation_matrix_to_quaternion(R):
    """
    从旋转矩阵提取四元数
    
    参数:
        R: 3x3旋转矩阵
    
    返回:
        [qx, qy, qz, qw] - 四元数
    """
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    
    # 确保四元数的w分量为正（标准形式）
    if qw < 0:
        qw = -qw
        qx = -qx
        qy = -qy
        qz = -qz
    
    return [qx, qy, qz, qw]


def euler_to_quaternion(xyzrpy):
    """
    将6维向量 [x, y, z, rx, ry, rz] 转换为7维向量 [x, y, z, qx, qy, qz, qw]
    其中 rx, ry, rz 是欧拉角（弧度），使用固定轴顺序 sxyz (X->Y->Z)
    
    参数:
        xyzrpy: [x, y, z, rx, ry, rz] - 位置和欧拉角
    
    返回:
        [x, y, z, qx, qy, qz, qw] - 位置和四元数
    """
    x, y, z, rx, ry, rz = xyzrpy
    
    # 将欧拉角转换为旋转矩阵 (axes='sxyz' 表示固定轴 X->Y->Z)
    R = euler_to_rotation_matrix(rx, ry, rz, axes='sxyz')
    
    # 从旋转矩阵提取四元数
    qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
    
    return [x, y, z, qx, qy, qz, qw]


def vector_to_pose(vector):
    """将7维向量 [x, y, z, qx, qy, qz, qw] 转换为 Pose 对象"""
    pose = Pose()
    pose.position.x = vector[0]
    pose.position.y = vector[1]
    pose.position.z = vector[2]
    pose.orientation.x = vector[3]
    pose.orientation.y = vector[4]
    pose.orientation.z = vector[5]
    pose.orientation.w = vector[6]
    return pose


def vectors_to_poses(vectors, frame_id="arm_base"):
    """将多个7维向量转换为 PoseStamped 列表"""
    poses = []
    for vec in vectors:
        pose_stamped = PoseStamped()
        pose_stamped.header = Header(frame_id=frame_id)
        pose_stamped.pose = vector_to_pose(vec)
        poses.append(pose_stamped)
    return poses


def main():
    """测试W2机器人的倒酒动作"""

    # ========================================================================
    # 配置参数
    # ========================================================================
    TRAJECTORY_EXECUTION_WAIT_TIME = 8.0  # 轨迹执行完成后的等待时间（秒）
    POURING_DURATION = 5.0  # 倒酒持续时间（秒）

    print("\n" + "=" * 70)
    print(" " * 20 + "W2 Robot Pour Beer Demo (O6 Hand)")
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

    # ========================================================================
    # 第二部分：切换到OCS2状态
    # ========================================================================
    print("-" * 70)
    print("[5] 切换到OCS2状态")
    print("-" * 70)
    try:
        print("  → 切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")
        
        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)
        print("  ✓ 已切换到HOME状态")

        print("  → 再切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")

        print("  → 切换到OCS2状态...")
        interface.send_fsm_command(3)  # 3 = OCS2/MOVE状态
        time.sleep(0.1)
        print("  ✓ 已切换到OCS2状态\n")
    except Exception as e:
        print(f"  ✗ 切换到OCS2状态失败: {e}\n")
        interface.disconnect()
        return 1

    # ========================================================================
    # 第三部分：准备倒酒轨迹数据（O6灵巧手版本）
    # ========================================================================
    print("-" * 70)
    print("[6] 准备倒酒轨迹数据（O6灵巧手版本）")
    print("-" * 70)

    # 左臂轨迹点（杯子）[x, y, z, qx, qy, qz, qw] 单位：米，四元数
    left_road_points_quat = {
        "p_0": [0.2414, 0.4167, -0.3037, 0.250251, -0.682559, 0.652591, -0.213572],
        "p_1": [0.2586, 0.2097, -0.3527, 0.278988, -0.776482, 0.538328, -0.171591],
        "p_2": [0.262800, 0.120400, -0.360000, 0.278988, -0.776482, 0.538328, -0.171591],
    }


    # 右臂轨迹点（酒枪）[x, y, z, qx, qy, qz, qw] 单位：米，四元数 - O6版本
    right_road_points_quat = {
        "p_0": [0.173600, -0.384100, -0.249900, -0.645109, 0.286136, -0.274507, 0.653152],
        "p_1": [0.431400, -0.388700, -0.053600, -0.982002, -0.091087, -0.011231, 0.165069],
        "p_2": [0.421100, -0.373100, -0.048600, -0.982010, -0.091081, -0.011285, 0.165025],
        "p_3": [0.427400, -0.371000, -0.240000, -0.981977, -0.091171, -0.010962, 0.165194],
    }

    print("  ✓ 轨迹数据已加载（四元数格式）")
    print(f"    左臂轨迹点: {len(left_road_points_quat)} 个")
    print(f"    右臂轨迹点: {len(right_road_points_quat)} 个")
    
    # 输出四元数数据
    print("\n  左臂轨迹点四元数:")
    for key, quat in left_road_points_quat.items():
        print(f"    {key}: [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}, {quat[4]:.6f}, {quat[5]:.6f}, {quat[6]:.6f}]")
        print(f"        (x, y, z, qx, qy, qz, qw)")
    
    print("\n  右臂轨迹点四元数:")
    for key, quat in right_road_points_quat.items():
        print(f"    {key}: [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}, {quat[4]:.6f}, {quat[5]:.6f}, {quat[6]:.6f}]")
        print(f"        (x, y, z, qx, qy, qz, qw)")
    print()

    # ========================================================================
    # 第四部分：构建倒酒轨迹（分为两段）
    # ========================================================================
    print("-" * 70)
    print("[7] 构建倒酒轨迹（分为两段）")
    print("-" * 70)

    # 第一段轨迹：移动到倒酒位置
    # 左臂轨迹: p_0 -> p_1 -> p_2（左臂保持在p_2位置）
    left_forward_trajectory = [
        left_road_points_quat["p_0"],  # 起始位置
        left_road_points_quat["p_1"],  # 中间位置
        left_road_points_quat["p_2"],  # 倒酒准备位置
    ]

    # 右臂轨迹: p_0 -> p_1 -> p_2 -> p_3（右臂移动到倒酒位置）
    right_forward_trajectory = [
        right_road_points_quat["p_0"],  # 起始位置
        right_road_points_quat["p_1"],  # 中间位置
        right_road_points_quat["p_2"],  # 倒酒准备位置
        right_road_points_quat["p_3"],  # 倒酒位置
    ]

    # 第二段轨迹：从倒酒位置返回到起始位置
    # 左臂轨迹: p_2 -> p_1 -> p_0
    left_return_trajectory = [
        left_road_points_quat["p_2"],  # 倒酒准备位置
        left_road_points_quat["p_1"],  # 中间位置
        left_road_points_quat["p_0"],  # 起始位置
    ]

    # 右臂轨迹: p_3 -> p_2 -> p_1 -> p_0
    right_return_trajectory = [
        right_road_points_quat["p_3"],  # 倒酒位置
        right_road_points_quat["p_2"],  # 倒酒准备位置
        right_road_points_quat["p_1"],  # 中间位置
        right_road_points_quat["p_0"],  # 起始位置
    ]

    # 将第一段轨迹转换为PoseStamped列表
    left_forward_poses = vectors_to_poses(left_forward_trajectory, frame_id="arm_base")
    right_forward_poses = vectors_to_poses(right_forward_trajectory, frame_id="arm_base")

    # 将第二段轨迹转换为PoseStamped列表
    left_return_poses = vectors_to_poses(left_return_trajectory, frame_id="arm_base")
    right_return_poses = vectors_to_poses(right_return_trajectory, frame_id="arm_base")

    # 确保第一段轨迹左右臂长度匹配
    max_forward_length = max(len(left_forward_poses), len(right_forward_poses))
    if len(left_forward_poses) < max_forward_length:
        last_left_pose = left_forward_poses[-1]
        while len(left_forward_poses) < max_forward_length:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(frame_id="arm_base")
            pose_stamped.pose = last_left_pose.pose
            left_forward_poses.append(pose_stamped)
    
    if len(right_forward_poses) < max_forward_length:
        last_right_pose = right_forward_poses[-1]
        while len(right_forward_poses) < max_forward_length:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(frame_id="arm_base")
            pose_stamped.pose = last_right_pose.pose
            right_forward_poses.append(pose_stamped)

    # 确保第二段轨迹左右臂长度匹配
    max_return_length = max(len(left_return_poses), len(right_return_poses))
    if len(left_return_poses) < max_return_length:
        last_left_pose = left_return_poses[-1]
        while len(left_return_poses) < max_return_length:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(frame_id="arm_base")
            pose_stamped.pose = last_left_pose.pose
            left_return_poses.append(pose_stamped)
    
    if len(right_return_poses) < max_return_length:
        last_right_pose = right_return_poses[-1]
        while len(right_return_poses) < max_return_length:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(frame_id="arm_base")
            pose_stamped.pose = last_right_pose.pose
            right_return_poses.append(pose_stamped)

    print("  ✓ 轨迹已构建")
    print(f"    第一段轨迹 - 左臂: {len(left_forward_poses)} 个点，右臂: {len(right_forward_poses)} 个点")
    print(f"    第二段轨迹 - 左臂: {len(left_return_poses)} 个点，右臂: {len(right_return_poses)} 个点")
    print(f"    第一段: 起始位置 -> 中间位置 -> 倒酒准备位置 -> 倒酒位置")
    print(f"    第二段: 倒酒位置 -> 倒酒准备位置 -> 中间位置 -> 起始位置\n")

    # ========================================================================
    # 第五部分：执行倒酒动作
    # ========================================================================
    print("=" * 70)
    print("[8] 执行倒酒动作")
    print("=" * 70)

    try:
        # 第一阶段：发送第一段轨迹到倒酒位置
        print("-" * 70)
        print("[8.1] 第一阶段：移动到倒酒位置")
        print("-" * 70)
        print("  → 发送第一段轨迹...")
        interface.send_target_path(left_forward_poses, right_forward_poses, frame_id="arm_base")
        print("  ✓ 第一段轨迹已发送")
        print(f"  → 等待到达倒酒位置（约{TRAJECTORY_EXECUTION_WAIT_TIME}秒）...")
        time.sleep(TRAJECTORY_EXECUTION_WAIT_TIME)
        print("  ✓ 已到达倒酒位置")

        # 确认到达倒酒位置并保持5秒
        print("-" * 70)
        print("[8.2] 在倒酒位置保持并执行倒酒动作")
        print("-" * 70)
        print("  ⚠ 注意：此处需要控制O6灵巧手执行扣扳机动作")
        print("     手势控制需要单独实现，或通过其他接口控制")
        print(f"  → 倒酒中...（保持{POURING_DURATION}秒）")
        time.sleep(POURING_DURATION)
        print("  ✓ 倒酒完成")

        # 第二阶段：发送第二段轨迹返回起始位置
        print("-" * 70)
        print("[8.3] 第二阶段：返回起始位置")
        print("-" * 70)
        print("  → 发送第二段轨迹...")
        interface.send_target_path(left_return_poses, right_return_poses, frame_id="arm_base")
        print("  ✓ 第二段轨迹已发送")
        print(f"  → 等待返回起始位置（约{TRAJECTORY_EXECUTION_WAIT_TIME}秒）...")
        time.sleep(TRAJECTORY_EXECUTION_WAIT_TIME)
        print("  ✓ 已返回起始位置")

        print("=" * 70)
        print("  ✓ 倒酒动作完成")
        print("=" * 70 + "\n")

    except Exception as e:
        print(f"  ✗ 执行倒酒动作失败: {e}\n")
        import traceback
        traceback.print_exc()
        interface.disconnect()
        return 1

    # ========================================================================
    # 第六部分：回到HOME位置并切换到HOLD状态
    # ========================================================================
    print("-" * 70)
    print("[9] 回到HOME位置并切换到HOLD状态")
    print("-" * 70)
    try:
        print("  → 先切换到HOLD状态...")
        interface.send_fsm_command(2)  # 2 = HOLD状态
        time.sleep(0.1)
        print("  ✓ 已切换到HOLD状态")
        
        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)
        print("  ✓ 已切换到HOME状态，机器人已回到HOME位置")
        
        print("  → 再切换到HOLD状态...")
        interface.send_fsm_command(2)  # 2 = HOLD状态
        time.sleep(0.1)
        print("  ✓ 已切换到HOLD状态\n")
    except Exception as e:
        print(f"  ✗ 回到HOME/HOLD状态失败: {e}\n")

    # ========================================================================
    # 第七部分：完成和清理
    # ========================================================================
    print("=" * 70)
    print("[10] 测试完成")
    print("=" * 70)
    print("  ✓ 倒酒动作测试已完成")
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

