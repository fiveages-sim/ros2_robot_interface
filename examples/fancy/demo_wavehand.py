"""
W2 机器人 挥手测试脚本 适用于分体控制
测试新的 target_path 接口，使用 w2轨迹测试.txt 中的轨迹数据
"""

import time
import sys
import math
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


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
    """测试W2机器人的路径接口"""

    # ========================================================================
    # 配置参数
    # ========================================================================
    TRAJECTORY_EXECUTION_WAIT_TIME = 6.0  # 轨迹执行完成后的等待时间（秒）

    print("\n" + "=" * 70)
    print(" " * 20 + "W2 Robot Wave Hand Demo")
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
        # 先切换到HOME状态
        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)
        print("  ✓ 已切换到HOME状态")

        # 先切换到HOME状态
        print("  → 再切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")

        # 再切换到OCS2状态
        print("  → 切换到OCS2状态...")
        interface.send_fsm_command(3)  # 3 = OCS2/MOVE状态
        time.sleep(0.1)  # 等待状态转换完成
        print("  ✓ 已切换到OCS2状态\n")
    except Exception as e:
        print(f"  ✗ 切换到OCS2状态失败: {e}\n")
        interface.disconnect()
        return 1

    # ========================================================================
    # 第三部分：准备轨迹数据
    # ========================================================================
    print("-" * 70)
    print("[6] 准备轨迹数据")
    print("-" * 70)

    # A点轨迹数据
    # A点 - Left Arm（左臂）初始位置（用于第一个点）
    a_left_initial = [
        [0.06, 0.44, -0.58, 0.69, -0.69, 0.12, -0.12],
    ]
    
    # 左臂挥手轨迹
    a_left_wave = [
        # [0.3, 0.27, 0.5, 0.22, 0.05, -0.05, 0.97],
        [0.28, 0.6, 0.44, 0.1, 0.06, -0.029, 0.99],
        [0.3, 0.27, 0.5, 0.22, 0.05, -0.05, 0.97],
        [0.28, 0.6, 0.44, -0.1, 0.06, -0.029, 0.99],
        # [0.3, 0.27, 0.5, 0.22, 0.05, -0.05, 0.97],
    ]

    # A点 - Right Arm（右臂）初始位置（用于第一个点）
    a_right_initial = [
        [0.06, -0.44, -0.58, 0.69, -0.69, 0.12, -0.12],
    ]
    
    # 右臂挥手轨迹
    a_right_wave = [
        # [0.3, -0.27, 0.5, 0.07, 0.18, 0.97, 0.04],
        [0.28, -0.6, 0.44, 0.06, -0.11, 0.989546, 0.062],
        [0.3, -0.27, 0.5, 0.07, 0.18, 0.97, 0.04],
        [0.28, -0.6, 0.44, 0.06, -0.11, 0.989546, 0.062],
        # [0.3, -0.27, 0.5, 0.07, 0.18, 0.97, 0.04],
    ]

    print("  ✓ 轨迹数据已加载")
    print(f"    左臂初始位置: {len(a_left_initial)} 个点")
    print(f"    左臂挥手轨迹: {len(a_left_wave)} 个点")
    print(f"    右臂初始位置: {len(a_right_initial)} 个点")
    print(f"    右臂挥手轨迹: {len(a_right_wave)} 个点")

    # ========================================================================
    # 第四部分：测试A点轨迹（重复执行两次）
    # ========================================================================
    print("=" * 70)
    print("[7] 测试挥手路径（将重复执行2次）")
    print("=" * 70)

    # 转换A点initial轨迹
    a_left_initial_poses = vectors_to_poses(a_left_initial, frame_id="arm_base")
    a_right_initial_poses = vectors_to_poses(a_right_initial, frame_id="arm_base")

    print(f"  → 准备A点initial路径...")
    print(f"    左臂: {len(a_left_initial_poses)} 个路径点")
    print(f"    右臂: {len(a_right_initial_poses)} 个路径点\n")

    # 提取右臂第一个点（用于后续保持位置）
    first_right_pose = None
    if len(a_right_initial_poses) > 0:
        first_right_pose = a_right_initial_poses[0].pose
    
    # 提取左臂第一个点（用于后续保持位置）
    first_left_pose = None
    if len(a_left_initial_poses) > 0:
        first_left_pose = a_left_initial_poses[0].pose

    # 准备挥手轨迹
    a_left_wave_poses = vectors_to_poses(a_left_wave, frame_id="arm_base")
    a_right_wave_poses = vectors_to_poses(a_right_wave, frame_id="arm_base")
    
    # 重复执行两次轨迹
    NUM_REPETITIONS = 2
    for repetition in range(1, NUM_REPETITIONS + 1):
        print("-" * 70)
        if repetition == 1:
            print(f"[7.{repetition}] 第 {repetition} 次执行轨迹：向右转，右臂挥手")
        else:
            print(f"[7.{repetition}] 第 {repetition} 次执行轨迹：向左转，左臂挥手")
        print("-" * 70)
        
        try:
            if repetition == 1:
                # 第一次：右臂挥手，左臂使用初始位置
                if first_left_pose is None:
                    raise ValueError("左臂初始位置未定义")
                # 使用左臂初始位置，重复以匹配右臂轨迹长度
                left_poses = []
                for _ in range(len(a_right_wave_poses)):
                    pose_stamped = PoseStamped()
                    pose_stamped.header = Header(frame_id="arm_base")
                    pose_stamped.pose = first_left_pose
                    left_poses.append(pose_stamped)
                
                right_poses = a_right_wave_poses
                print(f"  → 发送右臂挥手轨迹（{len(right_poses)}个点），左臂使用初始位置")
            else:
                # 第二次：左臂挥手，右臂使用初始位置
                if first_right_pose is None:
                    raise ValueError("右臂初始位置未定义")
                # 使用右臂初始位置，重复以匹配左臂轨迹长度
                right_poses = []
                for _ in range(len(a_left_wave_poses)):
                    pose_stamped = PoseStamped()
                    pose_stamped.header = Header(frame_id="arm_base")
                    pose_stamped.pose = first_right_pose
                    right_poses.append(pose_stamped)
                
                left_poses = a_left_wave_poses
                print(f"  → 发送左臂挥手轨迹（{len(left_poses)}个点），右臂使用初始位置")
            
            # 发送轨迹路径
            interface.send_target_path(left_poses, right_poses, frame_id="arm_base")
            print("  ✓ 轨迹路径已发送")
            
            # 同时发送腰部关节指令：前三个关节保持固定值，第四个关节根据执行次数设置
            # 第一次：+60度（向右转），第二次：-60度（向左转）
            if repetition == 1:
                waist_rotation_deg = 30.0
            else:
                waist_rotation_deg = -30.0
            
            waist_rotation_rad = math.radians(waist_rotation_deg)
            body_joint_positions = [-0.899, -1.714, -0.865, waist_rotation_rad]
            if interface.config.body_joint_controller_topic:
                interface.send_body_joint_positions(body_joint_positions)
                print(f"  ✓ 已发送腰部关节指令: {body_joint_positions} (第{repetition}次，旋转{waist_rotation_deg}度)")
            else:
                print("  ⚠ 未配置body_joint_controller_topic，跳过腰部关节指令")
            
            print(f"  → 等待轨迹执行完成（约{TRAJECTORY_EXECUTION_WAIT_TIME}秒）...")
            time.sleep(TRAJECTORY_EXECUTION_WAIT_TIME)
                
        except Exception as e:
            print(f"  ✗ 第 {repetition} 次执行失败: {e}\n")
            break

    print("=" * 70)
    print(f"  ✓ 所有 {NUM_REPETITIONS} 次轨迹执行完成")
    print("=" * 70 + "\n")

    # ========================================================================
    # 第五部分：回到HOME位置并切换到HOLD状态
    # ========================================================================
    print("-" * 70)
    print("[8] 回到HOME位置并切换到HOLD状态")
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
    print("[9] 测试完成")
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

