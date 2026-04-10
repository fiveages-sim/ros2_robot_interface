"""
测试 send_dual_arm_target_stamped() 功能
测试发送双臂目标位姿到 /dual_target/stamped 话题
"""

import time
import sys
import math
from geometry_msgs.msg import Pose, Point, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 全局配置参数
MAX_WAIT_TIME = 5.0  # 最大等待时间（秒）
CHECK_INTERVAL = 0.5  # 检查间隔（秒）
POSE_THRESHOLD = 0.002  # 位置距离阈值（米）


def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """创建位姿消息的辅助函数"""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def print_pose(pose, label="Pose"):
    """打印位姿信息的辅助函数"""
    print(f"  {label}:")
    print(f"    位置: ({pose.position.x:7.3f}, {pose.position.y:7.3f}, {pose.position.z:7.3f})")
    print(f"    方向: ({pose.orientation.x:6.3f}, {pose.orientation.y:6.3f}, "
          f"{pose.orientation.z:6.3f}, {pose.orientation.w:6.3f})")


def quaternion_multiply(q1, q2):
    """四元数乘法，输入/输出格式均为 (x, y, z, w)"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def normalize_quaternion(q):
    """归一化四元数，避免累计数值误差"""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def rotate_quaternion_world_x(q_current, angle_deg):
    """按世界坐标系 X 轴旋转指定角度（度）"""
    half_angle = math.radians(angle_deg) * 0.5
    q_delta_world_x = (math.sin(half_angle), 0.0, 0.0, math.cos(half_angle))
    q_new = quaternion_multiply(q_delta_world_x, q_current)
    return normalize_quaternion(q_new)


def wait_for_arrival(interface, max_wait_time=30.0, check_interval=0.5, pose_threshold=0.005):
    """等待双臂到达目标位置
    
    Args:
        interface: ROS2RobotInterface 实例
        max_wait_time: 最大等待时间（秒）
        check_interval: 检查间隔（秒）
        pose_threshold: 位置距离阈值（米），默认0.005m
    
    Returns:
        bool: True 如果双臂都已到达，False 如果超时
    """
    start_time = time.time()
    
    while time.time() - start_time < max_wait_time:
        left_result = interface.left_arm_handler.check_arrival(pose_threshold=pose_threshold)
        right_result = interface.right_arm_handler.check_arrival(pose_threshold=pose_threshold)
        
        if left_result['arrived'] and right_result['arrived']:
            elapsed_time = time.time() - start_time
            print(f"  ✓ 双臂均已到达目标位置（耗时 {elapsed_time:.1f} 秒）")
            return True
        
        time.sleep(check_interval)
    
    elapsed_time = time.time() - start_time
    print(f"  ⚠ 超时：{elapsed_time:.1f} 秒内未到达目标位置")
    left_result = interface.left_arm_handler.check_arrival(pose_threshold=pose_threshold)
    right_result = interface.right_arm_handler.check_arrival(pose_threshold=pose_threshold)
    print(f"    左臂到达状态: {'✓ 已到达' if left_result['arrived'] else '✗ 未到达'}")
    print(f"    右臂到达状态: {'✓ 已到达' if right_result['arrived'] else '✗ 未到达'}")

    left_current_pose = interface.left_arm_handler.get_pose()
    right_current_pose = interface.right_arm_handler.get_pose()
    left_target_pose = interface.left_arm_handler.get_target_pose()
    right_target_pose = interface.right_arm_handler.get_target_pose()

    def print_pose_error(arm_name, current_pose, target_pose, result):
        if current_pose is None or target_pose is None:
            print(f"    {arm_name}误差详情: 无法获取当前或目标位姿")
            return

        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        dz = target_pose.position.z - current_pose.position.z
        pos_dist = result.get('position_distance', float('inf'))

        print(f"    {arm_name}位置误差: dx={dx:+.4f}m, dy={dy:+.4f}m, dz={dz:+.4f}m, |d|={pos_dist:.4f}m")
        print(
            f"      {arm_name}Z: 当前={current_pose.position.z:.4f}m, "
            f"目标={target_pose.position.z:.4f}m, 差值={dz:+.4f}m"
        )

    print_pose_error("左臂", left_current_pose, left_target_pose, left_result)
    print_pose_error("右臂", right_current_pose, right_target_pose, right_result)
    return False


def main():
    """测试 send_dual_arm_target_stamped() 功能"""
    
    print("\n" + "=" * 70)
    print(" " * 15 + "Dual Arm Target Stamped Test")
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
        print("   此测试需要双臂机器人配置。")
        print("   请确保机器人配置为 dual_arm_mode = true\n")
        interface.disconnect()
        return 1
    
    print("✓ 检测到双臂模式\n")
    
    # ========================================================================
    # 第二部分：获取当前位姿
    # ========================================================================
    print("-" * 70)
    print("[5] 获取当前双臂位姿")
    print("-" * 70)
    
    left_current_pose = interface.left_arm_handler.get_pose()
    right_current_pose = interface.right_arm_handler.get_pose()
    
    if not left_current_pose or not right_current_pose:
        print("⚠ 错误: 无法获取当前位姿！")
        print("   请确保机器人正在运行并发布位姿数据。\n")
        interface.disconnect()
        return 1
    
    print_pose(left_current_pose, "左臂当前位姿")
    print()
    print_pose(right_current_pose, "右臂当前位姿")
    print()
    
    # ========================================================================
    # 第三部分：切换到OCS2状态
    # ========================================================================
    print("-" * 70)
    print("[6] 切换到OCS2状态（用于测试运动控制）")
    print("-" * 70)
    
    # 先切换到HOLD状态
    print("  → 切换到HOLD状态...")
    interface.send_fsm_command(2)  # HOLD
    time.sleep(1.0)
    
    # 再切换到OCS2状态
    print("  → 切换到OCS2状态...")
    interface.send_fsm_command(3)  # OCS2
    time.sleep(1.0)
    print("  ✓ 已切换到OCS2状态\n")
    
    # ========================================================================
    # 第五部分：运动 + 动态修改 movel_duration 测试
    # ========================================================================
    print("=" * 70)
    print("[7] 运动并动态修改 movel_duration 测试")
    print("=" * 70)
    
    # 记录初始位置
    initial_left_x = left_current_pose.position.x
    initial_left_y = left_current_pose.position.y
    initial_left_z = left_current_pose.position.z
    initial_right_x = right_current_pose.position.x
    initial_right_y = right_current_pose.position.y
    initial_right_z = right_current_pose.position.z
    
    print(f"\n  初始位置:")
    print(f"    左臂X: {initial_left_x:.3f}m")
    print(f"    左臂Y: {initial_left_y:.3f}m")
    print(f"    左臂Z: {initial_left_z:.3f}m")
    print(f"    右臂X: {initial_right_x:.3f}m")
    print(f"    右臂Y: {initial_right_y:.3f}m")
    print(f"    右臂Z: {initial_right_z:.3f}m")
    print(f"  方块边长: 0.10m")
    print(f"  最大等待时间: {MAX_WAIT_TIME}秒")
    print(f"  到达阈值: {POSE_THRESHOLD}m")
    print(f"  左臂轨迹: 上 -> 右 -> 下 -> 左 -> 后 -> 上 -> 前 -> 下")
    print(f"  右臂轨迹: 上 -> 左 -> 下 -> 右 -> 后 -> 上 -> 前 -> 下")
    print(f"  追加旋转: 世界X轴 +45° -> +45° -> -45° -> -45°\n")
    
    # 参数配置（按需修改为你的实际控制器节点/参数）
    CONTROLLER_NODE_NAME = "/ocs2_arm_controller"
    MOVEL_DURATION_PARAM_NAME = "movel_duration"
    DEFAULT_TRANSLATION_DURATION = 2.5
    DEFAULT_ROTATION_DURATION = 3.5
    
    # 执行12次动作：
    # 1) 双臂在 Y-Z 平面画方块（左右镜像）
    # 2) 追加一组动作：向后 -> 向上 -> 向前 -> 向下
    # 3) 按世界坐标系 X 轴旋转：+45° -> +45° -> -45° -> -45°
    step_size = 0.1
    motion_steps = [
        # 原有方块轨迹（Y-Z平面）
        {
            "type": "translation",
            "left_dx": 0.0, "left_dy": 0.0, "left_dz": step_size,
            "right_dx": 0.0, "right_dy": 0.0, "right_dz": step_size,
            "left_action": "上", "right_action": "上",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        {
            "type": "translation",
            "left_dx": 0.0, "left_dy": -step_size, "left_dz": 0.0,
            "right_dx": 0.0, "right_dy": step_size, "right_dz": 0.0,
            "left_action": "右", "right_action": "左",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        {
            "type": "translation",
            "left_dx": 0.0, "left_dy": 0.0, "left_dz": -step_size,
            "right_dx": 0.0, "right_dy": 0.0, "right_dz": -step_size,
            "left_action": "下", "right_action": "下",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        {
            "type": "translation",
            "left_dx": 0.0, "left_dy": step_size, "left_dz": 0.0,
            "right_dx": 0.0, "right_dy": -step_size, "right_dz": 0.0,
            "left_action": "左", "right_action": "右",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        # 追加动作（X-Z平面）
        {
            "type": "translation",
            "left_dx": -step_size, "left_dy": 0.0, "left_dz": 0.0,
            "right_dx": -step_size, "right_dy": 0.0, "right_dz": 0.0,
            "left_action": "后", "right_action": "后",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        {
            "type": "translation",
            "left_dx": 0.0, "left_dy": 0.0, "left_dz": step_size,
            "right_dx": 0.0, "right_dy": 0.0, "right_dz": step_size,
            "left_action": "上", "right_action": "上",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        {
            "type": "translation",
            "left_dx": step_size, "left_dy": 0.0, "left_dz": 0.0,
            "right_dx": step_size, "right_dy": 0.0, "right_dz": 0.0,
            "left_action": "前", "right_action": "前",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        {
            "type": "translation",
            "left_dx": 0.0, "left_dy": 0.0, "left_dz": -step_size,
            "right_dx": 0.0, "right_dy": 0.0, "right_dz": -step_size,
            "left_action": "下", "right_action": "下",
            "duration": DEFAULT_TRANSLATION_DURATION,
        },
        # 追加姿态旋转（世界坐标系 X 轴）
        {
            "type": "rotation_world_x",
            "angle_deg": 45.0,
            "left_action": "绕世界X轴旋转 +45°",
            "right_action": "绕世界X轴旋转 +45°",
            "duration": DEFAULT_ROTATION_DURATION,
        },
        {
            "type": "rotation_world_x",
            "angle_deg": 45.0,
            "left_action": "绕世界X轴旋转 +45°",
            "right_action": "绕世界X轴旋转 +45°",
            "duration": DEFAULT_ROTATION_DURATION,
        },
        {
            "type": "rotation_world_x",
            "angle_deg": -45.0,
            "left_action": "绕世界X轴旋转 -45°",
            "right_action": "绕世界X轴旋转 -45°",
            "duration": DEFAULT_ROTATION_DURATION,
        },
        {
            "type": "rotation_world_x",
            "angle_deg": -45.0,
            "left_action": "绕世界X轴旋转 -45°",
            "right_action": "绕世界X轴旋转 -45°",
            "duration": DEFAULT_ROTATION_DURATION,
        },
    ]

    current_left_x = initial_left_x
    current_left_y = initial_left_y
    current_left_z = initial_left_z
    current_right_x = initial_right_x
    current_right_y = initial_right_y
    current_right_z = initial_right_z
    current_left_q = (
        left_current_pose.orientation.x,
        left_current_pose.orientation.y,
        left_current_pose.orientation.z,
        left_current_pose.orientation.w,
    )
    current_right_q = (
        right_current_pose.orientation.x,
        right_current_pose.orientation.y,
        right_current_pose.orientation.z,
        right_current_pose.orientation.w,
    )

    for step_count, step in enumerate(motion_steps, start=1):
        if step["type"] == "translation":
            current_left_x += step["left_dx"]
            current_left_y += step["left_dy"]
            current_left_z += step["left_dz"]
            current_right_x += step["right_dx"]
            current_right_y += step["right_dy"]
            current_right_z += step["right_dz"]
        elif step["type"] == "rotation_world_x":
            current_left_q = rotate_quaternion_world_x(current_left_q, step["angle_deg"])
            current_right_q = rotate_quaternion_world_x(current_right_q, step["angle_deg"])

        print(
            f"[7.{step_count}] 左臂{step['left_action']} / 右臂{step['right_action']} "
            f"（边长 {step_size:.2f}m）"
        )
        print("-" * 70)
        print(
            f"  左臂目标(X,Y,Z)=({current_left_x:.3f}, {current_left_y:.3f}, {current_left_z:.3f})m, "
            f"右臂目标(X,Y,Z)=({current_right_x:.3f}, {current_right_y:.3f}, {current_right_z:.3f})m"
        )
        print(
            f"  左臂目标姿态(qx,qy,qz,qw)=({current_left_q[0]:.3f}, {current_left_q[1]:.3f}, "
            f"{current_left_q[2]:.3f}, {current_left_q[3]:.3f}), "
            f"右臂目标姿态=({current_right_q[0]:.3f}, {current_right_q[1]:.3f}, "
            f"{current_right_q[2]:.3f}, {current_right_q[3]:.3f})"
        )

        # 每段运动开始前，设置对应的 MoveL 时长
        target_duration = float(step["duration"])
        print(f"  → 设置当前段 MoveL 时长: {target_duration:.2f}s")
        print(f"  → 设置参数: 节点={CONTROLLER_NODE_NAME}, {MOVEL_DURATION_PARAM_NAME}={target_duration}")
        success = interface.set_node_parameters(
            full_node_name=CONTROLLER_NODE_NAME,
            parameters={MOVEL_DURATION_PARAM_NAME: target_duration},
        )
        if success:
            print("  ✓ 参数设置成功")
        else:
            print("  ⚠ 参数设置失败")

        # 创建目标位姿（平移，姿态保持不变）
        left_target_pose = create_pose(
            x=current_left_x,
            y=current_left_y,
            z=current_left_z,
            qx=current_left_q[0],
            qy=current_left_q[1],
            qz=current_left_q[2],
            qw=current_left_q[3],
        )
        
        right_target_pose = create_pose(
            x=current_right_x,
            y=current_right_y,
            z=current_right_z,
            qx=current_right_q[0],
            qy=current_right_q[1],
            qz=current_right_q[2],
            qw=current_right_q[3],
        )
        
        # 获取 frame_id（优先使用左臂的 frame_id，如果不可用则使用右臂的）
        frame_id = interface.left_arm_handler.get_frame_id()
        if frame_id is None:
            frame_id = interface.right_arm_handler.get_frame_id()
        if frame_id is None:
            print(f"  ⚠ frame_id 尚未设置（可能还未收到 pose 消息），使用默认值 'arm_base'")
            frame_id = "arm_base"
        else:
            print(f"  → 使用 frame_id: {frame_id}")
        
        # 发送目标位姿
        try:
            interface.send_dual_arm_target_stamped(
                left_target_pose,
                right_target_pose,
                frame_id=frame_id
            )
            print(f"  ✓ 目标位姿已发送")
        except Exception as e:
            print(f"  ✗ 发送失败: {e}")
            break
        
        # 等待到达目标位置
        print(f"  → 等待到达目标位置（最大等待 {MAX_WAIT_TIME} 秒，阈值 {POSE_THRESHOLD}m）...")
        arrived = wait_for_arrival(interface, MAX_WAIT_TIME, CHECK_INTERVAL, POSE_THRESHOLD)
        
        if not arrived:
            print(f"  ⚠ 超时，继续下一步...")
        
        print()
    
    # ========================================================================
    # 第六部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[8] 测试完成，断开连接")
    print("=" * 70)
    
    # 切换回HOLD状态
    print("  → 切换回HOLD状态...")
    interface.send_fsm_command(2)  # HOLD
    time.sleep(1.0)
    
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

