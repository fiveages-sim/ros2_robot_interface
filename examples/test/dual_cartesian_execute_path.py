"""
W2 机器人 ExecutePath 测试脚本
参考 dual_cartesian_path.py，改为使用 execute_path() 服务接口。
"""

import sys
import time
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


def wait_for_arrival(interface, max_wait_time=30.0, check_interval=0.5):
    """等待双臂到达目标位置。"""
    start_time = time.time()

    while time.time() - start_time < max_wait_time:
        left_result = interface.left_arm_handler.check_arrival()
        right_result = interface.right_arm_handler.check_arrival()

        if left_result["arrived"] and right_result["arrived"]:
            elapsed_time = time.time() - start_time
            print(f"  ✓ 双臂均已到达目标位置（耗时 {elapsed_time:.1f} 秒）")
            return True

        time.sleep(check_interval)

    elapsed_time = time.time() - start_time
    print(f"  ⚠ 超时：{elapsed_time:.1f} 秒内未到达目标位置")
    left_result = interface.left_arm_handler.check_arrival()
    right_result = interface.right_arm_handler.check_arrival()
    print(f"    左臂到达状态: {'✓ 已到达' if left_result['arrived'] else '✗ 未到达'}")
    print(f"    右臂到达状态: {'✓ 已到达' if right_result['arrived'] else '✗ 未到达'}")
    return False


def execute_segment(
    interface,
    title,
    left_vectors,
    right_vectors,
    frame_id,
    max_wait_time,
    check_interval,
    trajectory_duration=0.0,
):
    """执行单段双臂路径。"""
    print("-" * 70)
    print(title)
    print("-" * 70)

    try:
        left_poses = vectors_to_poses(left_vectors, frame_id=frame_id)
        right_poses = vectors_to_poses(right_vectors, frame_id=frame_id)

        print("  → 发送 ExecutePath 请求...")
        print(f"    左臂: {len(left_poses)} 个路径点")
        print(f"    右臂: {len(right_poses)} 个路径点")
        print(f"    trajectory_duration: {trajectory_duration:.2f} 秒")

        success = interface.execute_path(
            left_poses,
            right_poses,
            trajectory_duration=trajectory_duration,
            frame_id=frame_id,
        )

        if success:
            print("  ✓ ExecutePath 服务调用成功")
        else:
            print("  ⚠ ExecutePath 服务返回失败")

        print(f"  → 等待轨迹执行完成（最大等待 {max_wait_time} 秒）...")
        arrived = wait_for_arrival(interface, max_wait_time, check_interval)
        if arrived:
            print("  ✓ 轨迹执行完成（左右臂均已到达）\n")
        else:
            print("  ⚠ 轨迹执行完成（部分或全部未到达）\n")
    except Exception as e:
        print(f"  ✗ 执行轨迹失败: {e}\n")


def main():
    """测试 W2 机器人的 ExecutePath 接口。"""
    MAX_WAIT_TIME = 30.0
    CHECK_INTERVAL = 0.5
    TRAJECTORY_DURATION = 0.0

    print("\n" + "=" * 70)
    print(" " * 16 + "W2 Robot ExecutePath Test")
    print("=" * 70 + "\n")

    print("[1] 创建配置...")
    config = ROS2RobotInterfaceConfig()

    print("[2] 创建 ROS2RobotInterface 实例...")
    interface = ROS2RobotInterface(config)

    print("[3] 连接到 ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功!\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1

    is_dual_arm = interface.config.right_end_effector_target_topic is not None
    if not is_dual_arm:
        print("    ✗ 错误: 此测试需要双臂模式，但未检测到右臂 topic\n")
        interface.disconnect()
        return 1
    print("    ✓ 检测到双臂模式\n")

    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")

    print("[5] 切换到 OCS2 状态...")
    try:
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        interface.send_fsm_command(1)  # HOME
        time.sleep(5.0)
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        interface.send_fsm_command(3)  # OCS2/MOVE
        time.sleep(2.0)
        print("    ✓ 已切换到 OCS2 状态\n")
    except Exception as e:
        print(f"    ✗ 切换到 OCS2 状态失败: {e}\n")
        interface.disconnect()
        return 1

    # A点轨迹数据
    a_left_initial = [
        [0.243304, 0.287482, -0.166317, 0.669167, -0.144600, 0.714637, 0.143530],
        [0.464421, 0.188690, -0.078523, 0.669699, -0.139977, 0.715802, 0.139776],
    ]
    a_left_end = [
        [0.464421, 0.188690, -0.018523, 0.669699, -0.139977, 0.715802, 0.139776],
        [0.243304, 0.287482, -0.166317, 0.669167, -0.144600, 0.714637, 0.143530],
    ]
    a_right_initial = [
        [0.243304, -0.287482, -0.166317, 0.669167, 0.144600, 0.714637, -0.143530],
        [0.464421, -0.188690, -0.078523, 0.669699, 0.139977, 0.715802, -0.139776],
    ]
    a_right_end = [
        [0.464421, -0.188690, -0.018523, 0.669699, 0.139977, 0.715802, -0.139776],
        [0.243304, -0.287482, -0.166317, 0.669167, 0.144600, 0.714637, -0.143530],
    ]

    # B点轨迹数据
    b_left_initial = [
        [0.468027, 0.254521, -0.090424, 0.583926, 0.063293, 0.808647, 0.033391],
        [0.634115, 0.274643, 0.018583, 0.588553, 0.004895, 0.808427, -0.005291],
    ]
    b_left_end = [
        [0.655319, 0.276373, 0.036224, 0.700671, 0.022995, 0.713072, -0.007775],
        [0.654674, 0.331464, -0.028880, 0.485272, 0.506258, 0.515121, 0.492814],
        [0.385897, 0.444166, -0.046988, 0.502312, 0.483387, 0.521046, 0.492474],
    ]
    b_right_initial = [
        [0.468027, -0.254521, -0.090424, 0.583926, -0.063293, 0.808647, -0.033391],
        [0.634115, -0.274643, 0.018583, 0.588553, -0.004895, 0.808427, 0.005291],
    ]
    b_right_end = [
        [0.655319, -0.276373, 0.036224, 0.700671, -0.022995, 0.713072, 0.007775],
        [0.654674, -0.311464, -0.028880, 0.485272, -0.506258, 0.515121, -0.492814],
        [0.385897, -0.444166, -0.046988, 0.502312, -0.483387, 0.521046, -0.492474],
    ]

    frame_id = interface.left_arm_handler.get_frame_id()
    if frame_id is None:
        frame_id = interface.right_arm_handler.get_frame_id()
    if frame_id is None:
        frame_id = "arm_base"
        print("  ⚠ frame_id 未检测到，使用默认值 arm_base")
    else:
        print(f"  → 使用 frame_id: {frame_id}")

    execute_segment(
        interface,
        "[6] 仅左臂路径（左2点，右0点）",
        a_left_initial,
        [],
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=1.0,
    )
    time.sleep(1.5)

    execute_segment(
        interface,
        "[7] 仅右臂路径（左0点，右2点）",
        [],
        a_right_initial,
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=2.5,
    )
    time.sleep(1.5)

    execute_segment(
        interface,
        "[8] A点 Initial 路径（左2点，右1点）",
        a_left_initial,
        [a_right_initial[0]],
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=1.0,
    )
    time.sleep(1.0)

    execute_segment(
        interface,
        "[9] A点 End 路径（左2点，右2点）",
        a_left_end,
        a_right_end,
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=2.5,
    )
    time.sleep(2.0)

    execute_segment(
        interface,
        "[10] B点 Initial 路径（左2点，右1点）",
        b_left_initial,
        [b_right_initial[0]],
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=4.0,
    )
    time.sleep(1.0)

    execute_segment(
        interface,
        "[11] B点 End 路径（左3点，右3点）",
        b_left_end,
        b_right_end,
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=6.0,
    )
    time.sleep(2.0)

    execute_segment(
        interface,
        "[12] 不等数量路径点（左3点，右2点）",
        b_left_end,
        b_right_end[:2],
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=8.0,
    )
    time.sleep(2.0)

    execute_segment(
        interface,
        "[13] A点 Initial 路径（左1点，右2点）",
        [a_left_initial[0]],
        a_right_initial,
        frame_id,
        MAX_WAIT_TIME,
        CHECK_INTERVAL,
        trajectory_duration=10.0,
    )

    print("=" * 70)
    print("[14] 测试完成，开始断开连接")
    print("=" * 70)
    time.sleep(2.0)

    try:
        interface.disconnect()
        print("  ✓ 已断开连接")
    except Exception as e:
        print(f"  ⚠ 断开连接时出错: {e}")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n未预期的错误: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
