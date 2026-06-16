"""
W2 机器人 ExecutePath 左右臂独立轨迹拼接测试脚本。

测试场景：
1. 第一段：使用老接口 execute_path() 同时给左右臂发送较长轨迹。
2. 第一段轨迹尚未结束时，使用新接口 execute_right_path() 只更新右臂。
3. 右臂新轨迹尚未结束时，使用新接口 execute_left_path() 只更新左臂。

预期观察：
- 未更新的手臂继续执行已有 active reference buffer。
- 对应 buffer 结束后，未更新的手臂保持该手臂当前 reference 终点。
- 新更新的手臂从控制器接收时刻开始拼接执行新轨迹。
"""

import sys
import time

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def vector_to_pose(vector):
    """将7维向量 [x, y, z, qx, qy, qz, qw] 转换为 Pose 对象。"""
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
    """将多个7维向量转换为 PoseStamped 列表。"""
    poses = []
    for vec in vectors:
        pose_stamped = PoseStamped()
        pose_stamped.header = Header(frame_id=frame_id)
        pose_stamped.pose = vector_to_pose(vec)
        poses.append(pose_stamped)
    return poses


def wait_for_arrival(interface, max_wait_time=30.0, check_interval=0.5):
    """等待双臂到达当前目标位置。"""
    start_time = time.time()

    while time.time() - start_time < max_wait_time:
        left_result = interface.left_arm_handler.check_arrival()
        right_result = interface.right_arm_handler.check_arrival()

        if left_result["arrived"] and right_result["arrived"]:
            elapsed_time = time.time() - start_time
            print(f"  ✓ 双臂均已到达当前目标位置（耗时 {elapsed_time:.1f} 秒）")
            return True

        print(
            "  → 到达状态: "
            f"左臂={'✓' if left_result['arrived'] else '✗'}, "
            f"右臂={'✓' if right_result['arrived'] else '✗'}"
        )
        time.sleep(check_interval)

    elapsed_time = time.time() - start_time
    print(f"  ⚠ 超时：{elapsed_time:.1f} 秒内未到达当前目标位置")
    left_result = interface.left_arm_handler.check_arrival()
    right_result = interface.right_arm_handler.check_arrival()
    print(f"    左臂到达状态: {'✓ 已到达' if left_result['arrived'] else '✗ 未到达'}")
    print(f"    右臂到达状态: {'✓ 已到达' if right_result['arrived'] else '✗ 未到达'}")
    return False


def send_dual_execute_path(interface, title, left_vectors, right_vectors, frame_id, trajectory_duration):
    """使用老接口同时发送左右臂 ExecutePath 请求。"""
    print("-" * 70)
    print(title)
    print("-" * 70)

    left_poses = vectors_to_poses(left_vectors, frame_id=frame_id)
    right_poses = vectors_to_poses(right_vectors, frame_id=frame_id)

    print("  → 发送 execute_path 请求...")
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
        print("  ✓ execute_path 服务调用成功")
    else:
        print("  ⚠ execute_path 服务返回失败")

    return success


def send_single_arm_execute_path(interface, title, arm, vectors, frame_id, trajectory_duration):
    """使用新便捷接口只更新单侧手臂。"""
    print("-" * 70)
    print(title)
    print("-" * 70)

    poses = vectors_to_poses(vectors, frame_id=frame_id)

    print(f"  → 发送 execute_{arm}_path 请求...")
    print(f"    {arm}: {len(poses)} 个路径点")
    print("    未更新侧: 继续 active reference buffer，结束后保持当前 reference 终点")
    print(f"    trajectory_duration: {trajectory_duration:.2f} 秒")

    if arm == "left":
        success = interface.execute_left_path(
            poses,
            trajectory_duration=trajectory_duration,
            frame_id=frame_id,
        )
    elif arm == "right":
        success = interface.execute_right_path(
            poses,
            trajectory_duration=trajectory_duration,
            frame_id=frame_id,
        )
    else:
        raise ValueError(f"unsupported arm: {arm}")

    if success:
        print(f"  ✓ execute_{arm}_path 服务调用成功")
    else:
        print(f"  ⚠ execute_{arm}_path 服务返回失败")

    return success


def main():
    """测试 execute_left_path()/execute_right_path() 的独立轨迹拼接行为。"""
    first_trajectory_duration = 15.0
    right_update_delay = 5.0
    right_trajectory_duration = 4.0
    left_update_delay = 4.0
    left_trajectory_duration = 4.0
    max_wait_time = 30.0
    check_interval = 0.5

    print("\n" + "=" * 70)
    print(" " * 5 + "W2 Robot ExecutePath Independent-Arms Stitching Test")
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

    try:
        is_dual_arm = interface.config.right_end_effector_target_topic is not None
        if not is_dual_arm:
            print("    ✗ 错误: 此测试需要双臂模式，但未检测到右臂 topic\n")
            return 1
        print("    ✓ 检测到双臂模式\n")

        print("[4] 等待数据到达（2秒）...")
        time.sleep(2.0)
        print("    ✓ 数据收集已开始\n")

        print("[5] 切换到 OCS2 状态...")
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        interface.send_fsm_command(1)  # HOME
        time.sleep(5.0)
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        interface.send_fsm_command(3)  # OCS2/MOVE
        time.sleep(2.0)
        print("    ✓ 已切换到 OCS2 状态\n")

        first_left_path = [
            [0.6033803016355871, 0.41375307563578423, 0.29383709977844447,
             0.7144551252772559, 0.0003579242251742329, 0.6996737302328488, -0.0032275497456205203],
        ]
        first_right_path = [
            [0.5998637402562574, -0.4141385330426541, 0.11121182513583987,
             0.7153043563469663, -0.0011204641839848684, 0.6988029862372196, 0.0035789351780803175],
        ]

        right_only_path = [
            [0.3149393469605365, -0.6721904408017985, -0.06838125560927891,
             0.6968516693226728, -0.0014288331543221367, 0.7172028223615105, 0.0039775614018035055],
            [0.3068348123140658, -0.6729626894911632, -0.3581926217176511,
             0.7161100088042789, -0.0025534145339008597, 0.6979669755618071, 0.0046942933075830124],
            [0.3091508507547196, -0.3108020358673747, -0.3596782703940628,
             0.7149836818336867, -0.000854224497635221, 0.6991333785044492, 0.0031818348492363676],
        ]

        left_only_path = [
            [0.3587000970448418, 0.5683818668210182, 0.1307366052513367,
             0.6435602184806616, -0.053066362333830365, 0.7624722236724825, -0.04062406313648813],
            [0.2990542934121199, 0.5626040069162731, -0.1758828399206876,
             0.713803797669393, 0.0011934292676035582, 0.7003355052041234, -0.0035908647733899827],
        ]

        frame_id = interface.left_arm_handler.get_frame_id()
        if frame_id is None:
            frame_id = interface.right_arm_handler.get_frame_id()
        if frame_id is None:
            frame_id = "arm_base"
            print("  ⚠ frame_id 未检测到，使用默认值 arm_base")
        else:
            print(f"  → 使用 frame_id: {frame_id}")

        ok = send_dual_execute_path(
            interface,
            "[6] 第一段：老接口同时发送左右臂长轨迹",
            first_left_path,
            first_right_path,
            frame_id,
            first_trajectory_duration,
        )
        if not ok:
            return 1

        print(
            f"\n[7] 等待 {right_update_delay:.1f} 秒，确保第一段轨迹仍在执行中 "
            f"（第一段 duration={first_trajectory_duration:.1f}s）..."
        )
        time.sleep(right_update_delay)

        ok = send_single_arm_execute_path(
            interface,
            "[8] 第二段：新接口只更新右臂，左臂继续旧 reference",
            "right",
            right_only_path,
            frame_id,
            right_trajectory_duration,
        )
        if not ok:
            return 1

        print(
            f"\n[9] 等待 {left_update_delay:.1f} 秒，确保右臂新轨迹仍在执行中 "
            f"（右臂 duration={right_trajectory_duration:.1f}s）..."
        )
        time.sleep(left_update_delay)

        ok = send_single_arm_execute_path(
            interface,
            "[10] 第三段：新接口只更新左臂，右臂继续已有 reference",
            "left",
            left_only_path,
            frame_id,
            left_trajectory_duration,
        )
        if not ok:
            return 1

        time.sleep(2.0)
        # interface.send_dual_arm_target_stamped(
        #     left_pose=vector_to_pose(left_only_path[0]),
        #     right_pose=vector_to_pose(right_only_path[0]),
        #     frame_id=frame_id,
        # )
        # interface.left_arm_handler.send_target_stamped(vector_to_pose(left_only_path[0]))
        # interface.left_arm_handler.send_target(vector_to_pose(left_only_path[0]))


        print("\n[11] 等待独立拼接后的当前目标到达...")
        wait_for_arrival(interface, max_wait_time, check_interval)

        print("=" * 70)
        print("[12] 测试完成")
        print("=" * 70)
        return 0
    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        print("\n[结束] 断开连接...")
        try:
            interface.disconnect()
            print("  ✓ 已断开连接")
        except Exception as e:
            print(f"  ⚠ 断开连接时出错: {e}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n用户中断测试")
        sys.exit(1)
