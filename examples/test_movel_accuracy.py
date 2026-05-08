#!/usr/bin/env python3
import sys
import time
import math

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.quat_pose import quat_multiply, rotate_vector_by_quat


left_arm_joint_names = [
            "left_joint1", "left_joint2", "left_joint3", "left_joint4",
            "left_joint5", "left_joint6", "left_joint7"]

right_arm_joint_names = [
            "right_joint1", "right_joint2", "right_joint3", "right_joint4",
            "right_joint5", "right_joint6", "right_joint7"]

left_target_pose_list = [(0.35, 0.4, -0.4, 0.707, 0.0, 0.707, 0.0),
                         (0.32, 0.43, -0.42, 0.6, 0.2, 0.6, 0.2)]

left_joints_list = [[1.45, -1.06, -1.19, -0.80, -0.69, -0.68, -0.12,],
                    [1.55, -1.06, -1.39, -0.30, -0.79, -0.68, -0.12,]]

BASE_FRAME = "arm_base"
LEFT_EEF_FRAME = "left_eef"
LEFT_TCP_FRAME = "left_tcp"

def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose

def format_pose(pose: Pose) -> str:
    return (
        f"pos=({pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}), "
        f"quat=({pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
        f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f})"
    )

def convert_eef_pose_in_base_to_tcp_pose_in_base(
    eef_pose_in_base: Pose,
    tcp_in_eef_translation: tuple[float, float, float],
    tcp_in_eef_quaternion: tuple[float, float, float, float],
) -> Pose:
    eef_quaternion = (
        eef_pose_in_base.orientation.x,
        eef_pose_in_base.orientation.y,
        eef_pose_in_base.orientation.z,
        eef_pose_in_base.orientation.w,
    )
    offset_in_base = rotate_vector_by_quat(tcp_in_eef_translation, eef_quaternion)
    tcp_pose_in_base = Pose()
    tcp_pose_in_base.position.x = eef_pose_in_base.position.x + offset_in_base[0]
    tcp_pose_in_base.position.y = eef_pose_in_base.position.y + offset_in_base[1]
    tcp_pose_in_base.position.z = eef_pose_in_base.position.z + offset_in_base[2]
    tcp_quaternion = quat_multiply(eef_quaternion, tcp_in_eef_quaternion)
    tcp_pose_in_base.orientation = Quaternion(
        x=tcp_quaternion[0],
        y=tcp_quaternion[1],
        z=tcp_quaternion[2],
        w=tcp_quaternion[3],
    )
    return tcp_pose_in_base

def pose_position_distance(pose_a: Pose, pose_b: Pose) -> float:
    dx = pose_a.position.x - pose_b.position.x
    dy = pose_a.position.y - pose_b.position.y
    dz = pose_a.position.z - pose_b.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def pose_orientation_distance(pose_a: Pose, pose_b: Pose) -> float:
    q_a = pose_a.orientation
    q_b = pose_b.orientation
    norm_a = math.sqrt(q_a.x * q_a.x + q_a.y * q_a.y + q_a.z * q_a.z + q_a.w * q_a.w)
    norm_b = math.sqrt(q_b.x * q_b.x + q_b.y * q_b.y + q_b.z * q_b.z + q_b.w * q_b.w)
    if norm_a == 0.0 or norm_b == 0.0:
        raise ValueError("姿态四元数不能为零")

    dot = (
        q_a.x * q_b.x
        + q_a.y * q_b.y
        + q_a.z * q_b.z
        + q_a.w * q_b.w
    ) / (norm_a * norm_b)
    dot = min(1.0, max(-1.0, abs(dot)))
    return 2.0 * math.acos(dot)

def main():
    rclpy.init()

    interface = None
    try:
        print("\n[1] 创建配置和 ROS2RobotInterface 实例...")
        config = ROS2RobotInterfaceConfig()
        interface = ROS2RobotInterface(config)

        print("[2] 连接到 ROS 2...")
        interface.connect()
        print("    接口连接成功")

        if interface.config.right_end_effector_target_topic is None:
            print("    错误: 此测试需要双臂模式")
            return 1

        print("[3] 等待数据到达（1秒）...")
        time.sleep(1.0)

        print("\n[4] 发送直线 action...")
        duration = 3.0
        interface.set_node_parameters(
            full_node_name="/ocs2_arm_controller",
            parameters={"movej_duration": duration},
        )
        tcp_in_eef_transform = interface.lookup_transform(
            LEFT_EEF_FRAME,
            LEFT_TCP_FRAME,
            timeout=1.0,
        )
        if tcp_in_eef_transform is None:
            print(f"    错误: 无法查询 {LEFT_TCP_FRAME} 在 {LEFT_EEF_FRAME} 下的固定变换")
            return 1
        tcp_in_eef_translation = (
            tcp_in_eef_transform.transform.translation.x,
            tcp_in_eef_transform.transform.translation.y,
            tcp_in_eef_transform.transform.translation.z,
        )
        tcp_in_eef_quaternion = (
            tcp_in_eef_transform.transform.rotation.x,
            tcp_in_eef_transform.transform.rotation.y,
            tcp_in_eef_transform.transform.rotation.z,
            tcp_in_eef_transform.transform.rotation.w,
        )
        print(
            f"    固定变换 {LEFT_TCP_FRAME} in {LEFT_EEF_FRAME}: "
            f"trans={tcp_in_eef_translation}, quat={tcp_in_eef_quaternion}"
        )
        
        for index, (p, joints) in enumerate(zip(left_target_pose_list, left_joints_list), start=1):
            print(f"\n[5.{index}] 执行第 {index} 组测试")
            print(f"    MoveJ joints: {joints}")
            interface.left_arm_handler.send_joint_positions(joints)
            time.sleep(duration + 1.0)
            left_target_pose = create_pose(*p)
            # print(f"    left_target_pose ({BASE_FRAME} 下的 {LEFT_EEF_FRAME}): {format_pose(left_target_pose)}")
            left_target_pose_tcp = convert_eef_pose_in_base_to_tcp_pose_in_base(
                left_target_pose,
                tcp_in_eef_translation,
                tcp_in_eef_quaternion,
            )
            # print(f"    left_target_pose_tcp ({BASE_FRAME} 下的 {LEFT_TCP_FRAME}): {format_pose(left_target_pose_tcp)}")
            result = interface.execute_movel_action(
                "left",
                left_target_pose_tcp,
                duration=duration,
                time_mode=True,
                frame_id=BASE_FRAME,
            )
            if not result or not result.success:
                print(f"    MOVL 执行失败, result={result}")
                return 1
            time.sleep(0.2)
            left_final_pose = interface.left_arm_handler.get_pose()
            if left_final_pose is None:
                print("    错误: 未收到 left_final_pose")
                return 1
            # print(f"    left_final_pose (订阅话题): {format_pose(left_final_pose)}")
            left_distance = pose_position_distance(left_final_pose, left_target_pose)
            left_rotation_distance = pose_orientation_distance(left_final_pose, left_target_pose)
            print(
                f"    left_final_pose 与 left_target_pose "
                f"({BASE_FRAME} 下的 {LEFT_EEF_FRAME}) 的位置距离: {left_distance:.6f} m"
            )
            print(
                f"    left_final_pose 与 left_target_pose ({BASE_FRAME} 下的 {LEFT_EEF_FRAME}) 的旋转差距: "
                f"{left_rotation_distance:.6f} rad ({math.degrees(left_rotation_distance):.3f} deg)"
            )
        return 0

    except Exception as exc:
        print(f"执行失败: {exc}")
        return 1
    finally:
        if interface is not None:
            try:
                interface.send_fsm_command(2)
                interface.disconnect()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
