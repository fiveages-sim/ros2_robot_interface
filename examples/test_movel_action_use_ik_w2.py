#!/usr/bin/env python3
"""
直线运动 ROS2RobotInterface 示例
使用运动学测试脚本中的关节角度，先 MoveJ 到目标关节角度，
再读取当前位姿，计算 Z 轴移动 -0.2m 后的目标位姿，
最后通过 ROS2RobotInterface.execute_movel_action() 执行 MOVL。

Action: /ocs2_arm_controller/execute_linear, /ocs2_arm_controller/joint_trajectory_with_para
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


LEFT_TEST_JOINTS = [
    -0.3525227330, -0.7798290600, 0.8896949257,
    -1.8910405790, -2.7986485415, 0.4619120915, 0.8030297356,
]

RIGHT_TEST_JOINTS = [
    0.3525227330, -0.7798290600, -0.8896949257,
    -1.8910405790, 2.7986485415, 0.4619120915, -0.8030297356,
]


class LinearTrajectoryActionClient(Node):
    def __init__(self, interface=None):
        super().__init__("linear_trajectory_action_client")
        self.interface = interface

        self.left_arm_joint_names = [
            "left_joint1", "left_joint2", "left_joint3", "left_joint4",
            "left_joint5", "left_joint6", "left_joint7",
        ]
        self.right_arm_joint_names = [
            "right_joint1", "right_joint2", "right_joint3", "right_joint4",
            "right_joint5", "right_joint6", "right_joint7",
        ]

    def wait_for_servers(self, timeout=10.0):
        linear_ready = self.interface.wait_for_movel_action_server(timeout=timeout)
        trajectory_ready = self.interface.wait_for_joint_trajectory_action_server(timeout=timeout)
        if linear_ready and trajectory_ready:
            self.get_logger().info("所有服务/action 可用")
            return True

        self.get_logger().error(
            f"不可用 - 直线action: {linear_ready}, 轨迹action: {trajectory_ready}"
        )
        return False

    def send_movej_command(self, arm_name, joint_positions, duration=3.0):
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            self.get_logger().error("无法获取当前关节位置")
            return False

        if arm_name == "left":
            dual_joint_positions = joint_positions + current_positions["right"]
        else:
            dual_joint_positions = current_positions["left"] + joint_positions

        return self.send_dual_movej_command(
            dual_joint_positions[:7],
            dual_joint_positions[7:],
            duration,
        )

    def send_dual_movej_command(self, left_joints, right_joints, duration=3.0):
        self.get_logger().info("发送双臂 MoveJ action goal...")
        result = self.interface.execute_dual_arm_movej_action(
            left_joints,
            right_joints,
            duration=duration,
            time_mode=True,
            left_joint_names=self.left_arm_joint_names,
            right_joint_names=self.right_arm_joint_names,
            max_velocity=0.5,
            max_acceleration=1.0,
            max_jerk=2.0,
            feedback_callback=self.movej_feedback_callback,
            timeout=max(duration + 10.0, 30.0),
        )

        if result and result.success:
            self.get_logger().info(
                f"MoveJ 成功，消息: {result.message}, "
                f"规划时长: {result.planned_duration:.3f} 秒, "
                f"实际时长: {result.actual_duration:.3f} 秒"
            )
            return True

        self.get_logger().error(
            "MoveJ 失败"
            + (f"，消息: {result.message}" if result else "")
        )
        return False

    def movej_feedback_callback(self, feedback):
        # self.get_logger().info(
        #     f"MoveJ action反馈: 进度 {feedback.progress * 100.0:.1f}%, "
        #     f"已用 {feedback.elapsed_time:.2f}s, 剩余 {feedback.remaining_time:.2f}s"
        # )
        pass

    def get_current_joint_positions(self):
        if self.interface is None:
            return None

        joint_state = self.interface.get_joint_state(categorized=False)
        if joint_state is None:
            return None

        joint_name_to_position = dict(zip(
            joint_state.get("names", []),
            joint_state.get("positions", []),
        ))
        left_positions = [joint_name_to_position.get(name, 0.0) for name in self.left_arm_joint_names]
        right_positions = [joint_name_to_position.get(name, 0.0) for name in self.right_arm_joint_names]
        return {"left": left_positions, "right": right_positions}

    def get_current_pose(self, arm_name):
        if self.interface is None:
            return None
        handler = self.interface.left_arm_handler if arm_name == "left" else self.interface.right_arm_handler
        return handler.get_pose() if handler is not None else None


def build_offset_pose(source_pose, z_offset=-0.2):
    if source_pose is None:
        raise ValueError("当前位姿不可用")
    target_pose = Pose()
    target_pose.position.x = source_pose.position.x
    target_pose.position.y = source_pose.position.y
    target_pose.position.z = source_pose.position.z + z_offset
    target_pose.orientation = source_pose.orientation
    return target_pose


def format_pose(pose):
    if pose is None:
        return "不可用"
    return (
        f"position=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}), "
        f"orientation=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
        f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
    )


def print_current_pose(client, arm_name, label):
    current_pose = client.get_current_pose(arm_name)
    print(f"    {label}当前位姿: {format_pose(current_pose)}")


def print_action_feedback(feedback):
    # print(
    #     f"    action反馈: 进度 {feedback.progress * 100.0:.1f}%, "
    #     f"已用 {feedback.elapsed_time:.2f}s, 剩余 {feedback.remaining_time:.2f}s"
    # )
    pass


def main():
    print("=" * 70)
    print("直线运动 Action 测试 - 使用运动学测试脚本中的关节角度")
    print("步骤:")
    print("  1. MoveJ 移动到目标关节角度")
    print("  2. 读取当前位姿")
    print("  3. 计算 Z 轴下移 0.2m 后的目标位姿")
    print("  4. 通过 ROS2RobotInterface.execute_movel_action() 执行 MOVL")
    print("=" * 70)

    rclpy.init()

    interface = None
    client = None
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

        client = LinearTrajectoryActionClient(interface)

        print("[4] 等待服务和 action server...")
        if not client.wait_for_servers():
            return 1

        print("\n[5] 切换到 HOLD")
        interface.send_fsm_command(2)
        print("    状态切换完成")

        print("\n[6] 选择测试模式:")
        print("1. 左臂 - MoveJ 到目标关节 -> action 直线 Z 轴下移 0.2m")
        print("2. 右臂 - MoveJ 到目标关节 -> action 直线 Z 轴下移 0.2m")
        print("3. 双臂 - 同时 MoveJ 到目标关节 -> 单次 action 同时执行双臂直线运动")
        choice = input("请选择(1-3): ").strip()

        if choice not in ["1", "2", "3"]:
            print("无效选择")
            return 1

        if choice == "1":
            print("\n[7] MoveJ 移动到左臂目标关节角度...")
            if not client.send_movej_command("left", LEFT_TEST_JOINTS, duration=4.0):
                return 1
        elif choice == "2":
            print("\n[7] MoveJ 移动到右臂目标关节角度...")
            if not client.send_movej_command("right", RIGHT_TEST_JOINTS, duration=4.0):
                return 1
        else:
            print("\n[7] 双臂同时 MoveJ 到目标关节角度...")
            if not client.send_dual_movej_command(LEFT_TEST_JOINTS, RIGHT_TEST_JOINTS, duration=5.0):
                return 1

        print("\n[8] 发送直线 action...")
        duration = 3.0
        if choice == "1":
            left_target_pose = build_offset_pose(client.get_current_pose("left"), z_offset=-0.2)
            print(
                f"    左臂目标: ({left_target_pose.position.x:.4f}, "
                f"{left_target_pose.position.y:.4f}, {left_target_pose.position.z:.4f})"
            )
            result = interface.execute_movel_action(
                "left",
                left_target_pose,
                duration=duration,
                time_mode=True,
                max_linear_velocity=0.2,
                max_linear_acceleration=0.3,
                max_linear_jerk=2.0,
                max_angular_velocity=0.5,
                max_angular_acceleration=1.0,
                max_angular_jerk=3.0,
                feedback_callback=print_action_feedback,
            )
            if not result or not result.success:
                return 1
            time.sleep(0.2)
            print_current_pose(client, "left", "左臂")
        elif choice == "2":
            right_target_pose = build_offset_pose(client.get_current_pose("right"), z_offset=-0.2)
            print(
                f"    右臂目标: ({right_target_pose.position.x:.4f}, "
                f"{right_target_pose.position.y:.4f}, {right_target_pose.position.z:.4f})"
            )
            result = interface.execute_movel_action(
                "right",
                right_target_pose,
                duration=duration,
                time_mode=True,
                max_linear_velocity=0.2,
                max_linear_acceleration=0.3,
                max_linear_jerk=2.0,
                max_angular_velocity=0.5,
                max_angular_acceleration=1.0,
                max_angular_jerk=3.0,
                feedback_callback=print_action_feedback,
            )
            if not result or not result.success:
                return 1
            time.sleep(0.2)
            print_current_pose(client, "right", "右臂")
        else:
            left_target_pose = build_offset_pose(client.get_current_pose("left"), z_offset=-0.2)
            right_target_pose = build_offset_pose(client.get_current_pose("right"), z_offset=-0.2)
            print(
                f"    左臂目标: ({left_target_pose.position.x:.4f}, "
                f"{left_target_pose.position.y:.4f}, {left_target_pose.position.z:.4f})"
            )
            print(
                f"    右臂目标: ({right_target_pose.position.x:.4f}, "
                f"{right_target_pose.position.y:.4f}, {right_target_pose.position.z:.4f})"
            )
            dual_result = interface.execute_movel_action(
                "both",
                left_target_pose,
                duration=duration,
                right_endpoint_pose=right_target_pose,
                time_mode=True,
                max_linear_velocity=0.2,
                max_linear_acceleration=0.3,
                max_linear_jerk=2.0,
                max_angular_velocity=0.5,
                max_angular_acceleration=1.0,
                max_angular_jerk=3.0,
                feedback_callback=print_action_feedback,
            )
            if not dual_result or not dual_result.success:
                return 1
            time.sleep(0.2)
            print_current_pose(client, "left", "左臂")
            print_current_pose(client, "right", "右臂")

        print("\n直线 action 执行成功")
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
        if client is not None:
            client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
