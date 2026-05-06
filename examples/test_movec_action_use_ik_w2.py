#!/usr/bin/env python3
"""
圆弧运动 ROS2RobotInterface 示例
先 MoveJ 到圆弧起点关节角度，再通过 ROS2RobotInterface 执行 MOVEC。

Action: /ocs2_arm_controller/execute_circle_use_ik, /ocs2_arm_controller/joint_trajectory_with_para
支持两种圆弧定义方式:
  - 三点法: 起点(当前位姿) + 中间点 + 终点 (坐标硬编码)
  - 参数法: 圆心 + 旋转轴 + 旋转角度 (坐标硬编码, 终点姿态由当前位姿获取)
"""

import math
import sys
import time

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.node import Node

import rclpy

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 圆弧起点关节角度
LEFT_TEST_JOINTS = [
    1.5750129732309583,
    -0.7400602676933482,
    -1.1072540195779,
    -1.9906593861250388,
    -0.7295060947788113,
    -0.009940335195422034,
    0.6198394754996032,
]
RIGHT_TEST_JOINTS = [
    -1.5750129732309583,
    -0.7400602676933482,
    1.1072540195779,
    -1.9906593861250388,
    0.7295060947788113,
    -0.009940335195422034,
    -0.6198394754996032,
]

# 三点法参数: (x, y, z, qw, qx, qy, qz)
LEFT_THREE_POINT_MIDPOINT  = ( 
     0.3704625618078148,
    0.414126374455314,
    -0.29923553466780695,
    0.0032539334651132804,
    0.7149622341901515,
    -0.0009362579244783191,
    0.699154874845288,
)
LEFT_THREE_POINT_ENDPOINT  = ( 
    0.36712537577707993,
    0.3318733303285971,
    -0.4520389847935645,
    0.0033245439162966402,
    0.71558932020881,
    -0.0008963509947116495,
    0.6985127549055464,

)
LEFT_THREE_POINT_ANGLE     = 0.0

RIGHT_THREE_POINT_MIDPOINT = ( 
    0.3704625618078148,
    -0.414126374455314,
    -0.29923553466780695,
    0.0032539334651132804,
    0.7149622341901515,
    -0.0009362579244783191,
    0.699154874845288,
)
RIGHT_THREE_POINT_ENDPOINT = (
    0.36712537577707993,
    -0.3318733303285971,
    -0.4520389847935645,
    0.0033245439162966402,
    0.71558932020881,
    -0.0008963509947116495,
    0.6985127549055464,
)
RIGHT_THREE_POINT_ANGLE    = 0.0

# 参数法参数
LEFT_PARAMETRIC_CENTER  = (0.37, 0.29, -0.33)
LEFT_PARAMETRIC_AXIS    = (-1.0, 0.0, 0.0)
LEFT_PARAMETRIC_ANGLE   = (1.0 / 3.0) * math.pi

RIGHT_PARAMETRIC_CENTER = (0.37, -0.29, -0.33)
RIGHT_PARAMETRIC_AXIS   = (1.0, 0.0, 0.0)
RIGHT_PARAMETRIC_ANGLE  = (1.0 / 3.0) * math.pi


def make_pose(x, y, z, qw, qx, qy, qz):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)
    return pose


class CircleTrajectoryActionClient(Node):
    def __init__(self, interface=None): 
        super().__init__("circle_trajectory_action_client")
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
        circle_ready = self.interface.wait_for_movec_action_server(timeout=timeout)
        trajectory_ready = self.interface.wait_for_joint_trajectory_action_server(timeout=timeout)
        if circle_ready and trajectory_ready:
            self.get_logger().info("所有服务/action 可用")
            return True

        self.get_logger().error(
            f"不可用 - 圆弧action: {circle_ready}, 轨迹action: {trajectory_ready}"
        )
        return False

    def send_movej_command(self, arm_name, joint_positions, duration=4.0):
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

    def send_dual_movej_command(self, left_joints, right_joints, duration=4.0):
        self.get_logger().info("发送双臂 MoveJ action goal...")
        result = self.interface.execute_dual_arm_movej_action(
            left_joints,
            right_joints,
            duration=duration,
            time_mode=False,
            left_joint_names=self.left_arm_joint_names,
            right_joint_names=self.right_arm_joint_names,
            max_velocity=0.5,
            max_acceleration=1.0,
            max_jerk=2.0,
            feedback_callback=print_action_feedback,
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


def print_action_feedback(feedback):
    print(
        f"    action反馈: 进度 {feedback.progress * 100.0:.1f}%, "
        f"已用 {feedback.elapsed_time:.2f}s, 剩余 {feedback.remaining_time:.2f}s"
    )


def main():
    print("=" * 70)
    print("圆弧运动 Action 测试")
    print("步骤:")
    print("  1. MoveJ 移动到圆弧起点关节角度")
    print("  2. 三点法: 使用硬编码中间点/终点坐标")
    print("     参数法: 使用硬编码圆心/旋转轴/转角，读取当前位姿作为终点姿态")
    print("  3. 通过 ROS2RobotInterface.execute_movec_action_*() 执行 MOVEC")
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

        client = CircleTrajectoryActionClient(interface)

        print("[4] 等待服务和 action server...")
        if not client.wait_for_servers():
            return 1

        print("\n[5] 切换到 HOLD")
        interface.send_fsm_command(2)
        print("    状态切换完成")

        print("\n选择测试模式:")
        print("1. 左臂  - MoveJ 到起点 -> 圆弧 action (三点法, BFGS)")
        print("2. 右臂  - MoveJ 到起点 -> 圆弧 action (三点法, BFGS)")
        print("3. 双臂  - 同时 MoveJ 到起点 -> 单次圆弧 action 同时执行 (三点法, BFGS)")
        print("4. 左臂  - MoveJ 到起点 -> 圆弧 action (参数法, DLS)")
        print("5. 右臂  - MoveJ 到起点 -> 圆弧 action (参数法, DLS)")
        print("6. 双臂  - 同时 MoveJ 到起点 -> 单次圆弧 action 同时执行 (参数法, DLS)")
        choice = input("请选择(1-6): ").strip()

        if choice not in ["1", "2", "3", "4", "5", "6"]:
            print("无效选择")
            return 1

        if choice in ["1", "4"]:
            print("\n[6] MoveJ 移动到左臂起点...")
            if not client.send_movej_command("left", LEFT_TEST_JOINTS):
                return 1
        elif choice in ["2", "5"]:
            print("\n[6] MoveJ 移动到右臂起点...")
            if not client.send_movej_command("right", RIGHT_TEST_JOINTS):
                return 1
        else:
            print("\n[6] 双臂同时 MoveJ 到起点...")
            if not client.send_dual_movej_command(LEFT_TEST_JOINTS, RIGHT_TEST_JOINTS, duration=5.0):
                return 1

        print("\n[7] 发送圆弧 action...")
        duration = 6.0

        if choice == "1":
            midpoint = make_pose(*LEFT_THREE_POINT_MIDPOINT)
            endpoint = make_pose(*LEFT_THREE_POINT_ENDPOINT)
            print(f"    三点法 - 中间点: {LEFT_THREE_POINT_MIDPOINT[:3]}, 终点: {LEFT_THREE_POINT_ENDPOINT[:3]}")
            result = interface.execute_movec_action_three_point(
                "left",
                midpoint,
                endpoint,
                LEFT_THREE_POINT_ANGLE,
                duration=duration,
                feedback_callback=print_action_feedback,
            )
            if not result or not result.success:
                return 1

        elif choice == "2":
            midpoint = make_pose(*RIGHT_THREE_POINT_MIDPOINT)
            endpoint = make_pose(*RIGHT_THREE_POINT_ENDPOINT)
            print(f"    三点法 - 中间点: {RIGHT_THREE_POINT_MIDPOINT[:3]}, 终点: {RIGHT_THREE_POINT_ENDPOINT[:3]}")
            result = interface.execute_movec_action_three_point(
                "right",
                midpoint,
                endpoint,
                RIGHT_THREE_POINT_ANGLE,
                duration=duration,
                feedback_callback=print_action_feedback,
            )
            if not result or not result.success:
                return 1

        elif choice == "3":
            left_mid = make_pose(*LEFT_THREE_POINT_MIDPOINT)
            left_end = make_pose(*LEFT_THREE_POINT_ENDPOINT)
            right_mid = make_pose(*RIGHT_THREE_POINT_MIDPOINT)
            right_end = make_pose(*RIGHT_THREE_POINT_ENDPOINT)
            print(f"    左臂三点法 - 中间点: {LEFT_THREE_POINT_MIDPOINT[:3]}, 终点: {LEFT_THREE_POINT_ENDPOINT[:3]}")
            print(f"    右臂三点法 - 中间点: {RIGHT_THREE_POINT_MIDPOINT[:3]}, 终点: {RIGHT_THREE_POINT_ENDPOINT[:3]}")
            dual_result = interface.execute_movec_action_three_point(
                "both",
                left_mid,
                left_end,
                LEFT_THREE_POINT_ANGLE,
                duration=duration,
                right_midpoint_pose=right_mid,
                right_endpoint_pose=right_end,
                right_rotate_angle=RIGHT_THREE_POINT_ANGLE,
                feedback_callback=print_action_feedback,
            )
            if not dual_result or not dual_result.success:
                return 1

        elif choice == "4":
            print("    参数法 - 读取左臂当前位姿（用于终点姿态）...")
            end_pose = client.get_current_pose("left")
            center = Point(x=LEFT_PARAMETRIC_CENTER[0], y=LEFT_PARAMETRIC_CENTER[1], z=LEFT_PARAMETRIC_CENTER[2])
            axis = Vector3(x=LEFT_PARAMETRIC_AXIS[0], y=LEFT_PARAMETRIC_AXIS[1], z=LEFT_PARAMETRIC_AXIS[2])
            print(
                f"    圆心: {LEFT_PARAMETRIC_CENTER}, 轴: {LEFT_PARAMETRIC_AXIS}, "
                f"转角: {math.degrees(LEFT_PARAMETRIC_ANGLE):.1f}°"
            )
            result = interface.execute_movec_action_parametric(
                "left",
                center,
                axis,
                LEFT_PARAMETRIC_ANGLE,
                duration=duration,
                endpoint_pose=end_pose,
                feedback_callback=print_action_feedback,
            )
            if not result or not result.success:
                return 1

        elif choice == "5":
            print("    参数法 - 读取右臂当前位姿（用于终点姿态）...")
            end_pose = client.get_current_pose("right")
            center = Point(x=RIGHT_PARAMETRIC_CENTER[0], y=RIGHT_PARAMETRIC_CENTER[1], z=RIGHT_PARAMETRIC_CENTER[2])
            axis = Vector3(x=RIGHT_PARAMETRIC_AXIS[0], y=RIGHT_PARAMETRIC_AXIS[1], z=RIGHT_PARAMETRIC_AXIS[2])
            print(
                f"    圆心: {RIGHT_PARAMETRIC_CENTER}, 轴: {RIGHT_PARAMETRIC_AXIS}, "
                f"转角: {math.degrees(RIGHT_PARAMETRIC_ANGLE):.1f}°"
            )
            result = interface.execute_movec_action_parametric(
                "right",
                center,
                axis,
                RIGHT_PARAMETRIC_ANGLE,
                duration=duration,
                endpoint_pose=end_pose,
                feedback_callback=print_action_feedback,
            )
            if not result or not result.success:
                return 1

        elif choice == "6":
            print("    参数法 - 读取双臂当前位姿（用于终点姿态）...")
            left_end_pose = client.get_current_pose("left")
            right_end_pose = client.get_current_pose("right")
            left_center = Point(
                x=LEFT_PARAMETRIC_CENTER[0],
                y=LEFT_PARAMETRIC_CENTER[1],
                z=LEFT_PARAMETRIC_CENTER[2],
            )
            left_axis = Vector3(
                x=LEFT_PARAMETRIC_AXIS[0],
                y=LEFT_PARAMETRIC_AXIS[1],
                z=LEFT_PARAMETRIC_AXIS[2],
            )
            right_center = Point(
                x=RIGHT_PARAMETRIC_CENTER[0],
                y=RIGHT_PARAMETRIC_CENTER[1],
                z=RIGHT_PARAMETRIC_CENTER[2],
            )
            right_axis = Vector3(
                x=RIGHT_PARAMETRIC_AXIS[0],
                y=RIGHT_PARAMETRIC_AXIS[1],
                z=RIGHT_PARAMETRIC_AXIS[2],
            )
            print(
                f"    左臂参数法 - 圆心: {LEFT_PARAMETRIC_CENTER}, 轴: {LEFT_PARAMETRIC_AXIS}, "
                f"转角: {math.degrees(LEFT_PARAMETRIC_ANGLE):.1f}°"
            )
            print(
                f"    右臂参数法 - 圆心: {RIGHT_PARAMETRIC_CENTER}, 轴: {RIGHT_PARAMETRIC_AXIS}, "
                f"转角: {math.degrees(RIGHT_PARAMETRIC_ANGLE):.1f}°"
            )
            dual_result = interface.execute_movec_action_parametric(
                "both",
                left_center,
                left_axis,
                LEFT_PARAMETRIC_ANGLE,
                duration=duration,
                endpoint_pose=left_end_pose,
                right_center=right_center,
                right_axis=right_axis,
                right_rotate_angle=RIGHT_PARAMETRIC_ANGLE,
                right_endpoint_pose=right_end_pose,
                feedback_callback=print_action_feedback,
            )
            if not dual_result or not dual_result.success:
                return 1

        print("\n圆弧 action 执行成功")
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
