#!/usr/bin/env python3
"""
圆弧运动 Action 客户端示例
先 MoveJ 到圆弧起点关节角度，再通过 MovecUseIK action 执行 MOVEC。

Action: /ocs2_arm_controller/execute_circle_use_ik
服务: /ocs2_arm_controller/joint_trajectory_with_para, /kinematics_service
支持两种圆弧定义方式:
  - 三点法: 起点(当前位姿) + 中间点 + 终点 (坐标硬编码)
  - 参数法: 圆心 + 旋转轴 + 旋转角度 (坐标硬编码, 终点姿态由正解获取)
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from arms_ros2_control_msgs.action import MovecUseIK
from arms_ros2_control_msgs.msg import CircleMessage, JointWaypoint
from arms_ros2_control_msgs.srv import JointTrajectory, KinematicsService
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 圆弧起点关节角度
LEFT_TEST_JOINTS = [
    -0.48535481337577646, 0.7705844339342997, -0.20624623950835802,
    -0.6913082479497125, -0.025730988246958686, 1.1253353493101284, 0.0032744788190768545,
]
RIGHT_TEST_JOINTS = [
    -0.328385435813621, -0.22000419219377856, -0.2746054955857294,
    1.0731518620969906, 0.44726364968600296, -0.5811312841716576, 0.5052403055260443,
]

# 三点法参数: (x, y, z, qw, qx, qy, qz)
LEFT_THREE_POINT_MIDPOINT  = (0.2298, 0.1872, 0.1393, 1.0, 0.0, 0.0, 0.0)
LEFT_THREE_POINT_ENDPOINT  = (0.2291, 0.2564, 0.04, 1.0, 0.0, 0.0, 0.0)
LEFT_THREE_POINT_ANGLE     = 0.0

RIGHT_THREE_POINT_MIDPOINT = (0.2839, 0.5915, -0.4104, 1.0, 0.0, 0.0, 0.0)
RIGHT_THREE_POINT_ENDPOINT = (0.3214, 0.4782, -0.2665, 1.0, 0.0, 0.0, 0.0)
RIGHT_THREE_POINT_ANGLE    = 3.9551

# 参数法参数
LEFT_PARAMETRIC_CENTER  = (0.2520576827979941, 0.0, 0.09094741848884622)
LEFT_PARAMETRIC_AXIS    = (-1.0, 0.0, 0.0)
LEFT_PARAMETRIC_ANGLE   = (1.0 / 3.0) * math.pi

RIGHT_PARAMETRIC_CENTER = (0.2520576827979941, 0.0, 0.09094741848884622)
RIGHT_PARAMETRIC_AXIS   = (-1.0, 0.0, 0.0)
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

        self.circle_action_client = ActionClient(
            self,
            MovecUseIK,
            "/ocs2_arm_controller/execute_circle_use_ik",
        )
        self.kinematics_client = self.create_client(KinematicsService, "/kinematics_service")
        self.joint_trajectory_client = self.create_client(
            JointTrajectory,
            "/ocs2_arm_controller/joint_trajectory_with_para",
        )

        self.left_arm_joint_names = [
            "left_joint1", "left_joint2", "left_joint3", "left_joint4",
            "left_joint5", "left_joint6", "left_joint7",
        ]
        self.right_arm_joint_names = [
            "right_joint1", "right_joint2", "right_joint3", "right_joint4",
            "right_joint5", "right_joint6", "right_joint7",
        ]

    def wait_for_servers(self, timeout=10.0):
        circle_ready = self.circle_action_client.wait_for_server(timeout_sec=timeout)
        kinematics_ready = self.kinematics_client.wait_for_service(timeout_sec=timeout)
        trajectory_ready = self.joint_trajectory_client.wait_for_service(timeout_sec=timeout)
        if circle_ready and kinematics_ready and trajectory_ready:
            self.get_logger().info("所有服务/action 可用")
            return True

        self.get_logger().error(
            f"不可用 - 圆弧action: {circle_ready}, 运动学: {kinematics_ready}, 轨迹: {trajectory_ready}"
        )
        return False

    def get_pose_from_kinematics(self, arm_name, joint_angles):
        req = KinematicsService.Request()
        req.operation_type = "fk"
        req.arm_type = arm_name
        req.joint_angles = joint_angles

        future = self.kinematics_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success and future.result().result_poses:
            pose = future.result().result_poses[0]
            self.get_logger().info(
                f"正解位姿: 位置[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}], "
                f"四元数[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, "
                f"{pose.orientation.z:.3f}, {pose.orientation.w:.3f}]"
            )
            return pose
        self.get_logger().error("正解失败")
        return None

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
        req = JointTrajectory.Request()
        req.joint_names = self.left_arm_joint_names + self.right_arm_joint_names

        waypoint = JointWaypoint()
        waypoint.position = left_joints + right_joints
        waypoint.time_mode = True
        waypoint.total_time = duration
        waypoint.max_velocity = [0.5] * 14
        waypoint.max_acceleration = [1.0] * 14
        waypoint.max_jerk = [2.0] * 14
        req.waypoints = [waypoint]

        future = self.joint_trajectory_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
        if future.result() and future.result().success:
            planned_duration = future.result().planned_duration
            self.get_logger().info(f"MoveJ 成功，规划时长: {planned_duration:.3f} 秒")
            time.sleep(max(planned_duration, duration) + 1.0)
            return True

        error_msg = future.result().message if future.result() else "未知错误"
        self.get_logger().error(f"MoveJ 失败: {error_msg}")
        return False

    def create_circle_goal_three_point(
        self,
        arm_name,
        midpoint_pose,
        endpoint_pose,
        rotate_angle,
        duration=6.0,
        ik_type="DLS",
        right_midpoint_pose=None,
        right_endpoint_pose=None,
        right_rotate_angle=0.0,
    ):
        """三点法圆弧 goal: 起点为当前位姿，指定中间点和终点（硬编码坐标）。"""
        goal_msg = MovecUseIK.Goal()
        circle = CircleMessage()
        circle.arm_name = arm_name
        circle.duration = duration
        circle.time_mode = True
        circle.frame_id = "base_link"
        circle.ik_type = ik_type
        circle.use_three_point_method = True
        circle.use_slerp_for_orientation = True
        circle.max_linear_velocity = 0.3
        circle.max_linear_acceleration = 0.5
        circle.max_linear_jerk = 3.0
        circle.max_angular_velocity = 1.0
        circle.max_angular_acceleration = 2.0
        circle.max_angular_jerk = 5.0
        circle.midpoint = midpoint_pose
        circle.endpoint = endpoint_pose
        circle.rotate_angle = rotate_angle
        if right_midpoint_pose is not None:
            circle.right_midpoint = right_midpoint_pose
        if right_endpoint_pose is not None:
            circle.right_endpoint = right_endpoint_pose
        circle.right_rotate_angle = right_rotate_angle
        goal_msg.circle_params = circle
        return goal_msg

    def create_circle_goal_parametric(
        self,
        arm_name,
        center,
        axis,
        rotate_angle,
        duration=6.0,
        ik_type="DLS",
        end_pose=None,
        right_center=None,
        right_axis=None,
        right_rotate_angle=0.0,
        right_end_pose=None,
    ):
        """参数法圆弧 goal: 指定圆心、旋转轴和旋转角度，终点姿态由正解提供。"""
        goal_msg = MovecUseIK.Goal()
        circle = CircleMessage()
        circle.arm_name = arm_name
        circle.duration = duration
        circle.time_mode = True
        circle.frame_id = "base_link"
        circle.ik_type = ik_type
        circle.use_three_point_method = False
        circle.use_slerp_for_orientation = False
        circle.max_linear_velocity = 0.3
        circle.max_linear_acceleration = 0.5
        circle.max_linear_jerk = 3.0
        circle.max_angular_velocity = 1.0
        circle.max_angular_acceleration = 2.0
        circle.max_angular_jerk = 5.0
        circle.center = center
        circle.axis = axis
        circle.rotate_angle = rotate_angle
        circle.endpoint.position = Point(x=0.0, y=0.0, z=0.0)
        if end_pose is not None:
            circle.endpoint.orientation = end_pose.orientation
            self.get_logger().info(
                f"参数法终点姿态(来自正解): 四元数[{end_pose.orientation.x:.3f}, "
                f"{end_pose.orientation.y:.3f}, {end_pose.orientation.z:.3f}, "
                f"{end_pose.orientation.w:.3f}]"
            )
        else:
            circle.endpoint.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            self.get_logger().warn("未提供终点姿态，使用默认四元数")

        if right_center is not None:
            circle.right_center = right_center
        if right_axis is not None:
            circle.right_axis = right_axis
        circle.right_rotate_angle = right_rotate_angle
        circle.right_endpoint.position = Point(x=0.0, y=0.0, z=0.0)
        if right_end_pose is not None:
            circle.right_endpoint.orientation = right_end_pose.orientation
            self.get_logger().info(
                f"参数法右臂终点姿态(来自正解): 四元数[{right_end_pose.orientation.x:.3f}, "
                f"{right_end_pose.orientation.y:.3f}, {right_end_pose.orientation.z:.3f}, "
                f"{right_end_pose.orientation.w:.3f}]"
            )
        elif arm_name == "both":
            circle.right_endpoint.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            self.get_logger().warn("未提供右臂终点姿态，使用默认四元数")

        goal_msg.circle_params = circle
        return goal_msg

    def send_circle_action(self, goal_msg, arm_name, timeout=60.0):
        self.get_logger().info(f"发送 {arm_name} 臂圆弧 action goal...")
        send_goal_future = self.circle_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.circle_feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("圆弧 action goal 被拒绝")
            return None

        self.get_logger().info("圆弧 action goal 已接受")

        result_future = goal_handle.get_result_async()
        if not self.spin_until_result(result_future, timeout):
            self.get_logger().error("等待圆弧 action 结果超时，正在请求取消 goal")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return None

        result_response = result_future.result()
        result = result_response.result
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"圆弧 action 未成功结束，状态码: {result_response.status}, "
                f"消息: {result.message}, "
                f"预计时长: {result.estimated_duration:.3f}s, "
                f"实际时长: {result.actual_duration:.3f}s"
            )
        else:
            self.get_logger().info(
                f"圆弧 action 执行成功，消息: {result.message}, "
                f"预计时长: {result.estimated_duration:.3f}s, "
                f"实际时长: {result.actual_duration:.3f}s"
            )
        return result

    def circle_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"action反馈: 进度 {feedback.progress * 100.0:.1f}%, "
            f"已用 {feedback.elapsed_time:.2f}s, 剩余 {feedback.remaining_time:.2f}s"
        )

    def spin_until_result(self, future, timeout):
        start_time = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.time() - start_time > timeout:
                return False
        return future.done()

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


def send_fsm_command(interface, command, wait_time=0.5):
    interface.send_fsm_command(command)
    time.sleep(wait_time)


def main():
    print("=" * 70)
    print("圆弧运动 Action 测试")
    print("步骤:")
    print("  1. MoveJ 移动到圆弧起点关节角度")
    print("  2. 三点法: 使用硬编码中间点/终点坐标")
    print("     参数法: 使用硬编码圆心/旋转轴/转角，正解获取终点姿态")
    print("  3. 通过 MovecUseIK action 执行 MOVEC")
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

        print("[3] 等待数据到达（2秒）...")
        time.sleep(2.0)

        client = CircleTrajectoryActionClient(interface)

        print("[4] 等待服务和 action server...")
        if not client.wait_for_servers():
            return 1

        print("\n[5] 切换到 HOLD")
        send_fsm_command(interface, 2)
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

        print("\n[6] 切换到 MOVEJ 状态...")
        send_fsm_command(interface, 4)

        if choice in ["1", "4"]:
            print("\n[7] MoveJ 移动到左臂起点...")
            if not client.send_movej_command("left", LEFT_TEST_JOINTS):
                return 1
        elif choice in ["2", "5"]:
            print("\n[7] MoveJ 移动到右臂起点...")
            if not client.send_movej_command("right", RIGHT_TEST_JOINTS):
                return 1
        else:
            print("\n[7] 双臂同时 MoveJ 到起点...")
            if not client.send_dual_movej_command(LEFT_TEST_JOINTS, RIGHT_TEST_JOINTS, duration=5.0):
                return 1

        time.sleep(1.0)

        print("\n[8] 发送圆弧 action...")
        duration = 6.0

        if choice == "1":
            midpoint = make_pose(*LEFT_THREE_POINT_MIDPOINT)
            endpoint = make_pose(*LEFT_THREE_POINT_ENDPOINT)
            print(f"    三点法 - 中间点: {LEFT_THREE_POINT_MIDPOINT[:3]}, 终点: {LEFT_THREE_POINT_ENDPOINT[:3]}")
            goal_msg = client.create_circle_goal_three_point(
                "left", midpoint, endpoint, LEFT_THREE_POINT_ANGLE, duration,
            )
            result = client.send_circle_action(goal_msg, "left")
            if not result or not result.success:
                return 1

        elif choice == "2":
            midpoint = make_pose(*RIGHT_THREE_POINT_MIDPOINT)
            endpoint = make_pose(*RIGHT_THREE_POINT_ENDPOINT)
            print(f"    三点法 - 中间点: {RIGHT_THREE_POINT_MIDPOINT[:3]}, 终点: {RIGHT_THREE_POINT_ENDPOINT[:3]}")
            goal_msg = client.create_circle_goal_three_point(
                "right", midpoint, endpoint, RIGHT_THREE_POINT_ANGLE, duration,
            )
            result = client.send_circle_action(goal_msg, "right")
            if not result or not result.success:
                return 1

        elif choice == "3":
            left_mid = make_pose(*LEFT_THREE_POINT_MIDPOINT)
            left_end = make_pose(*LEFT_THREE_POINT_ENDPOINT)
            right_mid = make_pose(*RIGHT_THREE_POINT_MIDPOINT)
            right_end = make_pose(*RIGHT_THREE_POINT_ENDPOINT)
            print(f"    左臂三点法 - 中间点: {LEFT_THREE_POINT_MIDPOINT[:3]}, 终点: {LEFT_THREE_POINT_ENDPOINT[:3]}")
            print(f"    右臂三点法 - 中间点: {RIGHT_THREE_POINT_MIDPOINT[:3]}, 终点: {RIGHT_THREE_POINT_ENDPOINT[:3]}")
            dual_goal = client.create_circle_goal_three_point(
                "both",
                left_mid,
                left_end,
                LEFT_THREE_POINT_ANGLE,
                duration,
                right_midpoint_pose=right_mid,
                right_endpoint_pose=right_end,
                right_rotate_angle=RIGHT_THREE_POINT_ANGLE,
            )
            dual_result = client.send_circle_action(dual_goal, "both")
            if not dual_result or not dual_result.success:
                return 1

        elif choice == "4":
            print("    参数法 - 正解获取左臂当前位姿（用于终点姿态）...")
            current = client.get_current_joint_positions()
            end_pose = client.get_pose_from_kinematics("left", current["left"]) if current else None
            center = Point(x=LEFT_PARAMETRIC_CENTER[0], y=LEFT_PARAMETRIC_CENTER[1], z=LEFT_PARAMETRIC_CENTER[2])
            axis = Vector3(x=LEFT_PARAMETRIC_AXIS[0], y=LEFT_PARAMETRIC_AXIS[1], z=LEFT_PARAMETRIC_AXIS[2])
            print(
                f"    圆心: {LEFT_PARAMETRIC_CENTER}, 轴: {LEFT_PARAMETRIC_AXIS}, "
                f"转角: {math.degrees(LEFT_PARAMETRIC_ANGLE):.1f}°"
            )
            goal_msg = client.create_circle_goal_parametric(
                "left", center, axis, LEFT_PARAMETRIC_ANGLE, duration, end_pose=end_pose,
            )
            result = client.send_circle_action(goal_msg, "left")
            if not result or not result.success:
                return 1

        elif choice == "5":
            print("    参数法 - 正解获取右臂当前位姿（用于终点姿态）...")
            current = client.get_current_joint_positions()
            end_pose = client.get_pose_from_kinematics("right", current["right"]) if current else None
            center = Point(x=RIGHT_PARAMETRIC_CENTER[0], y=RIGHT_PARAMETRIC_CENTER[1], z=RIGHT_PARAMETRIC_CENTER[2])
            axis = Vector3(x=RIGHT_PARAMETRIC_AXIS[0], y=RIGHT_PARAMETRIC_AXIS[1], z=RIGHT_PARAMETRIC_AXIS[2])
            print(
                f"    圆心: {RIGHT_PARAMETRIC_CENTER}, 轴: {RIGHT_PARAMETRIC_AXIS}, "
                f"转角: {math.degrees(RIGHT_PARAMETRIC_ANGLE):.1f}°"
            )
            goal_msg = client.create_circle_goal_parametric(
                "right", center, axis, RIGHT_PARAMETRIC_ANGLE, duration, end_pose=end_pose,
            )
            result = client.send_circle_action(goal_msg, "right")
            if not result or not result.success:
                return 1

        elif choice == "6":
            print("    参数法 - 正解获取双臂当前位姿（用于终点姿态）...")
            current = client.get_current_joint_positions()
            left_end_pose = client.get_pose_from_kinematics("left", current["left"]) if current else None
            right_end_pose = client.get_pose_from_kinematics("right", current["right"]) if current else None
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
            dual_goal = client.create_circle_goal_parametric(
                "both",
                left_center,
                left_axis,
                LEFT_PARAMETRIC_ANGLE,
                duration,
                end_pose=left_end_pose,
                right_center=right_center,
                right_axis=right_axis,
                right_rotate_angle=RIGHT_PARAMETRIC_ANGLE,
                right_end_pose=right_end_pose,
            )
            dual_result = client.send_circle_action(dual_goal, "both")
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
                send_fsm_command(interface, 2)
                interface.disconnect()
            except Exception:
                pass
        if client is not None:
            client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
