#!/usr/bin/env python3
"""
直线运动 Action 客户端示例
使用运动学测试脚本中的关节角度，先 MoveJ 到目标关节角度，
再通过正解获取当前位姿，计算 Z 轴移动 -0.2m 后的目标位姿，
最后通过 ExecuteLinear action 执行 MOVL。

Action: /ocs2_arm_controller/execute_linear, /ocs2_arm_controller/joint_trajectory_with_para
服务: /kinematics_service
"""

import csv
import os
import sys
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from arms_ros2_control_msgs.action import ExecuteLinear, JointTrajectory as JointTrajectoryAction
from arms_ros2_control_msgs.msg import JointWaypoint, LinearMessage
from arms_ros2_control_msgs.srv import KinematicsService
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


LEFT_TEST_JOINTS = [
    -0.3525227330, -0.7798290600, 0.8896949257,
    -1.8910405790, -2.7986485415, 0.4619120915, 0.8030297356,
]

RIGHT_TEST_JOINTS = [
    0.0982891175, -0.8816540101, -0.5496532015,
    -1.8582004280, 2.8749939521, 0.3564035253, -1.0466912832,
]


class LinearTrajectoryActionClient(Node):
    def __init__(self, interface=None):
        super().__init__("linear_trajectory_action_client")
        self.interface = interface

        self.linear_action_client = ActionClient(
            self,
            ExecuteLinear,
            "/ocs2_arm_controller/execute_linear",
        )
        self.kinematics_client = self.create_client(KinematicsService, "/kinematics_service")
        self.joint_trajectory_client = ActionClient(
            self,
            JointTrajectoryAction,
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
        linear_ready = self.linear_action_client.wait_for_server(timeout_sec=timeout)
        kinematics_ready = self.kinematics_client.wait_for_service(timeout_sec=timeout)
        trajectory_ready = self.joint_trajectory_client.wait_for_server(timeout_sec=timeout)
        if linear_ready and kinematics_ready and trajectory_ready:
            self.get_logger().info("所有服务/action 可用")
            return True

        self.get_logger().error(
            f"不可用 - 直线action: {linear_ready}, 运动学: {kinematics_ready}, 轨迹action: {trajectory_ready}"
        )
        return False

    def get_pose_from_kinematics(self, arm_name, joint_angles):
        req = KinematicsService.Request()
        req.operation_type = "fk"
        req.arm_type = arm_name
        # req.solver_type = "AUTO"
        req.joint_angles = joint_angles

        future = self.kinematics_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success and future.result().result_poses:
            return future.result().result_poses[0]
        return None

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
        goal_msg = JointTrajectoryAction.Goal()
        goal_msg.joint_names = self.left_arm_joint_names + self.right_arm_joint_names

        waypoint = JointWaypoint()
        waypoint.position = left_joints + right_joints
        waypoint.time_mode = True
        waypoint.total_time = duration
        waypoint.max_velocity = [0.5] * 14
        waypoint.max_acceleration = [1.0] * 14
        waypoint.max_jerk = [2.0] * 14
        goal_msg.waypoints = [waypoint]

        self.get_logger().info("发送双臂 MoveJ action goal...")
        send_goal_future = self.joint_trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.joint_trajectory_feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveJ action goal 被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        if not self.spin_until_result(result_future, timeout=max(duration + 10.0, 30.0)):
            self.get_logger().error("等待 MoveJ action 结果超时，正在请求取消 goal")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False

        result_response = result_future.result()
        result = result_response.result
        if result_response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(
                f"MoveJ 成功，消息: {result.message}, "
                f"规划时长: {result.planned_duration:.3f} 秒, "
                f"实际时长: {result.actual_duration:.3f} 秒"
            )
            return True

        self.get_logger().error(
            f"MoveJ 失败，状态码: {result_response.status}, "
            f"消息: {result.message}, "
            f"规划时长: {result.planned_duration:.3f} 秒, "
            f"实际时长: {result.actual_duration:.3f} 秒"
        )
        return False

    def joint_trajectory_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"MoveJ action反馈: 进度 {feedback.progress * 100.0:.1f}%, "
            f"已用 {feedback.elapsed_time:.2f}s, 剩余 {feedback.remaining_time:.2f}s"
        )

    def create_linear_goal(
        self,
        arm_name,
        endpoint_pose,
        duration=3.0,
        ik_type="AUTO",
        right_endpoint_pose=None,
    ):
        goal_msg = ExecuteLinear.Goal()
        linear = LinearMessage()
        linear.arm_name = arm_name
        linear.duration = duration
        linear.time_mode = True
        linear.frame_id = "base_link"
        linear.ik_type = ik_type
        linear.max_linear_velocity = 0.2
        linear.max_linear_acceleration = 0.3
        linear.max_linear_jerk = 2.0
        linear.max_angular_velocity = 0.5
        linear.max_angular_acceleration = 1.0
        linear.max_angular_jerk = 3.0
        linear.endpoint = endpoint_pose
        if right_endpoint_pose is not None:
            linear.right_endpoint = right_endpoint_pose
        goal_msg.linear_params = linear
        return goal_msg

    def send_linear_action(self, goal_msg, arm_name, timeout=30.0):
        self.get_logger().info(f"发送 {arm_name} 臂直线 action goal...")
        send_goal_future = self.linear_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.linear_feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("直线 action goal 被拒绝")
            return None

        result_future = goal_handle.get_result_async()
        if not self.spin_until_result(result_future, timeout):
            self.get_logger().error("等待直线 action 结果超时，正在请求取消 goal")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return None

        result_response = result_future.result()
        result = result_response.result
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"直线 action 未成功结束，状态码: {result_response.status}, "
                f"消息: {result.message}, "
                f"预计时长: {result.estimated_duration:.3f}s, "
                f"实际时长: {result.actual_duration:.3f}s"
            )
        else:
            self.get_logger().info(
                f"直线 action 执行成功，消息: {result.message}, "
                f"预计时长: {result.estimated_duration:.3f}s, "
                f"实际时长: {result.actual_duration:.3f}s"
            )
        return result

    def linear_feedback_callback(self, feedback_msg):
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


def build_offset_pose(source_pose, z_offset=-0.2):
    target_pose = Pose()
    target_pose.position.x = source_pose.position.x
    target_pose.position.y = source_pose.position.y
    target_pose.position.z = source_pose.position.z + z_offset
    target_pose.orientation = source_pose.orientation
    return target_pose


def main():
    print("=" * 70)
    print("直线运动 Action 测试 - 使用运动学测试脚本中的关节角度")
    print("步骤:")
    print("  1. MoveJ 移动到目标关节角度")
    print("  2. 正解获取当前位姿")
    print("  3. 计算 Z 轴移动 +-0.05m 后的目标位姿")
    print("  4. 通过 ExecuteLinear action 执行 MOVL")
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
        send_fsm_command(interface, 2)
        print("    状态切换完成")

        print("\n[6] 通过正解获取目标关节角对应位姿...")
        left_current_pose = client.get_pose_from_kinematics("left", LEFT_TEST_JOINTS)
        right_current_pose = client.get_pose_from_kinematics("right", RIGHT_TEST_JOINTS)
        if left_current_pose is None or right_current_pose is None:
            print("    正解失败")
            return 1

        left_target_pose = build_offset_pose(left_current_pose, z_offset=-0.2)
        right_target_pose = build_offset_pose(right_current_pose, z_offset=-0.2)
        print(
            f"    左臂目标: ({left_target_pose.position.x:.4f}, "
            f"{left_target_pose.position.y:.4f}, {left_target_pose.position.z:.4f})"
        )
        print(
            f"    右臂目标: ({right_target_pose.position.x:.4f}, "
            f"{right_target_pose.position.y:.4f}, {right_target_pose.position.z:.4f})"
        )

        print("\n选择测试模式:")
        print("1. 左臂 - MoveJ 到目标关节 -> action 直线 Z 轴下移 0.2m")
        print("2. 右臂 - MoveJ 到目标关节 -> action 直线 Z 轴下移 0.2m")
        print("3. 双臂 - 同时 MoveJ 到目标关节 -> 单次 action 同时执行双臂直线运动")
        choice = input("请选择(1-3): ").strip()

        if choice not in ["1", "2", "3"]:
            print("无效选择")
            return 1

        print("\n[7] 切换到 MOVEJ 状态...")
        send_fsm_command(interface, 4)

        if choice == "1":
            print("\n[8] MoveJ 移动到左臂目标关节角度...")
            if not client.send_movej_command("left", LEFT_TEST_JOINTS, duration=4.0):
                return 1
        elif choice == "2":
            print("\n[8] MoveJ 移动到右臂目标关节角度...")
            if not client.send_movej_command("right", RIGHT_TEST_JOINTS, duration=4.0):
                return 1
        else:
            print("\n[8] 双臂同时 MoveJ 到目标关节角度...")
            if not client.send_dual_movej_command(LEFT_TEST_JOINTS, RIGHT_TEST_JOINTS, duration=5.0):
                return 1

        time.sleep(1.0)

        print("\n[9] 发送直线 action...")
        duration = 3.0
        if choice == "1":
            goal_msg = client.create_linear_goal("left", left_target_pose, duration)
            result = client.send_linear_action(goal_msg, "left")
            if not result or not result.success:
                return 1
        elif choice == "2":
            goal_msg = client.create_linear_goal("right", right_target_pose, duration)
            result = client.send_linear_action(goal_msg, "right")
            if not result or not result.success:
                return 1
        else:
            dual_goal = client.create_linear_goal(
                "both",
                left_target_pose,
                duration,
                right_endpoint_pose=right_target_pose,
            )
            dual_result = client.send_linear_action(dual_goal, "both")
            if not dual_result or not dual_result.success:
                return 1

        print("\n直线 action 执行成功")
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
