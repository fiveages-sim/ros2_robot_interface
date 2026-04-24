#!/usr/bin/env python3
"""
直线运动 Service 客户端 - 带数据记录功能
使用运动学测试脚本中的关节角度，先MoveJ到目标关节角度，
再通过正解获取当前位姿，计算Z轴移动-0.2m后的目标位姿作为MOVL的终点
服务: /ocs2_arm_controller/joint_trajectory_with_para, /ocs2_arm_controller/execute_linear
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os
import csv
import math
from datetime import datetime
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

# 导入服务类型
from arms_ros2_control_msgs.srv import ExecuteLinear, KinematicsService, JointTrajectory
from arms_ros2_control_msgs.msg import LinearMessage, JointWaypoint
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# ==================== 运动学测试脚本中的关节角度 ====================
LEFT_TEST_JOINTS = [-0.3525227330, -0.7798290600, 0.8896949257, 
                    -1.8910405790, -2.7986485415, 0.4619120915, 0.8030297356]
RIGHT_TEST_JOINTS = [0.0982891175, -0.8816540101, -0.5496532015, 
                     -1.8582004280, 2.8749939521, 0.3564035253, -1.0466912832]


class PoseDataRecorder(Node):
    """数据记录器 - 订阅PoseStamped数据并保存到CSV"""
    def __init__(self, output_dir="/home/lina/lina/data"):
        super().__init__('pose_data_recorder')
        
        self.output_dir = output_dir
        self.is_recording = False
        self.left_pose_count = 0
        self.right_pose_count = 0
        self.start_time = None
        
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            self.get_logger().info(f'创建输出目录: {output_dir}')
        
        self.left_subscription = self.create_subscription(
            PoseStamped, '/left_current_pose', self.left_pose_callback, 10)
        self.right_subscription = self.create_subscription(
            PoseStamped, '/right_current_pose', self.right_pose_callback, 10)
        
        self.left_csv_file = None
        self.right_csv_file = None
        self.left_csv_writer = None
        self.right_csv_writer = None
        self.recording_arm = 'both'
        self.recording_phase = 'linear'
        
    def start_recording(self, arm='both', phase='linear'):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.recording_arm = arm
        self.recording_phase = phase
        
        if arm == 'both' or arm == 'left':
            left_filename = os.path.join(self.output_dir, f'left_pose_{phase}_{timestamp}.csv')
            self.left_csv_file = open(left_filename, 'w', newline='')
            self.left_csv_writer = csv.writer(self.left_csv_file)
            self.left_csv_writer.writerow(['timestamp_sec', 'timestamp_nanosec', 'frame_id', 'phase',
                                           'position_x', 'position_y', 'position_z',
                                           'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z'])
            self.left_csv_file.flush()
            self.get_logger().info(f'左臂数据将记录到: {left_filename}')
        
        if arm == 'both' or arm == 'right':
            right_filename = os.path.join(self.output_dir, f'right_pose_{phase}_{timestamp}.csv')
            self.right_csv_file = open(right_filename, 'w', newline='')
            self.right_csv_writer = csv.writer(self.right_csv_file)
            self.right_csv_writer.writerow(['timestamp_sec', 'timestamp_nanosec', 'frame_id', 'phase',
                                            'position_x', 'position_y', 'position_z',
                                            'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z'])
            self.right_csv_file.flush()
            self.get_logger().info(f'右臂数据将记录到: {right_filename}')
        
        self.is_recording = True
        self.start_time = time.time()
        self.left_pose_count = 0
        self.right_pose_count = 0
        
    def stop_recording(self):
        self.is_recording = False
        if self.left_csv_file:
            self.left_csv_file.close()
            self.left_csv_file = None
        if self.right_csv_file:
            self.right_csv_file.close()
            self.right_csv_file = None
        elapsed_time = time.time() - self.start_time if self.start_time else 0
        total_count = self.left_pose_count + self.right_pose_count
        self.get_logger().info(f'数据记录停止，共记录 {total_count} 条数据，耗时 {elapsed_time:.2f} 秒')
        
    def left_pose_callback(self, msg):
        if not self.is_recording or self.recording_arm not in ['both', 'left']:
            return
        self._write_pose_data(msg, self.left_csv_writer, 'left')
        
    def right_pose_callback(self, msg):
        if not self.is_recording or self.recording_arm not in ['both', 'right']:
            return
        self._write_pose_data(msg, self.right_csv_writer, 'right')
        
    def _write_pose_data(self, msg, csv_writer, arm_name):
        try:
            row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
                   self.recording_phase,
                   msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                   msg.pose.orientation.w, msg.pose.orientation.x, 
                   msg.pose.orientation.y, msg.pose.orientation.z]
            if csv_writer:
                csv_writer.writerow(row)
                if arm_name == 'left':
                    self.left_pose_count += 1
                else:
                    self.right_pose_count += 1
        except Exception as e:
            self.get_logger().error(f'写入{arm_name}臂数据失败: {e}')


class LinearTrajectoryClient(Node):
    def __init__(self, interface=None):
        super().__init__('linear_trajectory_client')
        self.interface = interface
        self.pose_recorder = PoseDataRecorder()
        
        self.linear_client = self.create_client(ExecuteLinear, '/ocs2_arm_controller/execute_linear')
        self.kinematics_client = self.create_client(KinematicsService, '/kinematics_service')
        self.joint_trajectory_client = self.create_client(JointTrajectory, '/ocs2_arm_controller/joint_trajectory_with_para')
        
        self.left_arm_joint_names = ["left_joint1", "left_joint2", "left_joint3", "left_joint4",
                                     "left_joint5", "left_joint6", "left_joint7"]
        self.right_arm_joint_names = ["right_joint1", "right_joint2", "right_joint3", "right_joint4",
                                      "right_joint5", "right_joint6", "right_joint7"]
        
    def wait_for_services(self, timeout=10.0):
        linear_ready = self.linear_client.wait_for_service(timeout_sec=timeout)
        kinematics_ready = self.kinematics_client.wait_for_service(timeout_sec=timeout)
        trajectory_ready = self.joint_trajectory_client.wait_for_service(timeout_sec=timeout)
        if linear_ready and kinematics_ready and trajectory_ready:
            self.get_logger().info('所有服务可用')
            return True
        else:
            self.get_logger().error(f'服务不可用 - 直线: {linear_ready}, 运动学: {kinematics_ready}, 轨迹: {trajectory_ready}')
            return False

    def get_pose_from_kinematics(self, arm_name, joint_angles):
        """通过运动学服务获取关节角对应的末端位姿"""
        req = KinematicsService.Request()
        req.operation_type = "fk"
        req.arm_type = arm_name
        req.solver_type = "SDK"
        req.joint_angles = joint_angles
        
        if not self.kinematics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('运动学服务不可用')
            return None
        
        future = self.kinematics_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().success:
                response = future.result()
                if response.result_poses:
                    return response.result_poses[0]
        except Exception as e:
            self.get_logger().error(f'运动学服务调用异常: {e}')
        return None

    def send_movej_command(self, arm_name, joint_positions, duration=3.0):
        """使用JointTrajectory服务发送关节空间移动命令"""
        # 获取当前关节位置
        current_positions = self.get_current_joint_positions(arm_name)
        if current_positions is None:
            self.get_logger().error('无法获取当前关节位置')
            return False
        
        # 构建完整的14个关节位置（双臂）
        if arm_name == "left":
            dual_joint_positions = joint_positions + current_positions['right']
        else:
            dual_joint_positions = current_positions['left'] + joint_positions
        
        req = JointTrajectory.Request()
        req.joint_names = self.left_arm_joint_names + self.right_arm_joint_names
        
        wp = JointWaypoint()
        wp.position = dual_joint_positions
        wp.time_mode = True
        wp.total_time = duration
        
        wp.max_velocity = [0.5] * 14
        wp.max_acceleration = [1.0] * 14
        wp.max_jerk = [2.0] * 14
        
        req.waypoints = [wp]
        
        self.get_logger().info(f'发送{arm_name}臂MoveJ命令，目标关节: {[f"{j:.3f}" for j in joint_positions[:3]]}...')
        
        future = self.joint_trajectory_client.call_async(req)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
            if future.result() and future.result().success:
                planned_duration = future.result().planned_duration
                self.get_logger().info(f'✓ MoveJ成功，规划时长: {planned_duration:.3f}秒')
                wait_time = max(planned_duration, duration) + 1.0
                time.sleep(wait_time)
                return True
            else:
                error_msg = future.result().message if future.result() else '未知错误'
                self.get_logger().error(f'✗ MoveJ失败: {error_msg}')
                return False
        except Exception as e:
            self.get_logger().error(f'调用MoveJ服务失败: {e}')
            return False

    def send_dual_movej_command(self, left_joints, right_joints, duration=3.0):
        """同时移动双臂"""
        dual_joint_positions = left_joints + right_joints
        
        req = JointTrajectory.Request()
        req.joint_names = self.left_arm_joint_names + self.right_arm_joint_names
        
        wp = JointWaypoint()
        wp.position = dual_joint_positions
        wp.time_mode = True
        wp.total_time = duration
        
        wp.max_velocity = [0.5] * 14
        wp.max_acceleration = [1.0] * 14
        wp.max_jerk = [2.0] * 14
        
        req.waypoints = [wp]
        
        self.get_logger().info(f'发送双臂MoveJ命令...')
        
        future = self.joint_trajectory_client.call_async(req)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
            if future.result() and future.result().success:
                planned_duration = future.result().planned_duration
                self.get_logger().info(f'✓ 双臂MoveJ成功，规划时长: {planned_duration:.3f}秒')
                wait_time = max(planned_duration, duration) + 1.0
                time.sleep(wait_time)
                return True
            else:
                error_msg = future.result().message if future.result() else '未知错误'
                self.get_logger().error(f'✗ 双臂MoveJ失败: {error_msg}')
                return False
        except Exception as e:
            self.get_logger().error(f'调用MoveJ服务失败: {e}')
            return False

    def create_linear_request(self, arm_name, endpoint_pose, duration=3.0, ik_type="SDK"):
        """创建直线服务请求"""
        request = ExecuteLinear.Request()
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
        
        request.linear_params = linear
        return request

    def call_linear_service_with_recording(self, request, arm_name, timeout=15.0):
        """调用直线服务并记录位姿数据"""
        if not self.linear_client.service_is_ready():
            self.get_logger().error('直线服务未就绪')
            return None
        
        self.get_logger().info(f'调用{arm_name}臂直线服务...')
        future = self.linear_client.call_async(request)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if future.result() and future.result().success:
                response = future.result()
                self.get_logger().info(f'✓ 直线服务响应成功，开始记录位姿数据...')
                self.pose_recorder.start_recording(arm_name, 'linear')
                time.sleep(0.2)
                return response
            else:
                error_msg = future.result().message if future.result() else '未知错误'
                self.get_logger().error(f'直线服务调用失败: {error_msg}')
                return None
        except Exception as e:
            self.get_logger().error(f'直线服务调用异常: {e}')
            return None

    def get_current_joint_positions(self, arm_name="left"):
        """获取当前关节位置"""
        if self.interface is None:
            return None
        
        joint_state = self.interface.get_joint_state(categorized=False)
        if joint_state is None:
            return None
        
        all_joint_names = joint_state.get('names', [])
        all_joint_positions = joint_state.get('positions', [])
        joint_name_to_position = dict(zip(all_joint_names, all_joint_positions))
        
        left_positions = [joint_name_to_position.get(name, 0.0) for name in self.left_arm_joint_names]
        right_positions = [joint_name_to_position.get(name, 0.0) for name in self.right_arm_joint_names]
        
        return {'left': left_positions, 'right': right_positions}


def send_fsm_command(interface, command, wait_time=0.5):
    """发送FSM状态切换命令"""
    interface.send_fsm_command(command)
    time.sleep(wait_time)


def spin_recorders(client, duration):
    """在指定时间内持续处理记录器事件"""
    start_time = time.time()
    while time.time() - start_time < duration:
        rclpy.spin_once(client, timeout_sec=0.01)
        rclpy.spin_once(client.pose_recorder, timeout_sec=0.01)
        time.sleep(0.01)


def main():
    print("=" * 70)
    print("直线运动服务测试 - 使用运动学测试脚本中的关节角度")
    print("步骤:")
    print("  1. MoveJ移动到目标关节角度")
    print("  2. 正解获取当前位姿")
    print("  3. 计算Z轴移动-0.2m后的目标位姿")
    print("  4. MOVL直线运动")
    print("数据将保存到: /home/lina/lina/data")
    print("=" * 70)
    
    # 测试用的关节角度（来自运动学测试脚本）
    print("\n使用的关节角度:")
    print(f"  左臂: {[f'{j:.4f}' for j in LEFT_TEST_JOINTS]}")
    print(f"  右臂: {[f'{j:.4f}' for j in RIGHT_TEST_JOINTS]}")
    
    output_dir = "/home/lina/lina/data"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    rclpy.init()
    
    print("\n[1] 创建配置...")
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
    
    is_dual_arm = interface.config.right_end_effector_target_topic is not None
    if not is_dual_arm:
        print("    ✗ 错误: 此测试需要双臂模式\n")
        interface.disconnect()
        return 1
    print("    ✓ 检测到双臂模式\n")
    
    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")
    
    client = LinearTrajectoryClient(interface)
    
    print("[5] 等待服务...")
    if not client.wait_for_services():
        print("✗ 服务不可用")
        interface.disconnect()
        rclpy.shutdown()
        return 1
    print("✓ 服务已就绪")
    
    print("\n[6] 切换到HOLD状态...")
    try:
        send_fsm_command(interface, 2)
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    print("\n[7] 切换到HOME状态...")
    try:
        send_fsm_command(interface, 1)
        print("等待HOME完成 (5秒)...")
        time.sleep(5.0)
        print("✓ 已回到HOME位置")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    print("\n[8] 切换到HOLD状态...")
    try:
        send_fsm_command(interface, 2)
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    # ==================== 计算目标位姿 ====================
    print("\n[9] 通过正解获取目标关节角度对应的位姿...")
    
    print("\n   左臂:")
    left_current_pose = client.get_pose_from_kinematics("left", LEFT_TEST_JOINTS)
    if left_current_pose is None:
        print("      ✗ 左臂正解失败")
        return 1
    
    print(f"      位置: ({left_current_pose.position.x:.4f}, {left_current_pose.position.y:.4f}, {left_current_pose.position.z:.4f})")
    
    # 创建左臂目标位姿（Z轴向下移动0.2m）
    left_target_pose = Pose()
    left_target_pose.position.x = left_current_pose.position.x
    left_target_pose.position.y = left_current_pose.position.y
    left_target_pose.position.z = left_current_pose.position.z - 0.2
    left_target_pose.orientation = left_current_pose.orientation
    print(f"      目标位置(Z-0.2): ({left_target_pose.position.x:.4f}, {left_target_pose.position.y:.4f}, {left_target_pose.position.z:.4f})")
    
    print("\n   右臂:")
    right_current_pose = client.get_pose_from_kinematics("right", RIGHT_TEST_JOINTS)
    if right_current_pose is None:
        print("      ✗ 右臂正解失败")
        return 1
    
    print(f"      位置: ({right_current_pose.position.x:.4f}, {right_current_pose.position.y:.4f}, {right_current_pose.position.z:.4f})")
    
    # 创建右臂目标位姿（Z轴向下移动0.2m）
    right_target_pose = Pose()
    right_target_pose.position.x = right_current_pose.position.x
    right_target_pose.position.y = right_current_pose.position.y
    right_target_pose.position.z = right_current_pose.position.z - 0.2
    right_target_pose.orientation = right_current_pose.orientation
    print(f"      目标位置(Z-0.2): ({right_target_pose.position.x:.4f}, {right_target_pose.position.y:.4f}, {right_target_pose.position.z:.4f})")
    
    # ==================== 测试选择 ====================
    print("\n选择测试模式:")
    print("1. 左臂 - MoveJ到目标关节 -> 直线Z轴下移0.2m")
    print("2. 右臂 - MoveJ到目标关节 -> 直线Z轴下移0.2m")
    print("3. 双臂 - 分别MoveJ到目标关节 -> 分别直线Z轴下移0.2m")
    
    choice = input("请选择(1-3): ").strip()
    
    if choice == '1':
        test_arm = 'left'
        test_name = "左臂测试"
        duration = 3.0
    elif choice == '2':
        test_arm = 'right'
        test_name = "右臂测试"
        duration = 3.0
    elif choice == '3':
        test_arm = 'both'
        test_name = "双臂测试"
        duration = 3.0
    else:
        print("无效选择")
        return 1
    
    # ==================== 切换到MOVEJ状态 ====================
    print(f"\n[10] 切换到MOVEJ状态...")
    try:
        send_fsm_command(interface, 4)
        print("✓ 已切换到MOVEJ状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    # ==================== 执行MoveJ到目标关节角度 ====================
    if choice == '1':
        print(f"\n[11] MoveJ移动到左臂目标关节角度...")
        if not client.send_movej_command("left", LEFT_TEST_JOINTS, duration=4.0):
            print("✗ MoveJ到左臂目标关节失败")
            return 1
        print("✓ 左臂MoveJ完成")
        
    elif choice == '2':
        print(f"\n[11] MoveJ移动到右臂目标关节角度...")
        if not client.send_movej_command("right", RIGHT_TEST_JOINTS, duration=4.0):
            print("✗ MoveJ到右臂目标关节失败")
            return 1
        print("✓ 右臂MoveJ完成")
        
    elif choice == '3':
        print(f"\n[11] 双臂同时MoveJ到目标关节角度...")
        if not client.send_dual_movej_command(LEFT_TEST_JOINTS, RIGHT_TEST_JOINTS, duration=5.0):
            print("✗ 双臂MoveJ失败")
            return 1
        print("✓ 双臂MoveJ完成")
    
    # 等待稳定
    time.sleep(1.0)
    
    # ==================== 执行直线运动 ====================
    print(f"\n[12] 执行{test_name}直线运动...")
    
    if choice == '1':
        linear_req = client.create_linear_request("left", left_target_pose, duration, ik_type="SDK")
        response = client.call_linear_service_with_recording(linear_req, "left")
        
        if response and response.success:
            print(f"✓ 直线运动成功!")
            wait_time = response.estimated_duration + 2.0
            print(f"等待 {wait_time:.1f} 秒完成轨迹...")
            spin_recorders(client, wait_time)
            client.pose_recorder.stop_recording()
            print("✓ 位姿数据已保存")
        else:
            print("✗ 直线运动失败")
            
    elif choice == '2':
        linear_req = client.create_linear_request("right", right_target_pose, duration, ik_type="SDK")
        response = client.call_linear_service_with_recording(linear_req, "right")
        
        if response and response.success:
            print(f"✓ 直线运动成功!")
            wait_time = response.estimated_duration + 2.0
            print(f"等待 {wait_time:.1f} 秒完成轨迹...")
            spin_recorders(client, wait_time)
            client.pose_recorder.stop_recording()
            print("✓ 位姿数据已保存")
        else:
            print("✗ 直线运动失败")
            
    elif choice == '3':
        # 先左臂
        print("\n  执行左臂直线运动...")
        client.pose_recorder.start_recording("left", 'left_linear')
        left_req = client.create_linear_request("left", left_target_pose, duration, ik_type="SDK")
        left_response = client.call_linear_service_with_recording(left_req, "left")
        
        if left_response and left_response.success:
            wait_time = left_response.estimated_duration + 1.0
            print(f"等待 {wait_time:.1f} 秒完成左臂轨迹...")
            spin_recorders(client, wait_time)
            client.pose_recorder.stop_recording()
            print("✓ 左臂直线运动完成")
        else:
            print("✗ 左臂直线运动失败")
            return 1
        
        print("\n  等待2秒后执行右臂直线运动...")
        time.sleep(2.0)
        
        print("\n  执行右臂直线运动...")
        client.pose_recorder.start_recording("right", 'right_linear')
        right_req = client.create_linear_request("right", right_target_pose, duration, ik_type="SDK")
        right_response = client.call_linear_service_with_recording(right_req, "right")
        
        if right_response and right_response.success:
            wait_time = right_response.estimated_duration + 1.0
            print(f"等待 {wait_time:.1f} 秒完成右臂轨迹...")
            spin_recorders(client, wait_time)
            client.pose_recorder.stop_recording()
            print("✓ 右臂直线运动完成")
        else:
            print("✗ 右臂直线运动失败")
            return 1
    
    # ==================== 清理 ====================
    print("\n[13] 清理...")
    if client.pose_recorder.is_recording:
        client.pose_recorder.stop_recording()
    
    try:
        send_fsm_command(interface, 2)
        time.sleep(0.5)
        print("✓ 已切换回hold状态")
    except Exception as e:
        print(f"⚠ 状态切换失败: {e}")
    
    print("\n[14] 断开连接...")
    interface.disconnect()
    
    client.pose_recorder.destroy_node()
    client.destroy_node()
    
    rclpy.shutdown()
    
    print("\n" + "=" * 70)
    print("测试完成！")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)