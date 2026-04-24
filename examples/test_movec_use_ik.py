#!/usr/bin/env python3
"""
关节空间+圆弧轨迹 Service 客户端 - 带数据记录功能（只在圆弧时记录）
先使用关节空间移动到圆弧起点，再执行圆弧运动
服务: 
  - 关节移动: /ocs2_arm_controller/joint_trajectory_with_para (service)
  - 圆弧: /ocs2_arm_controller/execute_circle_use_ik (service)
  - 运动学: /kinematics_service (service)
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os
import csv
import math
import threading
import numpy as np
from datetime import datetime
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, PoseStamped
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

# 导入服务类型
from arms_ros2_control_msgs.srv import MovecUseIK, JointTrajectory, KinematicsService
from arms_ros2_control_msgs.msg import JointWaypoint, CircleMessage
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


class PoseDataRecorder(Node):
    """数据记录器 - 订阅PoseStamped数据并保存到CSV（只在圆弧时使用）"""
    def __init__(self, output_dir="/home/lina/lina/data"):
        super().__init__('pose_data_recorder')
        
        self.output_dir = output_dir
        self.is_recording = False
        self.left_pose_count = 0
        self.right_pose_count = 0
        self.start_time = None
        
        # 确保输出目录存在
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            self.get_logger().info(f'创建输出目录: {output_dir}')
        
        # 创建左臂数据订阅者
        self.left_subscription = self.create_subscription(
            PoseStamped,
            '/left_current_pose',  
            self.left_pose_callback,
            10
        )
        
        # 创建右臂数据订阅者
        self.right_subscription = self.create_subscription(
            PoseStamped,
            '/right_current_pose',
            self.right_pose_callback,
            10
        )
        
        # CSV文件句柄
        self.left_csv_file = None
        self.right_csv_file = None
        self.left_csv_writer = None
        self.right_csv_writer = None
        
        # 用于同步访问的标志
        self.recording_arm = 'both'  # 'left', 'right', 'both'
        self.recording_phase = 'unknown'  # 将用于标识圆弧运动
        
        self.get_logger().info('数据记录器初始化完成')
        
    def start_recording(self, arm='both', phase='circle'):
        """开始记录数据（只在圆弧时使用）"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.recording_arm = arm
        self.recording_phase = phase
        
        if arm == 'both' or arm == 'left':
            # 创建左臂CSV文件
            left_filename = os.path.join(self.output_dir, f'left_pose_{phase}_{timestamp}.csv')
            self.left_csv_file = open(left_filename, 'w', newline='')
            self.left_csv_writer = csv.writer(self.left_csv_file)
            
            # 写入表头
            header = ['timestamp_sec', 'timestamp_nanosec', 'frame_id', 'phase',
                     'position_x', 'position_y', 'position_z',
                     'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z']
            self.left_csv_writer.writerow(header)
            self.left_csv_file.flush()
            self.get_logger().info(f'左臂数据将记录到: {left_filename}')
        
        if arm == 'both' or arm == 'right':
            # 创建右臂CSV文件
            right_filename = os.path.join(self.output_dir, f'right_pose_{phase}_{timestamp}.csv')
            self.right_csv_file = open(right_filename, 'w', newline='')
            self.right_csv_writer = csv.writer(self.right_csv_file)
            
            # 写入表头
            header = ['timestamp_sec', 'timestamp_nanosec', 'frame_id', 'phase',
                     'position_x', 'position_y', 'position_z',
                     'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z']
            self.right_csv_writer.writerow(header)
            self.right_csv_file.flush()
            self.get_logger().info(f'右臂数据将记录到: {right_filename}')
        
        self.is_recording = True
        self.start_time = time.time()
        self.left_pose_count = 0
        self.right_pose_count = 0
        
    def update_phase(self, phase):
        """更新当前记录阶段"""
        self.recording_phase = phase
        
    def stop_recording(self):
        """停止记录数据"""
        self.is_recording = False
        
        # 关闭CSV文件
        if self.left_csv_file:
            self.left_csv_file.close()
            self.left_csv_file = None
            self.left_csv_writer = None
            
        if self.right_csv_file:
            self.right_csv_file.close()
            self.right_csv_file = None
            self.right_csv_writer = None
            
        elapsed_time = time.time() - self.start_time if self.start_time else 0
        total_count = self.left_pose_count + self.right_pose_count
        self.get_logger().info(f'数据记录停止，共记录 {total_count} 条数据，耗时 {elapsed_time:.2f} 秒')
        
    def left_pose_callback(self, msg):
        """左臂位姿回调函数"""
        if not self.is_recording or self.recording_arm not in ['both', 'left']:
            return
            
        self._write_pose_data(msg, self.left_csv_writer, 'left')
        
    def right_pose_callback(self, msg):
        """右臂位姿回调函数"""
        if not self.is_recording or self.recording_arm not in ['both', 'right']:
            return
            
        self._write_pose_data(msg, self.right_csv_writer, 'right')
        
    def _write_pose_data(self, msg, csv_writer, arm_name):
        """写入位姿数据到CSV"""
        try:
            # 提取数据
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            frame_id = msg.header.frame_id
            
            position = msg.pose.position
            orientation = msg.pose.orientation
            
            # 写入一行数据
            row = [
                timestamp_sec,
                timestamp_nanosec,
                frame_id,
                self.recording_phase,
                f'{position.x:.6f}',
                f'{position.y:.6f}',
                f'{position.z:.6f}',
                f'{orientation.w:.6f}',
                f'{orientation.x:.6f}',
                f'{orientation.y:.6f}',
                f'{orientation.z:.6f}'
            ]
            
            if csv_writer:
                csv_writer.writerow(row)
                
                # 更新计数
                if arm_name == 'left':
                    self.left_pose_count += 1
                    pose_count = self.left_pose_count
                else:
                    self.right_pose_count += 1
                    pose_count = self.right_pose_count
                
                # 每50条数据输出一次日志并刷新文件
                if pose_count % 50 == 0:
                    if arm_name == 'left' and self.left_csv_file:
                        self.left_csv_file.flush()
                    elif arm_name == 'right' and self.right_csv_file:
                        self.right_csv_file.flush()
                    
                    elapsed_time = time.time() - self.start_time
                    frequency = pose_count / elapsed_time if elapsed_time > 0 else 0
                    self.get_logger().info(
                        f'[{self.recording_phase}] {arm_name}臂: 已记录 {pose_count} 条数据，频率: {frequency:.2f} Hz'
                    )
                
        except Exception as e:
            self.get_logger().error(f'写入{arm_name}臂数据失败: {e}')


class MoveJAndCircleClient(Node):
    def __init__(self, interface=None):
        super().__init__('movej_and_circle_client')
        
        # 保存机器人接口
        self.interface = interface
        
        # 创建数据记录器（只用于圆弧运动）
        self.pose_recorder = PoseDataRecorder()
        
        # 创建圆弧服务客户端
        self.circle_client = self.create_client(MovecUseIK, '/ocs2_arm_controller/execute_circle_use_ik')
        
        # 创建关节轨迹服务客户端
        self.joint_trajectory_client = self.create_client(
            JointTrajectory, 
            '/ocs2_arm_controller/joint_trajectory_with_para'
        )
        
        # 创建运动学服务客户端
        self.kinematics_client = self.create_client(
            KinematicsService,
            '/kinematics_service'
        )
        
        # 定义关节名称（g1机器人）
        self.left_arm_joint_names = [
            "left_joint1", "left_joint2", "left_joint3",
            "left_joint4", "left_joint5", "left_joint6", "left_joint7"
        ]
        self.right_arm_joint_names = [
            "right_joint1", "right_joint2", "right_joint3",
            "right_joint4", "right_joint5", "right_joint6", "right_joint7"
        ]
        
        # 创建定时器用于检查记录状态
        self.create_timer(5.0, self._check_recording_status)
        
        # 添加一个标志表示是否正在等待圆弧开始
        self.waiting_for_circle_start = False
        
    def _check_recording_status(self):
        """检查记录状态"""
        if self.pose_recorder.is_recording:
            elapsed_time = time.time() - self.pose_recorder.start_time
            total_count = self.pose_recorder.left_pose_count + self.pose_recorder.right_pose_count
            self.get_logger().info(
                f'[圆弧位姿记录中] 阶段: {self.pose_recorder.recording_phase}, '
                f'左臂{self.pose_recorder.left_pose_count}条, '
                f'右臂{self.pose_recorder.right_pose_count}条, '
                f'总计{total_count}条, 运行时间: {elapsed_time:.1f}秒'
            )
        
    def wait_for_services(self, timeout=10.0):
        """等待服务可用"""
        circle_ready = self.circle_client.wait_for_service(timeout_sec=timeout)
        trajectory_ready = self.joint_trajectory_client.wait_for_service(timeout_sec=timeout)
        kinematics_ready = self.kinematics_client.wait_for_service(timeout_sec=timeout)
        
        if circle_ready and trajectory_ready and kinematics_ready:
            self.get_logger().info('所有服务可用')
            return True
        else:
            self.get_logger().error(f'服务不可用 - 圆弧: {circle_ready}, 轨迹: {trajectory_ready}, 运动学: {kinematics_ready}')
            return False

    def get_current_pose_from_kinematics(self, arm_name="right", joint_positions=None):
        """
        通过运动学服务获取当前关节角对应的末端位姿
        
        参数:
            arm_name: "left" 或 "right"
            joint_positions: 7个关节的位置列表（如果为None，则从当前机器人状态获取）
        
        返回:
            pose: geometry_msgs/Pose 对象，包含位置和四元数
        """
        if joint_positions is None:
            # 从当前机器人状态获取关节位置
            current_positions = self.get_current_joint_positions(arm_name)
            if current_positions is None:
                self.get_logger().error('无法获取当前关节位置')
                return None
            joint_positions = current_positions[arm_name]
        
        # 创建运动学请求
        req = KinematicsService.Request()
        req.operation_type = "fk"  # 正运动学
        req.arm_type = arm_name
        req.joint_angles = joint_positions
        
        # 等待服务
        if not self.kinematics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('运动学服务不可用')
            return None
        
        # 调用服务
        future = self.kinematics_client.call_async(req)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().success:
                response = future.result()
                if response.result_poses:
                    pose = response.result_poses[0]
                    self.get_logger().info(f'✓ 获取到位姿: 位置[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}], '
                                          f'四元数[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}]')
                    return pose
                else:
                    self.get_logger().error('运动学服务返回空位姿')
                    return None
            else:
                error_msg = future.result().message if future.result() else '未知错误'
                self.get_logger().error(f'运动学服务调用失败: {error_msg}')
                return None
        except Exception as e:
            self.get_logger().error(f'运动学服务调用异常: {e}')
            return None

    def send_movej_command(self, arm_name="left", joint_positions=None, duration=5.0):
        """
        使用JointTrajectory服务发送关节空间移动命令（不记录数据）
        
        参数:
            arm_name: "left" 或 "right"
            joint_positions: 7个关节的位置列表
            duration: 移动时间(秒)
        """
        if joint_positions is None:
            # 默认安全位置
            if arm_name == "left":
                joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                joint_positions = [-0.5468, -0.5792, 0.7379, -0.7889, -0.0640, 0.1741, 0.2269]
        
        # 获取当前关节位置
        current_positions = self.get_current_joint_positions(arm_name)
        if current_positions is None:
            self.get_logger().error('无法获取当前关节位置')
            return False
        
        # 构建完整的14个关节位置（双臂）
        if arm_name == "left":
            # 左臂移动到目标位置，右臂保持当前位置
            dual_joint_positions = joint_positions + current_positions['right']
        else:
            # 右臂移动到目标位置，左臂保持当前位置
            dual_joint_positions = current_positions['left'] + joint_positions
        
        # 创建请求
        req = JointTrajectory.Request()
        req.joint_names = self.left_arm_joint_names + self.right_arm_joint_names
        
        # 创建轨迹点
        wp = JointWaypoint()
        wp.position = dual_joint_positions
        wp.time_mode = True  # 使用总时间模式
        wp.total_time = duration
        
        # 设置运动限制（可根据需要调整）
        wp.max_velocity = [0.5] * 14  # 最大速度 rad/s
        wp.max_acceleration = [1.0] * 14  # 最大加速度 rad/s²
        wp.max_jerk = [2.0] * 14  # 最大加加速度 rad/s³
        
        req.waypoints = [wp]
        
        # 发送请求（不记录数据）
        self.get_logger().info(f'发送{arm_name}臂MoveJ命令，目标关节: {[f"{j:.3f}" for j in joint_positions[:3]]}...')
        
        future = self.joint_trajectory_client.call_async(req)
        
        # 等待结果
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
            if future.result() and future.result().success:
                planned_duration = future.result().planned_duration
                self.get_logger().info(f'✓ MoveJ成功，规划时长: {planned_duration:.3f}秒')
                
                # 等待运动完成
                wait_time = max(planned_duration, duration) + 1.0  # 多加一点等待时间
                self.get_logger().info(f'等待 {wait_time:.1f} 秒完成运动...')
                time.sleep(wait_time)
                return True
            else:
                error_msg = future.result().message if future.result() else '未知错误'
                self.get_logger().error(f'✗ MoveJ失败: {error_msg}')
                return False
        except Exception as e:
            self.get_logger().error(f'调用MoveJ服务失败: {e}')
            return False

    def get_current_joint_positions(self, arm_name="left"):
        """获取当前关节位置"""
        if self.interface is None:
            self.get_logger().error('机器人接口未初始化')
            return None
        
        joint_state = self.interface.get_joint_state(categorized=False)
        if joint_state is None:
            self.get_logger().error('无法获取关节状态')
            return None
        
        # 创建名称到位置的映射
        all_joint_names = joint_state.get('names', [])
        all_joint_positions = joint_state.get('positions', [])
        joint_name_to_position = dict(zip(all_joint_names, all_joint_positions))
        
        # 提取左臂和右臂位置
        left_positions = [joint_name_to_position.get(name, 0.0) for name in self.left_arm_joint_names]
        right_positions = [joint_name_to_position.get(name, 0.0) for name in self.right_arm_joint_names]
        
        return {'left': left_positions, 'right': right_positions}

    def create_circle_request_three_point(self, arm_name="left", 
                                           midpoint_x=0.2839, midpoint_y=0.5915, midpoint_z=-0.4104,midpoint_qw=1.0,
                                           midpoint_qx=0.0,midpoint_qy=0.0,midpoint_qz=0.0,
                                           endpoint_x=0.3214, endpoint_y=0.4782, endpoint_z=-0.2665,endpoint_qw=1.0,
                                           endpoint_qx=0.0,endpoint_qy=0.0,endpoint_qz=0.0,
                                           rotate_angle=3.9551,duration=5.0,ik_type="BFGS"):
        """创建三点法圆弧请求"""
        request = MovecUseIK.Request()
        circle = CircleMessage()
        
        circle.use_three_point_method = True
        circle.use_slerp_for_orientation = True
        circle.time_mode = True
        circle.frame_id = "base_link"
        circle.arm_name = arm_name
        circle.duration = duration
        
        # 运动参数
        circle.max_linear_velocity = 0.3
        circle.max_linear_acceleration = 0.5
        circle.max_linear_jerk = 3.0
        circle.max_angular_velocity = 1.0
        circle.max_angular_acceleration = 2.0
        circle.max_angular_jerk = 5.0
        
        circle.midpoint.position = Point(x=midpoint_x, y=midpoint_y, z=midpoint_z)
        circle.endpoint.position = Point(x=endpoint_x, y=endpoint_y, z=endpoint_z)
        
        circle.midpoint.orientation = Quaternion(x=midpoint_qx, y=midpoint_qy, z=midpoint_qz, w=midpoint_qw)
        circle.endpoint.orientation = Quaternion(x=endpoint_qx, y=endpoint_qy, z=endpoint_qz, w=endpoint_qw)
        circle.rotate_angle = rotate_angle
        circle.ik_type=ik_type
        request.circle_params = circle
        return request

    def create_circle_request_parametric(self, arm_name="left",
                                          center_x=0.3279, center_y=0.4871, center_z=-0.3826,
                                          axis_x=0.9104, axis_y=0.4054, axis_z=0.0822,
                                          rotate_angle=3.9551,
                                          duration=5.0,
                                          end_pose=None,ik_type="BFGS"):
        """
        创建参数法圆弧请求
        
        参数:
            end_pose: geometry_msgs/Pose 终点姿态（如果提供，将使用其中的四元数）
        """
        request = MovecUseIK.Request()
        circle = CircleMessage()
        
        circle.use_three_point_method = False
        circle.use_slerp_for_orientation = False  # 使用姿态插值
        circle.time_mode = True
        circle.frame_id = "base_link"
        circle.arm_name = arm_name
        circle.duration = duration
        
        circle.max_linear_velocity = 0.3
        circle.max_linear_acceleration = 0.5
        circle.max_linear_jerk = 3.0
        circle.max_angular_velocity = 1.0
        circle.max_angular_acceleration = 2.0
        circle.max_angular_jerk = 5.0
        
        circle.center = Point(x=center_x, y=center_y, z=center_z)
        circle.axis = Vector3(x=axis_x, y=axis_y, z=axis_z)
        circle.rotate_angle = rotate_angle
        
        # 设置终点位置
        circle.endpoint.position = Point(x=0.0, y=0.0, z=0.0)
        circle.ik_type=ik_type
        # 如果提供了终点姿态，使用它；否则使用默认四元数
        if end_pose is not None:
            circle.endpoint.orientation = end_pose.orientation
            self.get_logger().info(f'使用运动学计算的终点姿态: '
                                  f'四元数[{end_pose.orientation.x:.3f}, {end_pose.orientation.y:.3f}, '
                                  f'{end_pose.orientation.z:.3f}, {end_pose.orientation.w:.3f}]')
        else:
            circle.endpoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.get_logger().warn('未提供终点姿态，使用默认四元数')
        
        request.circle_params = circle
        return request

    def call_circle_service_with_recording(self, request, arm='left', timeout=15.0):
        """调用圆弧服务（只记录位姿数据，在服务成功响应后开始记录）"""
        if not self.circle_client.service_is_ready():
            self.get_logger().error('圆弧服务未就绪')
            return None
        
        self.get_logger().info(f'调用{arm}臂圆弧服务...')
        future = self.circle_client.call_async(request)
        
        try:
            # 等待服务响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.result() and future.result().success:
                response = future.result()
                self.get_logger().info(f'✓ 圆弧服务响应成功，开始记录位姿数据...')
                
                # 在服务成功响应后，再开始记录数据
                # 这样可以确保只记录圆弧运动的数据，不包含MoveJ的残留数据
                self.pose_recorder.update_phase('circle')
                self.pose_recorder.start_recording(arm, 'circle')
                
                # 给记录器一点时间初始化
                time.sleep(0.2)
                
                return response
            else:
                error_msg = future.result().message if future.result() else '未知错误'
                self.get_logger().error(f'圆弧服务调用失败: {error_msg}')
                return None
        except Exception as e:
            self.get_logger().error(f'圆弧服务调用异常: {e}')
            return None


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
    """主测试函数"""
    print("=" * 70)
    print("关节空间移动 + 圆弧轨迹服务测试 - 只在圆弧时记录位姿数据")
    print("步骤1: 使用 MoveJ 移动到圆弧起点（不记录数据）")
    print("步骤2: 使用 MoveC 执行圆弧运动（记录位姿数据）")
    print("数据将保存到: /home/lina/lina/data")
    print("=" * 70)
    
    # 检查输出目录
    output_dir = "/home/lina/lina/data"
    if not os.path.exists(output_dir):
        print(f"创建输出目录: {output_dir}")
        os.makedirs(output_dir)
    
    # 初始化ROS 2
    rclpy.init()
    
    # 创建ROS2RobotInterface配置
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
    
    # 创建客户端节点
    client = MoveJAndCircleClient(interface)
    
    # 等待服务
    print("[5] 等待服务...")
    if not client.wait_for_services():
        print("✗ 服务不可用")
        interface.disconnect()
        rclpy.shutdown()
        return 1
    print("✓ 服务已就绪")
    
    # 切换到HOLD状态
    print("\n[6] 切换到HOLD状态...")
    try:
        send_fsm_command(interface, 2)  # HOLD
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    # 切换到HOME状态
    print("\n[7] 切换到HOME状态...")
    try:
        send_fsm_command(interface, 1)  # HOME
        print("等待HOME完成 (5秒)...")
        time.sleep(5.0)
        print("✓ 已回到HOME位置")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    # 切换到HOLD状态
    print("\n[8] 切换到HOLD状态...")
    try:
        send_fsm_command(interface, 2)  # HOLD
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    # 选择测试模式
    print("\n选择测试模式:")
    print("1. 右臂 - MoveJ到起点 + 三点法圆弧")
    print("2. 右臂 - MoveJ到起点 + 参数法圆弧")
    print("3. 仅MoveJ移动（测试，不记录数据）")
    print("4. 仅圆弧运动（需要已在起点，记录数据）")
    
    arm_name = 'right'
    choice = input("请选择(1-4): ").strip()
    
    # 右臂圆弧起点的关节位置
    if choice in ['2','4']:
        # start_joints = [0.1606, -0.4771, -0.5085, 0.1539, 0.4943, -0.0472, 0.5657]
        # center_params = {
        #     'center': (0.076, -0.344, 0.118),
        #     'axis': (0.0, 1.0, 0.0),
        #     'angle': 2.0 * math.pi,
        #     'ik_type':"DLS"
        # }
        # start_joints = [-0.10041490563072086,-0.3431672560123358,-0.1591746416278058,0.3181722465663216,
        #                 0.311619984605325,-0.14628724397975174,0.20848141310625967]
        start_joints = [-0.328385435813621,-0.22000419219377856,-0.2746054955857294,1.0731518620969906,
                        0.44726364968600296,-0.5811312841716576,0.5052403055260443]
        center_params = {
            'center': (0.2520576827979941,0.0,0.09094741848884622),
            'axis': (-1.0, 0.0, 0.0),
            'angle': 1.0/3.0 * math.pi,
            'ik_type':"DLS"
        }
    elif choice in ['1']:
        ###下面的参数是宇树圆心[0.126298 -0.366388 0.135205],转轴：[-0.786041   0.616464 -0.0459511] 半径0.071357
        # start_joints = [0.1722, -0.2144, -0.4844, -0.0738, 0.2343, 0.017, -0.2333]
        # circle_params = {
        #     'midpoint': (0.125121, -0.373182, 0.064187,0.931845,-0.015346 ,0.012323, -0.362322),# [x,y,z,qw,qx,qy,qz]
        #     'endpoint': (0.082612, -0.421051, 0.149154,0.931845,-0.015346,0.012323,-0.362322),
        #     'angle': 2.0 * math.pi, #如果是0就会运动到第三个点的位置，不是整圆
        #     'ik_type':"DLS"
        # }
        
        #宇树测试点：
        # start_joints = [-0.09486280877359009, -0.31784775620251055, -0.12097406175381971, 0.2671227412429476,
        #                 0.29327939104456574, -0.12360212838689603, 0.15756105816112817]
        # circle_params = {
        #     'midpoint': (0.28493124081114657,-0.21010960167080947, 0.15160789617056938,
        #                  0.9998355240354676,-0.011837170939959271, -0.012956892810266867, -0.004574405924628916),# [x,y,z,qw,qx,qy,qz]
        #     'endpoint': (0.14703642497642724,-0.39914187594233796, 0.08492210997294118,
        #                  0.9124853769908218,-0.002862561769619186,0.012258643079609749, -0.40891560032344043),
        #     'angle': 0.0 * math.pi #如果是0就会运动到第三个点的位置，不是整圆
        # }

        ###w2画圆弧测试
        start_joints = [ -1.5009369600996312, -1.2722360477993053, 1.511559794610251, -0.8867350552610855,
                        0.36633719649439095, -0.5669138393503574, -0.33781478093279904]
        circle_params = {
            'midpoint': (0.29341990898031634, -0.5087368643682688, -0.4628460982440809,
                         0.177156205280351, 0.6982249217208578, -0.17696987626750169, 0.6706558733899475),
            'endpoint': (0.25315673326666455, -0.5852422198922865, -0.41319361638728647,
                         0.17900599274650042,0.696889216480679, -0.17734863484029192, 0.671453450534041),
            'angle': 0.0 * math.pi,
            'ik_type':"DLS"
        }
        # ik_type="BFGS"
    elif choice == '3':
        start_joints = [0.1606, -0.4771, -0.5085, 0.1539, 0.4943, -0.0472, 0.5657]
    
    # 切换到MOVEJ状态
    if choice not in ['4']:
        print(f"\n[9] 切换到MOVEJ状态...")
        try:
            send_fsm_command(interface, 4)  # MOVEJ
            print("✓ 已切换到MOVEJ状态")
        except Exception as e:
            print(f"✗ 状态切换失败: {e}")
            return 1
    
    # 关节空间移动到圆弧起点（不记录数据）
    if choice not in ['4']:
        print(f"\n[10] MoveJ移动到{arm_name}臂圆弧起点（不记录数据）...")
        if not client.send_movej_command(arm_name, start_joints, duration=3.0):
            print("✗ MoveJ到起点失败")
            return 1
        print("✓ MoveJ完成")
        
        # MoveJ完成后，额外等待一下确保完全稳定
        print("等待2秒确保机器人稳定...")
        time.sleep(2.0)
    
    # 对于参数法圆弧，先获取当前位姿用于终点姿态
    end_pose = None
    if choice in ['2']:
        print(f"\n[11] 获取{arm_name}臂当前位姿（用于圆弧终点姿态）...")
        end_pose = client.get_current_pose_from_kinematics(arm_name, start_joints)
        if end_pose is None:
            print("⚠ 警告: 无法获取当前位姿，将使用默认四元数")
    
    # 执行圆弧运动（记录数据）
    if choice not in ['3']:
        print(f"\n[12] 执行{arm_name}臂圆弧运动（记录位姿数据中）...")
        
        if choice in ['1']:
            circle_req = client.create_circle_request_three_point(
                arm_name=arm_name,
                midpoint_x=circle_params['midpoint'][0],
                midpoint_y=circle_params['midpoint'][1],
                midpoint_z=circle_params['midpoint'][2],
                midpoint_qw=circle_params['midpoint'][3],
                midpoint_qx=circle_params['midpoint'][4],
                midpoint_qy=circle_params['midpoint'][5],
                midpoint_qz=circle_params['midpoint'][6],
                endpoint_x=circle_params['endpoint'][0],
                endpoint_y=circle_params['endpoint'][1],
                endpoint_z=circle_params['endpoint'][2],
                endpoint_qw=circle_params['endpoint'][3],
                endpoint_qx=circle_params['endpoint'][4],
                endpoint_qy=circle_params['endpoint'][5],
                endpoint_qz=circle_params['endpoint'][6],
                rotate_angle=circle_params['angle'],
                duration=6.0,
                ik_type=circle_params['ik_type']
            )
            method_name = "三点法"
        elif choice in ['2']:
            circle_req = client.create_circle_request_parametric(
                arm_name=arm_name,
                center_x=center_params['center'][0],
                center_y=center_params['center'][1],
                center_z=center_params['center'][2],
                axis_x=center_params['axis'][0],
                axis_y=center_params['axis'][1],
                axis_z=center_params['axis'][2],
                rotate_angle=center_params['angle'],
                duration=6.0,
                end_pose=end_pose,
                ik_type=center_params['ik_type']
            )
            method_name = "参数法"
        elif choice == '4':
            # 仅圆弧运动，需要用户提供参数
            print("\n仅圆弧运动模式，请手动配置参数")
            method_type = input("选择方法 (1=三点法, 2=参数法): ").strip()
            if method_type == '1':
                circle_req = client.create_circle_request_three_point(arm_name=arm_name, duration=6.0)
                method_name = "三点法"
            else:
                circle_req = client.create_circle_request_parametric(arm_name=arm_name, duration=6.0)
                method_name = "参数法"
        
        circle_response = client.call_circle_service_with_recording(circle_req, arm_name)
        
        if circle_response and circle_response.success:
            print(f"✓ 圆弧运动({method_name})服务调用成功!")
            print(f"  消息: {circle_response.message}")
            print(f"  预计时长: {circle_response.estimated_duration:.2f}秒")
            
            wait_time = circle_response.estimated_duration + 2.0
            print(f"等待 {wait_time:.1f} 秒完成轨迹...")
            
            # 在等待期间持续处理记录器事件
            spin_recorders(client, wait_time)
            
            print("✓ 轨迹执行完成")
            
            # 停止记录
            client.pose_recorder.stop_recording()
            print("✓ 位姿数据已保存")
        else:
            error_msg = circle_response.message if circle_response else '未知错误'
            print(f"✗ 圆弧运动失败: {error_msg}")
            # 确保记录器停止
            if client.pose_recorder.is_recording:
                client.pose_recorder.stop_recording()
            return 1
    
    # 清理
    print("\n[13] 清理...")
    
    # 确保记录器已停止
    if client.pose_recorder.is_recording:
        client.pose_recorder.stop_recording()
    
    # 切换回hold状态
    try:
        send_fsm_command(interface, 2)  # HOLD
        time.sleep(0.5)
        print("✓ 已切换回hold状态")
    except Exception as e:
        print(f"⚠ 状态切换失败: {e}")
    
    # 断开连接
    print("\n[14] 断开连接...")
    interface.disconnect()
    
    # 销毁节点
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