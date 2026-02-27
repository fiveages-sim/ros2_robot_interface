#!/usr/bin/env python3
"""
圆弧轨迹 Service 客户端 - 带数据记录功能
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os
import csv
from datetime import datetime
import threading
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 导入自定义消息类型
from arms_ros2_control_msgs.srv import ExecuteCircle
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, PoseStamped
from arms_ros2_control_msgs.msg import CircleMessage


class PoseDataRecorder(Node):
    """数据记录器 - 订阅PoseStamped数据并保存到CSV"""
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
            '/right_current_pose',  # 你的PoseBasedReferenceManager发布的话题
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
        
        self.get_logger().info('数据记录器初始化完成')
        
    def start_recording(self, arm='both'):
        """开始记录数据"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.recording_arm = arm
        
        if arm == 'both' or arm == 'left':
            # 创建左臂CSV文件
            left_filename = os.path.join(self.output_dir, f'left_pose_{timestamp}.csv')
            self.left_csv_file = open(left_filename, 'w', newline='')
            self.left_csv_writer = csv.writer(self.left_csv_file)
            
            # 写入表头
            header = ['timestamp_sec', 'timestamp_nanosec', 'frame_id',
                     'position_x', 'position_y', 'position_z',
                     'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z']
            self.left_csv_writer.writerow(header)
            self.left_csv_file.flush()  # 立即写入文件
            self.get_logger().info(f'左臂数据将记录到: {left_filename}')
        
        if arm == 'both' or arm == 'right':
            # 创建右臂CSV文件
            right_filename = os.path.join(self.output_dir, f'right_pose_{timestamp}.csv')
            self.right_csv_file = open(right_filename, 'w', newline='')
            self.right_csv_writer = csv.writer(self.right_csv_file)
            
            # 写入表头
            header = ['timestamp_sec', 'timestamp_nanosec', 'frame_id',
                     'position_x', 'position_y', 'position_z',
                     'orientation_w', 'orientation_x', 'orientation_y', 'orientation_z']
            self.right_csv_writer.writerow(header)
            self.right_csv_file.flush()  # 立即写入文件
            self.get_logger().info(f'右臂数据将记录到: {right_filename}')
        
        self.is_recording = True
        self.start_time = time.time()
        self.left_pose_count = 0
        self.right_pose_count = 0
        
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
                        f'{arm_name}臂: 已记录 {pose_count} 条数据，频率: {frequency:.2f} Hz'
                    )
                
        except Exception as e:
            self.get_logger().error(f'写入{arm_name}臂数据失败: {e}')


class CircleServiceClient(Node):
    def __init__(self):
        super().__init__('circle_service_client')
        
        # 创建Service客户端
        self.left_client = self.create_client(ExecuteCircle, '/execute_left_circle')
        self.right_client = self.create_client(ExecuteCircle, '/execute_right_circle')
        
        # 创建数据记录器
        self.data_recorder = PoseDataRecorder()
        
        # 定时器用于检查记录状态
        self.create_timer(5.0, self._check_recording_status)
        
    def wait_for_service(self, arm='left', timeout=10.0):
        """等待服务可用"""
        client = self.left_client if arm == 'left' else self.right_client
        if client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f'{arm}臂圆弧服务可用')
            return True
        else:
            self.get_logger().error(f'{arm}臂圆弧服务不可用')
            return False

    def create_three_point_request(self, arm='left'):
        """创建三点法请求"""
        request = ExecuteCircle.Request()
        circle = CircleMessage()
        
        # 基本配置
        circle.use_three_point_method = True
        circle.use_slerp_for_orientation = True
        circle.time_mode = True
        circle.frame_id = "arm_base"
        circle.duration = 10.0
        
        # 运动参数
        circle.max_linear_velocity = 0.5
        circle.max_linear_acceleration = 1.0
        circle.max_linear_jerk = 6.0
        circle.max_angular_velocity = 2.0
        circle.max_angular_acceleration = 4.0
        circle.max_angular_jerk = 10.0
        
        if arm == 'left':
            # 左臂参数
            circle.midpoint.position = Point(x=0.2839, y=0.5915, z=-0.4104)
            circle.midpoint.orientation = Quaternion(x=0.7221, y=0.0005, z=0.6918, w=-0.0046)
            circle.endpoint.position = Point(x=0.3214, y=0.4782, z=-0.2665)
            circle.endpoint.orientation = Quaternion(x=0.7039, y=-0.1625, z=0.6717, w=-0.1643)
        else:
            # 右臂参数
            circle.midpoint.position = Point(x=0.3493, y=-0.4327, z=-0.3228)
            circle.midpoint.orientation = Quaternion(x=0.7173, y=0.0004, z=0.6967, w=0.0036)
            circle.endpoint.position = Point(x=0.2915, y=-0.2474, z=-0.3614)
            circle.endpoint.orientation = Quaternion(x=0.8192, y=0.0054, z=0.5735, w=0.0002)
        
        request.circle_params = circle
        return request

    def create_parametric_request(self, arm='left'):
        """创建参数法请求"""
        request = ExecuteCircle.Request()
        circle = CircleMessage()
        
        # 基本配置
        circle.use_three_point_method = False
        circle.use_slerp_for_orientation = True
        circle.time_mode = True
        circle.frame_id = "arm_base"
        circle.duration = 10.0
        
        # 运动参数
        circle.max_linear_velocity = 0.5
        circle.max_linear_acceleration = 1.0
        circle.max_linear_jerk = 6.0
        circle.max_angular_velocity = 2.0
        circle.max_angular_acceleration = 4.0
        circle.max_angular_jerk = 10.0
        
        if arm == 'left':
            # 左臂参数
            circle.endpoint.position = Point(x=0.3214, y=0.4782, z=-0.2665)
            circle.endpoint.orientation = Quaternion(x=0.7039, y=-0.1625, z=0.6717, w=-0.1643)
            circle.center = Point(x=0.3279, y=0.4871, z=-0.3826)
            circle.axis = Vector3(x=0.9104, y=0.4054, z=0.0822)
            circle.rotate_angle = 3.9551
        else:
            # 右臂参数
            circle.endpoint.position = Point(x=0.2915, y=-0.2474, z=-0.3614)
            circle.endpoint.orientation = Quaternion(x=0.8192, y=0.0054, z=0.5735, w=0.0002)
            circle.center = Point(x=0.3302, y=-0.3467, z=-0.3885)
            circle.axis = Vector3(x=-0.9345, y=-0.3227, z=-0.1502)
            circle.rotate_angle = 3.6985
        
        request.circle_params = circle
        return request

    def call_service_with_recording(self, request, arm='left', timeout=10.0):
        """调用服务，并在调用后开始记录数据"""
        client = self.left_client if arm == 'left' else self.right_client
        
        if not client.service_is_ready():
            self.get_logger().error(f'{arm}臂服务未就绪')
            return None
        
        # 先开始记录数据（在调用服务前）
        self.get_logger().info(f'开始记录{arm}臂数据...')
        self.data_recorder.start_recording(arm)
        
        # 确保记录器已经开始运行
        time.sleep(0.5)
        
        # 调用服务
        future = client.call_async(request)
        self.get_logger().info(f'{arm}臂服务已调用，等待响应...')
        
        # 等待响应
        start_time = time.time()
        response_received = False
        response = None
        
        while rclpy.ok() and not response_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                try:
                    response = future.result()
                    response_received = True
                    self.get_logger().info(f'{arm}臂服务响应已接收')
                except Exception as e:
                    self.get_logger().error(f'获取响应失败: {e}')
                    break
            
            if time.time() - start_time > timeout:
                self.get_logger().error('服务调用超时')
                break
        
        return response
    
    def _check_recording_status(self):
        """检查记录状态"""
        if self.data_recorder.is_recording:
            elapsed_time = time.time() - self.data_recorder.start_time
            total_count = self.data_recorder.left_pose_count + self.data_recorder.right_pose_count
            self.get_logger().info(
                f'记录中: 左臂{self.data_recorder.left_pose_count}条，'
                f'右臂{self.data_recorder.right_pose_count}条，'
                f'总计{total_count}条，运行时间: {elapsed_time:.1f}秒'
            )


def main():
    """主测试函数"""
    print("=" * 70)
    print("圆弧轨迹服务测试 - 带数据记录功能")
    print("数据将保存到: /home/lina/lina/data")
    print("=" * 70)
    
    # 检查输出目录
    output_dir = "/home/lina/lina/data"
    if not os.path.exists(output_dir):
        print(f"创建输出目录: {output_dir}")
        os.makedirs(output_dir)
    
    # 初始化ROS
    rclpy.init()
    
    # 创建接口
    print("[1] 创建机器人接口...")
    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)
    
    try:
        interface.connect()
        print("✓ 接口连接成功")
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        return 1
    
    time.sleep(2.0)

    # 检查是否是双臂模式
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    if not is_dual_arm:
        print("⚠ 警告: 当前不是双臂模式，将只控制左臂\n")
    
    # 切换到OCS2状态
    print("[2] 切换到OCS2状态...")
    try:
        interface.send_fsm_command(2)  # Hold
        time.sleep(0.1)
        interface.send_fsm_command(1)  # HOME
        time.sleep(5.0)
        interface.send_fsm_command(2)  # Hold
        time.sleep(0.1)
        interface.send_fsm_command(3)  # OCS2
        time.sleep(1.0)
        print("✓ 已切换到OCS2状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        interface.disconnect()
        return 1
    
    # 创建服务客户端和数据记录器
    print("[3] 创建圆弧服务客户端和数据记录器...")
    client = CircleServiceClient()
    
    # 等待服务
    print("等待圆弧服务可用...")
    if not client.wait_for_service('left'):
        print("✗ 服务不可用")
        interface.disconnect()
        return 1
    
    # 选择测试模式
    print("\n选择测试模式:")
    print("1. 左臂三点法")
    print("2. 左臂参数法")
    print("3. 右臂三点法")
    print("4. 右臂参数法")
    
    choice = input("请选择(1-4): ").strip()
    
    arm = 'left' if choice in ['1', '2'] else 'right'
    method = '三点法' if choice in ['1', '3'] else '参数法'
    
    print(f"\n开始{arm}臂{method}测试...")
    
    # 创建请求
    if choice in ['1', '3']:
        request = client.create_three_point_request(arm)
    else:
        request = client.create_parametric_request(arm)
    
    # 调用服务
    print(f"调用{arm}臂圆弧服务...")
    response = client.call_service_with_recording(request, arm)
    
    if response:
        if response.success:
            print(f"\n✓ 圆弧轨迹执行成功")
            print(f"消息: {response.message}")
            print(f"预计时长: {response.estimated_duration:.2f}秒")
            
            # 等待执行完成
            wait_time = response.estimated_duration + 3.0  # 额外加3秒缓冲
            print(f"\n等待执行完成 (约 {wait_time:.1f} 秒)...")
            
            # 创建单独的线程来等待并停止记录
            def wait_and_stop():
                time.sleep(wait_time)
                print("执行完成，停止数据记录...")
                client.data_recorder.stop_recording()
            
            wait_thread = threading.Thread(target=wait_and_stop)
            wait_thread.start()
            
            # 在主线程中继续处理ROS事件
            start_time = time.time()
            last_log_time = start_time
            
            while wait_thread.is_alive():
                # 处理ROS事件
                rclpy.spin_once(client, timeout_sec=0.1)
                rclpy.spin_once(client.data_recorder, timeout_sec=0.1)
                
                # 每2秒显示一次剩余时间
                current_time = time.time()
                if current_time - last_log_time >= 2.0:
                    elapsed = current_time - start_time
                    remaining = max(0, wait_time - elapsed)
                    
                    # 获取当前记录的数据量
                    left_count = client.data_recorder.left_pose_count
                    right_count = client.data_recorder.right_pose_count
                    total_count = left_count + right_count
                    
                    print(f"剩余时间: {remaining:.1f}秒 | "
                          f"已记录: 左臂{left_count}条，右臂{right_count}条，总计{total_count}条")
                    
                    last_log_time = current_time
            
            wait_thread.join()
            print("数据记录已停止")
            
        else:
            print(f"✗ 圆弧轨迹执行失败: {response.message}")
            # 等待一小段时间让可能的最后几条数据被记录
            time.sleep(0.5)
            client.data_recorder.stop_recording()
    else:
        print("✗ 服务调用失败")
        # 等待一小段时间让可能的最后几条数据被记录
        time.sleep(0.5)
        client.data_recorder.stop_recording()
    
    # 清理
    print("\n[4] 清理...")
    
    # 确保记录器已停止
    if client.data_recorder.is_recording:
        print("停止剩余的数据记录...")
        client.data_recorder.stop_recording()
    
    # 切换回HOLD状态
    try:
        print("切换回HOLD状态...")
        interface.send_fsm_command(2)
        time.sleep(1.0)
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"⚠ 状态切换失败: {e}")
    
    interface.disconnect()
    print("✓ 已断开连接")
    
    # 销毁节点
    client.data_recorder.destroy_node()
    client.destroy_node()
    
    rclpy.shutdown()
    
    # 显示记录的文件
    print("\n" + "=" * 70)
    print("测试完成！")
    print("生成的数据文件:")
    
    csv_files = [f for f in os.listdir(output_dir) if f.endswith('.csv')]
    
    if csv_files:
        for file in csv_files:
            file_path = os.path.join(output_dir, file)
            if os.path.exists(file_path):
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                        line_count = len(lines) - 1  # 减掉表头
                    file_size = os.path.getsize(file_path) / 1024  # KB
                    print(f"  {file}: {line_count} 行数据, {file_size:.2f} KB")
                except Exception as e:
                    print(f"  {file}: 读取失败 - {e}")
    else:
        print("  未找到CSV文件")
    
    print("=" * 70)
    
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n用户中断")
        # 尝试清理
        try:
            rclpy.shutdown()
        except:
            pass
        sys.exit(1)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        try:
            rclpy.shutdown()
        except:
            pass
        sys.exit(1)