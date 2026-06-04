#!/usr/bin/env python3
"""
圆弧轨迹 Service 客户端 - 简化版
"""

import rclpy
from rclpy.node import Node
import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 导入自定义消息类型
from arms_ros2_control_msgs.srv import ExecuteCircle
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from arms_ros2_control_msgs.msg import CircleMessage


class CircleServiceClient(Node):
    def __init__(self):
        super().__init__('circle_service_client')
        
        # 创建Service客户端
        self.left_client = self.create_client(ExecuteCircle, '/execute_left_circle')
        self.right_client = self.create_client(ExecuteCircle, '/execute_right_circle')
        
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
        circle.duration = 5.0
        
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
        circle.duration = 5.0
        
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

    def call_service(self, request, arm='left', timeout=10.0):
        """调用服务"""
        client = self.left_client if arm == 'left' else self.right_client
        
        if not client.service_is_ready():
            self.get_logger().error(f'{arm}臂服务未就绪')
            return None
        
        future = client.call_async(request)
        
        # 等待响应
        start_time = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('服务调用超时')
                return None
        
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f'获取响应失败: {e}')
            return None


def main():
    """主测试函数"""
    print("=" * 70)
    print("圆弧轨迹服务测试")
    print("=" * 70)
    
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
    
    # 创建服务客户端
    print("[3] 创建圆弧服务客户端...")
    client = CircleServiceClient()
    
    # 等待服务
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
    print("调用圆弧服务...")
    response = client.call_service(request, arm)
    
    if response:
        if response.success:
            print(f"✓ 圆弧轨迹执行成功")
            print(f"消息: {response.message}")
            print(f"预计时长: {response.estimated_duration:.2f}秒")
            
            # 等待执行完成
            wait_time = response.estimated_duration + 2.0
            print(f"等待 {wait_time:.1f} 秒...")
            time.sleep(wait_time)
        else:
            print(f"✗ 圆弧轨迹执行失败: {response.message}")
    else:
        print("✗ 服务调用失败")
    
    # 清理
    print("\n[4] 清理...")
    client.destroy_node()
    
    # 切换回HOLD状态
    try:
        interface.send_fsm_command(2)
        time.sleep(1.0)
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"⚠ 状态切换失败: {e}")
    
    interface.disconnect()
    print("✓ 已断开连接")
    
    rclpy.shutdown()
    print("\n测试完成！")
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