#!/usr/bin/env python3
"""
关节空间+圆弧轨迹 Service 客户端
先使用关节空间移动到圆弧起点，再执行圆弧运动
服务: 
  - 关节移动: /ocs2_arm_controller/target_joint_position (topic)
  - 圆弧: /ocs2_arm_controller/execute_circle_use_ik (service)
"""

import rclpy
from rclpy.node import Node
import time
import sys
import math
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from std_msgs.msg import Float64MultiArray, Int32

# 导入服务类型
from arms_ros2_control_msgs.srv import MovecUseIK
from arms_ros2_control_msgs.msg import CircleMessage


class MoveJAndCircleClient(Node):
    def __init__(self):
        super().__init__('movej_and_circle_client')
        
        # 创建圆弧服务客户端
        self.circle_client = self.create_client(MovecUseIK, '/ocs2_arm_controller/execute_circle_use_ik')
        
        # 创建关节位置发布器
        self.left_joint_pub = self.create_publisher(
            Float64MultiArray, 
            '/ocs2_arm_controller/target_joint_position',
            10
        )
        self.right_joint_pub = self.create_publisher(
            Float64MultiArray,
            '/ocs2_arm_controller/target_joint_position_right',  # 右臂的话题，根据实际情况调整
            10
        )
        
    def wait_for_services(self, timeout=10.0):
        """等待服务可用"""
        circle_ready = self.circle_client.wait_for_service(timeout_sec=timeout)
        
        if circle_ready:
            self.get_logger().info('圆弧服务可用')
            return True
        else:
            self.get_logger().error('圆弧服务不可用')
            return False

    def send_movej_command(self, arm_name="left", joint_positions=None, duration=3.0):
        """
        发送关节空间移动命令
        
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
                joint_positions = [0.-0.5468, -0.5792, 0.7379, -0.7889, -0.0640, 0.1741, 0.2269]
        
        msg = Float64MultiArray()
        msg.data = joint_positions
        
        if arm_name == "left":
            self.left_joint_pub.publish(msg)
            self.get_logger().info(f'发送左臂关节目标: {[f"{j:.3f}" for j in joint_positions[:3]]}...')
        else:
            self.right_joint_pub.publish(msg)
            self.get_logger().info(f'发送右臂关节目标: {[f"{j:.3f}" for j in joint_positions[:3]]}...')
        
        # 等待移动完成
        time.sleep(duration)
        return True

    def get_default_joint_positions(self, arm_name="left", pose_name="home"):
        """
        获取预设的关节位置
        
        参数:
            arm_name: "left" 或 "right"
            pose_name: "home", "rest", "ready", "circle_start"
        """
        if arm_name == "left":
            positions = {
                "home": [0.0, 0.2, 0.0, 0.0, -0.2, 0.0, 0.0],
                "rest": [0.0, 0.2, 0.0, 1.57, 0.0, 0.0, 0.0],
                "ready": [0.0, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0],
                "circle_start": [0.0, 0.2, 0.0, 0.3, 0.0, 0.0, 0.0]
            }
        else:
            positions = {
                "home": [0.0, -0.2, 0.0, 0.0, 0.2, 0.0, 0.0],
                "rest": [0.0, -0.2, 0.0, 1.57, 0.0, 0.0, 0.0],
                "ready": [0.0, -0.3, 0.0, 0.5, 0.0, 0.0, 0.0],
                "circle_start": [0.0, -0.2, 0.0, 0.3, 0.0, 0.0, 0.0]
            }
        
        return positions.get(pose_name, positions["home"])

    def create_circle_request_three_point(self, arm_name="left", 
                                           midpoint_x=0.2839, midpoint_y=0.5915, midpoint_z=-0.4104,
                                           endpoint_x=0.3214, endpoint_y=0.4782, endpoint_z=-0.2665,
                                           duration=5.0):
        """
        创建三点法圆弧请求
        """
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
        
        circle.midpoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        circle.endpoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        request.circle_params = circle
        return request

    def create_circle_request_parametric(self, arm_name="left",
                                          center_x=0.3279, center_y=0.4871, center_z=-0.3826,
                                          axis_x=0.9104, axis_y=0.4054, axis_z=0.0822,
                                          rotate_angle=3.9551,
                                          duration=5.0):
        """
        创建参数法圆弧请求
        """
        request = MovecUseIK.Request()
        circle = CircleMessage()
        
        circle.use_three_point_method = False
        circle.use_slerp_for_orientation = False
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
        
        circle.endpoint.position = Point(x=0.0, y=0.0, z=0.0)
        circle.endpoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        request.circle_params = circle
        return request

    def create_planar_circle_request(self, arm_name="left", 
                                      center_x=0.3, center_y=0.0, center_z=-0.2,
                                      radius=0.12, duration=6.0):
        """
        创建平面画圆请求
        """
        request = MovecUseIK.Request()
        circle = CircleMessage()
        
        circle.use_three_point_method = True
        circle.use_slerp_for_orientation = True
        circle.time_mode = True
        circle.frame_id = "base_link"
        circle.arm_name = arm_name
        circle.duration = duration
        
        circle.max_linear_velocity = 0.2
        circle.max_linear_acceleration = 0.3
        circle.max_linear_jerk = 2.0
        circle.max_angular_velocity = 0.8
        circle.max_angular_acceleration = 1.5
        circle.max_angular_jerk = 4.0
        
        circle.midpoint.position = Point(
            x=center_x + radius, 
            y=center_y, 
            z=center_z
        )
        circle.endpoint.position = Point(
            x=center_x, 
            y=center_y, 
            z=center_z + radius
        )
        
        circle.midpoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        circle.endpoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        request.circle_params = circle
        return request

    def call_circle_service(self, request, timeout=15.0):
        """调用圆弧服务"""
        if not self.circle_client.service_is_ready():
            self.get_logger().error('圆弧服务未就绪')
            return None
        
        future = self.circle_client.call_async(request)
        
        start_time = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('圆弧服务调用超时')
                return None
        
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f'获取圆弧响应失败: {e}')
            return None


def send_fsm_command(node, command, topic="/fsm_command"):
    """发送FSM状态切换命令"""
    publisher = node.create_publisher(Int32, topic, 10)
    msg = Int32()
    msg.data = command
    
    for _ in range(3):
        publisher.publish(msg)
        time.sleep(0.1)
    
    node.get_logger().info(f'发送FSM命令: {command}')
    time.sleep(0.5)


def main():
    """主测试函数"""
    print("=" * 70)
    print("关节空间移动 + 圆弧轨迹服务测试")
    print("步骤1: 使用 MoveJ 移动到圆弧起点")
    print("步骤2: 使用 MoveC 执行圆弧运动")
    print("=" * 70)
    
    rclpy.init()
    client = MoveJAndCircleClient()
    
    # 等待服务
    print("[1] 等待服务...")
    if not client.wait_for_services():
        print("✗ 服务不可用")
        rclpy.shutdown()
        return 1
    print("✓ 服务已就绪")
    
    # 创建FSM命令发布器
    fsm_pub = client.create_publisher(Int32, '/fsm_command', 10)
    
    # 切换到MOVEJ状态
    print("\n[2] 切换到MOVEJ状态...")
    try:
        send_fsm_command(client, 2)  # HOLD
        send_fsm_command(client, 1)  # HOME
        print("  等待HOME完成 (5秒)...")
        time.sleep(5.0)
        send_fsm_command(client, 2)  # HOLD
        send_fsm_command(client, 4)  # MOVEJ
        time.sleep(1.0)
        print("✓ 已切换到MOVEJ状态")
    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        return 1
    
    # 选择测试模式
    print("\n选择测试模式:")
    print("1. 左臂 - MoveJ到起点 + 三点法圆弧")
    print("2. 左臂 - MoveJ到起点 + 参数法圆弧")
    print("3. 左臂 - MoveJ到起点 + 平面画圆")
    print("4. 右臂 - MoveJ到起点 + 三点法圆弧")
    print("5. 右臂 - MoveJ到起点 + 参数法圆弧")
    print("6. 右臂 - MoveJ到起点 + 平面画圆")
    print("7. 仅MoveJ移动（测试）")
    print("8. 仅圆弧运动（需要已在起点）")
    
    choice = input("请选择(1-8): ").strip()
    
    if choice in ['1', '2', '3']:
        arm_name = 'left'
    elif choice in ['4', '5', '6', '7', '8']:
        arm_name = 'right'
    else:
        print("无效选择")
        return 1
    
    # 根据手臂设置起点关节位置和圆弧参数
    if arm_name == 'left':
        # 左臂圆弧起点的关节位置
        start_joints = [0.2, 0.3, 0.1, 0.5, 0.0, 0.0, 0.0]
        circle_params = {
            'midpoint': (0.2839, 0.5915, -0.4104),
            'endpoint': (0.3214, 0.4782, -0.2665)
        }
        center_params = {
            'center': (0.3279, 0.4871, -0.3826),
            'axis': (0.9104, 0.4054, 0.0822),
            'angle': 3.9551
        }
    else:
        # 右臂圆弧起点的关节位置
        start_joints = [0.2, -0.3, -0.1, 0.5, 0.0, 0.0, 0.0]
        circle_params = {
            'midpoint': (0.3493, -0.4327, -0.3228),
            'endpoint': (0.2915, -0.2474, -0.3614)
        }
        center_params = {
            'center': (0.3302, -0.3467, -0.3885),
            'axis': (-0.9345, -0.3227, -0.1502),
            'angle': 3.6985
        }
    
    # 先移动到HOME位置
    print(f"\n[3] MoveJ移动到{arm_name}臂HOME位置...")
    home_joints = client.get_default_joint_positions(arm_name, "home")
    client.send_movej_command(arm_name, home_joints, duration=3.0)
    
    # 关节空间移动到圆弧起点
    if choice not in ['8']:
        print(f"\n[4] MoveJ移动到{arm_name}臂圆弧起点...")
        client.send_movej_command(arm_name, start_joints, duration=3.0)
    
    # 切换到OCS2状态（圆弧运动需要OCS2状态）
    print(f"\n[5] 切换到OCS2状态...")
    send_fsm_command(client, 2)  # HOLD
    send_fsm_command(client, 3)  # OCS2
    time.sleep(1.0)
    
    # 执行圆弧运动
    if choice not in ['7']:
        print(f"\n[6] 执行{arm_name}臂圆弧运动...")
        
        if choice in ['1', '4']:
            circle_req = client.create_circle_request_three_point(
                arm_name=arm_name,
                midpoint_x=circle_params['midpoint'][0],
                midpoint_y=circle_params['midpoint'][1],
                midpoint_z=circle_params['midpoint'][2],
                endpoint_x=circle_params['endpoint'][0],
                endpoint_y=circle_params['endpoint'][1],
                endpoint_z=circle_params['endpoint'][2],
                duration=6.0
            )
            method_name = "三点法"
        elif choice in ['2', '5']:
            circle_req = client.create_circle_request_parametric(
                arm_name=arm_name,
                center_x=center_params['center'][0.0748],
                center_y=center_params['center'][-0.1274],
                center_z=center_params['center'][0.2670],
                axis_x=center_params['axis'][0.7071],
                axis_y=center_params['axis'][-0.7071],
                axis_z=center_params['axis'][0],
                rotate_angle=center_params['angle'][2.0*math.pi],
                duration=6.0
            )
            method_name = "参数法"
        elif choice in ['3', '6']:
            circle_req = client.create_planar_circle_request(
                arm_name=arm_name,
                center_x=0.25,
                center_y=0.0 if arm_name == 'left' else 0.0,
                center_z=-0.25,
                radius=0.1,
                duration=8.0
            )
            method_name = "平面画圆"
        else:
            print("无效选择")
            return 1
        
        circle_response = client.call_circle_service(circle_req)
        
        if circle_response and circle_response.success:
            print(f"✓ 圆弧运动({method_name})成功!")
            print(f"  消息: {circle_response.message}")
            print(f"  预计时长: {circle_response.estimated_duration:.2f}秒")
            
            wait_time = circle_response.estimated_duration + 2.0
            print(f"等待 {wait_time:.1f} 秒完成轨迹...")
            time.sleep(wait_time)
            print("✓ 轨迹执行完成")
        else:
            error_msg = circle_response.message if circle_response else '未知错误'
            print(f"✗ 圆弧运动失败: {error_msg}")
            return 1
    
    # 清理
    print("\n[7] 清理...")
    
    # 切换回MOVEJ状态
    send_fsm_command(client, 2)  # HOLD
    time.sleep(0.5)
    send_fsm_command(client, 4)  # MOVEJ
    print("✓ 已切换回MOVEJ状态")
    
    client.destroy_node()
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