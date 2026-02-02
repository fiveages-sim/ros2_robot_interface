#!/usr/bin/env python3
"""
CircleMessage Topic 发布器
用于向 /left_circle 发送圆形轨迹指令
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 导入自定义消息类型
from arms_ros2_control_msgs.msg import CircleMessage
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3


class CircleMessagePublisher(Node):
    def __init__(self):
        super().__init__('circle_message_publisher')
        
        # 配置QoS（服务质量），确保与控制器匹配
        qos_profile = QoSProfile(
            depth=10,  # 队列深度
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
            history=HistoryPolicy.KEEP_LAST  # 保留最后几条消息
        )
        
        # 创建Publisher
        self.publisher_ = self.create_publisher(
            CircleMessage, 
            '/left_circle', 
            qos_profile
        )
        
        # 等待1秒确保连接建立
        self.get_logger().info('等待Publisher连接建立...')
        rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().info('CircleMessage发布器已启动，准备发送消息')

    def publish_three_point_method(self):
        """使用三点法发送圆形轨迹"""
        msg = CircleMessage()
        
        # ====== 配置选项 ======
        msg.use_three_point_method = True
        msg.use_slerp_for_orientation = True
        msg.time_mode = True
        msg.frame_id = "arm_base"  # 重要：必须与控制器期望的坐标系一致
        
        # ====== 三点法参数 ======
        # 中间点
        msg.midpoint.position = Point(x=0.28393430066, y=0.59148, z=-0.4104)
        msg.midpoint.orientation = Quaternion(x=0.722056434171141, y=0.0004850863832760574, z=0.6918018220298489, w=-0.0046058688989129674)
        
        # 终点
        msg.endpoint.position = Point(x=0.3214, y=0.478193, z=-0.266509)
        msg.endpoint.orientation = Quaternion(x=0.7038671000308367, y=-0.16254370163904164, z=0.6716754422266904, w=-0.1643251376425349)
        
        # ====== 运动参数 ======
        msg.max_linear_velocity = 0.5
        msg.max_linear_acceleration = 1.0
        msg.max_linear_jerk = 6.0
        msg.max_angular_velocity = 2.0
        msg.max_angular_acceleration = 4.0
        msg.max_angular_jerk = 10.0
        msg.duration = 5.0  # 运动持续5秒
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info('已发送三点法圆形轨迹指令')
        self.print_message_summary(msg, "三点法")
        
        return msg

    def publish_parametric_method(self):
        """使用参数法发送圆形轨迹"""
        msg = CircleMessage()
        
        # ====== 配置选项 ======
        msg.use_three_point_method = False  # 使用参数法
        msg.use_slerp_for_orientation = True
        msg.time_mode = True
        msg.frame_id = "arm_base"
        # 终点
        msg.endpoint.position = Point(x=0.3214, y=0.478193, z=-0.266509)
        msg.endpoint.orientation = Quaternion(x=0.7038671000308367, y=-0.16254370163904164, z=0.6716754422266904, w=-0.1643251376425349)

        
        # ====== 参数法参数 ======
        # 圆心
        msg.center = Point(x=0.327903, y=0.487114, z=-0.382598)
        
        # 旋转轴（绕Z轴）
        msg.axis = Vector3(x=0.910422, y=0.405441, z=0.0821571)
        
        # 旋转角度（90度 = π/2 ≈ 1.57弧度）
        msg.rotate_angle = 3.9551
        
        # ====== 运动参数（与三点法相同）======
        msg.max_linear_velocity = 0.5
        msg.max_linear_acceleration = 1.0
        msg.max_linear_jerk = 6.0
        msg.max_angular_velocity = 2.0
        msg.max_angular_acceleration = 4.0
        msg.max_angular_jerk = 10.0
        msg.duration = 5.0  # 运动持续5秒
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info('已发送参数法圆形轨迹指令')
        self.print_message_summary(msg, "参数法")
        
        return msg

    def print_message_summary(self, msg, method_name):
        """打印消息摘要"""
        self.get_logger().info(f'=== {method_name} 消息摘要 ===')
        self.get_logger().info(f'坐标系: {msg.frame_id}')
        self.get_logger().info(f'时间模式: {msg.time_mode}, 时长: {msg.duration}秒')
        
        if msg.use_three_point_method:
            self.get_logger().info(f'中间点: ({msg.midpoint.position.x:.2f}, '
                                 f'{msg.midpoint.position.y:.2f}, '
                                 f'{msg.midpoint.position.z:.2f})')
            self.get_logger().info(f'终点: ({msg.endpoint.position.x:.2f}, '
                                 f'{msg.endpoint.position.y:.2f}, '
                                 f'{msg.endpoint.position.z:.2f})')
        else:
            self.get_logger().info(f'圆心: ({msg.center.x:.2f}, '
                                 f'{msg.center.y:.2f}, '
                                 f'{msg.center.z:.2f})')
            self.get_logger().info(f'旋转轴: ({msg.axis.x:.2f}, '
                                 f'{msg.axis.y:.2f}, '
                                 f'{msg.axis.z:.2f})')
            self.get_logger().info(f'旋转角度: {msg.rotate_angle:.2f} 弧度')
        
        self.get_logger().info('============================')

    def run_interactive(self):
        """交互式运行模式"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('CircleMessage 发布器 - 交互模式')
        self.get_logger().info('='*50)
        self.get_logger().info('1. 使用三点法发送')
        self.get_logger().info('2. 使用参数法发送')
        self.get_logger().info('3. 连续发送测试')
        self.get_logger().info('4. 退出')
        
        while rclpy.ok():
            try:
                choice = input('\n请选择操作 (1-4): ').strip()
                
                if choice == '1':
                    self.publish_three_point_method()
                elif choice == '2':
                    self.publish_parametric_method()
                elif choice == '3':
                    # 连续发送5次测试
                    count = int(input('输入发送次数: ') or '5')
                    interval = float(input('发送间隔(秒): ') or '1.0')
                    
                    for i in range(count):
                        self.get_logger().info(f'发送第 {i+1}/{count} 次')
                        self.publish_three_point_method()
                        rclpy.spin_once(self, timeout_sec=interval)
                        
                elif choice == '4':
                    self.get_logger().info('退出程序')
                    break
                else:
                    self.get_logger().warn('无效选择，请重新输入')
                    
            except KeyboardInterrupt:
                self.get_logger().info('用户中断')
                break
            except Exception as e:
                self.get_logger().error(f'发生错误: {e}')



def main(args=None):
    rclpy.init(args=args)
    """测试手臂笛卡尔位置控制（MoveC模式）"""
    
    print("\n" + "=" * 70)
    print(" " * 15 + "Arm MoveC Test")
    print("=" * 70 + "\n")
    
    # ========================================================================
    # 第一部分：初始化和连接
    # ========================================================================
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
    
    # 等待数据到达
    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")
    
    # 检查是否是双臂模式
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    if not is_dual_arm:
        print("⚠ 警告: 当前不是双臂模式，将只控制左臂\n")
    
    # ========================================================================
    # 第二部分：切换到OCS2状态
    # ========================================================================
    print("-" * 70)
    print("[5] 切换到OCS2状态")
    print("-" * 70)
    try:
        print("  → 切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")
        # 先切换到HOME状态
        print("  → 切换到HOME状态...")
        interface.send_fsm_command(1)  # 1 = HOME状态
        time.sleep(5.0)
        print("  ✓ 已切换到HOME状态")

        # 先切换到HOME状态
        print("  → 再切换到Hold状态...")
        interface.send_fsm_command(2)  # 2 = Hold状态
        time.sleep(0.1)
        print("  ✓ 已切换到Hold状态")

        # 再切换到OCS2状态
        print("  → 切换到OCS2状态...")
        interface.send_fsm_command(3)  # 3 = OCS2/MOVE状态
        time.sleep(0.1)  # 等待状态转换完成
        print("  ✓ 已切换到OCS2状态\n")
    except Exception as e:
        print(f"  ✗ 切换到OCS2状态失败: {e}\n")
        interface.disconnect()
        return 1
    # ========================================================================
    # 第三部分：准备轨迹数据
    # ========================================================================
    print("-" * 70)
    print("[6] 准备轨迹数据")
    print("-" * 70) 
    
    """使用三点法发送圆形轨迹"""
    try:
        publisher=CircleMessagePublisher()

        #方式1：直接发送一次（三点法）
        # publisher.publish_three_point_method()
        

        # 方式2: 直接发送一次（参数法）
        publisher.publish_parametric_method()

        # 方式3: 交互式发送（推荐）
        # publisher.run_interactive()
        time.sleep(6)
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        #清理资源
        if 'publisher' in locals():
            publisher.destroy_node()
        # rclpy.shutdown()
 
    
    # ========================================================================
    # 第四部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[7] 测试完成，断开连接")
    print("=" * 70)
     
    # 切换回HOLD状态
    print("  → 切换回HOLD状态...")
    try:
        interface.send_fsm_command(2)  # HOLD
        time.sleep(1.0)
    except Exception as e:
        print(f"  ⚠ 切换状态失败: {e}")
    
    # 断开连接
    interface.disconnect()
    print("  ✓ 已断开连接\n")
    
    print("=" * 70)
    print("测试完成！")
    print("=" * 70 + "\n")
    
    return 0



if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)