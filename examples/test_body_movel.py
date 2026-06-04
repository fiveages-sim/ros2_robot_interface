#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
import sys
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

def main(args=None):
    """主测试函数"""
    print("=" * 70)
    print("腰部电机movel测试")
    print("=" * 70)
    
    # 初始化ROS
    rclpy.init()
    
    # 创建节点（用于发布消息）
    node = rclpy.create_node('simple_body_movel_publisher')
    
    # 创建接口
    print("[1] 创建机器人接口...")
    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)

    try:
        interface.connect()
        print("✓ 接口连接成功")
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    
    time.sleep(2.0)

    # 切换到home位置再切换到hold
    print("[2] 切换到home位置再hold...")
    try:
        interface.send_fsm_command(2)  # Hold
        time.sleep(0.1)
        interface.send_fsm_command(1)  # HOME
        time.sleep(5.0)
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        print("✓ 已切换到hold状态")
        interface.send_fsm_command(3)  # move
        time.sleep(1.0)
        print("✓ 已切换到move状态")

    except Exception as e:
        print(f"✗ 状态切换失败: {e}")
        interface.disconnect()
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # 创建PoseStamped目标信息
    msg = Pose()
    
    # ✅ 确保使用float类型（显式转换）
    msg.position.x = float(0.3547)
    msg.position.y = float(0.0)
    msg.position.z = float(0.993962)
    
    # 设置姿态（四元数）- 也确保是float类型
    msg.orientation.w = float(0.897997)
    msg.orientation.x = float(-0.0588049)
    msg.orientation.y = float(0.126547)
    msg.orientation.z = float(-0.417287)

    # 创建发布者
    publisher = node.create_publisher(Pose, '/body_joint_controller/body_movel_target', 10)
    
    # 发布前检查所有值类型
    print("[3] 检查数据类型...")
    print(f"  position.x: {msg.position.x}, type: {type(msg.position.x)}")
    print(f"  position.y: {msg.position.y}, type: {type(msg.position.y)}")
    print(f"  position.z: {msg.position.z}, type: {type(msg.position.z)}")
    
    # 发布消息
    print("[4] 发布movel目标位置...")
    publisher.publish(msg)
    print(f"✓ 已发布目标位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
    
    # 等待运动完成
    time.sleep(5)

    # 切换回HOLD状态
    try:
        interface.send_fsm_command(2)
        time.sleep(1.0)
        print("✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"⚠ 状态切换失败: {e}")
    
    interface.disconnect()
    print("✓ 已断开连接")

    # 清理并退出
    node.destroy_node()
    rclpy.shutdown()
    print("✓ 测试完成，正常退出")
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