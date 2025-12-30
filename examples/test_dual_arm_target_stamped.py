"""
测试 send_dual_arm_target_stamped() 功能
测试发送双臂目标位姿到 /dual_target/stamped 话题
"""

import time
import sys
from geometry_msgs.msg import Pose, Point, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """创建位姿消息的辅助函数"""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def print_pose(pose, label="Pose"):
    """打印位姿信息的辅助函数"""
    print(f"  {label}:")
    print(f"    位置: ({pose.position.x:7.3f}, {pose.position.y:7.3f}, {pose.position.z:7.3f})")
    print(f"    方向: ({pose.orientation.x:6.3f}, {pose.orientation.y:6.3f}, "
          f"{pose.orientation.z:6.3f}, {pose.orientation.w:6.3f})")


def main():
    """测试 send_dual_arm_target_stamped() 功能"""
    
    print("\n" + "=" * 70)
    print(" " * 15 + "Dual Arm Target Stamped Test")
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
        print("⚠ 错误: 当前不是双臂模式！")
        print("   此测试需要双臂机器人配置。")
        print("   请确保机器人配置为 dual_arm_mode = true\n")
        interface.disconnect()
        return 1
    
    print("✓ 检测到双臂模式\n")
    
    # ========================================================================
    # 第二部分：获取当前位姿
    # ========================================================================
    print("-" * 70)
    print("[5] 获取当前双臂位姿")
    print("-" * 70)
    
    left_current_pose = interface.left_arm_handler.get_pose()
    right_current_pose = interface.right_arm_handler.get_pose()
    
    if not left_current_pose or not right_current_pose:
        print("⚠ 错误: 无法获取当前位姿！")
        print("   请确保机器人正在运行并发布位姿数据。\n")
        interface.disconnect()
        return 1
    
    print_pose(left_current_pose, "左臂当前位姿")
    print()
    print_pose(right_current_pose, "右臂当前位姿")
    print()
    
    # ========================================================================
    # 第三部分：切换到OCS2状态
    # ========================================================================
    print("-" * 70)
    print("[6] 切换到OCS2状态（用于测试运动控制）")
    print("-" * 70)
    
    # 先切换到HOLD状态
    print("  → 切换到HOLD状态...")
    interface.send_fsm_command(2)  # HOLD
    time.sleep(1.0)
    
    # 再切换到OCS2状态
    print("  → 切换到OCS2状态...")
    interface.send_fsm_command(3)  # OCS2
    time.sleep(1.0)
    print("  ✓ 已切换到OCS2状态\n")
    
    # ========================================================================
    # 第四部分：每隔2秒下降0.05m测试
    # ========================================================================
    print("=" * 70)
    print("[7] 每隔2秒双臂末端下降0.05m测试")
    print("=" * 70)
    
    # 记录初始Z位置
    initial_left_z = left_current_pose.position.z
    initial_right_z = right_current_pose.position.z
    
    print(f"\n  初始位置:")
    print(f"    左臂Z: {initial_left_z:.3f}m")
    print(f"    右臂Z: {initial_right_z:.3f}m")
    print(f"  每次下降: 0.05m")
    print(f"  发送间隔: 2秒")
    print(f"  按 Ctrl+C 停止测试\n")
    
    # 下降次数计数器
    step_count = 0
    interval = 2.0  # 2秒间隔
    
    try:
        while True:
            step_count += 1
            current_left_z = initial_left_z - (step_count * 0.05)
            current_right_z = initial_right_z - (step_count * 0.05)
            
            print(f"[7.{step_count}] 下降 {step_count * 0.05:.2f}m")
            print("-" * 70)
            print(f"  目标Z位置: 左臂={current_left_z:.3f}m, 右臂={current_right_z:.3f}m")
            
            # 创建目标位姿（只改变Z坐标，其他保持不变）
            left_target_pose = create_pose(
                x=left_current_pose.position.x,
                y=left_current_pose.position.y,
                z=current_left_z,
                qx=left_current_pose.orientation.x,
                qy=left_current_pose.orientation.y,
                qz=left_current_pose.orientation.z,
                qw=left_current_pose.orientation.w
            )
            
            right_target_pose = create_pose(
                x=right_current_pose.position.x,
                y=right_current_pose.position.y,
                z=current_right_z,
                qx=right_current_pose.orientation.x,
                qy=right_current_pose.orientation.y,
                qz=right_current_pose.orientation.z,
                qw=right_current_pose.orientation.w
            )
            
            # 发送目标位姿
            try:
                interface.send_dual_arm_target_stamped(
                    left_target_pose,
                    right_target_pose,
                    frame_id="arm_base"
                )
                print(f"  ✓ 目标位姿已发送")
            except Exception as e:
                print(f"  ✗ 发送失败: {e}")
                break
            
            # 等待指定间隔时间后发送下一个目标
            print(f"  → 等待 {interval} 秒后发送下一个目标...")
            time.sleep(interval)
            
    except KeyboardInterrupt:
        print(f"\n\n  ⚠ 用户中断测试（已执行 {step_count} 次下降）")
    
    # ========================================================================
    # 第五部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[8] 测试完成，断开连接")
    print("=" * 70)
    
    # 切换回HOLD状态
    print("  → 切换回HOLD状态...")
    interface.send_fsm_command(2)  # HOLD
    time.sleep(1.0)
    
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

