"""
测试 check_arrival 修复功能

验证以下修复是否生效：
1. 发送新目标后立即清除 latest_target_pose，避免误判为已到达
"""

import time
import sys

from geometry_msgs.msg import Pose
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def test_send_target_clear_behavior(interface, arm_handler, label):
    """测试发送目标后清除行为"""
    print(f"\n{'='*70}")
    print(f"测试 {label} - 发送目标后清除 latest_target_pose 的行为")
    print(f"{'='*70}")
    
    # 获取当前位置
    current_pose = arm_handler.get_pose()
    if current_pose is None:
        print(f"  ✗ 无法获取 {label} 当前位置（可能未订阅到数据）")
        return False
    
    print(f"  [当前位置] ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
    
    # 步骤1: 检查初始状态（可能已有目标）
    print(f"\n  [步骤1] 检查初始目标状态...")
    initial_target = arm_handler.get_target_pose()
    if initial_target is not None:
        print(f"    → 初始目标存在: ({initial_target.position.x:.4f}, {initial_target.position.y:.4f}, {initial_target.position.z:.4f})")
    else:
        print(f"    → 初始目标不存在（这是正常的）")
    
    # 步骤2: 获取 frame_id 并发送新目标
    print(f"\n  [步骤2] 获取 frame_id 并发送新目标...")
    frame_id = arm_handler.get_frame_id()
    if frame_id is None:
        print(f"    ⚠ frame_id 尚未设置（可能还未收到 pose 消息），等待中...")
        time.sleep(1.0)
        frame_id = arm_handler.get_frame_id()
        if frame_id is None:
            print(f"    ✗ 无法获取 frame_id，跳过测试")
            return False
    
    print(f"    → 使用 frame_id: {frame_id}")
    new_target = Pose()
    new_target.position.x = current_pose.position.x
    new_target.position.y = current_pose.position.y
    new_target.position.z = current_pose.position.z + 0.1  # z轴偏移10cm
    new_target.orientation = current_pose.orientation
    
    arm_handler.send_target_stamped(frame_id, new_target)
    print(f"    → 已发送新目标: ({new_target.position.x:.4f}, {new_target.position.y:.4f}, {new_target.position.z:.4f})")
    
    # 步骤3: 立即检查目标状态（应该被清除）
    print(f"\n  [步骤3] 立即检查目标状态（发送后立即检查）...")
    immediate_target = arm_handler.get_target_pose()
    
    if immediate_target is None:
        print(f"    ✓ 目标已被清除（latest_target_pose = None）")
        
        # 检查到达状态
        result = arm_handler.check_arrival()
        if result['status_message'] is None:
            print(f"    ✓ check_arrival 返回未到达（因为 target_pose 为 None）")
        else:
            print(f"    → check_arrival 结果: {result['status_message']}")
    else:
        print(f"    ✗ 目标未被清除！latest_target_pose 仍然存在")
        print(f"    → 这是错误的，应该被清除")
        return False
    
    # 步骤4: 等待一段时间，让新的 target topic 消息到达
    print(f"\n  [步骤4] 等待新的 target topic 消息到达（等待 2 秒）...")
    time.sleep(2.0)
    
    # 步骤5: 再次检查目标状态（应该已更新）
    print(f"\n  [步骤5] 再次检查目标状态（等待后）...")
    updated_target = arm_handler.get_target_pose()
    
    if updated_target is not None:
        print(f"    ✓ 目标已更新: ({updated_target.position.x:.4f}, {updated_target.position.y:.4f}, {updated_target.position.z:.4f})")
        
        # 检查到达状态
        result = arm_handler.check_arrival()
        print(f"    → check_arrival 结果: {result['status_message']}")
        print(f"    → 位置距离: {result['position_distance']:.4f} 米")
    else:
        print(f"    ⚠ 目标仍未更新（可能 target topic 未发布或延迟较大）")
        print(f"    → 这是正常的，如果系统没有发布 current_target topic")
    
    return True


def main():
    """主函数"""
    print("\n" + "="*70)
    print(" " * 15 + "check_arrival 修复功能验证测试")
    print("="*70 + "\n")
    
    # 创建配置
    print("[1] 创建配置...")
    config = ROS2RobotInterfaceConfig()
    print("    ✓ 配置创建完成\n")
    
    # 创建接口
    print("[2] 创建 ROS2RobotInterface 实例...")
    interface = ROS2RobotInterface(config)
    print("    ✓ 接口创建完成\n")
    
    # 连接
    print("[3] 连接到 ROS 2...")
    try:
        interface.connect()
        print("    ✓ 连接成功\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1
    
    # 等待数据到达
    print("[4] 等待数据到达（3秒）...")
    time.sleep(3.0)
    print("    ✓ 数据收集已开始\n")
    
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    
    # 测试左臂
    if interface.left_arm_handler:
        label = "LEFT_ARM" if is_dual_arm else "ARM"
        
        # 测试: 发送目标后清除行为
        test_result = test_send_target_clear_behavior(interface, interface.left_arm_handler, label)
        
        if test_result:
            print(f"\n{'='*70}")
            print(f"✓ {label} 测试通过！")
            print(f"{'='*70}\n")
        else:
            print(f"\n{'='*70}")
            print(f"✗ {label} 测试失败")
            print(f"{'='*70}\n")
    
    # 测试右臂（双臂模式）
    if is_dual_arm and interface.right_arm_handler:
        print("\n" + "="*70)
        print("测试右臂（双臂模式）")
        print("="*70)
        
        # 测试: 发送目标后清除行为
        test_result = test_send_target_clear_behavior(interface, interface.right_arm_handler, "RIGHT_ARM")
        
        if test_result:
            print(f"\n{'='*70}")
            print(f"✓ RIGHT_ARM 测试通过！")
            print(f"{'='*70}\n")
        else:
            print(f"\n{'='*70}")
            print(f"✗ RIGHT_ARM 测试失败")
            print(f"{'='*70}\n")
    
    # 断开连接
    print(f"\n{'='*70}")
    print("[5] 断开连接...")
    interface.disconnect()
    print("    ✓ 已断开连接\n")
    
    print("="*70)
    print("测试完成！")
    print("="*70 + "\n")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
