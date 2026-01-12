"""
check_arrival 功能测试脚本

专门用于测试手臂的到达判断功能。
测试包括：
- 手臂到达判断（位置和姿态）
- 默认阈值测试
- 自定义阈值测试
- 严格阈值测试
"""

import time
import sys

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def test_arm_arrival(interface, arm_handler, label):
    """测试手臂到达判断"""
    print(f"\n{'='*70}")
    print(f"测试 {label} 到达判断")
    print(f"{'='*70}")
    
    # 获取当前位置
    current_pose = arm_handler.get_pose()
    if current_pose is None:
        print(f"  ✗ 无法获取 {label} 当前位置（可能未订阅到数据）")
        return False
    
    print(f"  [当前位置] ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
    
    # 获取目标位置
    target_pose = arm_handler.get_target_pose()
    if target_pose is None:
        print(f"  ✗ 无法获取 {label} 目标位置（可能未设置或未订阅到数据）")
        print(f"  → 提示：请先发送目标位置（send_target() 或 send_target_stamped()）")
        print(f"  → 或者确保 /left_current_target 或 /right_current_target 话题正在发布")
        return False
    
    print(f"  [目标位置] ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
    
    # 测试默认阈值
    print(f"\n  [测试1] 使用默认阈值检查到达状态...")
    result = arm_handler.check_arrival()
    print(f"  → 到达状态: {'✓ 已到达' if result['arrived'] else '✗ 未到达'}")
    print(f"  → 位置距离: {result['position_distance']:.4f} 米")
    print(f"  → 姿态距离: {result['orientation_distance']:.4f}")
    print(f"  → 总距离: {result['distance']:.4f}")
    
    # 测试自定义阈值
    print(f"\n  [测试2] 使用自定义阈值检查到达状态（位置阈值=0.1米，姿态阈值=0.2）...")
    result = arm_handler.check_arrival(pose_threshold=0.1, orient_threshold=0.2)
    print(f"  → 到达状态: {'✓ 已到达' if result['arrived'] else '✗ 未到达'}")
    print(f"  → 位置距离: {result['position_distance']:.4f} 米")
    print(f"  → 姿态距离: {result['orientation_distance']:.4f}")
    
    # 测试严格阈值
    print(f"\n  [测试3] 使用严格阈值检查到达状态（位置阈值=0.01米，姿态阈值=0.01）...")
    result = arm_handler.check_arrival(pose_threshold=0.01, orient_threshold=0.01)
    print(f"  → 到达状态: {'✓ 已到达' if result['arrived'] else '✗ 未到达'}")
    print(f"  → 位置距离: {result['position_distance']:.4f} 米")
    print(f"  → 姿态距离: {result['orientation_distance']:.4f}")
    
    return True


def main():
    """主函数"""
    print("\n" + "="*70)
    print(" " * 20 + "手臂到达判断功能测试")
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
    
    # 循环判断到达状态
    print(f"{'='*70}")
    print("开始循环检查到达状态（按 Ctrl+C 停止）")
    print(f"{'='*70}\n")
    
    check_interval = 1.0  # 检查间隔（秒）
    check_count = 0
    
    try:
        while True:
            check_count += 1
            print(f"\n[检查 #{check_count}] {time.strftime('%H:%M:%S')}")
            print("-" * 70)
            
            # 测试左臂到达判断
            if interface.left_arm_handler:
                label = "LEFT_ARM" if is_dual_arm else "ARM"
                current_pose = interface.left_arm_handler.get_pose()
                target_pose = interface.left_arm_handler.get_target_pose()
                
                if current_pose is not None and target_pose is not None:
                    result = interface.left_arm_handler.check_arrival()
                    status = "✓ 已到达" if result['arrived'] else "✗ 未到达"
                    print(f"  [{label}] {status} | 位置距离: {result['position_distance']:.4f}m | 姿态距离: {result['orientation_distance']:.4f}")
                else:
                    print(f"  [{label}] ⚠ 数据不可用（当前位置或目标位置缺失）")
            
            # 测试右臂到达判断（双臂模式）
            if is_dual_arm and interface.right_arm_handler:
                current_pose = interface.right_arm_handler.get_pose()
                target_pose = interface.right_arm_handler.get_target_pose()
                
                if current_pose is not None and target_pose is not None:
                    result = interface.right_arm_handler.check_arrival()
                    status = "✓ 已到达" if result['arrived'] else "✗ 未到达"
                    print(f"  [RIGHT_ARM] {status} | 位置距离: {result['position_distance']:.4f}m | 姿态距离: {result['orientation_distance']:.4f}")
                else:
                    print(f"  [RIGHT_ARM] ⚠ 数据不可用（当前位置或目标位置缺失）")
            
            time.sleep(check_interval)
            
    except KeyboardInterrupt:
        print(f"\n\n{'='*70}")
        print("收到中断信号，停止检查")
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

