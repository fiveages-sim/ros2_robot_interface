"""
TF 变换查询和坐标转换测试脚本

测试 lookup_transform() 和 transform_pose() 接口：
1. 持续查询两个坐标系之间的变换关系
2. 测试将坐标从一个坐标系转换到另一个坐标系
"""

import time
import sys
from geometry_msgs.msg import Pose
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def format_transform(transform):
    """格式化 TransformStamped 输出（与 tf2_echo 格式一致）
    
    作用：将 TransformStamped 对象转换为易读的字符串格式
    - 提取平移信息（x, y, z）- 格式与 tf2_echo 一致
    - 提取旋转四元数（x, y, z, w）- 格式与 tf2_echo 一致
    - 计算并显示 RPY 角度（roll, pitch, yaw）- 格式与 tf2_echo 一致
    """
    if transform is None:
        return "  ✗ 变换查询失败"
    
    import math
    
    trans = transform.transform.translation
    rot = transform.transform.rotation
    
    # 从四元数计算 RPY 角度（与 tf2_echo 一致）
    # 四元数转欧拉角公式
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (rot.w * rot.x + rot.y * rot.z)
    cosr_cosp = 1.0 - 2.0 * (rot.x * rot.x + rot.y * rot.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (rot.w * rot.y - rot.z * rot.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用 90 度
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (rot.w * rot.z + rot.x * rot.y)
    cosy_cosp = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    
    result = f"  ✓ 变换查询成功:\n"
    result += f"    Translation: [{trans.x:6.3f}, {trans.y:6.3f}, {trans.z:6.3f}]\n"
    result += f"    Rotation (xyzw): [{rot.x:6.3f}, {rot.y:6.3f}, {rot.z:6.3f}, {rot.w:6.3f}]\n"
    result += f"    Rotation RPY (rad): [{roll:6.3f}, {pitch:6.3f}, {yaw:6.3f}]\n"
    result += f"    Rotation RPY (deg): [{roll_deg:7.3f}, {pitch_deg:7.3f}, {yaw_deg:7.3f}]"
    return result


def format_pose(pose, label="Pose"):
    """格式化 Pose 输出"""
    if pose is None:
        return f"  ✗ {label} 转换失败"
    
    result = f"  ✓ {label}:\n"
    result += f"    位置: ({pose.position.x:7.4f}, {pose.position.y:7.4f}, {pose.position.z:7.4f}) 米\n"
    result += f"    旋转: ({pose.orientation.x:6.4f}, {pose.orientation.y:6.4f}, "
    result += f"{pose.orientation.z:6.4f}, {pose.orientation.w:6.4f})"
    return result


def main():
    """测试 TF 变换查询接口"""
    
    print("\n" + "=" * 70)
    print(" " * 20 + "TF Transform Query Test")
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
    
    # 等待 TF 数据准备（给 TF 系统一些时间建立坐标系树）
    print("[4] 等待 TF 数据准备（3秒）...")
    time.sleep(3.0)
    print("    ✓ TF 系统已准备\n")
    
    # ========================================================================
    # 第二部分：定义要测试的坐标系对
    # ========================================================================
    print("-" * 70)
    print("[5] 定义要测试的坐标系对")
    print("-" * 70)
    
    # 测试 left_link1 和 left_link7 之间的变换
    # 注意：tf2_echo left_link1 left_link7 查询的是 left_link7 → left_link1 的变换
    # 所以要匹配 tf2_echo，应该使用 lookup_transform("left_link1", "left_link7")
    target_frame = "left_link1"  # 匹配 tf2_echo 的第一个参数（target）
    source_frame = "left_link7"  # 匹配 tf2_echo 的第二个参数（source）
    
    print(f"  测试坐标系对: {source_frame} → {target_frame}")
    print(f"  (查询 left_link7 在 left_link1 坐标系下的位姿)")
    print(f"  (对应命令: ros2 run tf2_ros tf2_echo {target_frame} {source_frame})\n")
    
    # ========================================================================
    # 第三部分：准备测试用的 Pose
    # ========================================================================
    print("-" * 70)
    print("[6] 准备测试用的 Pose")
    print("-" * 70)
    
    # 创建一个测试用的 Pose（在 source_frame 坐标系下）
    test_pose = Pose()
    test_pose.position.x = 0.1
    test_pose.position.y = 0.2
    test_pose.position.z = 0.3
    test_pose.orientation.x = 0.0
    test_pose.orientation.y = 0.0
    test_pose.orientation.z = 0.0
    test_pose.orientation.w = 1.0  # 单位四元数（无旋转）
    
    print(f"  测试 Pose（在 {source_frame} 坐标系下）:")
    print(f"    位置: ({test_pose.position.x:.3f}, {test_pose.position.y:.3f}, {test_pose.position.z:.3f}) 米")
    print(f"    旋转: ({test_pose.orientation.x:.4f}, {test_pose.orientation.y:.4f}, "
          f"{test_pose.orientation.z:.4f}, {test_pose.orientation.w:.4f})\n")
    
    # ========================================================================
    # 第四部分：持续查询变换和坐标转换
    # ========================================================================
    print("=" * 70)
    print("[7] 开始持续查询 TF 变换和坐标转换")
    print("=" * 70)
    print("  → 每 2 秒查询一次")
    print("  → 测试内容：")
    print(f"     1. 查询 {source_frame} → {target_frame} 的变换")
    print(f"     2. 将 Pose 从 {source_frame} 转换到 {target_frame}")
    print("  → 按 Ctrl+C 停止测试")
    print("-" * 70 + "\n")
    
    query_count = 0
    transform_success_count = 0
    transform_fail_count = 0
    pose_success_count = 0
    pose_fail_count = 0
    
    try:
        while True:
            query_count += 1
            print(f"[查询 #{query_count}] " + "=" * 60)
            print(f"时间: {time.strftime('%H:%M:%S')}\n")
            
            # 测试 1：查询变换关系
            print(f"  [测试 1] 查询变换: {source_frame} → {target_frame}")
            try:
                transform = interface.lookup_transform(target_frame, source_frame)
                
                if transform:
                    print(format_transform(transform))
                    transform_success_count += 1
                else:
                    print("  ✗ 变换查询失败（返回 None）")
                    transform_fail_count += 1
            except Exception as e:
                print(f"  ✗ 查询异常: {e}")
                transform_fail_count += 1
            
            print()
            
            # 测试 2：坐标转换
            print(f"  [测试 2] 坐标转换: {source_frame} → {target_frame}")
            try:
                # 将 test_pose 从 source_frame 转换到 target_frame
                transformed_pose = interface.transform_pose(
                    test_pose, source_frame, target_frame
                )
                
                if transformed_pose:
                    print(format_pose(test_pose, f"原始 Pose (在 {source_frame} 坐标系下)"))
                    print()
                    print(format_pose(transformed_pose, f"转换后的 Pose (在 {target_frame} 坐标系下)"))
                    pose_success_count += 1
                else:
                    print("  ✗ 坐标转换失败（返回 None）")
                    pose_fail_count += 1
            except Exception as e:
                print(f"  ✗ 转换异常: {e}")
                pose_fail_count += 1
            
            print()
            print(f"统计:")
            print(f"  变换查询: 成功 {transform_success_count}/{query_count}, 失败 {transform_fail_count}/{query_count}")
            print(f"  坐标转换: 成功 {pose_success_count}/{query_count}, 失败 {pose_fail_count}/{query_count}")
            print("-" * 70)
            print()
            
            # 等待 2 秒后继续下一次查询
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        # 用户中断（Ctrl+C）
        print("\n" + "-" * 70)
        print("  用户中断测试")
        print("-" * 70)
    
    # ========================================================================
    # 第五部分：断开连接和清理
    # ========================================================================
    print("\n" + "=" * 70)
    print("[8] 断开连接...")
    print("=" * 70)
    interface.disconnect()
    print("  ✓ 接口断开成功!")
    
    # 最终统计
    print(f"\n最终统计:")
    print(f"  总查询次数: {query_count}")
    print(f"  变换查询: 成功 {transform_success_count}/{query_count}, 失败 {transform_fail_count}/{query_count}")
    print(f"  坐标转换: 成功 {pose_success_count}/{query_count}, 失败 {pose_fail_count}/{query_count}")
    print("=" * 70)
    print("\n  测试完成!\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n测试被用户中断 - 正在清理...")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

