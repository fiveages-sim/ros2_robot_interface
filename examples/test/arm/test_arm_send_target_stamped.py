import time
import sys
from geometry_msgs.msg import Pose, Point, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# ============================================================================
# 测试参数（按需修改）
# ============================================================================
MAX_WAIT_TIME = 10.0    # 每步最大等待时间（秒）
CHECK_INTERVAL = 0.5    # 到位检查间隔（秒）
POSE_THRESHOLD = 0.05  # 位置距离阈值（米）
STEP_SIZE = 0.1       # 每次移动距离（米）


def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def print_pose(pose: Pose, label: str = "Pose") -> None:
    print(f"  {label}:")
    print(f"    位置: ({pose.position.x:7.3f}, {pose.position.y:7.3f}, {pose.position.z:7.3f})")
    print(
        f"    方向: ({pose.orientation.x:6.3f}, {pose.orientation.y:6.3f}, "
        f"{pose.orientation.z:6.3f}, {pose.orientation.w:6.3f})"
    )


def wait_for_arrival(handler, max_wait: float, interval: float, threshold: float) -> bool:
    """等待单臂到达目标位置，返回是否到达。"""
    start = time.time()
    while time.time() - start < max_wait:
        result = handler.check_arrival(pose_threshold=threshold)
        if result["arrived"]:
            print(f"  ✓ 已到达目标（耗时 {time.time() - start:.1f}s）")
            return True
        time.sleep(interval)

    result = handler.check_arrival(pose_threshold=threshold)
    print(f"  ⚠ 超时（{max_wait:.0f}s），当前距目标 {result.get('position_distance', float('inf')):.4f}m")
    return False


def main() -> int:
    print("\n" + "=" * 70)
    print(" " * 14 + "arm_handler.send_target_stamped() 功能测试")
    print("=" * 70 + "\n")

    # ========================================================================
    # 第一部分：初始化和连接
    # ========================================================================
    print("[1] 创建配置...")
    config = ROS2RobotInterfaceConfig()

    print("[2] 创建 ROS2RobotInterface 实例...")
    interface = ROS2RobotInterface(config)

    print("[3] 连接到 ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功!\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1

    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")

    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    print(f"[5] 检测到模式: {'双臂' if is_dual_arm else '单臂'}\n")

    # ========================================================================
    # 第二部分：切换到 OCS2 状态
    # ========================================================================
    print("-" * 70)
    print("[6] 切换状态：HOLD → HOME → OCS2")
    print("-" * 70)
    print("  → 切换到 HOLD 状态...")
    interface.send_fsm_command(2)   # HOLD
    print("  → 切换到 HOME 状态...")
    interface.send_fsm_command(1)   # HOME
    time.sleep(6.0)
    print("  → 切换到 OCS2 状态...")
    interface.send_fsm_command(2)   # HOLD
    interface.send_fsm_command(3)   # OCS2
    print("  ✓ 已切换到 OCS2 状态\n")

    # ========================================================================
    # 第三部分：获取初始位姿（HOME 完成后）
    # ========================================================================
    print("-" * 70)
    print("[7] 获取 HOME 后的当前位姿")
    print("-" * 70)

    left_pose = interface.left_arm_handler.get_pose()
    if left_pose is None:
        print("⚠ 无法获取左臂当前位姿，请检查 pose 话题是否正常发布")
        interface.disconnect()
        return 1
    print_pose(left_pose, "左臂当前位姿")

    right_pose = None
    if is_dual_arm:
        right_pose = interface.right_arm_handler.get_pose()
        if right_pose is None:
            print("⚠ 无法获取右臂当前位姿，请检查 pose 话题是否正常发布")
            interface.disconnect()
            return 1
        print()
        print_pose(right_pose, "右臂当前位姿")
    print()

    # ========================================================================
    # 第四部分：send_target_stamped() 测试
    # ========================================================================
    print("=" * 70)
    print(f"[8] send_target_stamped() 测试（步长 {STEP_SIZE}m）")
    print("=" * 70)

    # 每步的偏移量（dx, dy, dz）以及说明
    steps = [
        (0.0,  0.0,  STEP_SIZE, "向上"),
        (0.0,  0.0, -STEP_SIZE, "向下（返回）"),
        (0.0,  STEP_SIZE, 0.0, "向侧方"),
        (0.0, -STEP_SIZE, 0.0, "返回侧方原位"),
    ]

    # 维护左臂当前目标位置（以初始位姿为起点）
    lx, ly, lz = left_pose.position.x, left_pose.position.y, left_pose.position.z
    lqx = left_pose.orientation.x
    lqy = left_pose.orientation.y
    lqz = left_pose.orientation.z
    lqw = left_pose.orientation.w

    if is_dual_arm:
        rx, ry, rz = right_pose.position.x, right_pose.position.y, right_pose.position.z
        rqx = right_pose.orientation.x
        rqy = right_pose.orientation.y
        rqz = right_pose.orientation.z
        rqw = right_pose.orientation.w

    for step_idx, (dx, dy, dz, desc) in enumerate(steps, start=1):
        lx += dx
        ly += dy
        lz += dz
        left_target = create_pose(lx, ly, lz, lqx, lqy, lqz, lqw)

        print(f"\n[8.{step_idx}] 左臂 {desc}")
        print("-" * 70)
        print_pose(left_target, "目标位姿")

        try:
            interface.left_arm_handler.send_target_stamped("world", left_target)
            print("  ✓ send_target_stamped(world) 已发布")
        except Exception as e:
            print(f"  ✗ 发布失败: {e}")
            break

        arrived = wait_for_arrival(
            interface.left_arm_handler, MAX_WAIT_TIME, CHECK_INTERVAL, POSE_THRESHOLD
        )
        if not arrived:
            print("  ⚠ 未到达，继续下一步...")

        # 左臂到位后再发右臂目标（串行）
        if is_dual_arm:
            rx += dx
            ry -= dy  # 右臂关于中线镜像
            rz += dz
            right_target = create_pose(rx, ry, rz, rqx, rqy, rqz, rqw)

            print(f"\n  发送右臂目标（左臂已完成后）")
            print_pose(right_target, "右臂目标位姿")
            try:
                # 不传 frame_id，回退到 self.frame_id（最近订阅到的坐标系）
                interface.right_arm_handler.send_target_stamped(right_target)
                print("  ✓ 右臂 send_target_stamped(默认 frame_id) 已发布")
            except Exception as e:
                print(f"  ✗ 右臂发布失败: {e}")

            wait_for_arrival(
                interface.right_arm_handler, MAX_WAIT_TIME, CHECK_INTERVAL, POSE_THRESHOLD
            )

    # ========================================================================
    # 第五部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[9] 测试完成，断开连接")
    print("=" * 70)

    interface.send_fsm_command(2)   # 切回 HOLD
    time.sleep(0.5)
    interface.disconnect()
    print("  ✓ 已断开连接\n")

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
