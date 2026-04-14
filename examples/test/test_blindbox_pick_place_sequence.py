"""
盲盒点位抓放序列测试脚本。

运动顺序：
home -> 抓取up -> 抓取 -> 抓取up -> home -> 放置up -> 放置 -> 放置up -> home
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


MAX_WAIT_TIME = 5.0
CHECK_INTERVAL = 0.2
POSE_THRESHOLD = 0.01
BODY_THRESHOLD = 0.03
DEFAULT_FRAME_ID = "arm_base"


@dataclass
class Waypoint:
    name: str
    body_joints_rad: list[float] | None = None
    left_xyz: list[float] | None = None
    left_rpy: list[float] | None = None
    rpy_unit: str | None = None  # "deg" or "rad"
    right_xyz: list[float] | None = None
    right_rpy: list[float] | None = None


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """将欧拉角（弧度）转换为四元数 (x, y, z, w)。"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def create_pose_from_xyzrpy(xyz: list[float], rpy: list[float], unit: str) -> Pose:
    roll, pitch, yaw = rpy
    if unit == "deg":
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

    pose = Pose()
    pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def wait_left_arm_arrival(interface: ROS2RobotInterface, timeout: float) -> bool:
    start_time = time.time()
    while time.time() - start_time < timeout:
        result = interface.left_arm_handler.check_arrival(pose_threshold=POSE_THRESHOLD)
        if result["arrived"]:
            elapsed = time.time() - start_time
            print(f"    ✓ 左臂到位（{elapsed:.1f}s）")
            return True
        time.sleep(CHECK_INTERVAL)

    result = interface.left_arm_handler.check_arrival(pose_threshold=POSE_THRESHOLD)
    print(f"    ⚠ 左臂到位超时（{timeout:.1f}s），剩余误差 {result.get('position_distance', float('inf')):.4f}m")
    return False


def wait_right_arm_arrival(interface: ROS2RobotInterface, timeout: float) -> bool:
    start_time = time.time()
    while time.time() - start_time < timeout:
        result = interface.right_arm_handler.check_arrival(pose_threshold=POSE_THRESHOLD)
        if result["arrived"]:
            elapsed = time.time() - start_time
            print(f"    ✓ 右臂到位（{elapsed:.1f}s）")
            return True
        time.sleep(CHECK_INTERVAL)

    result = interface.right_arm_handler.check_arrival(pose_threshold=POSE_THRESHOLD)
    print(f"    ⚠ 右臂到位超时（{timeout:.1f}s），剩余误差 {result.get('position_distance', float('inf')):.4f}m")
    return False


def wait_body_arrival(interface: ROS2RobotInterface, timeout: float) -> bool:
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            result = interface.check_arrive("body", position_threshold=BODY_THRESHOLD)
            if result is not None and result.get("arrived", False):
                elapsed = time.time() - start_time
                print(f"    ✓ 腰部关节到位（{elapsed:.1f}s）")
                return True
        except Exception as exc:
            print(f"    ⚠ 腰部关节到位检查异常: {exc}，继续等待")
        time.sleep(CHECK_INTERVAL)

    print(f"    ⚠ 腰部关节到位超时（{timeout:.1f}s），继续执行")
    return False


def build_sequence(x: float, y: float, x1: float, y1: float) -> list[Waypoint]:
    home_0 = Waypoint(
        name="home0",
        body_joints_rad=[0.0, 0.0, 0.0, 0.0],
    )
    
    home = Waypoint(
        name="home",
        left_xyz=[0.0, 0.4, -0.15],
        left_rpy=[-175.2605, -84.5315, -6.0176],
        right_xyz=[0.0, -0.4, -0.15],
        right_rpy=[178.1795, -84.0944, 3.3536],
        rpy_unit="deg",
    )
    pick_up_0 = Waypoint(
        name="抓取up0",
        body_joints_rad=[0.0, 0.0, 0.0, np.pi],
    )
    pick_up_1 = Waypoint(
        name="抓取up1",
        body_joints_rad=[-0.8, -1.0, -0.2, np.pi],
    )
    pick_up = Waypoint(
        name="抓取up",
        left_xyz=[x-0.1, y+0.1, -0.2],
        left_rpy=[-2.7583, -1.5087, -0.3],
        rpy_unit="rad",
    )
    pick = Waypoint(
        name="抓取",
        left_xyz=[x, y, -0.4],
        left_rpy=[-2.7583, -1.5087, -0.3],
        rpy_unit="rad",
    )
    place_up = Waypoint(
        name="放置up",
        body_joints_rad=[0.0, 0.0, 0.0, 0.0],
        left_xyz=[x1, y1, -0.05],
        left_rpy=[-175.2605, -84.5315, -6.0176],
        rpy_unit="deg",
    )
    place = Waypoint(
        name="放置",
        body_joints_rad=[0.0, 0.0, 0.0, 0.0],
        left_xyz=[x1, y1, -0.15],
        left_rpy=[-175.2605, -84.5315, -6.0176],
        rpy_unit="deg",
    )

    return [home_0, home, pick_up_0, pick_up_1, pick_up, pick, pick_up, 
    home, pick_up_0, home_0, place_up, place, place_up, home, home_0]


def execute_waypoint(interface: ROS2RobotInterface, waypoint: Waypoint, frame_id: str) -> None:
    can_send_body = (
        waypoint.body_joints_rad is not None
        and
        interface.config.body_joint_controller_topic is not None
        and getattr(interface, "body_joint_controller_pub", None) is not None
    )
    can_send_left_arm = (
        waypoint.left_xyz is not None
        and waypoint.left_rpy is not None
        and waypoint.rpy_unit is not None
        and
        interface.left_arm_handler is not None
        and getattr(interface.left_arm_handler, "target_stamped_pub", None) is not None
    )
    can_send_right_arm = (
        waypoint.right_xyz is not None
        and waypoint.right_rpy is not None
        and waypoint.rpy_unit is not None
        and interface.config.right_end_effector_pose_topic is not None
        and interface.right_arm_handler is not None
        and getattr(interface.right_arm_handler, "target_stamped_pub", None) is not None
    )

    print(f"  → 点位: {waypoint.name}")
    if waypoint.body_joints_rad is not None:
        print(f"    腰部关节(rad): {waypoint.body_joints_rad}")
    else:
        print("    腰部关节: 未定义")
    if waypoint.left_xyz is not None and waypoint.left_rpy is not None and waypoint.rpy_unit is not None:
        print(f"    左臂xyz(m): {waypoint.left_xyz}")
        print(f"    左臂rpy({waypoint.rpy_unit}): {waypoint.left_rpy}")
    else:
        print("    左臂: 未定义")
    if can_send_right_arm:
        print(f"    右臂xyz(m): {waypoint.right_xyz}")
        print(f"    右臂rpy({waypoint.rpy_unit}): {waypoint.right_rpy}")
    else:
        print("    右臂: 未配置或不可用，跳过")

    if can_send_body:
        # 腰部关节运动：切换到 MOVEJ 后发送，等待真正到位再继续
        print("  → FSM: HOLD → MOVEJ（腰部关节）")
        interface.send_fsm_command(2)  # HOLD（内置 settle 等待）
        interface.send_fsm_command(4)  # MOVEJ（内置 settle 等待）
        interface.send_body_joint_positions(waypoint.body_joints_rad)
        wait_body_arrival(interface, MAX_WAIT_TIME)  # 确保到位后再切换 OCS2
    else:
        print("  ⚠ 腰部控制未配置或不可用，跳过")

    if can_send_left_arm:
        left_pose = create_pose_from_xyzrpy(waypoint.left_xyz, waypoint.left_rpy, waypoint.rpy_unit)
        # 左臂笛卡尔运动：切换到 OCS2 后发送，等待到位再继续
        print("  → FSM: HOLD → OCS2（手臂）")
        interface.send_fsm_command(2)  # HOLD（内置 settle 等待）
        interface.send_fsm_command(3)  # OCS2（内置 settle 等待）
        interface.left_arm_handler.send_target_stamped(frame_id, left_pose)
        wait_left_arm_arrival(interface, MAX_WAIT_TIME)
    else:
        print("  ⚠ 左臂控制未配置或不可用，跳过")

    # 右臂笛卡尔运动（仍在 OCS2 下）：等待到位再继续
    if can_send_right_arm:
        right_pose = create_pose_from_xyzrpy(waypoint.right_xyz, waypoint.right_rpy, waypoint.rpy_unit)
        interface.right_arm_handler.send_target_stamped(frame_id, right_pose)
        wait_right_arm_arrival(interface, MAX_WAIT_TIME)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="盲盒点位抓放测试程序（左臂+腰部）")
    parser.add_argument("--x", type=float, default=0.3, help="抓取点 x（米）")
    parser.add_argument("--y", type=float, default=0.3, help="抓取点 y（米）")
    parser.add_argument("--x1", type=float, default=0.5, help="放置点 x1（米）")
    parser.add_argument("--y1", type=float, default=0.2, help="放置点 y1（米）")
    parser.add_argument("--rounds", type=int, default=3, help="执行轮数，默认3")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rounds <= 0:
        print("✗ 参数错误：--rounds 必须大于 0")
        return 1

    print("\n" + "=" * 70)
    print(" " * 16 + "Blindbox Pick Place Sequence Test")
    print("=" * 70)
    print(f"参数: x={args.x}, y={args.y}, x1={args.x1}, y1={args.y1}, rounds={args.rounds}\n")

    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)

    print("[1] 连接 ROS 2...")
    try:
        interface.connect()
    except Exception as exc:
        print(f"✗ 连接失败: {exc}")
        return 1
    print("✓ 接口连接成功")
    time.sleep(2.0)

    print("[2] 初始化 FSM：切换到 HOLD")
    interface.send_fsm_command(2)  # HOLD
    time.sleep(1.0)

    frame_id = interface.left_arm_handler.get_frame_id()
    if frame_id is None:
        frame_id = DEFAULT_FRAME_ID
        print(f"[3] frame_id 未就绪，使用默认值: {frame_id}")
    else:
        print(f"[3] 使用 frame_id: {frame_id}")

    sequence = build_sequence(args.x, args.y, args.x1, args.y1)
    print("[4] 开始执行点位序列")
    print("    home -> 抓取up -> 抓取 -> 抓取up -> home -> 放置up -> 放置 -> 放置up -> home\n")

    try:
        for round_idx in range(1, args.rounds + 1):
            print("-" * 70)
            print(f"[轮次 {round_idx}/{args.rounds}]")
            print("-" * 70)
            for step_idx, waypoint in enumerate(sequence, start=1):
                print(f"[{round_idx}.{step_idx}/{len(sequence)}]")
                execute_waypoint(interface, waypoint, frame_id)
                print()
    except Exception as exc:
        print(f"✗ 执行过程中发生错误: {exc}")
        return 1
    finally:
        print("\n[5] 收尾：切回 HOLD 并断开连接")
        try:
            interface.send_fsm_command(2)  # HOLD
            time.sleep(1.0)
        except Exception as exc:
            print(f"⚠ 切回 HOLD 失败: {exc}")
        interface.disconnect()
        print("✓ 已断开连接")

    print("\n" + "=" * 70)
    print("测试完成")
    print("=" * 70 + "\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⚠ 用户中断测试")
        sys.exit(1)
    except Exception as exc:
        print(f"\n✗ 运行失败: {exc}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
