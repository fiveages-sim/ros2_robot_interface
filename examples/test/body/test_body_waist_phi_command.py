"""
测试 body_joint3 phi 速度控制接口。

通过 ROS2RobotInterface.send_waist_phi_velocity_scale() 发送：
1. 0.5 持续 5 秒
2. 0.0 停止
3. -0.5 持续 5 秒
4. 0.0 停止
"""

import argparse
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test waist phi velocity command interface.")
    parser.add_argument(
        "--topic",
        default=None,
        help=(
            "Override waist phi command topic. If omitted, ROS2RobotInterface "
            "auto-detects the active controller topic."
        ),
    )
    parser.add_argument("--duration", type=float, default=5.0, help="Duration for each non-zero command.")
    parser.add_argument("--speed", type=float, default=0.5, help="Positive velocity scale.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration < 0.0:
        raise ValueError("--duration must be non-negative")

    print("\n" + "=" * 70)
    print(" " * 16 + "Body Waist Phi Interface Test")
    print("=" * 70 + "\n")

    print("[1] 创建配置并初始化接口...")
    config = ROS2RobotInterfaceConfig()
    if args.topic is not None:
        config.waist_phi_command_topic = args.topic
    interface = ROS2RobotInterface(config)

    print("[2] 连接到 ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功\n")
    except Exception as exc:
        print(f"    ✗ 连接失败: {exc}\n")
        return 1

    try:
        print("[3] 等待初始数据（2秒）...")
        time.sleep(2.0)
        print("    ✓ 数据就绪\n")

        print("[4] 检查 waist phi 控制 topic 是否可用...")
        if interface.config.waist_phi_command_topic is None:
            print("    ✗ 未启用 waist_phi_command_topic")
            return 1
        print(f"    ✓ waist phi 控制 topic: {interface.config.waist_phi_command_topic}\n")

        print("-" * 70)
        print("[5] 正向 phi 速度控制")
        print("-" * 70)
        print(f"  发送 +{args.speed:.3f}，持续 {args.duration:.1f}s")
        interface.send_waist_phi_velocity_scale(args.speed)
        time.sleep(args.duration)

        print("  发送 0.0 停止")
        interface.send_waist_phi_velocity_scale(0.0)
        time.sleep(1.0)

        print("-" * 70)
        print("[6] 反向 phi 速度控制")
        print("-" * 70)
        print(f"  发送 -{args.speed:.3f}，持续 {args.duration:.1f}s")
        interface.send_waist_phi_velocity_scale(-args.speed)
        time.sleep(args.duration)

        print("  发送 0.0 停止")
        interface.send_waist_phi_velocity_scale(0.0)
        time.sleep(1.0)

        print("\n测试完成")
        print("=" * 70 + "\n")
        return 0
    finally:
        try:
            if interface.is_connected:
                interface.send_waist_phi_velocity_scale(0.0)
                interface.disconnect()
        except Exception as exc:
            print(f"    ⚠ 停止或断开连接失败: {exc}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⚠ 用户中断测试")
        sys.exit(1)
    except Exception as exc:
        print(f"\n✗ 测试失败: {exc}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
