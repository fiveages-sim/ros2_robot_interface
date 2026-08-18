"""
验证 ROS2RobotInterface.send_body_relative() 的身体一次笛卡尔相对位移下发。

send_body_relative() 行为:
    - 向 /body_target/relative 发布 TwistStamped（米 / 弧度 RPY）。
    - connect() 时自动检测 topic：/body_target/relative。
    - 自动切 FSM 到 OCS2，再切 BODY_TRACKING；发布前清空 body_current_target_pose。

本脚本流程:
    1. connect()，检查 body_target_relative_pub。
    2. 调用 send_body_relative(+DX)（默认 frame_id，内部 base_frame）。
    3. 调用 send_body_relative(-DX, frame_id="right_tcp")。
    4. finally 中切 HOLD 并 disconnect()。

成功判据:
    - 两次 send_body_relative() 均无异常。
    - 未检测到 body relative topic 时打印 skip 并正常退出。

前置条件:
    ROS 2 已 source；机器人/仿真在运行；WBC 且 body tracking 可用。

运行:
    .venv/bin/python examples/test/10_body_waist/check_send_body_relative.py

安全说明:
    会发送身体约 3 cm 平移；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

DX = 0.03  # 米，沿内部 base_frame 的 X
WAIT_SEC = 3.0


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            print("\n[cleanup] switch to HOLD")
            interface.send_fsm_command(2)
            time.sleep(0.5)
    except Exception as exc:
        print(f"[cleanup] warn: {exc}")
    finally:
        if interface.is_connected:
            interface.disconnect()
            print("[cleanup] disconnected")


def main() -> int:
    print("=" * 70)
    print("send_body_relative() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        if interface.body_target_relative_pub is None:
            print("skip: body target relative topic not detected")
            return 0

        print("-" * 70)
        print(f"send_body_relative({DX}, 0, 0)")
        interface.send_body_relative(DX, 0.0, 0.0)
        print(f"waiting {WAIT_SEC:.1f}s...")
        time.sleep(WAIT_SEC)

        print("-" * 70)
        print(f'send_body_relative({-DX}, 0, 0, frame_id="right_tcp")')
        interface.send_body_relative(-DX, 0.0, 0.0, frame_id="right_tcp")
        print(f"waiting {WAIT_SEC:.1f}s...")
        time.sleep(WAIT_SEC)

        print("done")
        return 0
    finally:
        cleanup(interface)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(0)
    except Exception as exc:
        print(f"failed: {exc}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
