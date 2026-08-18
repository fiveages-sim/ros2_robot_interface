"""
验证 ArmHandler.send_relative() 的左右臂一次笛卡尔相对位移下发。

send_relative() 行为:
    - 向 {end_effector_target_topic}/relative 发布 TwistStamped（米 / 弧度 RPY）。
    - publisher 由 connect() → ArmHandler.initialize() 从 target_topic 推导，无独立 config 字段。
    - 自动切 FSM 到 OCS2；发布前清空 latest_target_pose。

本脚本流程:
    1. connect()，检查左右臂 relative_pub。
    2. 左臂 send_relative(+DX, frame_id="left_tcp")。
    3. 右臂 send_relative(+DX, frame_id="right_tcp")。
    4. finally 中切 HOLD 并 disconnect()。

成功判据:
    - 两次 send_relative() 均无异常。
    - 某侧 relative publisher 未创建时对该侧打印 skip。

前置条件:
    ROS 2 已 source；机器人/仿真在运行；OCS2 笛卡尔目标话题存在。

运行:
    .venv/bin/python examples/test/03_arm_cartesian/check_send_relative.py

安全说明:
    会分别发送左臂、右臂约 3 cm 平移（各自 TCP 的 X）；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

DX = 0.03  # 米，沿各臂 TCP 的 X
WAIT_SEC = 3.0
LEFT_TCP_FRAME = "left_tcp"
RIGHT_TCP_FRAME = "right_tcp"


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
    print("send_relative() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        left = interface.left_arm_handler
        right = interface.right_arm_handler

        print("-" * 70)
        if left is None or left.relative_pub is None:
            print("skip: left arm relative publisher not initialized")
        else:
            print(f'left_arm_handler.send_relative({DX}, 0, 0, frame_id="{LEFT_TCP_FRAME}")')
            left.send_relative(DX, 0.0, 0.0, frame_id=LEFT_TCP_FRAME)
            print(f"waiting {WAIT_SEC:.1f}s...")
            time.sleep(WAIT_SEC)

        print("-" * 70)
        if right is None or right.relative_pub is None:
            print("skip: right arm relative publisher not initialized")
        else:
            print(f'right_arm_handler.send_relative({DX}, 0, 0, frame_id="{RIGHT_TCP_FRAME}")')
            right.send_relative(DX, 0.0, 0.0, frame_id=RIGHT_TCP_FRAME)
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
