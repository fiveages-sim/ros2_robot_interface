"""
验证 ROS2RobotInterface.send_body_target() 的 body 绝对位姿目标下发。

send_body_target() 行为:
    - 向 /body_target 发布 geometry_msgs/msg/Pose（控制器 base_frame 下，立即生效、不插值）。
    - connect() 时自动检测 topic：/body_target。
    - 自动切 FSM 到 OCS2，再切 BODY_TRACKING；发布前清空 body_current_target_pose。

本脚本流程:
    1. connect()，检查 body_target_pub。
    2. 读取当前 body 位姿（get_body_current_pose()），叠加 z 负方向偏移作为绝对目标。
    3. 调用 send_body_target()（base_frame 绝对位姿）。
    4. finally 中切 HOLD 并 disconnect()。

成功判据:
    - send_body_target() 调用无异常。
    - 未检测到 body target topic 或当前 body 位姿不可用时打印 skip 并正常退出。

前置条件:
    ROS 2 已 source；机器人/仿真在运行；WBC 且 body tracking 可用。

运行:
    .venv/bin/python examples/test/10_body_waist/check_send_body_target.py

安全说明:
    会发送 body 在 base_frame 下沿 z 负方向约 10 cm 的绝对位姿偏移；结束切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time
from copy import deepcopy

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

DZ = 0.1  # 米，沿 base_frame Z 负方向的绝对偏移
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
    print("send_body_target() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        if interface.body_target_pub is None:
            print("skip: body target topic not detected")
            return 0

        current = interface.get_body_current_pose()
        if current is None:
            print("skip: current body pose not available (wait for /body_current_pose)")
            return 0

        print("-" * 70)
        print(f"send_body_target(pose z-{DZ})")
        target = deepcopy(current)
        target.position.z -= DZ
        interface.send_body_target(target)
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
