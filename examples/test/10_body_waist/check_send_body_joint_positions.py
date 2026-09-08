"""
验证 ROS2RobotInterface.send_body_joint_positions() 的躯干关节目标下发。

send_body_joint_positions() 行为:
    - 向躯干关节控制器发布 Float64MultiArray（弧度）。
    - connect() 时自动检测 topic：
      /ocs2_wbc_controller/target_joint_position/body
      或 /body_joint_controller/target_joint_position。
    - 会缓存目标位置，供 check_arrive(part='body') 使用。

本脚本流程:
    1. connect()，检查 body_joint_controller_topic。
    2. 调用 send_body_joint_positions(TARGET)，等待数秒。
    3. 调用 send_body_joint_positions(HOME) 回位。
    4. finally 中切 HOLD 并 disconnect()。

成功判据:
    - 两次 send_body_joint_positions() 均无异常。
    - 未检测到 body topic 时打印 skip 并正常退出。

前置条件:
    ROS 2 已 source；机器人/仿真在运行；躯干为 4 关节（body_joint1–4）。

运行:
    conda run -n fa-ros2 python examples/test/10_body_waist/check_send_body_joint_positions.py

安全说明:
    会发送躯干小幅运动；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# body_joint1–4（弧度）；第 4 关节为腰部旋转，TARGET 相对 HOME 增加 0.3 rad
TARGET = [-0.8, -1.6, -0.8, 0.3]
HOME = [-0.8, -1.6, -0.8, 0.0]
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
    print("send_body_joint_positions() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        topic = interface.config.body_joint_controller_topic
        print(f"body_joint_controller_topic: {topic}")
        if topic is None or interface.body_joint_controller_pub is None:
            print("skip: body joint controller topic not detected")
            return 0

        print("-" * 70)
        print(f"send_body_joint_positions({TARGET})")
        interface.send_body_joint_positions(TARGET)
        print(f"waiting {WAIT_SEC:.1f}s...")
        time.sleep(WAIT_SEC)

        print("-" * 70)
        print(f"send_body_joint_positions({HOME})")
        interface.send_body_joint_positions(HOME)
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
