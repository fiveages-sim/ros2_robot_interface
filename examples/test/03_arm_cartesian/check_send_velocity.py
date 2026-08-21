"""
验证 ArmHandler.send_velocity() 的左右臂连续笛卡尔速度流下发。

send_velocity() 行为:
    - 向 {end_effector_target_topic}/twist 发布 geometry_msgs/msg/Twist（米/s、弧度/s）。
    - publisher 由 connect() → ArmHandler.initialize() 从 target_topic 推导，无独立 config 字段。
    - 自动切 FSM 到 OCS2；发布前清空 latest_target_pose。
    - 控制器 latch 每条 Twist 并按控制周期积分；0.2s 无新消息会自动停止。

本脚本流程:
    1. connect()，检查左右臂 twist_pub。
    2. 左臂：以 0.1 m/s 沿 base_frame X 持续发布约 2s（20Hz 循环）。
    3. 左臂：发布全零速度显式停止。
    4. 右臂：同样流程（若双臂）。
    5. finally 中切 HOLD 并 disconnect()。

成功判据:
    - 各 send_velocity() 调用均无异常。
    - 某侧 twist publisher 未创建时对该侧打印 skip。
    - 手臂沿 base_frame X 平移约 20 cm，肉眼可见。

前置条件:
    ROS 2 已 source；机器人/仿真在运行；OCS2 笛卡尔目标话题存在。

运行:
    .venv/bin/python examples/test/03_arm_cartesian/check_send_velocity.py

安全说明:
    会让左右臂各沿 base_frame X 平移约 20 cm（0.1 m/s × 2s），随后停止；
    请在手臂前方留有足够空间；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

VELOCITY = (0.1, 0.0, 0.0)  # 线速度 (vx, vy, vz)，m/s，base_frame X 正向
ANGULAR = (0.0, 0.0, 0.0)   # 角速度 (wx, wy, wz)，rad/s
ZERO = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))  # 显式停止
DURATION_SEC = 2.0   # 持续发布时长
PERIOD_SEC = 0.05    # 发布周期（20Hz，远高于 5Hz 维持下限）


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


def drive(handler, label: str) -> None:
    """对单个臂发布速度流并显式停止。"""
    if handler is None or handler.twist_pub is None:
        print(f"skip: {label} twist publisher not initialized")
        return

    displacement = VELOCITY[0] * DURATION_SEC
    print(f"{label}.send_velocity({VELOCITY}, {ANGULAR}) x{DURATION_SEC / PERIOD_SEC:.0f} 次")
    print(f"  → 观察 {label} 沿 base_frame X 平移约 {displacement * 100:.0f} cm")
    deadline = time.monotonic() + DURATION_SEC
    while time.monotonic() < deadline:
        handler.send_velocity(VELOCITY, ANGULAR)
        time.sleep(PERIOD_SEC)

    print(f"{label}.send_velocity(0,0,0 / 0,0,0) 显式停止")
    handler.send_velocity(*ZERO)
    time.sleep(0.5)


def main() -> int:
    print("=" * 70)
    print("send_velocity() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        print("-" * 70)
        drive(interface.left_arm_handler, "left_arm_handler")

        print("-" * 70)
        drive(interface.right_arm_handler, "right_arm_handler")

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
