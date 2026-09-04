"""测试腰部 MOVEJ：初始位姿到目标位姿，单段时长 2 秒。

给定左右臂关节在初始和目标位姿中相同；本测试仅下发 4 个腰部关节，
不驱动双臂。

运行：
    conda run -n fa-ros2 python examples/test-demo/demo_waist_movej.py

安全说明：
    会驱动真机腰部。确认周围安全后，按 Enter 开始执行。
"""

from __future__ import annotations

import math
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


INITIAL_BODY_DEG = [0.00, 0.00, 0.00, 0.00]
TARGET_BODY_DEG = [-71.78, -131.45, -59.60, 0.00]
MOVEJ_DURATION_SEC = 2.0
ARRIVAL_TIMEOUT_SEC = 15.0


def degrees_to_radians(values: list[float]) -> list[float]:
    return [math.radians(value) for value in values]


def move_body(
    interface: ROS2RobotInterface,
    name: str,
    target_deg: list[float],
) -> None:
    target_rad = degrees_to_radians(target_deg)
    print(f"[{name}] body deg: {target_deg}")
    print(f"[{name}] body rad: {[round(value, 4) for value in target_rad]}")
    interface.send_body_joint_positions(target_rad)

    result = interface.wait_until_joint_arrive(
        body_target_positions=target_rad,
        timeout=ARRIVAL_TIMEOUT_SEC,
    )
    if not result.get("arrived", False):
        raise RuntimeError(f"[{name}] 腰部未在超时内到位: {result}")

    print(f"[{name}] 腰部已到位")


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            print("\n[cleanup] 切换到 HOLD")
            interface.send_fsm_command(2)
            time.sleep(0.5)
    except Exception as exc:
        print(f"[cleanup] HOLD 失败: {exc}")
    finally:
        interface.disconnect()


def main() -> int:
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        print("连接 ROS 2 接口...")
        interface.connect()
        time.sleep(1.0)

        if (
            interface.config.body_joint_controller_topic is None
            or interface.body_joint_controller_pub is None
        ):
            print("错误：未检测到腰部关节控制 topic")
            return 1

        print(f"body controller: {interface.body_controller}")
        print(f"MOVEJ duration: {MOVEJ_DURATION_SEC:.1f}s")
        input("将驱动真机腰部，按 Enter 继续（Ctrl+C 取消）: ")

        if not interface.set_node_parameters(
            interface.body_controller,
            {"movej_duration": MOVEJ_DURATION_SEC},
        ):
            raise RuntimeError("设置 body controller 的 movej_duration 失败")

        move_body(interface, "初始位姿", INITIAL_BODY_DEG)
        move_body(interface, "目标位姿", TARGET_BODY_DEG)
        print("\n腰部 MOVEJ 测试完成")
        return 0
    except KeyboardInterrupt:
        print("\n操作人员中止腰部 MOVEJ 测试")
        return 130
    except Exception as exc:
        print(f"\n腰部 MOVEJ 测试失败: {exc}")
        return 1
    finally:
        cleanup(interface)


if __name__ == "__main__":
    raise SystemExit(main())