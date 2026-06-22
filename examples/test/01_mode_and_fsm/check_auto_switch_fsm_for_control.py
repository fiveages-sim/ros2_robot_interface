"""
验证 ROS2RobotInterface.auto_switch_fsm_for_control() 的 FSM 自动切换行为。

前置条件:
    须在 ROS2RobotInterfaceConfig 中开启 auto_switch_fsm_before_control=True，
    否则 auto_switch_fsm_for_control() 会直接返回 False，不会发送任何 FSM 切换命令。
    本脚本在连接时已显式开启该开关。

测试流程:
    连接机器人后，依次对下列 control_type 调用 auto_switch_fsm_for_control()，
    打印每次调用前后的 FSM 状态及返回值 switched：
        - True  : 已发送 FSM 切换命令
        - False : 当前状态已满足要求，或 control_type 为 other，或开关未开启

支持的 control_type 及预期切换目标:
    arm_pose    末端笛卡尔位姿 / 路径控制  -> OCS2 (3)
    arm_joint   手臂关节空间控制          -> MOVEJ (4)
    body_joint  身体 / 腰部关节控制        -> WBC: MOVEJ (4)；非 WBC: 已在 MOVEJ/OCS2 则不切，否则 OCS2 (3)
    head_joint  头部关节控制              -> 规则同 body_joint
    other       不触发自动切换            -> 始终返回 False

FSM 状态码: 1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ

同时输出 is_wbc 与 auto_switch_fsm_before_control，便于对照上述切换规则。
结束时在 finally 中尽量切回 HOLD 并断开连接。

运行:
    conda run -n fa-ros2 python examples/test/01_mode_and_fsm/check_auto_switch_fsm_for_control.py
"""

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


FSM_NAMES = {
    1: "HOME",
    2: "HOLD",
    3: "OCS2",
    4: "MOVEJ",
}

CONTROL_TYPES = [
    "arm_pose",
    "arm_joint",
    "body_joint",
    "head_joint",
    "other",
]


def fsm_name(state: int) -> str:
    return FSM_NAMES.get(state, f"UNKNOWN({state})")


def print_state(interface: ROS2RobotInterface, label: str) -> None:
    state = interface.get_fsm_state()
    print(f"{label}: {state} ({fsm_name(state)})")


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            print("\n[cleanup] switch to HOLD")
            interface.send_fsm_command(2)
            time.sleep(0.5)
    except Exception as exc:
        print(f"[cleanup] warn: failed to switch HOLD: {exc}")
    finally:
        try:
            interface.disconnect()
            print("[cleanup] disconnected")
        except Exception as exc:
            print(f"[cleanup] warn: failed to disconnect: {exc}")


def main() -> int:
    print("=" * 70)
    print("auto_switch_fsm_for_control check")
    print("=" * 70)

    config = ROS2RobotInterfaceConfig(auto_switch_fsm_before_control=True)
    interface = ROS2RobotInterface(config)
    interface.connect()
    time.sleep(1.0)

    try:
        print(f"is_wbc: {interface.is_wbc}")
        print(f"auto_switch_fsm_before_control: {interface.config.auto_switch_fsm_before_control}")
        print_state(interface, "initial")

        for control_type in CONTROL_TYPES:
            print("-" * 70)
            print(f"auto_switch_fsm_for_control({control_type!r})")
            print_state(interface, "before")
            switched = interface.auto_switch_fsm_for_control(control_type)
            time.sleep(3.0)
            print(f"switched: {switched}")
            print_state(interface, "after")

        return 0
    finally:
        cleanup(interface)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted by user")
        sys.exit(1)
    except Exception as exc:
        print(f"\nfailed: {exc}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
