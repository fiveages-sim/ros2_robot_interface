"""
验证 ROS2RobotInterface.send_fsm_command() 的 FSM 手动切换行为。

FSM 状态码:
    1=HOME  2=HOLD  3=OCS2  4=MOVEJ

send_fsm_command() 行为（本脚本直接调用该接口）:
    - 每次 publish 后固定等待 config.fsm_state_switch_settle_time（默认 0.3s），
      避免后续控制指令在旧状态下执行；这是盲等，不轮询 /fsm_state 确认。
    - 切换 HOME / OCS2 / MOVEJ 时，若当前不在 HOLD 且也不是目标状态，
      会自动先 publish HOLD，再 publish 目标状态（必要时 sleep 两次）。
    - 不做完整 FSM 转换图校验，其余非法转换由底层控制器处理。

本脚本流程:
    1. 连接接口，打印初始 get_fsm_state()。
    2. 先 HOLD -> HOME（合法路径，当前需在 HOLD）。
    3. 在 HOME 状态下直接 send_fsm_command(OCS2)，不手动先发 HOLD；
       预期 send_fsm_command 自动插入 HOLD 中转（HOME -> HOLD -> OCS2）。
    4. 每次 send_fsm_command() 前后打印 FSM 状态；发送后再额外等待 wait_sec。
    5. finally 中尽量切回 HOLD 并断开连接。

运行:
    conda run -n fa-ros2 python examples/test/01_mode_and_fsm/check_fsm_commands.py
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

SEQUENCE = [
    ("HOLD", 2, 1.0),
    ("HOME", 1, 5.0),
    # 从 HOME 直接切 OCS2，不手动先发 HOLD；验证 send_fsm_command 自动插入 HOLD
    ("OCS2 (direct from HOME)", 3, 1.0),
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
    print("FSM command check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        print_state(interface, "initial")
        for name, command, wait_sec in SEQUENCE:
            print("-" * 70)
            print(f"send_fsm_command({command}) -> {name}")
            print_state(interface, "before")
            interface.send_fsm_command(command)
            time.sleep(wait_sec)
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
