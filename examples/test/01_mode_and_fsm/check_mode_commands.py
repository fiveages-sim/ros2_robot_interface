"""
验证 ROS2RobotInterface.send_mode_command() 的模式命令发布行为。

前置条件:
    发送 mode 命令前，FSM 须处于 OCS2 (3)；否则底层控制器通常不会响应模式切换。
    本脚本连接后会先 send_fsm_command(3) 切到 OCS2，再依次测试各 mode 命令。

send_mode_command(command: str) 用法:
    在已连接的 ROS2RobotInterface 实例上调用，向 /mode_command 发布模式字符串。
    无返回值；未连接时抛出 ROS2NotConnectedError。

    示例:
        from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

        interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
        interface.connect()
        interface.send_fsm_command(3)              # 先切到 OCS2
        interface.send_mode_command("BODY_FREE")   # 切换为身体自由模式
        interface.send_mode_command("BASE_LOCK")   # 锁定底盘
        interface.disconnect()

send_mode_command() 行为（本脚本依次调用该接口）:
    - 向 /mode_command 话题 publish std_msgs/String 命令字符串。
    - publish 后固定等待 MODE_SWITCH_SETTLE_TIME_SEC（默认 0.1s），
      避免后续指令在模式尚未切换时发出；这是盲等，不轮询对端确认。
    - 若需确认生效，请在发完后调用 wait_until_mode_commands_applied([...])，
      对照 /ocs2_wbc_controller/current_state（多条命令合并检查，勿按条重复 wait）。
    - 若 mode_command_pub 未初始化（对端无 /mode_command），仅 warning 并 return，
      不会抛出异常。

与 FSM 的关系:
    mode 命令控制 WBC / 底盘等行为模式；生效前提是 FSM 已在 OCS2 (3)。
    本脚本在每条 mode 命令发送前后打印 get_fsm_state()，确认测试期间 FSM 保持 OCS2。

内置测试序列（字符串需与底层控制器约定一致）:
    BODY_FREE         身体自由模式
    BODY_LOCK         身体锁定
    BODY_RELATIVE     身体相对控制（别名 BODY_VERTICAL）
    BODY_TRACKING     身体跟踪
    BODY_HEAD_COUPLED 身体-头耦合
    BASE_LOCK         底盘锁定
    BASE_UNLOCK       底盘解锁

本脚本流程:
    1. 连接接口，打印初始 get_fsm_state()。
    2. send_fsm_command(3) 切到 OCS2，等待 settle。
    3. 按 SEQUENCE 依次 send_mode_command()，每条命令前后打印 FSM 状态。
    4. 发送后再额外等待 wait_sec，便于观察对端响应。
    5. finally 中尽量切回 HOLD 并断开连接。

运行:
    conda run -n fa-ros2 python examples/test/01_mode_and_fsm/check_mode_commands.py
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
    ("身体自由模式", "BODY_FREE", 3.0),
    ("身体锁定", "BODY_LOCK", 3.0),
    ("身体相对控制", "BODY_RELATIVE", 3.0),
    ("身体跟踪", "BODY_TRACKING", 3.0),
    ("身体-头耦合", "BODY_HEAD_COUPLED", 3.0),
    ("底盘锁定", "BASE_LOCK", 3.0),
    ("底盘解锁", "BASE_UNLOCK", 3.0),
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
    print("Mode command check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        print_state(interface, "initial")
        print("-" * 70)
        print("send_fsm_command(3) -> OCS2 (required before mode commands)")
        print_state(interface, "before OCS2")
        interface.send_fsm_command(3)
        time.sleep(1.0)
        print_state(interface, "after OCS2")

        for label, command, wait_sec in SEQUENCE:
            print("-" * 70)
            print(f"send_mode_command({command!r}) -> {label}")
            print_state(interface, "before")
            interface.send_mode_command(command)
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
