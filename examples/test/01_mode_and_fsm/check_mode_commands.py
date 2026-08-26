"""
验证 ROS2RobotInterface.send_mode_command() 的模式命令发布行为。

前置条件（通用）:
    - ROS 2 已 source；WBC 仿真/真机在运行。
    - 已 connect()，且对端存在 /mode_command 话题。
    - 发送 mode 命令前，FSM 须处于 OCS2 (3)；否则底层控制器通常不会响应模式切换。
    - 本脚本连接后会先 send_fsm_command(3) 切到 OCS2，再依次测试各 mode 命令。

左右臂使能 / 双臂耦合 专用前置条件:
    - 须为 WBC 栈（检测到 ocs2_wbc_controller，即 interface.is_wbc=True）。
      非 WBC 时 LEFT_ARM_* / RIGHT_ARM_* / ARMS_* 命令通常无效，脚本会 skip 该段序列。
    - 发送 LEFT_ARM_ENABLE/DISABLE、RIGHT_ARM_ENABLE/DISABLE 前，须先处于双臂独立
      （ARMS_INDEPENDENT）。若当前为 ARMS_COUPLED，单臂使能切换会被底层忽略（与 RViz
      OCS2FSMPanel 一致：耦合开启时左右臂开关不可点）。
    - ARMS_COUPLED 要求左右臂均已启用，且机器人具备双臂耦合能力（has_bimanual_coupling）。
    - 本脚本在臂使能序列开头会先发 ARMS_INDEPENDENT，末尾再发 ARMS_INDEPENDENT 恢复。

HOME_JOINT_ON / HOME_JOINT_OFF 专用前置条件:
    - 须为 WBC 栈（interface.is_wbc=True），且 task 文件已启用 HOME 关节参考能力
      （homeJointReference.activate=true，参见 fa-w2-description/config/ocs2/*.info）。
      未启用时控制器会忽略该命令并打印 "HOME joint reference command ignored"。
    - HOME_JOINT_* 是参考约束开关，不是 HumanoidMode 位，由 WBC 控制器在任何 FSM 状态
      （HOLD/MOVEJ/OCS2）都会应用；但仍建议保持脚本默认流程（先切 OCS2）。
    - send_mode_command 只负责发布；HOME_JOINT_* 没有对应的 current_state 字段映射，
      无法用 wait_until_mode_commands_applied 确认，需直接读
      current_state.home_joint_reference_enabled 判断是否生效。

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
    BODY_FREE           身体自由模式
    BODY_LOCK           身体锁定
    BODY_RELATIVE       身体相对控制（别名 BODY_VERTICAL）
    BODY_TRACKING       身体跟踪
    BODY_HEAD_COUPLED   身体-头耦合
    BASE_LOCK           底盘锁定
    BASE_UNLOCK         底盘解锁
    ARMS_INDEPENDENT    双臂独立（解除耦合；左右臂使能切换前提）
    RIGHT_ARM_ENABLE    启用右臂
    RIGHT_ARM_DISABLE   禁用右臂
    LEFT_ARM_ENABLE     启用左臂
    LEFT_ARM_DISABLE    禁用左臂
    ARMS_COUPLED        双臂耦合（要求左右臂均已启用）
    HOME_JOINT_ON       参考关节开启（软拉向 controller HOME）
    HOME_JOINT_OFF      参考关节关闭

本脚本流程:
    1. 连接接口，打印初始 get_fsm_state()。
    2. send_fsm_command(3) 切到 OCS2，等待 settle。
    3. 按 BASE_SEQUENCE 依次 send_mode_command()，每条命令前后打印 FSM 状态。
    4. 若 is_wbc=True，再按 ARM_SEQUENCE 测试左右臂使能与双臂耦合。
    5. 发送后再额外等待 wait_sec，便于观察对端响应。
    6. finally 中尽量切回 HOLD 并断开连接。

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

BASE_SEQUENCE = [
    ("身体自由模式", "BODY_FREE", 3.0),
    ("身体锁定", "BODY_LOCK", 3.0),
    ("身体相对控制", "BODY_RELATIVE", 3.0),
    ("身体跟踪", "BODY_TRACKING", 3.0),
    ("身体-头耦合", "BODY_HEAD_COUPLED", 3.0),
    ("底盘锁定", "BASE_LOCK", 3.0),
    ("底盘解锁", "BASE_UNLOCK", 3.0),
]

# 须先 ARMS_INDEPENDENT，再测单臂使能；ARMS_COUPLED 放在两臂均 enable 之后。
ARM_SEQUENCE = [
    ("双臂独立（解除耦合）", "ARMS_INDEPENDENT", 2.0),
    ("启用右臂", "RIGHT_ARM_ENABLE", 2.0),
    ("禁用右臂", "RIGHT_ARM_DISABLE", 2.0),
    ("启用右臂（恢复）", "RIGHT_ARM_ENABLE", 2.0),
    ("禁用左臂", "LEFT_ARM_DISABLE", 2.0),
    ("启用左臂（恢复）", "LEFT_ARM_ENABLE", 2.0),
    ("双臂耦合", "ARMS_COUPLED", 2.0),
    ("双臂独立（恢复）", "ARMS_INDEPENDENT", 2.0),
]

# HOME_JOINT_ON/OFF 走 /mode_command，由 WBC 控制器 handleHomeJointReferenceCommand 处理
# （先于 resolveHumanoidModeCommand），不进入 mode 位解析；末尾恢复为「开启」。
# 生效确认读 current_state.home_joint_reference_enabled（wait_until_* 无 HOME_JOINT 映射）。
HOME_JOINT_SEQUENCE = [
    ("参考关节开启（软拉向 HOME）", "HOME_JOINT_ON", 2.0),
    ("参考关节关闭", "HOME_JOINT_OFF", 2.0),
    ("参考关节开启（恢复）", "HOME_JOINT_ON", 2.0),
]


def fsm_name(state: int) -> str:
    return FSM_NAMES.get(state, f"UNKNOWN({state})")


def print_state(interface: ROS2RobotInterface, label: str) -> None:
    state = interface.get_fsm_state()
    print(f"{label}: {state} ({fsm_name(state)})")


def run_sequence(
    interface: ROS2RobotInterface,
    sequence: list[tuple[str, str, float]],
    section: str,
) -> None:
    print("=" * 70)
    print(section)
    print("=" * 70)
    for label, command, wait_sec in sequence:
        print("-" * 70)
        print(f"send_mode_command({command!r}) -> {label}")
        print_state(interface, "before")
        interface.send_mode_command(command)
        time.sleep(wait_sec)
        print_state(interface, "after")


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

        run_sequence(interface, BASE_SEQUENCE, "body / base mode commands")

        if not interface.is_wbc:
            print("-" * 70)
            print("skip arm mode commands: WBC controller not detected (is_wbc=False)")
            print("  LEFT_ARM_* / RIGHT_ARM_* / ARMS_* require ocs2_wbc_controller")
        else:
            run_sequence(
                interface,
                ARM_SEQUENCE,
                "arm enable / bimanual coupling mode commands",
            )
            run_sequence(
                interface,
                HOME_JOINT_SEQUENCE,
                "HOME joint reference mode commands",
            )
            enabled = (
                interface.wbc_state.home_joint_reference_enabled
                if interface.wbc_state is not None
                else None
            )
            print(f"current_state.home_joint_reference_enabled: {enabled}")
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
