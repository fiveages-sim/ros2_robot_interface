"""
验证 ROS2RobotInterface.get_fsm_state() 的 FSM 状态读取与变化监听。

FSM 状态码:
    1=HOME  2=HOLD  3=OCS2  4=MOVEJ

get_fsm_state() 行为（本脚本直接调用该接口）:
    - 返回最近一次从 /fsm_state 订阅回调缓存的状态码（int）。
    - connect() 后、尚未收到 /fsm_state 前，内部默认值为 2（HOLD）。
    - 只读查询，不 publish /fsm_command。
    - 未知状态码由订阅回调过滤，不会写入缓存。

本脚本流程:
    1. connect()，等待 --wait-sec 秒以便 /fsm_state 回调更新缓存。
    2. 输出当前 FSM 状态。
    3. 以 --poll-interval 轮询 get_fsm_state()；仅在状态变化时打印新旧状态。
    4. 持续运行，直到用户 Ctrl+C 终止；finally 中 disconnect()。

前置条件:
    ROS 2 环境已 source；机器人或仿真栈在运行时可收到 /fsm_state。

运行:
    conda run -n fa-ros2 python examples/test/01_mode_and_fsm/check_get_fsm_state.py
    conda run -n fa-ros2 python examples/test/01_mode_and_fsm/check_get_fsm_state.py --poll-interval 0.1

安全说明:
    本脚本只读查询 FSM 状态，不发送控制命令。按 Ctrl+C 退出。
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

VALID_FSM_STATES = {1, 2, 3, 4}

FSM_NAMES = {
    1: "HOME",
    2: "HOLD",
    3: "OCS2",
    4: "MOVEJ",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor ROS2RobotInterface.get_fsm_state() changes.")
    parser.add_argument("--wait-sec", type=float, default=1.0, help="seconds to wait after connect before first read")
    parser.add_argument("--poll-interval", type=float, default=0.2, help="seconds between get_fsm_state() polls")
    return parser.parse_args()


def fsm_label(state: int) -> str:
    return f"{state} ({FSM_NAMES.get(state, f'UNKNOWN({state})')})"


def read_fsm_state(interface: ROS2RobotInterface) -> int:
    state = interface.get_fsm_state()
    if state not in VALID_FSM_STATES:
        raise ValueError(
            f"invalid FSM state {state}, expected one of {sorted(VALID_FSM_STATES)}"
        )
    return state


def print_current(state: int) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] current FSM: {fsm_label(state)}")


def print_change(previous: int, current: int) -> None:
    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] FSM changed: "
        f"{fsm_label(previous)} -> {fsm_label(current)}"
    )


def monitor_fsm(interface: ROS2RobotInterface, poll_interval: float) -> None:
    last_state = read_fsm_state(interface)
    print_current(last_state)
    print("monitoring FSM changes... (Ctrl+C to stop)")

    while True:
        time.sleep(poll_interval)
        current_state = read_fsm_state(interface)
        if current_state != last_state:
            print_change(last_state, current_state)
            last_state = current_state


def main() -> int:
    args = parse_args()

    print("=" * 70)
    print("get_fsm_state() monitor")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()

    try:
        print(f"connected: is_connected={interface.is_connected}")
        if args.wait_sec > 0:
            print(f"waiting {args.wait_sec:.1f}s for /fsm_state subscription...")
            time.sleep(args.wait_sec)

        monitor_fsm(interface, args.poll_interval)
        return 0
    finally:
        if interface.is_connected:
            interface.disconnect()
            print("[cleanup] disconnected")


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
