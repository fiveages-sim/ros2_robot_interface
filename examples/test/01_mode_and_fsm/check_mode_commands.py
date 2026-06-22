import argparse
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


FSM_NAMES = {
    1: "HOME",
    2: "HOLD",
    3: "OCS2",
    4: "MOVEJ",
}


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check ROS2RobotInterface.send_mode_command().")
    parser.add_argument(
        "--command",
        type=str,
        default=None,
        help="Mode command to publish, for example BODY_FREE, BODY_LOCK, BASE_LOCK, BASE_UNLOCK.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("=" * 70)
    print("Mode command check")
    print("=" * 70)

    if not args.command:
        print("No --command provided. Dry run only; no mode command will be sent.")
        print("Example: --command BODY_FREE")
        return 0

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        print_state(interface, "fsm before mode command")
        print(f"send_mode_command({args.command!r})")
        interface.send_mode_command(args.command)
        print_state(interface, "fsm after mode command")
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
