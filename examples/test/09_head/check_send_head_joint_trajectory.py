"""
验证 ROS2RobotInterface.send_head_joint_trajectory() 的头部多路点轨迹下发（split 拓扑）。

send_head_joint_trajectory() 行为:
    - 向 /head_joint_controller/target_joint_trajectory 发布 trajectory_msgs/JointTrajectory。
    - 调用方至少提供 2 个路点；控制器会把当前位置自动插到轨迹起点。
    - 传入 trajectory_duration 时，发布前将 head 控制器参数 movej_trajectory_duration 设为该值。
    - 内部 auto_switch_fsm_for_control("head_joint")；缓存末路点供 check_arrive(part='head')。

本脚本流程:
    1. connect()；无 head_joint_trajectory_pub 则 skip。
    2. 从 get_joint_state(categorized=True) 取 head 关节名与当前位置。
    3. 构造两个路点：首关节 + 小偏移，再回到当前。
    4. send_head_joint_trajectory(..., trajectory_duration=TRAJ_DURATION)。
    5. 等待后打印 check_arrive(part='head')。
    6. finally 切 HOLD 并 disconnect。

成功判据:
    - send_head_joint_trajectory() 无异常。
    - 未检测到轨迹 publisher 或 head 关节状态不可用时打印 skip 并 return 0。

前置条件:
    ROS 2 已 source；split 拓扑（head_joint_controller）在运行。

运行:
    .venv/bin/python examples/test/09_head/check_send_head_joint_trajectory.py

安全说明:
    会让头部首关节小幅运动（约 0.1 rad）再复位；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

DELTA = 0.1  # 弧度，首关节偏移量
TRAJ_DURATION = 3.0
WAIT_MARGIN = 2.0


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


def _head_joint_state(interface: ROS2RobotInterface) -> tuple[list[str], list[float]] | None:
    categorized = interface.get_joint_state(categorized=True)
    if not categorized:
        return None
    data = categorized.get("head") or {}
    names = data.get("names") or []
    positions = data.get("positions") or []
    if not names or not positions or len(names) != len(positions):
        return None
    return [str(n) for n in names], [float(p) for p in positions]


def main() -> int:
    print("=" * 70)
    print("send_head_joint_trajectory() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        if interface.head_joint_trajectory_pub is None:
            print("skip: head joint trajectory publisher not initialized "
                  "(target_joint_trajectory topic not detected)")
            return 0

        state = _head_joint_state(interface)
        if state is None:
            print("skip: head joint state not available")
            return 0
        joint_names, current = state
        print(f"head joints ({len(joint_names)}): {joint_names}")

        wp_off = list(current)
        wp_off[0] += DELTA
        waypoints = [wp_off, list(current)]

        print("-" * 70)
        print(f"send_head_joint_trajectory(names, {len(waypoints)} waypoints, duration={TRAJ_DURATION})")
        print(f"  waypoint0[0] = {waypoints[0][0]:.4f} (current[0] + {DELTA})")
        interface.send_head_joint_trajectory(
            joint_names, waypoints, trajectory_duration=TRAJ_DURATION
        )

        wait = TRAJ_DURATION + WAIT_MARGIN
        print(f"waiting {wait:.1f}s...")
        time.sleep(wait)

        arrive = interface.check_arrive(part="head")
        print(f"check_arrive(part='head'): {arrive}")
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
