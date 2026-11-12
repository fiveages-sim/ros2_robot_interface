"""
验证 ROS2RobotInterface.send_joint_trajectory() 的多路点关节轨迹下发。

send_joint_trajectory() 行为:
    - 向统一话题 ``/{arm_controller}/target_joint_trajectory`` 发布
      ``trajectory_msgs/JointTrajectory``。
    - 由 ``joint_names`` 决定控制左臂 / 右臂 / 双臂（名称含 left_/right_）。
    - 调用方至少提供 2 个路点；控制器会把当前位置自动插到轨迹起点。
    - 若传入 ``trajectory_duration``，发布前将控制器参数
      ``movej_trajectory_duration`` 设为该值；``None`` 则沿用当前参数。
    - 内部会 ``auto_switch_fsm_for_control("arm_joint")``。

前置条件:
    - ROS 2 已 source；机器人/仿真在运行。
    - 已检测到臂控制器与 ``target_joint_trajectory`` 话题。

本脚本流程:
    1. connect()；无 arm_trajectory_pub 则 skip。
    2. 打印左臂当前关节状态（便于对照）。
    3. 使用复现问题的固定 joint_names / waypoints。
    4. send_joint_trajectory(..., movej_trajectory_duration=TRAJ_DURATION)。
    5. 等待 TRAJ_DURATION + 余量后打印关节状态与相对末路点误差。
    6. finally 切 HOLD 并 disconnect。

成功判据:
    - send_joint_trajectory() 无异常。
    - 未检测到轨迹 publisher 时打印 skip 并 return 0。

运行:
    conda run -n fa-ros2 python examples/test/05_arm_jointspace/check_send_joint_trajectory.py

安全说明:
    会发送左臂到下方固定绝对关节角（幅度较大）；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# 复现「动一下然后突然停止」的关节轨迹
JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
]
WAYPOINTS = [
    [1.87641669974313, -0.5687477016984709, -1.730039779714491,
     -1.9825839235531273, -1.0040110954577952, 0.07270804751898144, 0.2687643399448287],
    [0.9644474437188942, -0.5046140506501708, -1.5921780158023264,
     -0.47424058673172265, -0.7691065252830858, -0.0748338879732527, -0.8967314065858463],
    [1.5177486656402732, -1.2694007275883998, -0.89838601010572,
     -1.6746200861523974, -0.27997303648008287, -0.02395193698124802, 0.678350940785112],
]
TRAJ_DURATION = 5.0
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


def _left_arm_state(interface: ROS2RobotInterface) -> tuple[list[str], list[float]] | None:
    categorized = interface.get_joint_state(categorized=True)
    if not categorized:
        return None
    is_dual = interface.config.right_end_effector_pose_topic is not None
    key = "left_arm" if is_dual else "arm"
    data = categorized.get(key) or {}
    names = data.get("names") or []
    positions = data.get("positions") or []
    if not names or not positions or len(names) != len(positions):
        return None
    return [str(n) for n in names], [float(p) for p in positions]


def main() -> int:
    print("=" * 70)
    print("send_joint_trajectory() check (repro waypoints)")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        if interface.arm_trajectory_pub is None:
            print("skip: arm trajectory publisher not initialized "
                  "(target_joint_trajectory topic not detected)")
            return 0

        print(f"arm_controller: {interface.arm_controller}")
        print(f"arm_trajectory_pub: ready")

        state = _left_arm_state(interface)
        if state is not None:
            names, current = state
            print(f"current joints ({len(names)}): {names}")
            print(f"current positions: {[f'{p:.4f}' for p in current]}")
        else:
            print("warn: cannot read left arm joint state (continue anyway)")

        print("-" * 70)
        print(f"joint_names: {JOINT_NAMES}")
        print(f"waypoints ({len(WAYPOINTS)}):")
        for i, wp in enumerate(WAYPOINTS):
            print(f"  [{i}] {[f'{p:.4f}' for p in wp]}")
        print(f"movej_trajectory_duration={TRAJ_DURATION:.1f}s")

        interface.send_joint_trajectory(
            joint_names=JOINT_NAMES,
            waypoints=WAYPOINTS,
            movej_trajectory_duration=10.0,
            movej_trajectory_blend_ratio=0.5,
        )
        print("send_joint_trajectory() published")

        wait_s = TRAJ_DURATION + WAIT_MARGIN
        print(f"waiting {wait_s:.1f}s for trajectory execution...")
        time.sleep(wait_s)

        after = _left_arm_state(interface)
        if after is not None:
            _, positions = after
            final_wp = WAYPOINTS[-1]
            print(f"positions after: {[f'{p:.4f}' for p in positions]}")
            if len(positions) == len(final_wp):
                errs = [abs(c - t) for c, t in zip(positions, final_wp)]
                print(f"abs error vs final waypoint: {[f'{e:.4f}' for e in errs]}")
                print(f"max abs error: {max(errs):.4f} rad")

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
