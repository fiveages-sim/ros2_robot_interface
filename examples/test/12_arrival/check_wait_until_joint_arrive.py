"""
验证 ROS2RobotInterface.wait_until_joint_arrive() 的双臂关节空间到位判定。

wait_until_joint_arrive() 行为:
    - 在关节空间对比当前关节位置与显式传入的目标位置列表。
    - 所有指定组（left / right / body）的最大绝对误差 <= joint_tolerance 才算到达。
    - 返回 arrived、elapsed、left_error_max_abs、right_error_max_abs、reason 等字段。
    - 注意：此接口需显式传目标（无内部缓存），与 check_arrive(part='left_arm')
      （基于笛卡尔位姿）不同。

前置条件:
    - ROS 2 已 source；双臂仿真/真机在运行。
    - 已检测到统一双臂关节 topic（ocs2_wbc_controller 或 ocs2_arm_controller）。

本脚本流程:
    1. connect()；非双臂或无统一 topic 则 skip。
    2. 读当前 left_arm / right_arm 关节；末关节 ±0.05 rad。
    3. 发送 perturb 后的关节目标。
    4. 用 wait_until_joint_arrive() 等待双臂关节到位。
    5. 恢复原始关节位置。
    6. 再次用 wait_until_joint_arrive() 等待双臂关节到位。
    7. finally 切 HOLD 并 disconnect。

成功判据:
    - 两次 wait_until_joint_arrive() 均在超时内 arrived=True。
    - 返回结果中的 left_error_max_abs / right_error_max_abs 合理。
    - 非双臂 / 无统一 topic 时打印 skip 并 return 0。

运行:
    conda run -n fa-ros2 python examples/test/12_arrival/check_wait_until_joint_arrive.py

安全说明:
    会发送双臂小幅关节运动（末关节 ±0.05 rad）；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

PERTURB_RAD = 0.05
TIMEOUT_SEC = 10.0
JOINT_TOLERANCE = 0.03


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


def _arm_positions(categorized: dict, side: str) -> list[float] | None:
    data = categorized.get(side) or {}
    positions = data.get("positions")
    if not positions:
        return None
    return [float(p) for p in positions]


def _print_result(label: str, result: dict) -> None:
    arrived = result.get("arrived", False)
    elapsed = result.get("elapsed", 0.0)
    left_err = result.get("left_error_max_abs")
    right_err = result.get("right_error_max_abs")
    reason = result.get("reason", "?")
    print(
        f"  {label}: arrived={arrived} elapsed={elapsed:.2f}s "
        f"reason={reason} "
        f"left_err={left_err} "
        f"right_err={right_err}"
    )


def main() -> int:
    print("=" * 70)
    print("wait_until_joint_arrive() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        if not interface.config.right_end_effector_pose_topic:
            print("skip: dual-arm only")
            return 0

        unified = interface.config.unified_arm_joint_controller_topic
        print(f"unified_arm_joint_controller_topic: {unified}")
        print(f"is_wbc: {interface.is_wbc}")
        if not unified or interface.unified_arm_joint_controller_pub is None:
            print("skip: unified dual-arm joint controller topic not detected")
            return 0

        # 读取当前双臂关节位置
        categorized = interface.get_joint_state(categorized=True) or {}
        left = _arm_positions(categorized, "left_arm")
        right = _arm_positions(categorized, "right_arm")
        if not left or not right:
            print("failed: cannot read left/right arm joint positions")
            return 1

        # 构造 perturb 目标（末关节 ±0.05 rad）
        left_target = list(left)
        right_target = list(right)
        left_target[-1] += PERTURB_RAD
        right_target[-1] -= PERTURB_RAD

        print(f"\nleft  current[-1]={left[-1]:.4f} -> target[-1]={left_target[-1]:.4f}")
        print(f"right current[-1]={right[-1]:.4f} -> target[-1]={right_target[-1]:.4f}")

        # ---- 第 1 轮：发送 perturb 目标 ----
        print("-" * 70)
        print("[1/2] send_dual_arm_joint_positions(perturbed)")
        interface.send_dual_arm_joint_positions(left_target, right_target)

        print(f"[1/2] wait_until_joint_arrive(timeout={TIMEOUT_SEC}s, tol={JOINT_TOLERANCE}) ...")
        result = interface.wait_until_joint_arrive(
            left_target_positions=left_target,
            right_target_positions=right_target,
            timeout=TIMEOUT_SEC,
            joint_tolerance=JOINT_TOLERANCE,
        )
        _print_result("perturb", result)
        if not result.get("arrived"):
            print("FAILED: left/right arm did not arrive at perturbed target")
            return 1

        # ---- 第 2 轮：恢复原位 ----
        print("-" * 70)
        print("[2/2] send_dual_arm_joint_positions(restore)")
        interface.send_dual_arm_joint_positions(left, right)

        print(f"[2/2] wait_until_joint_arrive(timeout={TIMEOUT_SEC}s, tol={JOINT_TOLERANCE}) ...")
        result = interface.wait_until_joint_arrive(
            left_target_positions=left,
            right_target_positions=right,
            timeout=TIMEOUT_SEC,
            joint_tolerance=JOINT_TOLERANCE,
        )
        _print_result("restore", result)
        if not result.get("arrived"):
            print("FAILED: left/right arm did not arrive at restored target")
            return 1

        print("-" * 70)
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
