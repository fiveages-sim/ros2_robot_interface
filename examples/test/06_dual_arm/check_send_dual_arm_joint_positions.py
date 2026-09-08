"""
验证 ROS2RobotInterface.send_dual_arm_joint_positions() 的双臂关节目标下发。

send_dual_arm_joint_positions() 行为:
    - 向统一关节 topic 发布左右臂目标（MoveJ 语义）。
    - WBC 下可附带 body_positions / head_positions，按 config.joint_names 合成统一消息。
    - WBC 合成路径会缓存 body/head 目标，供 check_arrive(part='body'/'head') 使用。
    - 省略 body/head 时从 joint_states hold 当前角。

前置条件:
    - ROS 2 已 source；双臂仿真/真机在运行。
    - 已检测到统一双臂关节 topic（ocs2_wbc_controller 或 ocs2_arm_controller）。

本脚本流程:
    1. connect()；非双臂或无统一 topic 则 skip。
    2. 读当前 left_arm / right_arm 关节；末关节 ±0.05 rad。
    3. 发送第一组目标（含 body/head 固定值）。
    4. 用同一轮询同时 wait check_arrive(head) 与 check_arrive(body)，都到位后再发第二组。
    5. 第二组发送后再做同样的 head+body 到位判定；finally 切 HOLD 并 disconnect。

成功判据:
    - 两次 send_dual_arm_joint_positions() 均无异常。
    - 两组发送后 head 与 body 均在超时内同时 arrived=True。
    - 非双臂 / 无统一 topic 时打印 skip 并 return 0。

运行:
    conda run -n fa-ros2 python examples/test/06_dual_arm/check_send_dual_arm_joint_positions.py

安全说明:
    会发送双臂小幅关节运动；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

PERTURB_RAD = 0.05
TIMEOUT_SEC = 10.0
POLL_SEC = 0.2
POSITION_THRESHOLD = 0.01


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


def wait_head_and_body(interface: ROS2RobotInterface, label: str) -> bool:
    """同一轮询循环内同时等待 head 与 body 到位。"""
    start = time.time()
    last_head = None
    last_body = None
    while time.time() - start < TIMEOUT_SEC:
        last_head = interface.check_arrive(part="head", position_threshold=POSITION_THRESHOLD)
        last_body = interface.check_arrive(part="body", position_threshold=POSITION_THRESHOLD)
        head_ok = bool(last_head and last_head.get("arrived"))
        body_ok = bool(last_body and last_body.get("arrived"))
        if head_ok and body_ok:
            elapsed = time.time() - start
            print(
                f"  {label}: head/body arrived=True elapsed={elapsed:.2f}s "
                f"head_dist={last_head.get('distance'):.4f} "
                f"body_dist={last_body.get('distance'):.4f}"
            )
            return True
        time.sleep(POLL_SEC)

    print(
        f"  {label}: timeout after {TIMEOUT_SEC:.1f}s "
        f"head={last_head} body={last_body}"
    )
    return False


def main() -> int:
    print("=" * 70)
    print("send_dual_arm_joint_positions() check")
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

        categorized = interface.get_joint_state(categorized=True) or {}
        left = _arm_positions(categorized, "left_arm")
        right = _arm_positions(categorized, "right_arm")
        if not left or not right:
            print("failed: cannot read left/right arm joint positions")
            return 1

        left_target = list(left)
        right_target = list(right)
        left_target[-1] += PERTURB_RAD
        right_target[-1] -= PERTURB_RAD

        print(f"left current[-1]={left[-1]:.4f} -> target[-1]={left_target[-1]:.4f}")
        print(f"right current[-1]={right[-1]:.4f} -> target[-1]={right_target[-1]:.4f}")
        print("body=[-0.9, -0.18, -0.9, 0.0] head=[0.0, -0.26]")

        print("-" * 70)
        print("send_dual_arm_joint_positions(perturbed)")
        interface.send_dual_arm_joint_positions(
            left_target,
            right_target,
            body_positions=[-0.9, -1.8, -0.9, 0.0],
            head_positions=[0.0, -0.26],
        )
        print("check_arrive(head+body) after first send...")
        if not wait_head_and_body(interface, "perturbed"):
            return 1

        print("-" * 70)
        print("send_dual_arm_joint_positions(restore)")
        interface.send_dual_arm_joint_positions(
            left,
            right,
            body_positions=[-0.7, -1.4, -0.7, 0.0],
            head_positions=[0.0, 0.26],
        )
        print("check_arrive(head+body) after restore...")
        if not wait_head_and_body(interface, "restore"):
            return 1

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
