"""
验证 ROS2RobotInterface.check_arrive(part='head') 的头部关节到位判定。

check_arrive(part='head') 行为:
    - 比较 head_target_positions 与当前 categorized joint state 中的 head 位置。
    - 返回 {'arrived': bool, 'distance': float}；无目标或长度不一致时 arrived=False。

前置条件:
    - ROS 2 已 source；机器人/仿真在运行。
    - 已检测到 head_joint_controller_topic。

本脚本流程:
    1. connect()，检查 head topic；无则 skip。
    2. 从 get_joint_state(categorized=True)['head'] 取当前位置。
    3. 每关节 ±0.1 rad 小扰动后 send_head_joint_positions。
    4. 轮询 check_arrive(part='head', position_threshold=0.01) 直至到位或超时。
    5. 回发原始位置并再次等待到位；finally 切 HOLD 并 disconnect。

成功判据:
    - 扰动目标与回中目标两次 check_arrive 均在超时内 arrived=True。
    - 未检测到 head topic 时打印 skip 并 return 0。

运行:
    conda run -n fa-ros2 python examples/test/09_head/check_arrive_head.py

安全说明:
    会发送头部小幅关节运动；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

PERTURB_RAD = 0.1
TIMEOUT_SEC = 8.0
POLL_SEC = 0.2
POSITION_THRESHOLD = 0.001  # 头部到位阈值（弧度）


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


def wait_head_arrive(interface: ROS2RobotInterface, label: str) -> bool:
    start = time.time()
    last = None
    while time.time() - start < TIMEOUT_SEC:
        last = interface.check_arrive(part="head", position_threshold=POSITION_THRESHOLD)
        if last and last.get("arrived"):
            elapsed = time.time() - start
            print(f"  {label}: arrived=True elapsed={elapsed:.2f}s distance={last.get('distance'):.4f}")
            return True
        time.sleep(POLL_SEC)

    distance = last.get("distance") if last else None
    print(f"  {label}: timeout after {TIMEOUT_SEC:.1f}s last={last} distance={distance}")
    return False


def main() -> int:
    print("=" * 70)
    print("check_arrive(part='head') check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        topic = interface.config.head_joint_controller_topic
        print(f"head_joint_controller_topic: {topic}")
        if topic is None or interface.head_joint_controller_pub is None:
            print("skip: head joint controller topic not detected")
            return 0

        categorized = interface.get_joint_state(categorized=True) or {}
        head = categorized.get("head") or {}
        current = head.get("positions")
        if not current:
            print("failed: no head positions in joint state")
            return 1

        current = [float(p) for p in current]
        # 交替 ±0.05，避免所有关节同向大幅运动
        target = [
            p + (PERTURB_RAD if i % 2 == 0 else -PERTURB_RAD) for i, p in enumerate(current)
        ]
        print(f"current: {current}")
        print(f"target:  {target}")

        print("-" * 70)
        print("send_head_joint_positions(perturbed)")
        interface.send_head_joint_positions(target)
        if not wait_head_arrive(interface, "perturb"):
            return 1

        print("-" * 70)
        print("send_head_joint_positions(restore)")
        interface.send_head_joint_positions(current)
        if not wait_head_arrive(interface, "restore"):
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
