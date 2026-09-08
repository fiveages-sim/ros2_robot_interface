"""
验证 left_arm_handler.check_arrival() 的单臂笛卡尔到位判定。

check_arrival() 行为:
    - 比较当前末端位姿与 target topic / 缓存目标位姿。
    - 返回 arrived、position_distance、orientation_angle_deg、status_message 等字段。
    - 无 target/current 时 arrived=False，status_message 可为 None。

前置条件:
    - ROS 2 已 source；机器人/仿真在运行。
    - 可获取 left_arm 位姿；笛卡尔控制需 OCS2（FSM=3）。

本脚本流程:
    1. connect()，等待位姿。
    2. HOLD → HOME → OCS2，再取当前位姿。
    3. 相对当前 z 轴 +3cm 构造目标，send_target_stamped。
    4. 轮询 left_arm_handler.check_arrival() 直至到位或超时。
    5. 回原位并再次等待；finally 切 HOLD 并 disconnect。

成功判据:
    - 去程与回程两次 check_arrival 均在超时内 arrived=True。
    - 无法获取当前位姿时失败退出。

运行:
    conda run -n fa-ros2 python examples/test/12_arrival/check_arm_check_arrival.py

安全说明:
    会发送手臂小幅笛卡尔运动（约 3cm）；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import copy
import sys
import time

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

STEP_M = 0.03
TIMEOUT_SEC = 10.0
POLL_SEC = 0.2
POSE_THRESHOLD = 0.05
ORIENT_THRESHOLD = 5.0


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


def print_pose(pose: Pose, label: str) -> None:
    p = pose.position
    o = pose.orientation
    print(f"  {label}: pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
          f"ori=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})")


def wait_arm_arrival(handler, label: str) -> bool:
    start = time.time()
    last = None
    while time.time() - start < TIMEOUT_SEC:
        last = handler.check_arrival(
            pose_threshold=POSE_THRESHOLD,
            orient_threshold=ORIENT_THRESHOLD,
        )
        if last and last.get("arrived"):
            elapsed = time.time() - start
            print(
                f"  {label}: arrived=True elapsed={elapsed:.2f}s "
                f"pos_dist={last.get('position_distance'):.4f}m "
                f"orient_deg={last.get('orientation_angle_deg'):.2f}"
            )
            return True
        time.sleep(POLL_SEC)

    print(f"  {label}: timeout after {TIMEOUT_SEC:.1f}s last={last}")
    return False


def offset_pose(src: Pose, dz: float) -> Pose:
    pose = copy.deepcopy(src)
    pose.position.z += dz
    return pose


def main() -> int:
    print("=" * 70)
    print("left_arm_handler.check_arrival() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        handler = interface.left_arm_handler
        if handler is None:
            print("skip: left_arm_handler not available")
            return 0

        print("-" * 70)
        print("switch FSM: HOLD → HOME → OCS2")
        interface.send_fsm_command(2)  # HOLD
        interface.send_fsm_command(1)  # HOME
        time.sleep(6.0)
        interface.send_fsm_command(2)  # HOLD
        interface.send_fsm_command(3)  # OCS2
        time.sleep(1.0)

        home_pose = handler.get_pose()
        if home_pose is None:
            print("failed: cannot get left arm pose")
            return 1
        print_pose(home_pose, "home_pose")

        frame_id = handler.get_frame_id() or "world"
        print(f"frame_id: {frame_id}")

        up_pose = offset_pose(home_pose, STEP_M)
        print("-" * 70)
        print_pose(up_pose, "target(+z)")
        handler.send_target_stamped(frame_id, up_pose)
        if not wait_arm_arrival(handler, "up"):
            return 1

        print("-" * 70)
        print_pose(home_pose, "target(home)")
        handler.send_target_stamped(frame_id, home_pose)
        if not wait_arm_arrival(handler, "home"):
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
