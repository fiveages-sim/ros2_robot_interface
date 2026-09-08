"""
验证 ROS2RobotInterface.check_arrive(part='body_pose') 的躯干笛卡尔到位判定。

check_arrive(part='body_pose') 行为:
    - 比较 get_body_current_pose() 与 get_body_current_target_pose()。
    - 使用与手臂相同的笛卡尔判定（位置欧氏距离 + 姿态角度双阈值）。
    - 返回 arrived、position_distance、orientation_angle_deg、status_message 等字段；
      无 current/target 时 arrived=False。

前置条件:
    - ROS 2 已 source；WBC 仿真/真机在运行。
    - connect() 能自动检测到 /body_current_pose 与 /body_current_target。
    - 双臂 handler 可用（通过 send_dual_arm_target_stamped 下发 body_pose）。
    - 控制前切入 OCS2（FSM=3）。

本脚本流程:
    1. connect()，检查 body pose topics；无则 skip。
    2. HOLD → HOME → OCS2。
    3. 读取当前双臂位姿与 body 位姿。
    4. body z +0.02 m 小扰动后 send_dual_arm_target_stamped(..., body_pose=...)。
    5. 轮询 check_arrive(part='body_pose') 直至到位或超时。
    6. 回发原始 body 位姿并再次等待到位；finally 切 HOLD 并 disconnect。

成功判据:
    - 扰动目标与回中目标两次 check_arrive 均在超时内 arrived=True。
    - 未检测到 body pose topics / 未收到消息 / 无双臂 handler 时打印 skip 并 return 0。

运行:
    conda run -n fa-ros2 python examples/test/10_body_waist/check_body_pose_arrive.py

安全说明:
    会发送躯干小幅笛卡尔运动（约 2cm）并保持双臂当前目标；结束时切 HOLD 并断开连接。
"""

from __future__ import annotations

import copy
import sys
import time

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

STEP_M = 0.05
TIMEOUT_SEC = 10.0
POLL_SEC = 0.2
POSE_THRESHOLD = 0.02
ORIENT_THRESHOLD = 1.0


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
    print(
        f"  {label}: pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
        f"ori=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
    )


def wait_body_pose_arrive(interface: ROS2RobotInterface, label: str) -> bool:
    start = time.time()
    last = None
    while time.time() - start < TIMEOUT_SEC:
        last = interface.check_arrive(
            part="body_pose",
            arm_pose_threshold=POSE_THRESHOLD,
            arm_orient_threshold=ORIENT_THRESHOLD,
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
    print("check_arrive(part='body_pose') check")
    print("=" * 70)

    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)
    interface.connect()
    time.sleep(1.0)

    try:
        pose_topic = interface.config.body_current_pose_topic
        target_topic = interface.config.body_current_target_pose_topic
        print(f"body_current_pose_topic: {pose_topic}")
        print(f"body_current_target_pose_topic: {target_topic}")
        if pose_topic is None or target_topic is None:
            print("skip: body pose topics not detected")
            return 0

        home_body = interface.get_body_current_pose()
        if home_body is None:
            print("skip: /body_current_pose 未收到消息")
            return 0

        if interface.left_arm_handler is None or interface.right_arm_handler is None:
            print("skip: dual-arm handlers not available")
            return 0

        print("-" * 70)
        print("switch FSM: HOLD → HOME → OCS2")
        interface.send_fsm_command(2)  # HOLD
        interface.send_fsm_command(1)  # HOME
        time.sleep(6.0)
        interface.send_fsm_command(2)  # HOLD
        interface.send_fsm_command(3)  # OCS2
        time.sleep(1.0)

        home_body = interface.get_body_current_pose()
        left_pose = interface.left_arm_handler.get_pose()
        right_pose = interface.right_arm_handler.get_pose()
        if home_body is None or left_pose is None or right_pose is None:
            print("failed: cannot get body/left/right current poses")
            return 1

        print_pose(home_body, "home_body")
        print_pose(left_pose, "left_hold")
        print_pose(right_pose, "right_hold")

        frame_id = interface.left_arm_handler.get_frame_id() or "arm_base"
        target_body = offset_pose(home_body, STEP_M)

        print("-" * 70)
        print_pose(target_body, "target_body(+z)")
        print("send_dual_arm_target_stamped(body_pose=perturbed)")
        interface.send_dual_arm_target_stamped(
            left_pose=left_pose,
            right_pose=right_pose,
            frame_id=frame_id,
            body_pose=target_body,
            body_frame_id="base_footprint",
        )
        if not wait_body_pose_arrive(interface, "perturb"):
            return 1

        print("-" * 70)
        print_pose(home_body, "target_body(home)")
        print("send_dual_arm_target_stamped(body_pose=restore)")
        interface.send_dual_arm_target_stamped(
            left_pose=left_pose,
            right_pose=right_pose,
            frame_id=frame_id,
            body_pose=home_body,
            body_frame_id="base_footprint",
        )
        if not wait_body_pose_arrive(interface, "restore"):
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
