#!/usr/bin/env python3
"""从 HOME 起点使用 MoveL Action 将左臂沿基坐标系 +Z 移动 30 cm。

HOME 稳定后会自动以 ``--plot`` 启动 ``record_joint_interfaces.py``，运动结束、
异常或中断时自动停止录制，并在 ``record_data/movel_action_*`` 中生成 CSV 与图表。

运行（从 fa-py-libraries 根目录）：
    .venv/bin/python ros2_robot_interface/examples/test/16_joint_trajectory_compare/move_left_up_movel_action.py

安全说明：
    机器人会先回 HOME，然后执行 30 cm 直线运动。运行前请确认左臂上方空间充足。
"""

from __future__ import annotations

import copy
import math
import sys
import time

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

from _recording_process import RecorderProcess, finish_recorder, start_recorder


Z_OFFSET_M = 0.30
HOME_WAIT_SEC = 6.0
DATA_WAIT_SEC = 2.0
MOVE_DURATION_SEC = 5.0  # MoveL Action 请求运动时长
ACTION_SERVER_TIMEOUT_SEC = 10.0
ACTION_RESULT_TIMEOUT_SEC = 20.0
FSM_WAIT_SEC = 3.0
HOME_STABLE_TIMEOUT_SEC = 10.0
ARRIVAL_TIMEOUT_SEC = 5.0
POLL_SEC = 0.2
STABLE_SAMPLE_COUNT = 5
HOME_STABLE_POSITION_M = 0.002
HOME_STABLE_ORIENTATION_DEG = 1.0
POSITION_THRESHOLD_M = 0.02
ORIENTATION_THRESHOLD_DEG = 3.0
COUNTDOWN_SEC = 3
HOLD_RETRY_SEC = 0.2


def print_pose(pose: Pose, label: str) -> None:
    p = pose.position
    q = pose.orientation
    print(
        f"  {label}: pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
        f"ori=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
    )


def build_up_target(home_pose: Pose) -> Pose:
    target = copy.deepcopy(home_pose)
    target.position.z += Z_OFFSET_M
    return target


def pose_error(current: Pose, target: Pose) -> tuple[float, float]:
    dx = current.position.x - target.position.x
    dy = current.position.y - target.position.y
    dz = current.position.z - target.position.z
    position_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    cq = current.orientation
    tq = target.orientation
    current_norm = math.sqrt(cq.x * cq.x + cq.y * cq.y + cq.z * cq.z + cq.w * cq.w)
    target_norm = math.sqrt(tq.x * tq.x + tq.y * tq.y + tq.z * tq.z + tq.w * tq.w)
    if current_norm == 0.0 or target_norm == 0.0:
        return position_distance, float("inf")
    dot = abs(
        (cq.x * tq.x + cq.y * tq.y + cq.z * tq.z + cq.w * tq.w)
        / (current_norm * target_norm)
    )
    orientation_angle = 2.0 * math.degrees(math.acos(min(1.0, dot)))
    return position_distance, orientation_angle


def wait_for_fsm_state(interface: ROS2RobotInterface, expected: int) -> None:
    deadline = time.monotonic() + FSM_WAIT_SEC
    while time.monotonic() < deadline:
        if interface.get_fsm_state() == expected:
            return
        time.sleep(POLL_SEC)
    raise RuntimeError(
        f"FSM 未进入期望状态 {expected}，当前状态: {interface.get_fsm_state()}"
    )


def wait_for_stable_home_pose(handler) -> Pose:
    """等待 HOME 后的新鲜位姿连续稳定，避免从运动中或陈旧位姿构造目标。"""
    time.sleep(HOME_WAIT_SEC)
    previous_object = handler.get_pose()
    previous_pose = copy.deepcopy(previous_object) if previous_object is not None else None
    stable_samples = 0
    deadline = time.monotonic() + HOME_STABLE_TIMEOUT_SEC

    while time.monotonic() < deadline:
        current = handler.get_pose()
        if current is None or current is previous_object:
            time.sleep(POLL_SEC)
            continue

        if previous_pose is not None:
            position_delta, orientation_delta = pose_error(current, previous_pose)
            if (
                position_delta <= HOME_STABLE_POSITION_M
                and orientation_delta <= HOME_STABLE_ORIENTATION_DEG
            ):
                stable_samples += 1
            else:
                stable_samples = 0

        previous_object = current
        previous_pose = copy.deepcopy(current)
        if stable_samples >= STABLE_SAMPLE_COUNT:
            return previous_pose
        time.sleep(POLL_SEC)

    raise RuntimeError("HOME 后未收到连续稳定的新鲜左臂位姿")


def wait_for_target_pose(handler, target_pose: Pose) -> Pose:
    deadline = time.monotonic() + ARRIVAL_TIMEOUT_SEC
    last_error = (float("inf"), float("inf"))
    while time.monotonic() < deadline:
        current = handler.get_pose()
        if current is not None:
            last_error = pose_error(current, target_pose)
            if (
                last_error[0] <= POSITION_THRESHOLD_M
                and last_error[1] <= ORIENTATION_THRESHOLD_DEG
            ):
                return copy.deepcopy(current)
        time.sleep(POLL_SEC)
    raise RuntimeError(
        "MoveL 实测末端未到位: "
        f"position_distance={last_error[0]:.4f} m, "
        f"orientation_angle={last_error[1]:.2f} deg"
    )


def countdown() -> None:
    print(f"左臂将在基坐标系 +Z 方向移动 {Z_OFFSET_M:.2f} m，请确认上方无障碍物。")
    for remaining in range(COUNTDOWN_SEC, 0, -1):
        print(f"  {remaining}...")
        time.sleep(1.0)


def ensure_hold(interface: ROS2RobotInterface) -> None:
    deadline = time.monotonic() + FSM_WAIT_SEC
    last_error: BaseException | None = None
    while time.monotonic() < deadline:
        try:
            interface.send_fsm_command(2)
        except BaseException as exc:
            last_error = exc
        if interface.get_fsm_state() == 2:
            return
        time.sleep(HOLD_RETRY_SEC)
    raise RuntimeError(
        f"FSM 未进入 HOLD，当前状态: {interface.get_fsm_state()}, last_error={last_error}"
    )


def cleanup(interface: ROS2RobotInterface) -> list[str]:
    failures: list[str] = []
    try:
        if interface.is_connected:
            print("\n[cleanup] 切换到 HOLD")
            ensure_hold(interface)
    except BaseException as exc:
        message = f"切换并确认 HOLD 失败: {exc}"
        failures.append(message)
        print(f"[cleanup] 警告: {message}")
    finally:
        if interface.is_connected:
            try:
                interface.disconnect()
                print("[cleanup] 已断开连接")
            except BaseException as exc:
                message = f"断开连接失败: {exc}"
                failures.append(message)
                print(f"[cleanup] 警告: {message}")
    return failures


def main() -> int:
    print("=" * 70)
    print("左臂 HOME → 基坐标系 +Z 30 cm（MoveL Action）")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    recorder: RecorderProcess | None = None

    try:
        interface.connect()
        print(f"[1] 等待位姿数据 {DATA_WAIT_SEC:.1f}s...")
        time.sleep(DATA_WAIT_SEC)

        handler = interface.left_arm_handler
        if handler is None:
            raise RuntimeError("left_arm_handler 不可用")
        if not interface.wait_for_movel_action_server(
            timeout=ACTION_SERVER_TIMEOUT_SEC
        ):
            raise RuntimeError(
                f"MoveL Action Server 不可用: {interface.config.movel_action_name}"
            )

        print("[2] 切换状态：HOLD → HOME")
        interface.send_fsm_command(2)
        interface.send_fsm_command(1)
        wait_for_fsm_state(interface, 1)
        print(f"    等待 HOME 运动完成并确认位姿稳定（至少 {HOME_WAIT_SEC:.1f}s）...")
        home_pose = wait_for_stable_home_pose(handler)

        frame_id = handler.get_frame_id()
        if not frame_id:
            raise RuntimeError("无法获取左臂位姿对应的基坐标系 frame_id")

        target_pose = build_up_target(home_pose)
        print(f"[3] 基坐标系: {frame_id}")
        print_pose(home_pose, "HOME")
        print_pose(target_pose, "目标(+Z 0.30m)")

        print("    启动关节接口录制（--plot）...")
        recorder = start_recorder("movel_action")

        # MoveL Action 内部执行 IK 关节轨迹，会自动从 HOLD 切换到 MOVEJ。
        ensure_hold(interface)
        countdown()
        print("[4] 调用 execute_movel_action()...")
        result = interface.execute_movel_action(
            "left",
            target_pose,
            duration=MOVE_DURATION_SEC,
            time_mode=True,
            frame_id=frame_id,
            timeout=ACTION_RESULT_TIMEOUT_SEC,
            wait_for_server_timeout=ACTION_SERVER_TIMEOUT_SEC,
        )
        if result is None or not getattr(result, "success", False):
            message = getattr(result, "message", "无返回结果")
            raise RuntimeError(f"MoveL Action 执行失败: {message}")

        print(f"[5] MoveL Action 返回成功: {getattr(result, 'message', '')}")
        actual_pose = wait_for_target_pose(handler, target_pose)
        print_pose(actual_pose, "执行后实际位姿")
        actual_dz = actual_pose.position.z - home_pose.position.z
        print(f"  相对 HOME 的实际 Z 位移: {actual_dz:.4f} m")
        return 0
    finally:
        active_exception = sys.exc_info()[0] is not None
        failures: list[str] = []
        try:
            if recorder is not None:
                recorder.request_stop()
        except BaseException as exc:
            failures.append(f"请求 recorder 停止失败: {exc}")
        failures.extend(cleanup(interface))
        try:
            finish_recorder(recorder)
        except KeyboardInterrupt:
            raise
        except BaseException as exc:
            failures.append(f"recorder/plot 清理失败: {exc}")
        if failures:
            print(f"[cleanup] 未完整成功: {'; '.join(failures)}")
            if not active_exception:
                raise RuntimeError("清理未完整成功: " + "; ".join(failures))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n用户中断")
        sys.exit(1)
    except Exception as exc:  # noqa: BLE001
        print(f"执行失败: {exc}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
