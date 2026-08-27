"""使用右臂 MoveL Action 以两组速度约束逐段执行笛卡尔路径并分别录制。

输入点格式为 [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]，坐标系为
arm_base。运行前需要启动双臂分体控制器。

每轮都会在录制外切到 OCS2，用 5 秒单点 ExecutePath 回到第 0 个基准点，再
切到 MOVEJ。开启录制后立即从第 1 个路径点开始逐段调用阻塞式 MoveL Action。
连续重复点会被跳过，避免提交零长度 MoveL。每轮应分别生成 cal_right.csv 和
real_right.csv。

运行：
    conda run -n fa-ros2 python examples/test-demo/check_movel_right_path_speed_mode.py
"""

import math
import time
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import PoseStamped

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.quat_pose import euler_rpy_to_quat_xyzw


FRAME_ID = "arm_base"
FSM_OCS2 = 3
FSM_MOVEJ = 4
BASELINE_MOVE_DURATION = 5.0
ACTION_SERVER_TIMEOUT = 5.0
MOVEL_TIMEOUT = 30.0

# MoveL 速度约束模式参数。角加速度和角加加速度不下发，使用控制器默认值。
SPEED_PROFILES = (
    (
        "linear_0.2",
        {
            "max_linear_velocity": 0.2,
            "max_linear_acceleration": 0.5,
            "max_linear_jerk": 4.0,
            "max_angular_velocity": 0.5,
        },
    ),
    (
        "linear_0.05",
        {
            "max_linear_velocity": 0.05,
            "max_linear_acceleration": 0.5,
            "max_linear_jerk": 2.0,
            "max_angular_velocity": 0.5,
        },
    ),
)

RECORD_ROOT = (
    Path(__file__).resolve().parents[1]
    / "test"
    / "cart_trajectory_compare"
    / "record_data"
)

PATH_LIST_MM_DEG = [
    [350, -500, -367, 171, -88, 7],  # 0 基准点（运行前应已到达）
    [350, -330, -367, 171, -88, 7],  # 1
    [360, -330, -367, 171, -88, 7],  # 2
    [382, -352, -367, 171, -88, 7],  # 3
    [414, -352, -367, 171, -88, 7],  # 4
    [444, -330, -367, 171, -88, 7],  # 5
    [444, -354, -367, 171, -88, 7],  # 6
    [454, -354, -367, 171, -88, 7],  # 7
    [454, -398, -367, 171, -88, 7],  # 8
    [444, -398, -367, 171, -88, 7],  # 9
    [444, -428, -367, 171, -88, 7],  # 10
    [454, -428, -367, 171, -88, 7],  # 11
    [454, -472, -367, 171, -88, 7],  # 12
    [444, -472, -367, 171, -88, 7],  # 13
    [444, -496, -367, 171, -88, 7],  # 14
    [414, -474, -367, 171, -88, 7],  # 15
    [382, -474, -367, 171, -88, 7],  # 16
    [360, -496, -367, 171, -88, 7],  # 17
    [350, -496, -367, 171, -88, 7],  # 18 终点
    [350, -496, -367, 171, -88, 7],  # 19 重复终点（执行时跳过）
]


def to_pose_stamped(point: list[float]) -> PoseStamped:
    """将毫米/角度表示的 XYZ-RPY 转为米/四元数表示的 PoseStamped。"""
    x, y, z, roll, pitch, yaw = point
    qx, qy, qz, qw = euler_rpy_to_quat_xyzw(
        math.radians(roll), math.radians(pitch), math.radians(yaw)
    )
    result = PoseStamped()
    result.header.frame_id = FRAME_ID
    result.pose.position.x = x / 1000.0
    result.pose.position.y = y / 1000.0
    result.pose.position.z = z / 1000.0
    result.pose.orientation.x = qx
    result.pose.orientation.y = qy
    result.pose.orientation.z = qz
    result.pose.orientation.w = qw
    return result


def build_movel_targets() -> list[PoseStamped]:
    """忽略第 0 个基准点，并过滤后续连续重复点。"""
    unique_points: list[list[float]] = []
    previous = PATH_LIST_MM_DEG[0]
    for point in PATH_LIST_MM_DEG[1:]:
        if point != previous:
            unique_points.append(point)
        previous = point
    return [to_pose_stamped(point) for point in unique_points]


def make_session_dir() -> Path:
    """创建本次 MoveL 录制会话目录，并避免覆盖历史数据。"""
    RECORD_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = RECORD_ROOT / f"right_movel_path_speed_{stamp}"
    suffix = 2
    while session_dir.exists():
        session_dir = RECORD_ROOT / f"right_movel_path_speed_{stamp}_{suffix}"
        suffix += 1
    session_dir.mkdir()
    return session_dir


def set_record_dir(
    interface: ROS2RobotInterface, controller_node: str, record_dir: Path
) -> bool:
    ok = interface.set_node_parameters(
        controller_node, {"traj_record_dir": str(record_dir)}
    )
    print(f"  设置录制目录 -> {'成功' if ok else '失败'}: {record_dir}")
    return ok


def set_record_enabled(
    interface: ROS2RobotInterface, controller_node: str, enabled: bool
) -> bool:
    ok = interface.set_node_parameters(
        controller_node, {"traj_record_enabled": enabled}
    )
    print(f"  traj_record_enabled={enabled} -> {'成功' if ok else '失败'}")
    return ok


def has_record_parameters(
    interface: ROS2RobotInterface, controller_node: str
) -> bool:
    """确认控制器节点及轨迹录制参数在用户确认运动前可用。"""
    parameters = interface.list_node_parameters(controller_node)
    parameter_names = {str(parameter.get("name", "")) for parameter in parameters}
    required_names = {"traj_record_dir", "traj_record_enabled"}
    missing_names = sorted(required_names - parameter_names)
    if missing_names:
        print(f"错误：控制器缺少录制参数: {', '.join(missing_names)}")
        return False
    return True


def prepare_movej(interface: ROS2RobotInterface) -> bool:
    """在开启录制前切换并确认 MOVEJ，避免 Action 内部切换产生空录制。"""
    print("切换到 MOVEJ...")
    interface.auto_switch_fsm_for_control("arm_joint")
    if interface.get_fsm_state() != FSM_MOVEJ:
        print(
            "错误：FSM 未进入 MOVEJ，"
            f"当前状态={interface.get_fsm_state()}，期望={FSM_MOVEJ}"
        )
        return False
    return True


def execute_movel_path(
    interface: ROS2RobotInterface,
    targets: list[PoseStamped],
    speed_parameters: dict[str, float],
) -> bool:
    """以阻塞式右臂 MoveL Action 依次执行所有目标点。"""
    target_count = len(targets)
    for index, target in enumerate(targets, start=1):
        print(f"  [{index}/{target_count}] 下发右臂 MoveL Action...")
        result = interface.execute_movel_action(
            "right",
            target,
            time_mode=False,
            frame_id=FRAME_ID,
            **speed_parameters,
            auto_switch_fsm=False,
            timeout=MOVEL_TIMEOUT,
            wait_for_server_timeout=ACTION_SERVER_TIMEOUT,
        )
        if result is None:
            print(f"错误：第 {index} 段 MoveL Action 未返回结果")
            return False
        if not bool(getattr(result, "success", False)):
            message = getattr(result, "message", "")
            print(f"错误：第 {index} 段 MoveL Action 失败: {message}")
            return False
    return True


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            print("\n[cleanup] 切换到 HOLD")
            interface.send_fsm_command(2)
            time.sleep(0.5)
    except Exception as exc:
        print(f"[cleanup] HOLD 失败: {exc}")
    finally:
        interface.disconnect()


def main() -> int:
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    controller_node = ""
    record_started = False
    try:
        print("连接 ROS 2 接口...")
        interface.connect()
        time.sleep(1.0)

        if interface.is_wbc:
            print("错误：检测到 WBC；本测试要求双臂分体控制模式")
            return 1
        if interface.right_arm_handler is None or interface.execute_path_client is None:
            print("错误：未检测到右臂或 ExecutePath 接口")
            return 1
        if not interface.execute_path_client.service_is_ready():
            print("错误：ExecutePath 服务不可用")
            return 1
        if not interface.wait_for_movel_action_server(timeout=ACTION_SERVER_TIMEOUT):
            print(f"错误：MoveL Action 不可用: {interface.config.movel_action_name}")
            return 1
        controller_node = interface.arm_controller
        if not controller_node:
            print("错误：无法确定 OCS2 手臂控制器节点")
            return 1
        if not has_record_parameters(interface, controller_node):
            return 1

        targets = build_movel_targets()
        skipped_count = len(PATH_LIST_MM_DEG) - 1 - len(targets)
        print(f"基准点: PATH_LIST_MM_DEG[0]（每轮录制前自动回到此点）")
        print(f"MoveL 目标数: {len(targets)}，跳过连续重复点: {skipped_count}")
        print(f"frame: {FRAME_ID}，time_mode=False")
        for label, parameters in SPEED_PROFILES:
            print(f"{label}: {parameters}")
        print(f"录制控制器: {controller_node}")
        if input("将驱动右臂运动，输入 EXECUTE 继续: ").strip() != "EXECUTE":
            print("已取消")
            return 0

        # 建立明确的 False -> True 边沿，确保开启时清空旧缓存。
        if not set_record_enabled(interface, controller_node, False):
            print("错误：无法初始化轨迹录制状态")
            return 1

        session_dir = make_session_dir()
        print(f"\n本次录制会话目录: {session_dir}")

        baseline = to_pose_stamped(PATH_LIST_MM_DEG[0])
        for index, (label, speed_parameters) in enumerate(SPEED_PROFILES, start=1):
            step_dir = session_dir / f"{index:02d}_{label}"
            step_dir.mkdir()
            print(f"\n[{index}/{len(SPEED_PROFILES)}] {label}，目录={step_dir.name}")

            interface.auto_switch_fsm_for_control("arm_pose")
            if interface.get_fsm_state() != FSM_OCS2:
                print(f"错误：FSM 未进入 OCS2，当前状态={interface.get_fsm_state()}")
                return 1
            print(f"  录制外回到第 0 个基准点，duration={BASELINE_MOVE_DURATION:.1f}s...")
            if not interface.execute_right_path(
                [baseline], BASELINE_MOVE_DURATION, FRAME_ID
            ):
                print("错误：移动到基准点的 ExecutePath 服务返回失败")
                return 1
            time.sleep(BASELINE_MOVE_DURATION)
            if not prepare_movej(interface):
                return 1

            if not set_record_dir(interface, controller_node, step_dir):
                return 1
            print("  开启录制后立即开始逐段 MoveL 规划与执行...")
            # 参数服务超时时，控制器仍可能已经收到 True；提前标记以便 finally 关闭。
            record_started = True
            if not set_record_enabled(interface, controller_node, True):
                return 1

            motion_ok = False
            stop_ok = False
            try:
                motion_ok = execute_movel_path(interface, targets, speed_parameters)
            finally:
                if not motion_ok:
                    try:
                        print("  运动未完成，先切换到 HOLD...")
                        interface.send_fsm_command(2)
                    except Exception as exc:
                        print(f"  警告：运动失败后切换 HOLD 失败: {exc}")
                stop_ok = set_record_enabled(interface, controller_node, False)
                if stop_ok:
                    record_started = False
                time.sleep(0.2)

            if not motion_ok:
                return 1
            if not stop_ok:
                print("错误：无法停止录制并写入 CSV")
                return 1

            expected_files = (
                step_dir / "cal_right.csv",
                step_dir / "real_right.csv",
            )
            missing_files = [file.name for file in expected_files if not file.is_file()]
            if missing_files:
                print(f"错误：缺少录制文件: {', '.join(missing_files)}")
                return 1
            print("  已保存 cal_right.csv 和 real_right.csv")

        print(f"\n两轮 MoveL 轨迹录制完成: {session_dir}")
        return 0
    finally:
        if record_started and controller_node:
            try:
                set_record_enabled(interface, controller_node, False)
            except Exception as exc:
                print(f"[cleanup] 关闭轨迹录制失败: {exc}")
        cleanup(interface)


if __name__ == "__main__":
    raise SystemExit(main())
