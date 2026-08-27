"""录制右臂笛卡尔路径在 25 秒下的规划与实际轨迹。

输入点格式为 [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]，坐标系为
arm_base。该脚本会驱动真机，运行前需要启动双臂分体 OCS2 控制器。

每段录制前先确保处于 OCS2，并用 5 秒单点路径移动到第 0 个基准点；随后开启
录制并立即调用 execute_right_path。所有运动仅等待对应的 trajectory_duration，
不进行到位判断。CSV 写入 cart_trajectory_compare/record_data。

运行：
    conda run -n fa-ros2 python examples/test-demo/check_execute_right_path_three_durations.py
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
BASELINE_MOVE_DURATION = 5.0
TRAJECTORY_DURATION = 25.0
RECORD_ROOT = (
    Path(__file__).resolve().parents[1]
    / "test"
    / "cart_trajectory_compare"
    / "record_data"
)

PATH_LIST_MM_DEG = [
    [350, -500, -367, 171, -88, 7],  # 0 基准点
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
    [350, -496, -367, 171, -88, 7],  # 18
    [350, -496, -367, 171, -88, 7],  # 19 终点
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


def make_session_dir() -> Path:
    """创建可辨识且不覆盖历史数据的本次录制会话目录。"""
    RECORD_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = RECORD_ROOT / f"right_execute_path_{stamp}"
    suffix = 2
    while session_dir.exists():
        session_dir = RECORD_ROOT / f"right_execute_path_{stamp}_{suffix}"
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
            print("错误：未检测到双臂 ExecutePath 接口")
            return 1
        if not interface.execute_path_client.service_is_ready():
            print("错误：ExecutePath 服务不可用")
            return 1
        controller_node = interface.arm_controller
        if not controller_node:
            print("错误：无法确定 OCS2 手臂控制器节点")
            return 1

        path = [to_pose_stamped(point) for point in PATH_LIST_MM_DEG]
        print(f"路径点数: {len(path)}，frame: {FRAME_ID}")
        print(f"路径执行时长: {TRAJECTORY_DURATION:.1f} 秒")
        print(f"每轮录制前回基准点: {BASELINE_MOVE_DURATION:.1f} 秒")
        print(f"录制控制器: {controller_node}")
        if input("将驱动右臂运动，输入 EXECUTE 继续: ").strip() != "EXECUTE":
            print("已取消")
            return 0

        # 确保首次开启录制时存在明确的 False -> True 边沿，从而清空旧缓存。
        if not set_record_enabled(interface, controller_node, False):
            print("错误：无法初始化轨迹录制状态")
            return 1

        session_dir = make_session_dir()
        print(f"\n本次录制会话目录: {session_dir}")

        step_dir = session_dir / f"01_right_path_{TRAJECTORY_DURATION:g}s"
        step_dir.mkdir()
        print(f"\nduration={TRAJECTORY_DURATION:.1f}s，目录={step_dir.name}")
        interface.auto_switch_fsm_for_control("arm_pose")
        if interface.get_fsm_state() != FSM_OCS2:
            print(f"错误：FSM 未进入 OCS2，当前状态={interface.get_fsm_state()}")
            return 1
        print(f"  移动到第 0 个基准点，duration={BASELINE_MOVE_DURATION:.1f}s...")
        if not interface.execute_right_path(
            [path[0]], BASELINE_MOVE_DURATION, FRAME_ID
        ):
            print("错误：移动到基准点的 ExecutePath 服务返回失败")
            return 1
        time.sleep(BASELINE_MOVE_DURATION)

        if not set_record_dir(interface, controller_node, step_dir):
            return 1
        # 参数服务超时时，控制器仍可能已经收到 True；提前标记以便 finally 关闭。
        record_started = True
        if not set_record_enabled(interface, controller_node, True):
            return 1

        motion_ok = False
        stop_ok = False
        try:
            print("  下发 execute_right_path...")
            motion_ok = interface.execute_right_path(
                path, TRAJECTORY_DURATION, FRAME_ID
            )
            if motion_ok:
                time.sleep(TRAJECTORY_DURATION)
        finally:
            stop_ok = set_record_enabled(interface, controller_node, False)
            if stop_ok:
                record_started = False
            time.sleep(0.2)

        if not motion_ok:
            print("错误：ExecutePath 服务返回失败")
            return 1
        if not stop_ok:
            print("错误：无法停止录制并写入 CSV")
            return 1

        expected_files = (step_dir / "pred_right.csv", step_dir / "real_right.csv")
        missing_files = [file.name for file in expected_files if not file.is_file()]
        if missing_files:
            print(f"错误：缺少录制文件: {', '.join(missing_files)}")
            return 1
        print("  已保存 pred_right.csv 和 real_right.csv")

        print(f"\n轨迹录制完成: {session_dir}")
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
