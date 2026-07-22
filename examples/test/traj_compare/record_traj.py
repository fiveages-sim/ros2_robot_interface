#!/usr/bin/env python3
"""
规划轨迹与实际末端轨迹录制脚本（框架版）
==========================================

配合控制器内的 TrajectoryRecorder 使用：通过 ROS 参数控制录制的开始/结束，
录制窗口内执行要测试的运动（MoveL / MoveC / MoveJ 等），控制器会在窗口结束时
把 cal（规划）与 real（实际 FK）轨迹落盘为 CSV，供 compare_pose_traj.py 对比。

录制流程
--------
1. 连接 ROS2RobotInterface。
2. 设置控制器参数 traj_record_dir = <保存目录>。
3. 设置 traj_record_enabled = True（false->true 会清空缓存并开始录制）。
4. 执行要测试的运动（见 run_test_motion，当前为占位，待填充）。
5. 设置 traj_record_enabled = False（true->false 触发控制器同步落盘）。
6. 提示产物路径。

产物（由控制器写入 traj_record_dir）
------------------------------------
    cal_left.csv / cal_right.csv    规划轨迹（MoveL/MoveC 才有；纯 MoveJ 不产生 cal）
    real_left.csv / real_right.csv  实际末端轨迹（录制窗口内始终记录）

注意
----
- 控制器参数在控制器节点上（默认 /ocs2_arm_controller），本脚本用
  interface.set_node_parameters(CONTROLLER_NODE, {...}) 远程设置。
- 必须先设 dir 再设 enabled=True，否则用控制器默认目录。
- 运动 action 同步返回后再关闭录制，保证窗口恰好包住运动。
- 控制器与本脚本需在同一台机器（CSV 落在控制器端的 traj_record_dir）。
"""

import os
import sys
import time
from datetime import datetime

import rclpy

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


# ============================ 手动配置区 ============================
# 控制器节点名由连接后的 interface.is_wbc 自动推导（见 resolve_controller_node）：
#   分体控制 -> /ocs2_arm_controller
#   全身控制 -> /ocs2_wbc_controller
# 两者只会同时存在一个。如需强制指定，可将下面改为具体字符串（非 None 时优先）。
CONTROLLER_NODE_OVERRIDE = None

# 录制产物根目录：脚本同目录下的 record_data/
# 每次运行会在其下新建一个带时间戳的子目录（见 make_run_dir），避免覆盖历史数据。
# 注意：CSV 由控制器进程按此路径写盘，故用绝对路径（同机下即为此目录）。
RECORD_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "record_data")
# ==================================================================


def make_run_dir(root: str) -> str:
    """为本次运行生成带时间戳的子目录（不创建，仅返回路径；控制器落盘时会自动建）。

    形如 <root>/20260722_174130；同秒重复运行时追加 _2、_3 后缀避免冲突。
    """
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(root, stamp)
    suffix = 2
    while os.path.exists(run_dir):
        run_dir = os.path.join(root, f"{stamp}_{suffix}")
        suffix += 1
    return run_dir




def resolve_controller_node(interface: ROS2RobotInterface) -> str:
    """按 interface.is_wbc 推导承载录制参数的控制器节点名。

    复用 ros_interface 在 connect() 时的自动检测：ROS 图中存在
    /ocs2_wbc_controller/target_joint_position 时 is_wbc=True（全身控制），
    否则为分体控制（ocs2_arm_controller）。
    """
    if CONTROLLER_NODE_OVERRIDE:
        return CONTROLLER_NODE_OVERRIDE
    return "/ocs2_wbc_controller" if interface.is_wbc else "/ocs2_arm_controller"


def set_record_enabled(interface: ROS2RobotInterface, node: str, enabled: bool) -> bool:
    """设置控制器的 traj_record_enabled 参数。"""
    ok = interface.set_node_parameters(node, {"traj_record_enabled": bool(enabled)})
    print(f"    set traj_record_enabled={enabled} -> {'OK' if ok else 'FAILED'}")
    return ok


def set_record_dir(interface: ROS2RobotInterface, node: str, out_dir: str) -> bool:
    """设置控制器的 traj_record_dir 参数。"""
    ok = interface.set_node_parameters(node, {"traj_record_dir": str(out_dir)})
    print(f"    set traj_record_dir={out_dir} -> {'OK' if ok else 'FAILED'}")
    return ok


# ---------------------- 测试运动参数（按需修改） ----------------------
MOTION_FRAME = "arm_base"      # 目标位姿所在坐标系
MOTION_DZ = -0.10           # 末端 Z 方向偏移（米）
ARRIVE_TIMEOUT = 15.0       # 等待到位超时（秒）
ARRIVE_INTERVAL = 0.3       # 到位检查间隔（秒）
POSE_THRESHOLD = 0.02       # 到位位置阈值（米）
ORIENT_THRESHOLD = 3.0      # 到位姿态阈值（度）
# ---------------------------------------------------------------------


def _offset_pose(src, dz: float):
    """基于当前位姿，仅在 Z 方向施加偏移，姿态保持不变。"""
    from geometry_msgs.msg import Pose, Point, Quaternion
    return Pose(
        position=Point(x=src.position.x, y=src.position.y, z=src.position.z + dz),
        orientation=Quaternion(
            x=src.orientation.x, y=src.orientation.y,
            z=src.orientation.z, w=src.orientation.w,
        ),
    )


def _wait_arrival(handler, label: str) -> bool:
    """轮询等待单臂到位。"""
    start = time.time()
    while time.time() - start < ARRIVE_TIMEOUT:
        result = handler.check_arrival(
            pose_threshold=POSE_THRESHOLD, orient_threshold=ORIENT_THRESHOLD
        )
        if result.get("arrived"):
            print(f"    {label} 已到位（耗时 {time.time() - start:.1f}s）")
            return True
        time.sleep(ARRIVE_INTERVAL)
    print(f"    {label} 等待到位超时（{ARRIVE_TIMEOUT:.0f}s）")
    return False


def run_test_motion(interface: ROS2RobotInterface) -> bool:
    """测试运动：左、右臂各发一次 send_target_stamped（走 MPC 链路，产生 cal）。

    读取当前末端位姿，令其沿 Z 偏移 MOTION_DZ 作为目标，分别发左臂、右臂并等待到位。
    单臂机器人只发左臂。
    """
    is_dual = interface.config.right_end_effector_pose_topic is not None

    # 左臂
    left_pose = interface.left_arm_handler.get_pose()
    if left_pose is None:
        print("    无法获取左臂当前位姿（检查 pose 话题）")
        return False
    left_target = _offset_pose(left_pose, MOTION_DZ)
    print(f"    发送左臂目标: z {left_pose.position.z:.3f} -> {left_target.position.z:.3f}")
    interface.left_arm_handler.send_target_stamped(MOTION_FRAME, left_target)
    ok_left = _wait_arrival(interface.left_arm_handler, "左臂")

    ok_right = True
    if is_dual:
        right_pose = interface.right_arm_handler.get_pose()
        if right_pose is None:
            print("    无法获取右臂当前位姿（检查 pose 话题）")
            return False
        right_target = _offset_pose(right_pose, MOTION_DZ)
        print(f"    发送右臂目标: z {right_pose.position.z:.3f} -> {right_target.position.z:.3f}")
        interface.right_arm_handler.send_target_stamped(MOTION_FRAME, right_target)
        ok_right = _wait_arrival(interface.right_arm_handler, "右臂")
    else:
        print("    单臂模式，跳过右臂")

    return ok_left and ok_right


def main() -> int:
    run_dir = make_run_dir(RECORD_ROOT)
    print("=" * 70)
    print("规划/实际轨迹录制（框架版）")
    print(f"  本次保存目录: {run_dir}")
    print("=" * 70)

    rclpy.init()
    interface = None
    record_started = False
    controller_node = None
    try:
        print("\n[1] 创建并连接 ROS2RobotInterface...")
        interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
        interface.connect()
        controller_node = resolve_controller_node(interface)
        print(f"    连接成功；控制器节点: {controller_node} "
              f"({'全身控制 WBC' if interface.is_wbc else '分体控制'})")

        print("\n[2] 设置录制目录...")
        if not set_record_dir(interface, controller_node, run_dir):
            print("    错误: 无法设置 traj_record_dir")
            return 1

        print("\n[3] 开始录制...")
        if not set_record_enabled(interface, controller_node, True):
            print("    错误: 无法开启录制")
            return 1
        record_started = True
        # 给控制器一点时间应用参数、并采到运动前的静止基线
        time.sleep(0.5)

        print("\n[4] 执行测试运动...")
        motion_ok = run_test_motion(interface)
        if not motion_ok:
            print("    警告: 测试运动未成功完成，仍会停止并落盘已录数据")

        print("\n[5] 停止录制并落盘...")
        set_record_enabled(interface, controller_node, False)
        record_started = False
        # 落盘为同步操作，设置返回后产物应已写出
        time.sleep(0.2)

        print("\n完成。产物位于本次运行目录（控制器端）:")
        print(f"    {run_dir}/cal_left.csv   ")
        print(f"    {run_dir}/cal_right.csv  ")
        print(f"    {run_dir}/real_left.csv")
        print(f"    {run_dir}/real_right.csv ")
        return 0 if motion_ok else 1

    except Exception as exc:  # noqa: BLE001
        print(f"执行失败: {exc}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # 异常路径下确保关闭录制，避免录制一直开着
        if interface is not None and record_started and controller_node is not None:
            try:
                set_record_enabled(interface, controller_node, False)
            except Exception:
                pass
        if interface is not None:
            try:
                interface.disconnect()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
