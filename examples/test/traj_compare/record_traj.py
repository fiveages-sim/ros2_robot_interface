#!/usr/bin/env python3
"""
规划轨迹与实际末端轨迹录制脚本（框架版）
==========================================

配合控制器内的 TrajectoryRecorder 使用：通过 ROS 参数控制录制的开始/结束，
录制窗口内执行要测试的运动（MoveL / MoveC / MoveJ 等），控制器会在窗口结束时
把 pred（MPC 预测）与 real（实际 FK）轨迹落盘为 CSV，供 compare_pose_traj.py 对比。

录制流程
--------
1. 连接 ROS2RobotInterface。
2. 设置控制器参数 traj_record_dir = <保存目录>。
3. 设置 traj_record_enabled = True（false->true 会清空缓存并开始录制）。
4. 执行要测试的运动（见 run_test_motion，切换注释选定测试项）。
5. 设置 traj_record_enabled = False（true->false 触发控制器同步落盘）。
6. 提示产物路径。

产物（由控制器写入 traj_record_dir）
------------------------------------
    pred_left.csv / pred_right.csv  MPC 预测末端轨迹（被预测时刻绝对时间戳）
    real_left.csv / real_right.csv  实际 FK 末端轨迹（录制窗口内始终记录）

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
ARRIVE_TIMEOUT = 15.0       # 等待到位超时（秒）
ARRIVE_INTERVAL = 0.3       # 到位检查间隔（秒）
POSE_THRESHOLD = 0.02       # 到位位置阈值（米）
ORIENT_THRESHOLD = 3.0      # 到位姿态阈值（度）
# ---------------------------------------------------------------------


def _offset_pose(src, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
    """基于当前位姿施加平移，姿态保持不变。"""
    from geometry_msgs.msg import Pose, Point, Quaternion
    return Pose(
        position=Point(x=src.position.x + dx, y=src.position.y + dy,
                       z=src.position.z + dz),
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


def test_send_target_stamped(interface: ROS2RobotInterface) -> bool:
    """带 frame 的单点目标：左右臂各发一次 send_target_stamped（走 MPC，产 pred）。

    不显式传 frame_id：send_target_stamped 会回退到 handler 从 pose 话题订阅到的
    frame（self.frame_id），与目标 pose 的来源坐标系一致；能取到 pose 即说明该
    frame 已就绪。
    """
    is_dual = interface.config.right_end_effector_pose_topic is not None
    left_pose = interface.left_arm_handler.get_pose()
    if left_pose is None:
        print("    无法获取左臂当前位姿（检查 pose 话题）")
        return False
    left_target = _offset_pose(left_pose, dx=-0.2, dy=0.0, dz=-0.10)
    print(f"    左臂 stamped: z {left_pose.position.z:.3f} -> {left_target.position.z:.3f}")
    interface.left_arm_handler.send_target_stamped(left_target)
    ok_left = _wait_arrival(interface.left_arm_handler, "左臂")
    if not is_dual:
        print("    单臂模式，跳过右臂")
        return ok_left
    right_pose = interface.right_arm_handler.get_pose()
    if right_pose is None:
        print("    无法获取右臂当前位姿（检查 pose 话题）")
        return False
    right_target = _offset_pose(right_pose, dx=-0.2, dy=0.0, dz=-0.10)
    print(f"    右臂 stamped: z {right_pose.position.z:.3f} -> {right_target.position.z:.3f}")
    interface.right_arm_handler.send_target_stamped(right_target)
    ok_right = _wait_arrival(interface.right_arm_handler, "右臂")
    return ok_left and ok_right


def test_send_target(interface: ROS2RobotInterface) -> bool:
    """无 frame 的单点目标：左右臂各发一次 send_target（Pose，基准坐标系解释）。"""
    is_dual = interface.config.right_end_effector_pose_topic is not None
    left_pose = interface.left_arm_handler.get_pose()
    if left_pose is None:
        print("    无法获取左臂当前位姿（检查 pose 话题）")
        return False
    left_target = _offset_pose(left_pose, dx=-0.2, dy=0.0, dz=-0.10)
    print(f"    左臂 target: z {left_pose.position.z:.3f} -> {left_target.position.z:.3f}")
    interface.left_arm_handler.send_target(left_target)
    ok_left = _wait_arrival(interface.left_arm_handler, "左臂")
    if not is_dual:
        print("    单臂模式，跳过右臂")
        return ok_left
    right_pose = interface.right_arm_handler.get_pose()
    if right_pose is None:
        print("    无法获取右臂当前位姿（检查 pose 话题）")
        return False
    right_target = _offset_pose(right_pose, dx=-0.2, dy=0.0, dz=-0.10)
    print(f"    右臂 target: z {right_pose.position.z:.3f} -> {right_target.position.z:.3f}")
    interface.right_arm_handler.send_target(right_target)
    ok_right = _wait_arrival(interface.right_arm_handler, "右臂")
    return ok_left and ok_right


def test_send_dual_arm_target_stamped(interface: ROS2RobotInterface) -> bool:
    """双臂同步 stamped：一次调用 send_dual_arm_target_stamped 下发左右臂目标。"""
    if interface.config.right_end_effector_target_topic is None:
        print("    需要双臂模式（未检测到右臂 target topic），跳过")
        return False
    left_pose = interface.left_arm_handler.get_pose()
    right_pose = interface.right_arm_handler.get_pose()
    if left_pose is None or right_pose is None:
        print("    无法获取左/右臂当前位姿（检查 pose 话题）")
        return False
    left_target = _offset_pose(left_pose, dx=-0.2, dy=0.0, dz=-0.10)
    right_target = _offset_pose(right_pose, dx=-0.2, dy=0.0, dz=-0.10)
    print(f"    双臂 stamped: 左 z->{left_target.position.z:.3f} 右 z->{right_target.position.z:.3f}")
    interface.send_dual_arm_target_stamped(left_target, right_target, frame_id="arm_base")
    ok_left = _wait_arrival(interface.left_arm_handler, "左臂")
    ok_right = _wait_arrival(interface.right_arm_handler, "右臂")
    return ok_left and ok_right


def test_send_dual_arm_body_target(interface: ROS2RobotInterface) -> bool:
    """双臂 + body：WBC 下 send_dual_arm_target_stamped 附带 body_pose。"""
    if interface.config.right_end_effector_target_topic is None:
        print("    需要双臂模式（未检测到右臂 target topic），跳过")
        return False
    if not interface.is_wbc:
        print("    需要 WBC 模式（interface.is_wbc=False），跳过")
        return False
    left_pose = interface.left_arm_handler.get_pose()
    right_pose = interface.right_arm_handler.get_pose()
    if left_pose is None or right_pose is None:
        print("    无法获取左/右臂当前位姿（检查 pose 话题）")
        return False
    left_target = _offset_pose(left_pose, dx=-0.2, dy=0.0, dz=-0.10)
    right_target = _offset_pose(right_pose, dx=-0.2, dy=0.0, dz=-0.10)
    body_target = _offset_pose(left_pose, dz=-0.03)  # body 目标：保守的小平移
    print("    双臂+body: body dz=-0.030")
    interface.send_dual_arm_target_stamped(
        left_target, right_target, frame_id="arm_base",
        body_pose=body_target, body_mode="BODY_TRACKING",
    )
    ok_left = _wait_arrival(interface.left_arm_handler, "左臂")
    ok_right = _wait_arrival(interface.right_arm_handler, "右臂")
    return ok_left and ok_right


# 注意：send_target_path()（发布 /target_path topic）为第一代双臂路径接口，
# 现已被第二代 ExecutePath service 接口取代（execute_path / execute_left_path /
# execute_right_path，支持 trajectory_duration、返回 success、左右可不等长、空侧保持
# 原参考）。因此不再为 send_target_path 单列测试项，路径类测试统一走下面的 execute_* 系列。


def test_execute_path(interface: ROS2RobotInterface) -> bool:
    """ExecutePath service：双臂各一条多路点路径。需双臂配置。"""
    if interface.config.right_end_effector_target_topic is None:
        print("    需要双臂模式（未检测到右臂 target topic），跳过")
        return False
    left_pose = interface.left_arm_handler.get_pose()
    right_pose = interface.right_arm_handler.get_pose()
    if left_pose is None or right_pose is None:
        print("    无法获取左/右臂当前位姿（检查 pose 话题）")
        return False
    left_poses = [
        _offset_pose(left_pose, dx=0.0, dy=0.0, dz=0.0),
        _offset_pose(left_pose, dx=0.03, dy=0.0, dz=-0.03),
        _offset_pose(left_pose, dx=-0.06, dy=0.0, dz=0.0),
    ]
    right_poses = [
        _offset_pose(right_pose, dx=0.0, dy=0.0, dz=0.0),
        _offset_pose(right_pose, dx=0.03, dy=0.0, dz=-0.03),
        _offset_pose(right_pose, dx=-0.06, dy=0.0, dz=0.0),
    ]
    print(f"    execute_path: 左 {len(left_poses)} 点 右 {len(right_poses)} 点")
    ok = interface.execute_path(left_poses, right_poses,
                                trajectory_duration=3.0, frame_id="arm_base")
    print(f"    execute_path service success={ok}")
    ok_left = _wait_arrival(interface.left_arm_handler, "左臂")
    ok_right = _wait_arrival(interface.right_arm_handler, "右臂")
    return ok and ok_left and ok_right


def test_execute_left_path(interface: ROS2RobotInterface) -> bool:
    """ExecutePath 左臂包装：只动左臂，右臂保持原参考。需双臂配置。"""
    if interface.config.right_end_effector_target_topic is None:
        print("    需要双臂模式（未检测到右臂 target topic），跳过")
        return False
    left_pose = interface.left_arm_handler.get_pose()
    if left_pose is None:
        print("    无法获取左臂当前位姿（检查 pose 话题）")
        return False
    left_poses = [
        _offset_pose(left_pose, dx=0.0, dy=0.0, dz=0.0),
        _offset_pose(left_pose, dx=0.03, dy=0.0, dz=-0.03),
        _offset_pose(left_pose, dx=-0.06, dy=0.0, dz=0.0),
    ]
    print(f"    execute_left_path: 左 {len(left_poses)} 点")
    ok = interface.execute_left_path(left_poses, trajectory_duration=3.0,
                                     frame_id="arm_base")
    print(f"    execute_left_path service success={ok}")
    return ok and _wait_arrival(interface.left_arm_handler, "左臂")


def test_execute_right_path(interface: ROS2RobotInterface) -> bool:
    """ExecutePath 右臂包装：只动右臂，左臂保持原参考。需双臂配置。"""
    if interface.config.right_end_effector_target_topic is None:
        print("    需要双臂模式（未检测到右臂 target topic），跳过")
        return False
    right_pose = interface.right_arm_handler.get_pose()
    if right_pose is None:
        print("    无法获取右臂当前位姿（检查 pose 话题）")
        return False
    right_poses = [
        _offset_pose(right_pose, dx=0.0, dy=0.0, dz=0.0),
        _offset_pose(right_pose, dx=0.03, dy=0.0, dz=-0.03),
        _offset_pose(right_pose, dx=-0.06, dy=0.0, dz=0.0),
    ]
    print(f"    execute_right_path: 右 {len(right_poses)} 点")
    ok = interface.execute_right_path(right_poses, trajectory_duration=3.0,
                                      frame_id="arm_base")
    print(f"    execute_right_path service success={ok}")
    return ok and _wait_arrival(interface.right_arm_handler, "右臂")


def run_test_motion(interface: ROS2RobotInterface) -> bool:
    """测试入口：取消目标函数的注释，只保留一个调用。

    每次运行只执行一个测试项，录制为一段 pred + real。
    """
    # return test_send_target_stamped(interface)
    # return test_send_target(interface)
    # return test_send_dual_arm_target_stamped(interface)
    # return test_send_dual_arm_body_target(interface)
    # return test_execute_left_path(interface)
    # return test_execute_right_path(interface)
    return test_execute_path(interface)


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

        # 录制已关闭后再回 HOME，回程轨迹不会被录入 CSV。
        print("\n[6] 回到 HOME 位置...")
        try:
            interface.send_fsm_command(1)  # 1: HOME（内部会自动先切 HOLD）
            print("    已发送 HOME 指令")
        except Exception as home_exc:  # noqa: BLE001
            print(f"    警告: 回 HOME 失败: {home_exc}")

        print("\n完成。产物位于本次运行目录（控制器端）:")
        print(f"    {run_dir}/pred_left.csv")
        print(f"    {run_dir}/pred_right.csv")
        print(f"    {run_dir}/real_left.csv")
        print(f"    {run_dir}/real_right.csv")
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
