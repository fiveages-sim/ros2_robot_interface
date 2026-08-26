"""W2 双臂抓箱并倾倒任务 OCS2 规划示例。

动作点位来自项目根目录的《倾倒任务.txt》。本示例只控制双臂和腰部，
不控制夹爪，也不会在结束时自动回 HOME。
第 1、6 步使用 MoveJ；第 2～5、7 步使用 OCS2 笛卡尔参考规划。

启用 ENABLE_TRAJ_RECORD 时，MPC 步骤（2～5、7）分段录制到
examples/test/cart_trajectory_compare/record_data/<会话时间戳>/step*/，
可由 compare_pose_traj.py 按会话→step 选择对比（pred↔real）。
"""

from __future__ import annotations

import math
import os
import sys
import time
from collections.abc import Iterator, Sequence
from contextlib import contextmanager, nullcontext
from datetime import datetime
from typing import Optional

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


FRAME_ID = "arm_base"
DATA_SETTLE_TIME = 2.0
ARRIVAL_TIMEOUT = 30.0
MOVEJ_DURATION = 5.0
MOVEL_DURATION = 5.0
POUR_TRAJECTORY_DURATION = 10.0

# 分段轨迹录制：产物落在 cart_trajectory_compare/record_data，便于对比脚本直接选取
ENABLE_TRAJ_RECORD = True
CONTROLLER_NODE_OVERRIDE: Optional[str] = None
RECORD_ROOT = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "..", "test", "cart_trajectory_compare", "record_data"
    )
)
LEFT_INITIAL_JOINTS = [
    2.72979,
    -1.1976,
    -2.56379,
    -2.396,
    -1.1947,
    -0.3511,
    -0.2927,
]
RIGHT_INITIAL_JOINTS = [
    -2.68059,
    -1.134,
    2.59399,
    -2.298,
    1.167,
    -0.3477,
    0.3972,
]
INITIAL_BODY_JOINTS = [-0.8639, -1.7582, -0.9, 0.0]
POUR_BODY_JOINTS = [0.0, 0.0, math.radians(18.95), 0.0]

APPROACH_LEFT = [
    0.707604, 0.412536, -0.104300, 0.508490, -0.030031, 0.860121, 0.026981
]
APPROACH_RIGHT = [
    0.688860, -0.327791, -0.124593, 0.547873, 0.006481, 0.836531, 0.003056
]
GRASP_LEFT = [
    0.707604, 0.372536, -0.104300, 0.508490, -0.030031, 0.860121, 0.026981
]
GRASP_RIGHT = [
    0.688860, -0.287791, -0.124593, 0.547873, 0.006481, 0.836531, 0.003056
]
LIFT_LEFT = [
    0.707604, 0.372536, -0.054300, 0.508490, -0.030031, 0.860121, 0.026981
]
LIFT_RIGHT = [
    0.688860, -0.287791, -0.074593, 0.547873, 0.006481, 0.836531, 0.003056
]
POUR_READY_LEFT = [
    0.6791592, 0.334456, 0.416901, 0.553139, 0.030244, 0.833914, 0.023025
]
POUR_READY_RIGHT = [
    0.679955, -0.334456, 0.4168409, 0.550124, 0.030224, 0.834296, -0.020011
]

POUR_LEFT_PATH = [
    [0.541105, 0.336896, -0.044200, 0.905963, 0.012358, 0.420665, 0.046046],
    [0.499752, 0.334456, -0.104795, 0.985850, 0.011287, 0.156928, 0.057849],
    POUR_READY_LEFT,
]
POUR_RIGHT_PATH = [
    [0.557352, -0.335782, -0.044258, 0.913185, 0.005028, 0.404357, -0.050627],
    [0.499823, -0.334456, -0.104719, 0.985842, -0.011260, 0.156970, -0.057863],
    POUR_READY_RIGHT,
]


def vector_to_pose(vector: Sequence[float]) -> Pose:
    """将 [x, y, z, qx, qy, qz, qw] 转换为 Pose。"""
    if len(vector) != 7:
        raise ValueError(f"Pose vector must contain 7 values, got {len(vector)}")

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, vector[:3])
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = map(float, vector[3:])
    return pose


def require_arrival(result: dict, step_name: str, part: str) -> None:
    """到位失败时终止任务，并保留接口返回的诊断信息。"""
    if not result.get("arrived", False):
        raise RuntimeError(
            f"{step_name}: {part} did not arrive within {ARRIVAL_TIMEOUT:.1f}s; "
            f"result={result.get('result', result)}"
        )


@contextmanager
def record_step(
    interface: ROS2RobotInterface,
    controller_node: str,
    session_dir: str,
    step_label: str,
) -> Iterator[str]:
    """开录 → 运动由调用方执行 → 退出时关录落盘。"""
    out_dir = os.path.join(session_dir, step_label)
    os.makedirs(out_dir, exist_ok=True)
    if not interface.set_node_parameters(controller_node, {"traj_record_dir": out_dir}):
        raise RuntimeError(f"无法设置 traj_record_dir={out_dir}")
    if not interface.set_node_parameters(
        controller_node, {"traj_record_enabled": True}
    ):
        raise RuntimeError(f"无法开启录制: {step_label}")
    print(f"    开始录制: {out_dir}")
    try:
        yield out_dir
    finally:
        interface.set_node_parameters(controller_node, {"traj_record_enabled": False})
        print(f"    已落盘: {out_dir}")


def execute_dual_pose(
    interface: ROS2RobotInterface,
    step_name: str,
    left_vector: Sequence[float],
    right_vector: Sequence[float],
    movel_duration: float,
) -> None:
    """发送双臂单点目标，等待请求时长后要求左右臂均到位。"""
    print(f"\n{step_name}")
    interface.send_dual_arm_target_stamped(
        vector_to_pose(left_vector),
        vector_to_pose(right_vector),
        frame_id=FRAME_ID,
        movel_duration=movel_duration,
    )
    time.sleep(movel_duration)
    require_arrival(
        interface.wait_until_arrive(part="left_arm", timeout=ARRIVAL_TIMEOUT),
        step_name,
        "left_arm",
    )
    require_arrival(
        interface.wait_until_arrive(part="right_arm", timeout=ARRIVAL_TIMEOUT),
        step_name,
        "right_arm",
    )


def run_task(interface: ROS2RobotInterface) -> None:
    """严格按《倾倒任务.txt》的顺序执行双臂和腰部动作。"""
    if interface.config.right_end_effector_target_topic is None:
        raise RuntimeError("This demo requires dual-arm mode")

    session_dir: Optional[str] = None
    controller_node: Optional[str] = None
    if ENABLE_TRAJ_RECORD:
        os.makedirs(RECORD_ROOT, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(RECORD_ROOT, stamp)
        suffix = 2
        while os.path.exists(session_dir):
            session_dir = os.path.join(RECORD_ROOT, f"{stamp}_{suffix}")
            suffix += 1
        os.makedirs(session_dir)
        controller_node = (
            CONTROLLER_NODE_OVERRIDE
            or getattr(interface, "arm_controller", None)
            or ("/ocs2_wbc_controller" if interface.is_wbc else "/ocs2_arm_controller")
        )
        print(f"轨迹录制会话目录: {session_dir}")
        print(f"控制器节点: {controller_node}")

    def step_ctx(step_label: str):
        if session_dir is None or controller_node is None:
            return nullcontext()
        return record_step(interface, controller_node, session_dir, step_label)

    # 固定等待：DATA_SETTLE_TIME=2.0s，仅用于等待状态数据，不是机器人运动时长。
    print(f"等待状态数据 {DATA_SETTLE_TIME:.1f}s...")
    time.sleep(DATA_SETTLE_TIME)

    # [1/7] MoveJ：不录制
    print("\n[1/7] 移动到初始关节位置")
    interface.send_coordinated_joint_positions(
        body_positions=INITIAL_BODY_JOINTS,
        left_arm_positions=LEFT_INITIAL_JOINTS,
        right_arm_positions=RIGHT_INITIAL_JOINTS,
    )
    time.sleep(MOVEJ_DURATION)
    initial_result = interface.wait_until_joint_arrive(
        left_target_positions=LEFT_INITIAL_JOINTS,
        right_target_positions=RIGHT_INITIAL_JOINTS,
        body_target_positions=INITIAL_BODY_JOINTS,
        timeout=ARRIVAL_TIMEOUT,
    )
    if not initial_result.get("arrived", False):
        raise RuntimeError(f"[1/7] Initial joints did not arrive: {initial_result}")

    with step_ctx("step2_approach"):
        execute_dual_pose(
            interface, "[2/7] 移动到抓取点两侧",
            APPROACH_LEFT, APPROACH_RIGHT, MOVEL_DURATION,
        )
    with step_ctx("step3_grasp"):
        execute_dual_pose(
            interface, "[3/7] 移动到抓取点",
            GRASP_LEFT, GRASP_RIGHT, MOVEL_DURATION,
        )
    with step_ctx("step4_lift"):
        execute_dual_pose(
            interface, "[4/7] 抬高箱体",
            LIFT_LEFT, LIFT_RIGHT, MOVEL_DURATION,
        )
    with step_ctx("step5_pour_ready"):
        execute_dual_pose(
            interface, "[5/7] 移动到准备倾倒点",
            POUR_READY_LEFT, POUR_READY_RIGHT, MOVEL_DURATION,
        )

    # [6/7] MoveJ 腰：不录制
    print("\n[6/7] 移动腰部到倾倒起始角度")
    interface.send_coordinated_joint_positions(body_positions=POUR_BODY_JOINTS)
    time.sleep(MOVEJ_DURATION)
    body_result = interface.wait_until_joint_arrive(
        body_target_positions=POUR_BODY_JOINTS,
        timeout=ARRIVAL_TIMEOUT,
    )
    if not body_result.get("arrived", False):
        raise RuntimeError(f"[6/7] Body joints did not arrive: {body_result}")

    with step_ctx("step7_pour_path"):
        print("\n[7/7] 执行三点连续倾倒路径")
        path_accepted = interface.execute_path(
            left_poses=[vector_to_pose(vector) for vector in POUR_LEFT_PATH],
            right_poses=[vector_to_pose(vector) for vector in POUR_RIGHT_PATH],
            trajectory_duration=POUR_TRAJECTORY_DURATION,
            frame_id=FRAME_ID,
        )
        if not path_accepted:
            raise RuntimeError("[7/7] ExecutePath service returned success=False")
        time.sleep(POUR_TRAJECTORY_DURATION)
        require_arrival(
            interface.wait_until_arrive(part="left_arm", timeout=ARRIVAL_TIMEOUT),
            "[7/7] 连续倾倒路径",
            "left_arm",
        )
        require_arrival(
            interface.wait_until_arrive(part="right_arm", timeout=ARRIVAL_TIMEOUT),
            "[7/7] 连续倾倒路径",
            "right_arm",
        )

    if session_dir is not None:
        print(f"\n轨迹录制完成，会话目录: {session_dir}")


def main() -> int:
    """连接 ROS2，执行任务并保证资源清理。"""
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        print("连接 ROS2RobotInterface...")
        interface.connect()
        run_task(interface)
        print("\n倾倒任务执行完成")
        return 0
    except KeyboardInterrupt:
        print("\n操作人员中止倾倒任务")
        return 130
    except Exception as exc:
        print(f"\n倾倒任务失败: {exc}")
        return 1
    finally:
        interface.disconnect()


if __name__ == "__main__":
    sys.exit(main())
