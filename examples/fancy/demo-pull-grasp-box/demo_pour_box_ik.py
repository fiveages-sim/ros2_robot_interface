"""W2 双臂抓箱并倾倒任务 IK Motion Planning 示例。

动作点位来自项目根目录的《倾倒任务.txt》。本示例只控制双臂和腰部，
不控制夹爪，也不会在结束时自动回 HOME。
第 1、6 步使用 MoveJ；第 2～5 步使用双臂 MoveL + DLS IK；
第 7 步使用两段双臂 MoveL + AUTO IK 倾倒，再用 MoveL + DLS IK 回正。

运行前需确保 /ocs2_arm_controller/execute_linear Action 可用。
"""

import math
import sys
import time
from collections.abc import Sequence

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


FRAME_ID = "arm_base"
DATA_SETTLE_TIME = 2.0
ARRIVAL_TIMEOUT = 30.0
ACTION_SERVER_TIMEOUT = 5.0
MOVEJ_DURATION = 5.0
MOVEL_DURATION = 5.0
POUR_MOVEL_SEGMENT_DURATION = 5.0
RETURN_MOVEL_DURATION = 5.0

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


def require_motion_success(result: object, step_name: str) -> None:
    """Motion Planning Action 失败时终止任务并保留返回消息。"""
    if result is None or not bool(getattr(result, "success", False)):
        message = (
            getattr(result, "message", "no action result")
            if result is not None
            else "no action result"
        )
        raise RuntimeError(f"{step_name}: motion planning failed: {message}")


def execute_dual_movel(
    interface: ROS2RobotInterface,
    step_name: str,
    left_vector: Sequence[float],
    right_vector: Sequence[float],
    duration: float,
    ik_type: str = "DLS",
) -> None:
    """通过阻塞式 MoveL Action 规划并执行双臂逐点 IK 轨迹。"""
    print(f"\n{step_name}")
    result = interface.execute_movel_action(
        "both",
        vector_to_pose(left_vector),
        right_endpoint_pose=vector_to_pose(right_vector),
        duration=duration,
        time_mode=True,
        frame_id=FRAME_ID,
        ik_type=ik_type,
        timeout=max(ARRIVAL_TIMEOUT, duration + ACTION_SERVER_TIMEOUT),
        wait_for_server_timeout=ACTION_SERVER_TIMEOUT,
    )
    require_motion_success(result, step_name)


def run_task(interface: ROS2RobotInterface) -> None:
    """按《倾倒任务.txt》顺序执行 MoveJ 和 MoveL IK。"""
    if interface.config.right_end_effector_target_topic is None:
        raise RuntimeError("This demo requires dual-arm mode")

    print(f"等待状态数据 {DATA_SETTLE_TIME:.1f}s...")
    time.sleep(DATA_SETTLE_TIME)

    if not interface.wait_for_movel_action_server(timeout=ACTION_SERVER_TIMEOUT):
        raise RuntimeError(
            f"{interface.config.movel_action_name} Action is not available"
        )
    print("\n[1/7] 移动到初始关节位置（MoveJ）")
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

    execute_dual_movel(
        interface,
        "[2/7] 移动到抓取点两侧（MoveL + DLS IK）",
        APPROACH_LEFT,
        APPROACH_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_movel(
        interface,
        "[3/7] 移动到抓取点（MoveL + DLS IK）",
        GRASP_LEFT,
        GRASP_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_movel(
        interface,
        "[4/7] 抬高箱体（MoveL + DLS IK）",
        LIFT_LEFT,
        LIFT_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_movel(
        interface,
        "[5/7] 移动到准备倾倒点（MoveL + DLS IK）",
        POUR_READY_LEFT,
        POUR_READY_RIGHT,
        MOVEL_DURATION,
    )

    print("\n[6/7] 移动腰部到倾倒起始角度（MoveJ）")
    interface.send_coordinated_joint_positions(body_positions=POUR_BODY_JOINTS)
    time.sleep(MOVEJ_DURATION)
    body_result = interface.wait_until_joint_arrive(
        body_target_positions=POUR_BODY_JOINTS,
        timeout=ARRIVAL_TIMEOUT,
    )
    if not body_result.get("arrived", False):
        raise RuntimeError(f"[6/7] Body joints did not arrive: {body_result}")

    execute_dual_movel(
        interface,
        "[7A-1] 准备点到 point1（MoveL + AUTO IK）",
        POUR_LEFT_PATH[0],
        POUR_RIGHT_PATH[0],
        POUR_MOVEL_SEGMENT_DURATION,
        ik_type="AUTO",
    )
    execute_dual_movel(
        interface,
        "[7A-2] point1 到 point2（MoveL + AUTO IK）",
        POUR_LEFT_PATH[1],
        POUR_RIGHT_PATH[1],
        POUR_MOVEL_SEGMENT_DURATION,
        ik_type="AUTO",
    )

    execute_dual_movel(
        interface,
        "[7B] point2 回到准备点（MoveL + DLS IK）",
        POUR_READY_LEFT,
        POUR_READY_RIGHT,
        RETURN_MOVEL_DURATION,
    )


def main() -> int:
    """连接 ROS2，执行 IK Motion Planning 任务并保证资源清理。"""
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        print("连接 ROS2RobotInterface...")
        interface.connect()
        run_task(interface)
        print("\nIK Motion Planning 倾倒任务执行完成")
        return 0
    except KeyboardInterrupt:
        print("\n操作人员中止倾倒任务")
        return 130
    except Exception as exc:
        print(f"\nIK Motion Planning 倾倒任务失败: {exc}")
        return 1
    finally:
        interface.disconnect()


if __name__ == "__main__":
    sys.exit(main())
