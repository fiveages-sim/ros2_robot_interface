"""W2 拉并抓料箱任务示例。

动作点位来自项目根目录的《拉并抓料箱.txt》。左夹爪先拉箱，
右夹爪随后抓取并抬起料箱。
"""

import sys
import time
from collections.abc import Sequence

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


FRAME_ID = "arm_base"
DATA_SETTLE_TIME = 2.0
ARRIVAL_TIMEOUT = 30.0
MOVEJ_DURATION = 5.0
MOVEL_DURATION = 5.0
APPROACH_TRAJECTORY_DURATION = 6.0
GRIPPER_OPEN_PERCENT = 1.0
GRIPPER_CLOSED_PERCENT = 0.0

LEFT_INITIAL_JOINTS = [
    -0.087266,
    1.256637,
    0.680678,
    -2.286381,
    -1.029744,
    -0.453786,
    -0.383972,
]
RIGHT_INITIAL_JOINTS = [
    0.087266,
    1.256637,
    -0.680678,
    -2.286381,
    1.029744,
    -0.453786,
    0.383972,
]
INITIAL_BODY_JOINTS = [-0.900066, -1.799958, -0.900066, 0.0]

VISUAL_FIRST_LEFT = [
    0.7775, 0.4002, -0.1842, -0.70721, 0.055572, -0.703763, -0.038516
]
VISUAL_FIRST_RIGHT = [
    0.761, -0.3758, -0.1854, 0.704799, 0.082384, 0.701853, -0.062235
]
VISUAL_SECOND_LEFT = [
    0.7663, 0.1843, -0.1705, -0.708125, 0.049176, -0.703646, -0.031979
]
VISUAL_SECOND_RIGHT = [
    0.7676, -0.5756, -0.1842, 0.704039, 0.08846, 0.701228, -0.069165
]

PULL_APPROACH_LEFT_PATH = [
    [0.5354, 0.3909, -0.1558, -0.708125, 0.049176, -0.703646, -0.031979],
    [0.7663, 0.2143, -0.1505, -0.708125, 0.049176, -0.703646, -0.031979],
    [0.7663, 0.2143, -0.1705, -0.708125, 0.049176, -0.703646, -0.031979],
    [0.7663, 0.1843, -0.1705, -0.708125, 0.049176, -0.703646, -0.031979],
]
PULL_APPROACH_RIGHT_PATH = [
    [0.3043, -0.4668, -0.1606, 0.704039, 0.08846, 0.701228, -0.069165],
    [0.3043, -0.4668, -0.1606, 0.704039, 0.08846, 0.701228, -0.069165],
    [0.3043, -0.4668, -0.1606, 0.704039, 0.08846, 0.701228, -0.069165],
    [0.3043, -0.4668, -0.1606, 0.704039, 0.08846, 0.701228, -0.069165],
]

PULL_LIFT_LEFT = [
    0.7663, 0.1843, -0.1405, -0.708125, 0.049176, -0.703646, -0.031979
]
PULL_LIFT_RIGHT = PULL_APPROACH_RIGHT_PATH[-1]
PULL_END_LEFT = [
    0.7667, 0.4846, -0.1475, -0.708125, 0.049176, -0.703646, -0.031979
]
PULL_END_RIGHT = [
    0.3049, -0.4681, -0.1614, 0.704039, 0.08846, 0.701228, -0.069165
]

VISUAL_THIRD_LEFT = [
    0.7443, 0.4578, -0.1201, -0.703532, 0.089012, -0.705067, 0.000267
]
VISUAL_THIRD_RIGHT = [
    0.7635, -0.3025, -0.1953, -0.694838, -0.047969, -0.710836, 0.098037
]

GRASP_APPROACH_LEFT_PATH = [
    [0.7665, 0.4844, -0.1473, -0.703532, 0.089012, -0.705067, 0.000267],
    [0.7665, 0.4844, -0.1473, -0.703532, 0.089012, -0.705067, 0.000267],
    [0.7665, 0.4844, -0.1473, -0.703532, 0.089012, -0.705067, 0.000267],
    [0.7665, 0.4844, -0.1473, -0.703532, 0.089012, -0.705067, 0.000267],
]
GRASP_APPROACH_RIGHT_PATH = [
    [0.5345, -0.4509, -0.1688, -0.694838, -0.047969, -0.710836, 0.098037],
    [0.7635, -0.3325, -0.1753, -0.694838, -0.047969, -0.710836, 0.098037],
    [0.7635, -0.3325, -0.1953, -0.694838, -0.047969, -0.710836, 0.098037],
    [0.7635, -0.3025, -0.1953, -0.694838, -0.047969, -0.710836, 0.098037],
]

GRASP_LIFT_LEFT = GRASP_APPROACH_LEFT_PATH[-1]
GRASP_LIFT_RIGHT = [
    0.7635, -0.3025, -0.1653, -0.694838, -0.047969, -0.710836, 0.098037
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


def execute_dual_path(
    interface: ROS2RobotInterface,
    step_name: str,
    left_path: Sequence[Sequence[float]],
    right_path: Sequence[Sequence[float]],
) -> None:
    """通过 ExecutePath Service 执行双臂路径并检查最终到位。"""
    print(f"\n{step_name}")
    accepted = interface.execute_path(
        left_poses=[vector_to_pose(vector) for vector in left_path],
        right_poses=[vector_to_pose(vector) for vector in right_path],
        trajectory_duration=APPROACH_TRAJECTORY_DURATION,
        frame_id=FRAME_ID,
    )
    if not accepted:
        raise RuntimeError(f"{step_name}: ExecutePath service returned success=False")

    time.sleep(APPROACH_TRAJECTORY_DURATION)
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


def command_gripper(
    interface: ROS2RobotInterface,
    step_name: str,
    side: str,
    percent: float,
) -> None:
    """发送单侧夹爪百分比目标并等待到位。"""
    handler = (
        interface.left_gripper_handler
        if side == "left"
        else interface.right_gripper_handler
    )
    if handler is None:
        raise RuntimeError(f"{step_name}: {side} gripper handler is unavailable")

    handler.send_position_percent(percent)
    require_arrival(
        interface.wait_until_arrive(
            part=f"{side}_gripper",
            timeout=ARRIVAL_TIMEOUT,
        ),
        step_name,
        f"{side}_gripper",
    )


def validate_capabilities(interface: ROS2RobotInterface) -> None:
    """在机器人运动前验证双臂及百分比夹爪控制能力。"""
    if interface.config.right_end_effector_target_topic is None:
        raise RuntimeError("This demo requires dual-arm mode")
    for side, handler in (
        ("left", interface.left_gripper_handler),
        ("right", interface.right_gripper_handler),
    ):
        if handler is None or handler.target_percent_pub is None:
            raise RuntimeError(
                f"This demo requires {side} gripper target_percent control"
            )


def run_task(interface: ROS2RobotInterface) -> None:
    """严格按《拉并抓料箱.txt》的顺序执行双臂、腰部和夹爪动作。"""
    validate_capabilities(interface)

    print(f"等待状态数据 {DATA_SETTLE_TIME:.1f}s...")
    time.sleep(DATA_SETTLE_TIME)

    print("\n[准备] 完全打开左右夹爪")
    command_gripper(interface, "[准备] 打开左夹爪", "left", GRIPPER_OPEN_PERCENT)
    command_gripper(interface, "[准备] 打开右夹爪", "right", GRIPPER_OPEN_PERCENT)

    print("\n[1/9] 移动到初始关节位置")
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
        raise RuntimeError(f"[1/9] Initial joints did not arrive: {initial_result}")

    execute_dual_pose(
        interface,
        "[2/9] 移动到视觉 first 点",
        VISUAL_FIRST_LEFT,
        VISUAL_FIRST_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_pose(
        interface,
        "[3/9] 单独移动到视觉 second 点",
        VISUAL_SECOND_LEFT,
        VISUAL_SECOND_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_path(
        interface,
        "[4/9] 执行拉箱四段逼近路径",
        PULL_APPROACH_LEFT_PATH,
        PULL_APPROACH_RIGHT_PATH,
    )

    print("\n[5/9] 左夹爪抓取并轻提")
    command_gripper(
        interface,
        "[5/9] 关闭左夹爪",
        "left",
        GRIPPER_CLOSED_PERCENT,
    )
    execute_dual_pose(
        interface,
        "[5/9] 合爪后轻提",
        PULL_LIFT_LEFT,
        PULL_LIFT_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_pose(
        interface,
        "[6/9] 横拉箱体",
        PULL_END_LEFT,
        PULL_END_RIGHT,
        MOVEL_DURATION,
    )

    execute_dual_pose(
        interface,
        "[7/9] 单独移动到视觉 third 点",
        VISUAL_THIRD_LEFT,
        VISUAL_THIRD_RIGHT,
        MOVEL_DURATION,
    )
    execute_dual_path(
        interface,
        "[8/9] 执行抓取四段逼近路径",
        GRASP_APPROACH_LEFT_PATH,
        GRASP_APPROACH_RIGHT_PATH,
    )

    print("\n[9/9] 右夹爪抓取并抬起")
    command_gripper(
        interface,
        "[9/9] 关闭右夹爪",
        "right",
        GRIPPER_CLOSED_PERCENT,
    )
    execute_dual_pose(
        interface,
        "[9/9] 抓取后抬起",
        GRASP_LIFT_LEFT,
        GRASP_LIFT_RIGHT,
        MOVEL_DURATION,
    )


def main() -> int:
    """连接 ROS2，执行任务并保证资源清理。"""
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        print("连接 ROS2RobotInterface...")
        interface.connect()
        run_task(interface)
        print("\n拉并抓料箱任务执行完成")
        return 0
    except KeyboardInterrupt:
        print("\n操作人员中止拉并抓料箱任务")
        return 130
    except Exception as exc:
        print(f"\n拉并抓料箱任务失败: {exc}")
        return 1
    finally:
        interface.disconnect()


if __name__ == "__main__":
    sys.exit(main())
