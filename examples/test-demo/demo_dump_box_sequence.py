"""双臂倒箱四段位姿序列 Demo（基于倒箱任务 JSON 点位）。

流程（与任务 JSON 一一对应）：
    1_预备抬臂  prep   躯干前倾 10°，双臂抬起箱子（3s）
    2_半翻倒箱  dump   双臂前伸下探，半倾倒（4s）
    3_终倾      dump   双臂继续前倾到底，终倾（5s，运动结束后再保持 3s）
    4_回正持箱  dump   躯干回正、双臂收回持箱（5s）

- 双臂：全部通过 `send_dual_arm_target_stamped()` 下发
  （/dual_target/stamped，frame=arm_base，OCS2 笛卡尔参考规划）
- 躯干：`body_deg` 为角度制（度），脚本内统一 `math.radians` 转弧度后
  经 `send_body_joint_positions()` 下发；`body_deg=None` 表示该步躯干不动。
- 时序：躯干与双臂共用同一个 `/fsm_command`（分体栈下双臂 OCS2 与躯干
  MOVEJ 无法同帧切换），因此每步**先躯干（MOVEJ）到位，再双臂（OCS2）**
  顺序执行，避免 FSM 切换打断躯干运动。`sleep_s` 用于运动结束后的额外保持。
- 若检测到 WBC（`is_wbc=True`），躯干关节目标在 OCS2 下可能不生效，
  建议改用 `send_dual_arm_target_stamped(..., body_pose=...)` 单发双臂+躯干。

运行：
    conda run -n fa-ros2 python examples/test-demo/demo_dump_box_sequence.py
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


FRAME_ID = "arm_base"
ARRIVAL_TIMEOUT = 30.0
# 是否下发躯干关节指令。调试双臂抖动时置 False，只跑双臂。
ENABLE_BODY = False

# 倒箱任务点位：left_pose / right_pose 为 [x, y, z, qx, qy, qz, qw]（arm_base 下）
# body_deg 为角度制（度），脚本内转弧度；None 表示该步躯干保持不变。
STEPS: list[dict] = [
    {
        "name": "1_预备抬臂",
        "phase": "prep",
        "body_deg": [0, 0, 10, 0],
        "left_pose": [0.66, 0.245, 0.07, 0.520745, -0.021212, 0.853175, 0.021602],
        "right_pose": [0.66, -0.245, 0.07, -0.533299, -0.014676, -0.845797, 0.002115],
        "duration_s": 3.0,
    },
    {
        "name": "2_半翻倒箱",
        "phase": "dump",
        "body_deg": [0, 0, 10, 0],
        "left_pose": [0.71, 0.245, -0.08, 0.807602, -0.011331, 0.588951, 0.028075],
        "right_pose": [0.71, -0.245, -0.08, -0.816376, -0.012749, -0.57733, 0.00757],
        "duration_s": 4.0,
    },
    {
        "name": "3_终倾",
        "phase": "dump",
        "body_deg": None,
        "left_pose": [0.56, 0.245, -0.12, 0.988299, 0.002913, 0.149497, 0.030135],
        "right_pose": [0.56, -0.245, -0.12, -0.990716, -0.007813, -0.135137, 0.012602],
        "duration_s": 5.0,
        "sleep_s": 3.0,
    },
    {
        "name": "4_回正持箱",
        "phase": "dump",
        "body_deg": [0, 0, 0, 0],
        "left_pose": [0.66, 0.245, 0.07, 0.520745, -0.021212, 0.853175, 0.021602],
        "right_pose": [0.66, -0.245, 0.07, -0.533299, -0.014676, -0.845797, 0.002115],
        "duration_s": 5.0,
    },
]


def vector_to_pose(vector: Sequence[float]) -> Pose:
    """将 [x, y, z, qx, qy, qz, qw] 转为 geometry_msgs/msg/Pose。"""
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


def degrees_to_radians(values: Sequence[float]) -> list[float]:
    """角度制转弧度制。"""
    return [math.radians(value) for value in values]


def execute_step(interface: ROS2RobotInterface, step: dict) -> None:
    """执行单步：先躯干（MOVEJ）到位，再双臂（OCS2）到位，最后按 sleep_s 保持。"""
    name = step["name"]
    duration_s = step["duration_s"]

    # 1) 躯干：MOVEJ 先行并等待到位，避免随后双臂切换 OCS2 的 HOLD 命令打断它
    #    调试双臂抖动时置 ENABLE_BODY=False 可完全跳过躯干指令。
    body_deg = step.get("body_deg")
    if ENABLE_BODY and body_deg is not None:
        body_rad = degrees_to_radians(body_deg)
        print(f"[{name}] 躯干关节(rad) -> {[round(v, 4) for v in body_rad]}（MOVEJ 先行）")
        interface.send_body_joint_positions(body_rad)
        body_result = interface.wait_until_joint_arrive(
            body_target_positions=body_rad,
            timeout=ARRIVAL_TIMEOUT,
        )
        if not body_result.get("arrived", False):
            raise RuntimeError(f"[{name}] 躯干未到位: {body_result}")

    # 2) 双臂：OCS2 笛卡尔参考规划
    left_pose = vector_to_pose(step["left_pose"])
    right_pose = vector_to_pose(step["right_pose"])
    print(f"[{name}] 双臂(arm_base) duration={duration_s}s（OCS2）")
    interface.send_dual_arm_target_stamped(
        left_pose=left_pose,
        right_pose=right_pose,
        frame_id=FRAME_ID,
        movel_duration=duration_s,
    )
    time.sleep(duration_s)

    left_result = interface.wait_until_arrive(
        part="left_arm",
        timeout=ARRIVAL_TIMEOUT,
        arm_pose_threshold=0.015,
        arm_orient_threshold=10.0,
    )
    right_result = interface.wait_until_arrive(
        part="right_arm",
        timeout=ARRIVAL_TIMEOUT,
        arm_pose_threshold=0.015,
        arm_orient_threshold=10.0,
    )
    for part, result in (("left_arm", left_result), ("right_arm", right_result)):
        if not result.get("arrived", False):
            print(f"[{name}] 警告: {part} 未到位: {result.get('result')}")

    # 3) 运动结束后的额外保持
    sleep_s = step.get("sleep_s", 0.0)
    if sleep_s:
        print(f"[{name}] 保持 {sleep_s}s")
        time.sleep(sleep_s)


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
    try:
        print("连接 ROS 2 接口...")
        interface.connect()
        time.sleep(1.0)

        if interface.right_arm_handler is None:
            print("错误：未检测到双臂模式")
            return 1

        body_topic = interface.config.body_joint_controller_topic
        print(f"is_wbc: {interface.is_wbc}")
        print(f"body_joint_controller_topic: {body_topic}")
        print(f"body_joint_controller_pub 初始化: {interface.body_joint_controller_pub is not None}")
        if body_topic is None or interface.body_joint_controller_pub is None:
            print("警告: 未检测到躯干关节 topic，body_deg 将不会下发/生效！")
        if interface.is_wbc:
            print("注意: 检测到 WBC。躯干关节目标在 OCS2 下可能不生效；")
            print("      若躯干不动，请改用 send_dual_arm_target_stamped(..., body_pose=...) 单发。")

        print(f"共 {len(STEPS)} 步，frame: {FRAME_ID}")
        if input("将驱动真机双臂与躯干，输入 EXECUTE 继续: ").strip() != "EXECUTE":
            print("已取消")
            return 0

        for step in STEPS:
            execute_step(interface, step)

        print("\n倒箱任务执行完成")
        return 0
    except KeyboardInterrupt:
        print("\n操作人员中止倒箱任务")
        return 130
    except Exception as exc:
        print(f"\n倒箱任务失败: {exc}")
        return 1
    finally:
        cleanup(interface)


if __name__ == "__main__":
    raise SystemExit(main())
