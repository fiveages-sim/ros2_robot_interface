"""
W2 右臂：MOVEJ servo 连续来回摆动

servo 下没有 duration，快慢由发送侧决定：
以 RATE_HZ 持续发送一条连续曲线的时间采样点，让手臂在两个关节位姿之间来回摆动。

要点：目标是"路径上的采样点"，不是"终点"。相邻两条目标只差一点点，
所以滤波器是在追一个移动的点，不会在每个目标处减速到 0。
"""

import math
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


# 右臂 7 个关节的两个目标位置（rad）
JOINT_POSE_A = [
    -1.0878204719791311,
    -1.289885601473769,
    1.7209154738965002,
    -1.9466650573167845,
    0.3389286829785926,
    -0.6165078096092568,
    -0.7253940986693612,
]
JOINT_POSE_B = [
    -1.1836672623121074,
    -1.0343347122467803,
    1.8194790625350763,
    -1.804429317898399,
    0.6680073579719592,
    -0.7823042889006794,
    0.38241552380606325,
]

PERIOD = 6.0  # 一个来回 A -> B -> A 的周期（秒）
RATE_HZ = 100.0  # 目标发布频率（Hz）
SETTLE = 4.0  # 下发 A 之后等它收敛（秒）
HOME_WAIT = 5.0  # 等待回到 HOME（秒）


def main():
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()

    # 回 HOME
    interface.send_fsm_command(2)  # HOLD
    time.sleep(0.1)
    interface.send_fsm_command(1)  # HOME
    time.sleep(HOME_WAIT)

    # 打开 MOVEJ servo（下一条位置目标生效）。servo 下 movej_duration 不参与
    interface.set_node_parameters(
        interface.arm_controller, {"movej_interpolation_type": "servo"}
    )

    arm = interface.right_arm_handler

    # 先到 A 并等收敛（servo 会平滑地把手臂追到 A）
    arm.send_joint_positions(JOINT_POSE_A)
    time.sleep(SETTLE)

    # 连续曲线的采样点：q(t) = mid - half*cos(wt)，t=0 在 A、t=T/2 在 B、t=T 回到 A
    mid = [(a + b) / 2.0 for a, b in zip(JOINT_POSE_A, JOINT_POSE_B)]
    half = [(b - a) / 2.0 for a, b in zip(JOINT_POSE_A, JOINT_POSE_B)]
    w = 2.0 * math.pi / PERIOD
    dt = 1.0 / RATE_HZ

    print("servo 摆动中，Ctrl+C 停止")
    t0 = time.monotonic()
    try:
        while True:
            c = math.cos(w * (time.monotonic() - t0))
            arm.send_joint_positions([m - h * c for m, h in zip(mid, half)])
            time.sleep(dt)
    except KeyboardInterrupt:
        print()

    # 收尾
    interface.send_fsm_command(2)  # HOLD
    time.sleep(0.2)
    interface.send_fsm_command(1)  # HOME
    time.sleep(HOME_WAIT)
    interface.disconnect()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
