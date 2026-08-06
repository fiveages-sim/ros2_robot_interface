"""
验证 get_wrench / enter_compliance / set_compliance_force 联调冒烟。

前置条件:
    - ROS 2 已 source；真机/仿真控制器在运行。
    - 当前真机仅左臂装 FT：robot.local.yaml 启用 left_ft（如 kwr75_485），
      存在 /left_ft_broadcaster/wrench；本脚本只读左侧。
    - COMPLIANCE 段需要 ocs2_arm_controller；须能从 HOLD 切入 COMPLIANCE。
    - 进入 COMPLIANCE 后若控制器仍有内部零力校准，调用方自行等待足够时间再设力。
      本接口不提供 wait_zero_force_calibration（无 /compliance_force_status）。

流程:
    1. connect
    2. 打印左臂 get_wrench（失败则警告，不中断）
    3. （默认）enter_compliance → set_compliance_force（示例用 0 N，避免危险接触）
    4. send_fsm_command(FSM_HOLD) 退出
    5. disconnect

运行:
    cd /home/fiveages/dev_ws/fa-py-libraries
    uv run python ros2_robot_interface/examples/test/13_ft_and_compliance/check_ft_and_compliance.py
    uv run python ros2_robot_interface/examples/test/13_ft_and_compliance/check_ft_and_compliance.py --skip-compliance
"""

from __future__ import annotations

import argparse
import sys
import time

from ros2_robot_interface import (
    FSM_HOLD,
    ROS2InterfaceError,
    ROS2RobotInterface,
    ROS2RobotInterfaceConfig,
)

parser = argparse.ArgumentParser(description="FT wrench + COMPLIANCE smoke check")
parser.add_argument(
    "--skip-compliance",
    action="store_true",
    help="Only read wrench; skip enter_compliance / set_compliance_force",
)
parser.add_argument(
    "--compliance-settle-sec",
    type=float,
    default=2.0,
    help="Wait after enter_compliance before set_compliance_force (default 2.0)",
)
args = parser.parse_args()

print("=" * 70)
print("FT wrench + COMPLIANCE check")
print("=" * 70)

interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
interface.connect()
time.sleep(1.0)

try:
    print(f"left_ft_wrench_topic={interface.config.left_ft_wrench_topic}")
    print(f"arm_controller={interface.arm_controller}")
    print(f"fsm_state={interface.get_fsm_state()}")

    print("-" * 70)
    # 当前真机仅左臂装 FT
    try:
        wrench = interface.get_wrench("left")
        print(
            f"get_wrench('left'): force={wrench['force']} "
            f"torque={wrench['torque']} frame_id={wrench['frame_id']!r} "
            f"stamp={wrench['stamp']}"
        )
    except (ROS2InterfaceError, ValueError) as exc:
        print(f"[warn] get_wrench('left') failed: {exc}")

    if args.skip_compliance:
        print("-" * 70)
        print("skip compliance (--skip-compliance)")
    else:
        print("-" * 70)
        print("enter_compliance()")
        interface.enter_compliance()
        print(f"fsm_state after enter={interface.get_fsm_state()}")
        if args.compliance_settle_sec > 0:
            print(f"waiting {args.compliance_settle_sec}s for possible internal zero-force cal...")
            time.sleep(args.compliance_settle_sec)

        # 示例用 0 N，避免危险接触；X 轴力控、其余位置控
        task_selection = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        force_setpoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print(f"set_compliance_force({task_selection}, {force_setpoint})")
        interface.set_compliance_force(task_selection, force_setpoint)
        print("set_compliance_force ok")

        print("-" * 70)
        print("send_fsm_command(FSM_HOLD)")
        interface.send_fsm_command(FSM_HOLD)
        print(f"fsm_state after HOLD={interface.get_fsm_state()}")
finally:
    if interface.is_connected:
        print("\n[cleanup] switch to HOLD")
        interface.send_fsm_command(FSM_HOLD)
        time.sleep(0.5)
        interface.disconnect()
        print("[cleanup] disconnected")
