"""
测试 arm_handler.send_joint_positions() 功能
向手臂发送关节角度目标（MoveJ 模式），验证手臂能否运动到指定关节角
支持单臂和双臂机器人

注意：
- send_joint_positions() 内部会自动切换到 MOVEJ 状态（FSM 命令 4）
- 关节数量和顺序必须与实际硬件配置一致
- 到位判断依赖 /left_current_target 话题（笛卡尔空间），
  MoveJ 模式下如果未订阅此话题，check_arrival() 可能无法判断，
  请观察实际机器人运动或打印当前关节状态确认
"""

import time
import sys
from typing import List

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

# ============================================================================
# 测试参数（按需修改）
# ============================================================================
MAX_WAIT_TIME = 15.0    # 每步最大等待时间（秒）
CHECK_INTERVAL = 0.5    # 到位检查间隔（秒）
POSE_THRESHOLD = 0.05   # 位置距离阈值（米，用于 check_arrival）
STEP_WAIT = 3.0         # 每步发完后等待机器人运动的时间（秒）


def print_joint_state(interface, is_dual_arm: bool) -> None:
    """打印当前关节状态。"""
    joint_state = interface.get_joint_state(categorized=True)
    if not joint_state:
        print("  ⚠ 无法获取关节状态")
        return

    key = "left_arm" if is_dual_arm else "arm"
    left_data = joint_state.get(key, {})
    names = left_data.get("names", [])
    positions = left_data.get("positions", [])
    if names and positions:
        print(f"  左臂关节状态:")
        for name, pos in zip(names, positions):
            print(f"    {name}: {pos:.4f} rad")
    else:
        print(f"  ⚠ 未获取到左臂关节数据（key='{key}'）")

    if is_dual_arm:
        right_data = joint_state.get("right_arm", {})
        r_names = right_data.get("names", [])
        r_positions = right_data.get("positions", [])
        if r_names and r_positions:
            print(f"  右臂关节状态:")
            for name, pos in zip(r_names, r_positions):
                print(f"    {name}: {pos:.4f} rad")



def send_and_wait(
    interface,
    handler,
    positions: List[float],
    desc: str,
    is_dual_arm: bool,
    step_wait: float,
) -> bool:
    """发送关节目标并等待运动完成。"""
    print(f"  → 目标关节角: {[f'{p:.3f}' for p in positions]}")
    try:
        handler.send_joint_positions(positions)
        print(f"  ✓ send_joint_positions() 已发布（已自动切换到 MOVEJ 状态）")
    except Exception as e:
        print(f"  ✗ 发布失败: {e}")
        return False

    # MoveJ 不一定有 current_target 话题，先固定等待运动完成
    print(f"  → 等待运动完成（{step_wait:.0f}s）...")
    time.sleep(step_wait)


    print(f"  → 当前关节状态：")
    print_joint_state(interface, is_dual_arm)
    return True


def main() -> int:
    print("\n" + "=" * 70)
    print(" " * 12 + "arm_handler.send_joint_positions() 功能测试")
    print("=" * 70 + "\n")

    # ========================================================================
    # 第一部分：初始化和连接
    # ========================================================================
    print("[1] 创建配置...")
    config = ROS2RobotInterfaceConfig()

    print("[2] 创建 ROS2RobotInterface 实例...")
    interface = ROS2RobotInterface(config)

    print("[3] 连接到 ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功!\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1

    print("[4] 等待数据到达（2秒）...")
    time.sleep(2.0)
    print("    ✓ 数据收集已开始\n")

    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    print(f"[5] 检测到模式: {'双臂' if is_dual_arm else '单臂'}\n")

    # 检查关节控制器话题是否存在
    if interface.left_arm_handler.joint_controller_pub is None:
        print("⚠ 未检测到左臂关节控制器话题，send_joint_positions() 无法使用")
        print("  请确认 /ocs2_wbc_controller/target_joint_position/left 或类似话题已发布")
        interface.disconnect()
        return 1

    # ========================================================================
    # 第二部分：切换到 HOME 状态并获取初始关节角
    # ========================================================================
    print("-" * 70)
    print("[6] 切换状态：HOLD → HOME")
    print("-" * 70)
    print("  → 切换到 HOLD 状态...")
    interface.send_fsm_command(2)   # HOLD
    print("  → 切换到 HOME 状态...")
    interface.send_fsm_command(1)   # HOME
    time.sleep(6.0)
    print("  ✓ HOME 完成\n")

    print("-" * 70)
    print("[7] 获取 HOME 后的关节状态")
    print("-" * 70)
    print_joint_state(interface, is_dual_arm)
    print()

    # ========================================================================
    # 第三部分：读取当前关节角作为测试基准
    # ========================================================================
    joint_state = interface.get_joint_state(categorized=True)
    if not joint_state:
        print("⚠ 无法获取关节状态，退出测试")
        interface.disconnect()
        return 1

    arm_key = "left_arm" if is_dual_arm else "arm"
    left_positions = joint_state.get(arm_key, {}).get("positions", [])
    if not left_positions:
        print(f"⚠ 无法获取左臂关节位置（key='{arm_key}'），退出测试")
        interface.disconnect()
        return 1

    num_joints = len(left_positions)
    print(f"  左臂检测到 {num_joints} 个关节\n")

    # ========================================================================
    # 第四部分：send_joint_positions() 测试
    # ========================================================================
    print("=" * 70)
    print(f"[8] send_joint_positions() 测试")
    print("=" * 70)

    # 以 HOME 位姿为基准，对第 0 关节做小幅度偏移测试
    # ⚠️ 请根据实际机器人的关节限位调整偏移量，避免超限
    DELTA = 0.2  # 偏移量（弧度），按需修改

    test_steps = [
        (f"关节 0 偏移 +{DELTA} rad",
         [p + (DELTA if i == 0 else 0.0) for i, p in enumerate(left_positions)]),
        ("恢复到 HOME 位姿",
         list(left_positions)),
    ]

    for step_idx, (desc, target_positions) in enumerate(test_steps, start=1):
        print(f"\n[8.{step_idx}] 左臂 {desc}")
        print("-" * 70)
        send_and_wait(
            interface,
            interface.left_arm_handler,
            target_positions,
            desc,
            is_dual_arm,
            STEP_WAIT,
        )

    # 双臂模式下同样测试右臂
    if is_dual_arm and interface.right_arm_handler.joint_controller_pub is not None:
        right_positions = joint_state.get("right_arm", {}).get("positions", [])
        if right_positions:
            right_test_steps = [
                (f"关节 0 偏移 +{DELTA} rad",
                 [p + (DELTA if i == 0 else 0.0) for i, p in enumerate(right_positions)]),
                ("恢复到 HOME 位姿",
                 list(right_positions)),
            ]
            for step_idx, (desc, target_positions) in enumerate(right_test_steps, start=1):
                print(f"\n[8.右臂.{step_idx}] 右臂 {desc}")
                print("-" * 70)
                send_and_wait(
                    interface,
                    interface.right_arm_handler,
                    target_positions,
                    desc,
                    is_dual_arm,
                    STEP_WAIT,
                )

    # ========================================================================
    # 第五部分：清理和断开连接
    # ========================================================================
    print("\n" + "=" * 70)
    print("[9] 测试完成，切回 HOLD 并断开连接")
    print("=" * 70)
    interface.send_fsm_command(2)   # HOLD
    interface.disconnect()
    print("  ✓ 已断开连接\n")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
