#!/usr/bin/env python3
"""验证左右臂 stamped 目标使用独立轨迹，不会互相抢占。"""

import sys
import time

from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


MOVEL_DURATION = 5.0
SECOND_ARM_DELAY = 2.0
MAX_WAIT_TIME = 15.0
CHECK_INTERVAL = 0.5
CONTROLLER_NODE = "/ocs2_arm_controller"
FRAME_ID = "base_footprint"


def vector_to_pose(vector):
    """将 [x, y, z, qx, qy, qz, qw] 转换为 Pose。"""
    pose = Pose()
    pose.position.x = vector[0]
    pose.position.y = vector[1]
    pose.position.z = vector[2]
    pose.orientation.x = vector[3]
    pose.orientation.y = vector[4]
    pose.orientation.z = vector[5]
    pose.orientation.w = vector[6]
    return pose


def wait_for_arrival(interface, max_wait_time=MAX_WAIT_TIME):
    """等待双臂到达当前目标；超时返回 False。"""
    start_time = time.monotonic()
    while time.monotonic() - start_time < max_wait_time:
        left_result = interface.left_arm_handler.check_arrival()
        right_result = interface.right_arm_handler.check_arrival()
        left_arrived = left_result["arrived"]
        right_arrived = right_result["arrived"]

        print(
            "  → 到达状态: "
            f"左臂={'✓' if left_arrived else '✗'}, "
            f"右臂={'✓' if right_arrived else '✗'}"
        )
        if left_arrived and right_arrived:
            elapsed = time.monotonic() - start_time
            print(f"  ✓ 双臂均已到达（等待 {elapsed:.1f} 秒）")
            return True

        time.sleep(CHECK_INTERVAL)

    print(f"  ✗ 超时：{max_wait_time:.1f} 秒内双臂未全部到达")
    return False


def run_staggered_scenario(
    interface,
    title,
    first_arm,
    first_pose,
    second_arm,
    second_pose,
):
    """先后发送两侧 stamped 目标，并等待二者到达。"""
    handlers = {
        "left": interface.left_arm_handler,
        "right": interface.right_arm_handler,
    }
    labels = {"left": "左臂", "right": "右臂"}

    print("\n" + "-" * 70)
    print(title)
    print("-" * 70)
    print(f"  → 发送{labels[first_arm]} stamped 目标")
    handlers[first_arm].send_target_stamped(FRAME_ID, first_pose)

    print(
        f"  → 等待 {SECOND_ARM_DELAY:.1f} 秒；"
        f"{labels[first_arm]}的 {MOVEL_DURATION:.1f} 秒轨迹仍在执行"
    )
    time.sleep(SECOND_ARM_DELAY)

    print(f"  → 发送{labels[second_arm]} stamped 目标")
    handlers[second_arm].send_target_stamped(FRAME_ID, second_pose)
    print(
        f"  → 观察要求：{labels[first_arm]}继续原轨迹，"
        "不得停顿、跳到终点或重新计时"
    )
    return wait_for_arrival(interface)


def main():
    """运行左右两个镜像的 stamped 独立轨迹测试场景。"""
    print("\n" + "=" * 70)
    print("W2 Robot Dual Target Stamped Independent-Arms Test")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())

    try:
        print("\n[1] 连接到 ROS 2...")
        interface.connect()
        print("  ✓ 接口连接成功")

        if interface.config.right_end_effector_target_topic is None:
            print("  ✗ 此测试需要双臂模式，但未检测到右臂 target topic")
            return 1

        print("\n[2] 等待数据到达（2 秒）...")
        time.sleep(2.0)

        print("\n[3] 切换到 OCS2/MOVE 状态...")
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        interface.send_fsm_command(1)  # HOME
        time.sleep(5.0)
        interface.send_fsm_command(2)  # Hold
        time.sleep(1.0)
        interface.send_fsm_command(3)  # OCS2/MOVE
        time.sleep(2.0)

        print(f"\n[4] 设置 movel_duration={MOVEL_DURATION:.1f} 秒...")
        parameter_set = interface.set_node_parameters(
            full_node_name=CONTROLLER_NODE,
            parameters={"movel_duration": MOVEL_DURATION},
        )
        if not parameter_set:
            print(f"  ✗ 无法设置 {CONTROLLER_NODE} 的 movel_duration")
            return 1
        print("  ✓ 参数设置成功")

        left_target_a = vector_to_pose([
            0.732282, 0.414557, 1.49396,
            0.714695, 0.000156307, 0.699429, -0.0030065,
        ])
        right_target_a = vector_to_pose([
            0.732227, -0.414569, 1.49161,
            0.714696, -0.000154616, 0.699429, 0.00299888,
        ])
        left_target_b = vector_to_pose([
            0.724042, 0.649565, 1.16098,
            0.715409, 0.000935087, 0.698696, -0.0034755,
        ])
        right_target_b = vector_to_pose([
            0.723077, -0.714907, 1.14925,
            0.71541, -0.00133699, 0.698693, 0.00379536,
        ])

        if not run_staggered_scenario(
            interface,
            "[5] 场景 A：左臂先发送，2 秒后发送右臂",
            "left",
            left_target_a,
            "right",
            right_target_a,
        ):
            return 1

        print("\n[6] 等待 1 秒，确保场景 A 稳定...")
        time.sleep(1.0)

        if not run_staggered_scenario(
            interface,
            "[7] 场景 B：右臂先发送，2 秒后发送左臂",
            "right",
            right_target_b,
            "left",
            left_target_b,
        ):
            return 1

        print("\n" + "=" * 70)
        print("[8] 两个镜像场景均执行完成")
        print("=" * 70)
        return 0
    except Exception as exc:
        print(f"\n✗ 测试失败: {exc}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        print("\n[结束] 切换到 Hold 并断开连接...")
        try:
            interface.send_fsm_command(2)
        except Exception as exc:
            print(f"  ⚠ 切换 Hold 失败: {exc}")
        try:
            interface.disconnect()
            print("  ✓ 已断开连接")
        except Exception as exc:
            print(f"  ⚠ 断开连接时出错: {exc}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n用户中断测试")
        sys.exit(1)
