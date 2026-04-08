
import math
import time
import sys
from typing import List

from copy import deepcopy
from geometry_msgs.msg import Pose

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def print_pose(pose: Pose, title: str) -> None:
    print(title)
    print(f"  position: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}")
    print(
        "  orientation: "
        f"x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, "
        f"z={pose.orientation.z:.6f}, w={pose.orientation.w:.6f}"
    )


def main() -> int:

    expected_lateral_distance_m = 0.50  # 50cm
    arm_descend_m = 0.20  # 双臂下降距离（米）

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
    time.sleep(5.5)
    print("  ✓ HOME 完成\n")
    interface.send_fsm_command(2)
    interface.send_fsm_command(3)

    # ========================================================================
    # 第三部分：读取当前末端位姿作为测试基准
    # ========================================================================
    left_pose = interface.left_arm_handler.get_pose()
    if left_pose is None:
        print("⚠ 无法获取左臂当前位姿，请检查 pose 话题是否正常发布")
        interface.disconnect()
        return 1
    print_pose(left_pose, "左臂当前位姿")

    right_pose = None
    current_y_diff_m = None
    current_lateral_distance_m = None
    target_frame = "base_footprint"
    left_source_frame = None
    right_source_frame = None
    left_pose_in_base = None
    right_pose_in_base = None
    if is_dual_arm:
        right_pose = interface.right_arm_handler.get_pose()
        if right_pose is None:
            print("⚠ 无法获取右臂当前位姿，请检查 pose 话题是否正常发布")
            interface.disconnect()
            return 1
        left_source_frame = interface.left_arm_handler.get_frame_id()
        right_source_frame = interface.right_arm_handler.get_frame_id()

        # 在执行横向距离调整前，先固定左右臂四元数
        left_pose.orientation.x = 0.715033
        left_pose.orientation.y = 0.001095
        left_pose.orientation.z = 0.679082
        left_pose.orientation.w = -0.003379
        right_pose.orientation.x = 0.715033
        right_pose.orientation.y = -0.001095
        right_pose.orientation.z = 0.699082
        right_pose.orientation.w = 0.003379
        interface.send_dual_arm_target_stamped(left_pose, right_pose)
        time.sleep(5.0)

        dy = left_pose.position.y - right_pose.position.y
        current_y_diff_m = dy
        current_lateral_distance_m = abs(dy)
        print(
            f"左右臂当前 y 差值(left - right): {dy:.6f}  (abs: {abs(dy):.6f}) | "
            f"期望横向距离: {expected_lateral_distance_m:.2f}m"
        )

        print_pose(right_pose, "右臂当前位姿")

    print()

    # ========================================================================
    # 第四部分
    # ========================================================================
    if is_dual_arm and current_lateral_distance_m is not None:
        delta_lateral_distance_m = current_lateral_distance_m - expected_lateral_distance_m
        print(
            f"横向距离差值(当前-期望): {delta_lateral_distance_m:.6f}m  "
            f"(当前: {current_lateral_distance_m:.6f}m, 期望: {expected_lateral_distance_m:.2f}m)"
        )
        if right_pose is not None:
            # 统一处理正负号：
            # delta>0（当前更大）=> 左臂沿 -y 移动 delta/2，右臂沿 +y 移动 delta/2
            # delta<0（当前更小）=> 左臂沿 +y 移动 |delta|/2，右臂沿 -y 移动 |delta|/2
            left_target: Pose = deepcopy(left_pose)
            right_target: Pose = deepcopy(right_pose)
            dy_each = delta_lateral_distance_m / 2.0
            left_target.position.y -= dy_each
            right_target.position.y += dy_each

            print(
                f"调整指令：left y {left_pose.position.y:.6f} -> {left_target.position.y:.6f}, "
                f"right y {right_pose.position.y:.6f} -> {right_target.position.y:.6f}"
            )
            interface.send_dual_arm_target_stamped(left_target, right_target)
            time.sleep(5.0)

            left_pose_now = interface.left_arm_handler.get_pose()
            right_pose_now = interface.right_arm_handler.get_pose()

            if left_source_frame and left_pose_now is not None:
                left_pose_in_base = interface.transform_pose(
                    left_pose_now, left_source_frame, target_frame
                )
                if left_pose_in_base is not None:
                    print_pose(left_pose_in_base, f"左臂末端位姿（{target_frame}）")
                    print(f"  frame: {target_frame}")
                else:
                    print(f"⚠ 左臂位姿转换到 {target_frame} 失败")
            else:
                print(f"⚠ 左臂缺少 frame_id 或 pose，无法转换到 {target_frame}")

            if right_source_frame and right_pose_now is not None:
                right_pose_in_base = interface.transform_pose(
                    right_pose_now, right_source_frame, target_frame
                )
                if right_pose_in_base is not None:
                    print_pose(right_pose_in_base, f"右臂末端位姿（{target_frame}）")
                    print(f"  frame: {target_frame}")
                else:
                    print(f"⚠ 右臂位姿转换到 {target_frame} 失败")
            else:
                print(f"⚠ 右臂缺少 frame_id 或 pose，无法转换到 {target_frame}")


    categorized_joint_state = interface.get_joint_state(categorized=True)
    if categorized_joint_state and categorized_joint_state.get("body", {}).get("names"):
        body = categorized_joint_state["body"]
        body_names = body["names"]
        body_positions: List[float] = body["positions"].copy()
        print(
            "当前 body joints: "
            + ", ".join(f"{n}={p:.6f}" for n, p in zip(body_names, body_positions))
        )

        if "body_joint3" in body_names:
            idx = body_names.index("body_joint3")
            old_joint3 = body_positions[idx]
            body_positions[idx] = -0.0796
            print(f"设置 body_joint3: {old_joint3:.6f} -> {body_positions[idx]:.6f} rad")

            interface.send_body_joint_positions(body_positions)
            # time.sleep(5.0)

            # 同时发送双臂姿态：xyz 不变，四元数绕末端自身 y 轴逆时针转 45°（右乘旋转四元数）
            if is_dual_arm and interface.right_arm_handler is not None:
                left_pose_now = interface.left_arm_handler.get_pose()
                right_pose_now = interface.right_arm_handler.get_pose()
                if left_pose_now is not None and right_pose_now is not None:
                    # 绕局部 y 轴逆时针 45° → q_rot = (x=0, y=+sin(π/8), z=0, w=cos(π/8))
                    s = math.sin(math.pi / 8)   # sin(22.5°)
                    c = math.cos(math.pi / 8)   # cos(22.5°)
                    left_target = deepcopy(left_pose_now)
                    right_target = deepcopy(right_pose_now)
                    for target in (left_target, right_target):
                        target.position.x += 0.2
                        cx = target.orientation.x
                        cy = target.orientation.y
                        cz = target.orientation.z
                        cw = target.orientation.w
                        # q_new = q_cur * (0, +s, 0, c)
                        target.orientation.x =  cx * c - cz * s
                        target.orientation.y =  cy * c + cw * s
                        target.orientation.z =  cx * s + cz * c
                        target.orientation.w =  cw * c - cy * s
                    print("同时发送双臂姿态（x +0.1m，绕末端 y 轴逆时针 45°）")
                    interface.send_dual_arm_target_stamped(left_target, right_target)
                else:
                    print("⚠ 无法获取当前末端位姿，跳过双臂姿态发送")

            time.sleep(5.0)
        else:
            print("⚠ 当前 body joints 中未找到 body_joint3，跳过设置")
    else:
        print("⚠ 未获取到 body joint 状态，跳过 body_joint3 设置")


    # 获取当前左右末端在 base_footprint 下的位姿，向下移动 0.3m 后发送
    if is_dual_arm and interface.right_arm_handler is not None:
        left_pose_cur = interface.left_arm_handler.get_pose()
        right_pose_cur = interface.right_arm_handler.get_pose()
        if (left_pose_cur is not None and right_pose_cur is not None
                and left_source_frame and right_source_frame):
            left_in_base = interface.transform_pose(left_pose_cur, left_source_frame, target_frame)
            right_in_base = interface.transform_pose(right_pose_cur, right_source_frame, target_frame)
            if left_in_base is not None and right_in_base is not None:
                print_pose(left_in_base, f"左臂当前位姿（{target_frame}）")
                print_pose(right_in_base, f"右臂当前位姿（{target_frame}）")
                left_down = deepcopy(left_in_base)
                right_down = deepcopy(right_in_base)
                left_down.position.z -= arm_descend_m
                right_down.position.z -= arm_descend_m
                print(
                    f"双臂向下移动 {arm_descend_m}m: "
                    f"left z {left_in_base.position.z:.6f} -> {left_down.position.z:.6f}, "
                    f"right z {right_in_base.position.z:.6f} -> {right_down.position.z:.6f}"
                )
                interface.send_dual_arm_target_stamped(left_down, right_down, target_frame)
                time.sleep(5.0)
            else:
                print(f"⚠ 位姿转换到 {target_frame} 失败，跳过下移")
        else:
            print("⚠ 无法获取末端位姿或 frame_id，跳过下移")

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
