"""
验证 ROS2RobotInterface.send_dual_arm_target_stamped() 的双臂目标下发。

send_dual_arm_target_stamped() 行为（本脚本直接调用该接口）:
    - 向 /dual_target/stamped 发布 Path：[left, right]；WBC 下可附带 body 为第 3 个 pose。
    - 传入 body_pose 时仅 WBC 模式支持，且 body_mode 只能为 BODY_TRACKING 或省略
      （省略时内部自动使用 BODY_TRACKING）。
    - movel_duration 在发布目标前写入控制器节点参数；该参数全局生效，直到再次修改。
    - 非法用法示例（勿执行）：body_pose + body_mode="BODY_FREE" 会抛 ValueError。

本脚本流程:
    1. 连接接口；非双臂模式则 skip。
    2. 第一段：双臂小幅偏移目标，movel_duration=3s，等待 3s。
    3. 第二段（仅 WBC）：双臂 + body_pose，movel_duration=10s，等待 10s。
    4. finally 中尽量切回 HOLD 并断开连接。

前置条件:
    ROS 2 已 source；双臂仿真/真机在运行。

运行:
    conda run -n fa-ros2 python examples/test/06_dual_arm/check_send_dual_arm_target_stamped.py
"""

import sys
import time

from geometry_msgs.msg import Point, Pose, Quaternion

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


# 两段不同目标：相对当前末端位姿的偏移量（米）
OFFSET_1 = (0.03, 0.0, 0.0)   # 第一段：沿 x +3cm
OFFSET_2 = (0.0, 0.0, 0.05)   # 第二段：沿 z +5cm
MOVEL_DURATION_1 = 3.0
MOVEL_DURATION_2 = 10.0


def create_pose(x, y, z, qw=1.0):
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=qw)
    return pose


def offset_pose(source, dx=0.0, dy=0.0, dz=0.0):
    pose = Pose()
    pose.position = Point(
        x=source.position.x + dx,
        y=source.position.y + dy,
        z=source.position.z + dz,
    )
    pose.orientation = Quaternion(
        x=source.orientation.x,
        y=source.orientation.y,
        z=source.orientation.z,
        w=source.orientation.w,
    )
    return pose


def cleanup(interface):
    try:
        if interface.is_connected:
            print("\n[cleanup] switch to HOLD")
            interface.send_fsm_command(2)
            time.sleep(0.5)
    except Exception as exc:
        print(f"[cleanup] warn: failed to switch HOLD: {exc}")
    finally:
        try:
            interface.disconnect()
            print("[cleanup] disconnected")
        except Exception as exc:
            print(f"[cleanup] warn: failed to disconnect: {exc}")


def main() -> int:
    print("=" * 70)
    print("Dual-arm target stamped check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        if (
            interface.config.right_end_effector_target_topic is None
            or interface.left_arm_handler is None
            or interface.right_arm_handler is None
        ):
            print("skip: dual-arm only")
            return 0

        left_current = interface.left_arm_handler.get_pose()
        right_current = interface.right_arm_handler.get_pose()
        if left_current is None or right_current is None:
            print("skip: cannot read current arm poses")
            return 0

        frame_id = interface.left_arm_handler.get_frame_id() or "arm_base"
        left_pose_1 = offset_pose(left_current, *OFFSET_1)
        right_pose_1 = offset_pose(right_current, *OFFSET_1)
        left_pose_2 = offset_pose(left_current, *OFFSET_2)
        right_pose_2 = offset_pose(right_current, *OFFSET_2)
        print(f"is_wbc={interface.is_wbc}, frame_id={frame_id}")
        print(f"arm_controller={interface.arm_controller}")

        print("-" * 70)
        print(
            "send_dual_arm_target_stamped(left, right) -> dual-arm only, "
            f"offset={OFFSET_1}, movel_duration={MOVEL_DURATION_1}"
        )
        interface.send_dual_arm_target_stamped(
            left_pose_1,
            right_pose_1,
            frame_id=frame_id,
            movel_duration=MOVEL_DURATION_1,
        )
        time.sleep(MOVEL_DURATION_1)

        if not interface.is_wbc:
            print("-" * 70)
            print("skip: WBC body_pose example (interface.is_wbc=False)")
            return 0

        body_pose = create_pose(0.1, 0.0, 0.75)
        print("-" * 70)
        print(
            "send_dual_arm_target_stamped(left, right, body_pose, "
            f"offset={OFFSET_2}, body_mode='BODY_TRACKING', "
            f"movel_duration={MOVEL_DURATION_2})"
        )
        interface.send_dual_arm_target_stamped(
            left_pose_2,
            right_pose_2,
            frame_id=frame_id,
            body_pose=body_pose,
            body_mode="BODY_TRACKING",
            movel_duration=MOVEL_DURATION_2,
        )
        time.sleep(MOVEL_DURATION_2)
        return 0
    finally:
        cleanup(interface)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted by user")
        sys.exit(1)
    except Exception as exc:
        print(f"\nfailed: {exc}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
