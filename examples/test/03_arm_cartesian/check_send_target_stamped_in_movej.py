"""在 MOVEJ 下发送带坐标系的末端目标，并观察控制器目标反馈与到位结果。

运行：python check_send_target_stamped_in_movej.py
到位检测依赖 *_current_target；控制器须支持 MOVEJ 下的 stamped 目标。
"""

from copy import deepcopy
import sys
import time

from ros2_robot_interface import (
    FSM_HOLD,
    FSM_HOME,
    FSM_MOVEJ,
    ROS2RobotInterface,
    ROS2RobotInterfaceConfig,
)

from check_send_target_stamped import print_pose


STEP_SIZE = 0.1          # 每步移动距离（米），按实机空间调整
MAX_WAIT_TIME = 10.0     # 每步等待到位的超时（秒）
CHECK_INTERVAL = 0.5
POSE_THRESHOLD = 0.01   # 位置容差（米）
ORIENT_THRESHOLD = 5.0  # 姿态容差（度）


def main() -> int:
    # 必须关闭，否则 send_target_stamped() 会自动切到 OCS2。
    config = ROS2RobotInterfaceConfig(auto_switch_fsm_before_control=False)
    interface = ROS2RobotInterface(config)
    try:
        interface.connect()
        time.sleep(2.0)
        print("切换状态：HOLD → HOME → HOLD → MOVEJ")
        interface.send_fsm_command(FSM_HOLD)
        interface.send_fsm_command(FSM_HOME)
        time.sleep(6.0)
        interface.send_fsm_command(FSM_HOLD)
        interface.send_fsm_command(FSM_MOVEJ)

        deadline = time.monotonic() + MAX_WAIT_TIME
        while interface.get_fsm_state() != FSM_MOVEJ:
            if time.monotonic() >= deadline:
                raise RuntimeError("等待 MOVEJ 状态反馈超时")
            time.sleep(CHECK_INTERVAL)

        handlers = [interface.left_arm_handler]
        if interface.config.right_end_effector_pose_topic is not None:
            handlers.append(interface.right_arm_handler)

        initial_targets = []
        for handler in handlers:
            pose = handler.get_pose()
            frame_id = handler.get_frame_id()
            if pose is None or not frame_id:
                raise RuntimeError(f"{handler.label}: 缺少当前位姿或坐标系")
            initial_targets.append((handler, deepcopy(pose), frame_id))

        for dx, dy, dz, label in (
            (0.0, 0.0, STEP_SIZE, "向上"),
            (0.0, 0.0, 0.0, "返回"),
            (0.0, STEP_SIZE, 0.0, "向侧方"),
            (0.0, 0.0, 0.0, "返回"),
        ):
            for arm_index, (handler, initial, frame_id) in enumerate(initial_targets):
                if interface.get_fsm_state() != FSM_MOVEJ:
                    raise RuntimeError("已离开 MOVEJ，停止发送目标")
                target = deepcopy(initial)
                target.position.x += dx
                target.position.y += dy if arm_index == 0 else -dy
                target.position.z += dz
                print(f"\n{handler.label}: {label}，frame_id={frame_id}")
                print_pose(target, "发送目标")
                handler.send_target_stamped(frame_id, target)

                deadline = time.monotonic() + MAX_WAIT_TIME
                while True:
                    if interface.get_fsm_state() != FSM_MOVEJ:
                        raise RuntimeError("运动期间离开 MOVEJ")
                    result = handler.check_arrival(POSE_THRESHOLD, ORIENT_THRESHOLD)
                    print(f"  到位反馈: {result}")
                    if result["arrived"]:
                        break
                    if time.monotonic() >= deadline:
                        if handler.get_target_pose() is None:
                            raise RuntimeError(
                                f"{handler.label}: 未收到 current_target，无法判定到位"
                            )
                        raise RuntimeError(f"{handler.label}: 到位超时，停止后续步骤")
                    time.sleep(CHECK_INTERVAL)
        return 0
    finally:
        try:
            if interface.is_connected:
                interface.send_fsm_command(FSM_HOLD)
        finally:
            interface.disconnect()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, Exception) as exc:
        print(f"\n测试终止: {exc}")
        sys.exit(1)
