"""每秒打印右手力传感器原始读数，按 Ctrl+C 停止。

前置条件：已 source ROS 2，控制器正在运行并发布右手 FT wrench。

运行：
    cd /home/fiveages/dev_ws/fa-py-libraries
    uv run python ros2_robot_interface/examples/test/13_ft_and_compliance/check_right_ft.py
"""

import time

from ros2_robot_interface import (
    ROS2InterfaceError,
    ROS2RobotInterface,
    ROS2RobotInterfaceConfig,
)


def main():
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        interface.connect()
        print(f"右手 FT topic: {interface.config.right_ft_wrench_topic}", flush=True)
        print("每秒打印一次，按 Ctrl+C 停止。", flush=True)
        while True:
            time.sleep(1.0)
            try:
                wrench = interface.get_original_wrench("right")
                print(
                    f"force [N] (x, y, z)={wrench['force']} "
                    f"torque [N·m] (x, y, z)={wrench['torque']} "
                    f"frame_id={wrench['frame_id']!r} stamp={wrench['stamp']}",
                    flush=True,
                )
            except ROS2InterfaceError as exc:
                print(f"[等待右手 FT 数据] {exc}", flush=True)
    except KeyboardInterrupt:
        print("\n已停止。", flush=True)
    finally:
        interface.disconnect()


if __name__ == "__main__":
    main()
