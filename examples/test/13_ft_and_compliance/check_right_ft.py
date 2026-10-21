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
        start_time = time.monotonic()
        while True:
            time.sleep(1.0)
            try:
                wrench = interface.get_original_wrench("right")
                fx, fy, fz = wrench["force"]
                tx, ty, tz = wrench["torque"]
                print(
                    f"\n[{time.monotonic() - start_time:7.1f} s] "
                    f"坐标系: {wrench['frame_id']} | "
                    f"消息时间戳: {wrench['stamp']:.3f}\n"
                    f"  力   [N]   Fx={fx:+9.3f}  Fy={fy:+9.3f}  Fz={fz:+9.3f}\n"
                    f"  力矩 [N·m] Tx={tx:+9.3f}  Ty={ty:+9.3f}  Tz={tz:+9.3f}",
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
