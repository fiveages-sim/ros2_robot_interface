"""验证 ROS2RobotInterface 的 body pose（笛卡尔）到位判定。

前置条件:
    - ROS 2 已 source；WBC 仿真/真机在运行。
    - body 控制器发布 /body_current_pose 与 /body_current_target（均 PoseStamped）。

运行:
    conda run -n fa-ros2 python examples/test/12_arrival/check_body_pose_arrive.py
"""

from __future__ import annotations

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

TIMEOUT_SEC = 10.0
POSE_THRESHOLD = 0.05
ORIENT_THRESHOLD = 5.0


def main() -> int:
    config = ROS2RobotInterfaceConfig(
        body_current_pose_topic="/body_current_pose",
        body_current_target_pose_topic="/body_current_target",
    )
    interface = ROS2RobotInterface(config)
    interface.connect()
    time.sleep(1.0)

    try:
        cur = interface.get_body_current_pose()
        tgt = interface.get_body_current_target_pose()
        print(f"body current pose: {cur}")
        print(f"body target  pose: {tgt}")
        if cur is None or tgt is None:
            print("skip: /body_current_pose 或 /body_current_target 未收到消息")
            return 0

        result = interface.wait_until_arrive(
            part="body_pose",
            timeout=TIMEOUT_SEC,
            arm_pose_threshold=POSE_THRESHOLD,
            arm_orient_threshold=ORIENT_THRESHOLD,
        )
        print(f"arrived={result.get('arrived')} elapsed={result.get('elapsed'):.2f}s")
        print(f"detail={result.get('result')}")
        return 0 if result.get("arrived") else 1
    finally:
        if interface.is_connected:
            interface.disconnect()
            print("[cleanup] disconnected")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(0)
    except Exception as exc:
        print(f"failed: {exc}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
