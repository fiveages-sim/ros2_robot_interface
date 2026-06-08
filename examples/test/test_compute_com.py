"""
Compute and print robot center of mass from /robot_description and /joint_states.

Run with a robot or simulation already publishing those topics. The script prints
one CoM estimate every second until interrupted.
"""

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.dynamics import ComEstimatorError


def main():
    """Continuously print center-of-mass estimates."""
    print("\n" + "=" * 70)
    print(" " * 18 + "Center of Mass Monitor")
    print("=" * 70 + "\n")

    config = ROS2RobotInterfaceConfig(joint_states_topic="/joint_states")
    interface = ROS2RobotInterface(config)

    print("[1] Connecting to ROS 2...")
    try:
        interface.connect()
        print("    Connected\n")
    except Exception as exc:
        print(f"    Failed to connect: {exc}\n")
        return 1

    print("[2] Waiting for /robot_description and /joint_states...")
    try:
        while True:
            try:
                estimate = interface.get_center_of_mass()
            except ComEstimatorError as exc:
                print(f"[CoM unavailable] {exc}")
                time.sleep(1.0)
                continue

            if estimate is None:
                print("[CoM unavailable] waiting for cached robot_description and joint_states")
                time.sleep(1.0)
                continue

            x, y, z = estimate.xyz
            print(
                f"[CoM] frame={estimate.frame_id} "
                f"x={x:.4f} y={y:.4f} z={z:.4f}"
            )
            if estimate.missing_joints:
                print("      missing:", ", ".join(estimate.missing_joints))
            if estimate.unsupported_joints:
                print("      unsupported multi-q joints:", ", ".join(estimate.unsupported_joints))

            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        print("[3] Disconnecting...")
        interface.disconnect()
        print("    Disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
