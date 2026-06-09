"""
Compute and print robot center of mass from /robot_description and /joint_states.

Run with a robot or simulation already publishing those topics. The script prints
one CoM estimate every second until interrupted.
"""

import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.dynamics import (
    ComEstimatorError,
    SupportMargins,
    SupportRectangle,
    SupportStatus,
    evaluate_support_margins,
)

DEFAULT_SUPPORT_RECTANGLE = SupportRectangle(
    x_min=-0.22,
    x_max=0.22,
    y_min=-0.22,
    y_max=0.22,
    margin=0.05,
    frame_id="base_footprint",
)

_SUPPORT_STATUS_MARKERS = {
    SupportStatus.SAFE: "✓",
    SupportStatus.WARNING: "!",
    SupportStatus.VIOLATION: "✗",
}


def _print_triggered_margins(title: str, entries: list[tuple[str, float]]) -> None:
    """Print only margin entries that breached their threshold (< 0)."""
    triggered = [(label, value) for label, value in entries if value < 0.0]
    if not triggered:
        return
    print(f"  {title}")
    for label, value in triggered:
        print(f"    {label}  {value:+.4f}")


def _print_support_margins(margins: SupportMargins) -> None:
    """Print support margins in a multi-line, scannable layout."""
    marker = _SUPPORT_STATUS_MARKERS[margins.status]
    print(f"  support  {marker} {margins.status.value}")
    _print_triggered_margins(
        "margins",
        [
            ("x min", margins.safe_margin_x_min),
            ("x max", margins.safe_margin_x_max),
            ("y min", margins.safe_margin_y_min),
            ("y max", margins.safe_margin_y_max),
        ],
    )
    _print_triggered_margins(
        "raw margins",
        [
            ("x min", margins.raw_margin_x_min),
            ("x max", margins.raw_margin_x_max),
            ("y min", margins.raw_margin_y_min),
            ("y max", margins.raw_margin_y_max),
        ],
    )


def _print_com_estimate(
    estimate,
    *,
    support_rectangle: SupportRectangle = DEFAULT_SUPPORT_RECTANGLE,
) -> None:
    """Print one CoM estimate block."""
    x, y, z = estimate.xyz
    print("[CoM]")
    print(f"  frame    {estimate.frame_id}")
    print(f"  position x={x:+.4f}  y={y:+.4f}  z={z:+.4f}")

    if estimate.frame_id != support_rectangle.frame_id:
        print(
            f"  support  ? unavailable "
            f"(com_frame={estimate.frame_id}, support_frame={support_rectangle.frame_id})"
        )
    else:
        _print_support_margins(evaluate_support_margins((x, y), support_rectangle))

    if estimate.missing_joints:
        print("  missing  ", ", ".join(estimate.missing_joints))
    if estimate.unsupported_joints:
        print("  skipped  ", ", ".join(estimate.unsupported_joints))


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
    printed_frame_diagnostics = False
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

            if not printed_frame_diagnostics and estimate.frame_diagnostics is not None:
                diag = estimate.frame_diagnostics
                print("[Frame diagnostics]")
                print(f"      Pinocchio universe: {diag.pinocchio_universe_name}")
                print(f"      URDF root link: {diag.urdf_root_link}")
                print(f"      requested CoM frame: {diag.requested_frame_id}")
                print(f"      relation: {diag.relation}")
                if not diag.frame_id_matches_root_link:
                    print(
                        "      warning: requested CoM frame differs from URDF root link; "
                        "current estimator does not apply a frame transform"
                    )
                printed_frame_diagnostics = True

            _print_com_estimate(estimate)
            print()

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
