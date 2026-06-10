"""
Prompt for an axis-aligned box in base_footprint and print head-camera FOV checks.

The box is assumed to be aligned with the base_footprint axes. The script
queries the current transform from base_footprint to the head camera frame,
projects the box corners into the camera image, and prints visibility and
image-center status.
"""

from __future__ import annotations

import argparse
from math import isfinite
import sys
from threading import Lock
import time
from typing import Iterable

from geometry_msgs.msg import Point
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.dynamics import (
    DEFAULT_BASE_FRAME,
    HEAD_CAMERA_FRAME,
    BoxFovEstimatorError,
    ComEstimatorError,
    GroundFovEstimatorError,
    SupportRectangle,
    estimate_box_fov_from_transform_stamped,
    estimate_ground_fov_from_transform_stamped,
    evaluate_support_margins,
)
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_CENTERED_THRESHOLD = 0.08
DEFAULT_FOV_MARKER_TOPIC = "/head_camera_ground_fov"
DEFAULT_BOX_MARKER_TOPIC = "/axis_aligned_box_fov"
DEFAULT_BOX_POSE_SIZE_TOPIC = "/axis_aligned_box_pose_size"
DEFAULT_SUPPORT_RECTANGLE = SupportRectangle(
    x_min=-0.34,
    x_max=0.34,
    y_min=-0.23,
    y_max=0.23,
    margin=0.02,
    frame_id="base_footprint",
)


def _format_xyz(values: Iterable[float]) -> str:
    x, y, z = tuple(values)
    return f"x={x:+.4f}  y={y:+.4f}  z={z:+.4f}"


def _prompt_float(prompt: str) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print("  invalid number, please enter a numeric value")
            continue
        if not (float("-inf") < value < float("inf")):
            print("  invalid number, value must be finite")
            continue
        return value


def _prompt_box() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    print("[box input]")
    center = (
        _prompt_float("  box center x in base_footprint [m]: "),
        _prompt_float("  box center y in base_footprint [m]: "),
        _prompt_float("  box center z in base_footprint [m]: "),
    )
    size = (
        _prompt_float("  box length along base x [m]: "),
        _prompt_float("  box width along base y [m]: "),
        _prompt_float("  box height along base z [m]: "),
    )
    for label, value in zip(("length", "width", "height"), size, strict=True):
        if value <= 0.0:
            raise ValueError(f"box {label} must be > 0, got {value}")
    return center, size


def _box_from_pose_size_values(
    values: Iterable[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    data = tuple(float(value) for value in values)
    if len(data) != 6:
        raise ValueError(f"expected 6 values [x, y, z, length, width, height], got {len(data)}")
    if not all(isfinite(value) for value in data):
        raise ValueError("all box pose/size values must be finite")

    center = (data[0], data[1], data[2])
    size = (data[3], data[4], data[5])
    for label, value in zip(("length", "width", "height"), size, strict=True):
        if value <= 0.0:
            raise ValueError(f"box {label} must be > 0, got {value}")
    return center, size


def _axis_aligned_box_corners(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    x, y, z = center
    length, width, height = size
    hx = length / 2.0
    hy = width / 2.0
    hz = height / 2.0

    return (
        (x - hx, y - hy, z - hz),
        (x - hx, y - hy, z + hz),
        (x - hx, y + hy, z - hz),
        (x - hx, y + hy, z + hz),
        (x + hx, y - hy, z - hz),
        (x + hx, y - hy, z + hz),
        (x + hx, y + hy, z - hz),
        (x + hx, y + hy, z + hz),
    )


def _print_box_corners(corners: tuple[tuple[float, float, float], ...]) -> None:
    print("[box corners in base_footprint]")
    for index, corner in enumerate(corners, start=1):
        print(f"  C{index}: {_format_xyz(corner)}")


def _print_box_fov_estimate(estimate, center_ray_at_box_z=None) -> None:
    if estimate.center_score is None:
        score_text = "None"
    else:
        score_text = f"{estimate.center_score:+.6f}"

    in_fov = [f"C{index}" for index, corner in enumerate(estimate.corners, start=1) if corner.in_image]
    out_fov = [f"C{index}" for index, corner in enumerate(estimate.corners, start=1) if not corner.in_image]

    print("[box fov]")
    print(f"  center_score  {score_text}")
    if center_ray_at_box_z is None:
        print("  center_ray_at_box_z  unavailable")
    else:
        point_text = (
            _format_xyz(center_ray_at_box_z.point_base)
            if center_ray_at_box_z.point_base is not None
            else "None"
        )
        distance_text = (
            "None"
            if center_ray_at_box_z.distance_along_ray is None
            else f"{center_ray_at_box_z.distance_along_ray:+.4f}"
        )
        print(
            "  center_ray_at_box_z  "
            f"status={center_ray_at_box_z.status.value}  "
            f"t={distance_text}  "
            f"point={point_text}"
        )
    print(f"  in_fov:   {' '.join(in_fov) if in_fov else '(none)'}")
    print(f"  out_fov:  {' '.join(out_fov) if out_fov else '(none)'}")


def _print_com_support_estimate(estimate, support_rectangle: SupportRectangle = DEFAULT_SUPPORT_RECTANGLE) -> None:
    x, y, z = estimate.xyz
    print("[CoM support]")
    print(f"  position  {_format_xyz((x, y, z))}")
    if estimate.frame_id != support_rectangle.frame_id:
        print(
            f"  support   unavailable "
            f"(com_frame={estimate.frame_id}, support_frame={support_rectangle.frame_id})"
        )
        return

    margins = evaluate_support_margins((x, y), support_rectangle)
    in_base_range = margins.raw_margin_x_min >= 0.0 and margins.raw_margin_x_max >= 0.0
    in_base_range = in_base_range and margins.raw_margin_y_min >= 0.0 and margins.raw_margin_y_max >= 0.0
    print(f"  in_base_range  {in_base_range}")
    print(f"  support_status {margins.status.value}")
    print(
        "  support_range "
        f"x_min={support_rectangle.x_min:+.4f}  "
        f"x_max={support_rectangle.x_max:+.4f}  "
        f"y_min={support_rectangle.y_min:+.4f}  "
        f"y_max={support_rectangle.y_max:+.4f}"
    )


def _point(values: Iterable[float]) -> Point:
    x, y, z = tuple(values)
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def _base_marker(frame_id: str, namespace: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _delete_all_marker(frame_id: str, namespace: str) -> MarkerArray:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def _build_ground_fov_markers(estimate) -> MarkerArray:
    points = list(estimate.polygon_base)
    if not points:
        return _delete_all_marker(estimate.source_frame_id, "head_camera_ground_fov")

    outline = _base_marker(estimate.source_frame_id, "head_camera_ground_fov", 0, Marker.LINE_STRIP)
    outline.scale.x = 0.035
    _set_color(outline, (0.05, 0.75, 1.0, 1.0))
    outline.points = [_point(point) for point in points]
    outline.points.append(_point(points[0]))

    rays = _base_marker(estimate.source_frame_id, "head_camera_ground_fov", 1, Marker.LINE_LIST)
    rays.scale.x = 0.018
    _set_color(rays, (1.0, 0.65, 0.05, 0.8))
    camera_origin = _point(estimate.camera_origin_base)
    for point in points:
        rays.points.append(camera_origin)
        rays.points.append(_point(point))

    markers = [outline, rays]

    if estimate.center_intersection.point_base is not None:
        center_ray = _base_marker(estimate.source_frame_id, "head_camera_ground_fov", 3, Marker.LINE_LIST)
        center_ray.scale.x = 0.018
        _set_color(center_ray, (0.8, 0.0, 1.0, 1.0))
        center_ray.points.append(camera_origin)
        center_ray.points.append(_point(estimate.center_intersection.point_base))
        markers.append(center_ray)

    if len(points) >= 3:
        fill = _base_marker(estimate.source_frame_id, "head_camera_ground_fov", 2, Marker.TRIANGLE_LIST)
        fill.scale.x = 1.0
        fill.scale.y = 1.0
        fill.scale.z = 1.0
        _set_color(fill, (0.05, 0.75, 1.0, 0.22))
        for index in range(1, len(points) - 1):
            fill.points.append(_point(points[0]))
            fill.points.append(_point(points[index]))
            fill.points.append(_point(points[index + 1]))
        markers.append(fill)

    return MarkerArray(markers=markers)


def _build_box_markers(
    corners: tuple[tuple[float, float, float], ...],
    center: tuple[float, float, float],
    camera_origin_base: tuple[float, float, float] | None,
    frame_id: str,
) -> MarkerArray:
    edges = (
        (0, 2), (2, 6), (6, 4), (4, 0),
        (1, 3), (3, 7), (7, 5), (5, 1),
        (0, 1), (2, 3), (4, 5), (6, 7),
    )

    box_edges = _base_marker(frame_id, "axis_aligned_box_fov", 0, Marker.LINE_LIST)
    box_edges.scale.x = 0.025
    _set_color(box_edges, (0.0, 0.85, 0.25, 1.0))
    for start, end in edges:
        box_edges.points.append(_point(corners[start]))
        box_edges.points.append(_point(corners[end]))

    center_marker = _base_marker(frame_id, "axis_aligned_box_fov", 1, Marker.SPHERE)
    center_marker.pose.position = _point(center)
    center_marker.scale.x = 0.08
    center_marker.scale.y = 0.08
    center_marker.scale.z = 0.08
    _set_color(center_marker, (1.0, 0.1, 0.1, 1.0))

    markers = [box_edges, center_marker]

    if camera_origin_base is not None:
        center_line = _base_marker(frame_id, "axis_aligned_box_fov", 2, Marker.LINE_LIST)
        center_line.scale.x = 0.018
        _set_color(center_line, (1.0, 0.1, 0.1, 0.75))
        center_line.points.append(_point(camera_origin_base))
        center_line.points.append(_point(center))
        markers.append(center_line)

    return MarkerArray(markers=markers)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prompt for an axis-aligned base_footprint box and print head-camera FOV checks."
    )
    parser.add_argument(
        "--camera-frame",
        default=HEAD_CAMERA_FRAME,
        help=f"TF target/camera frame. Default: {HEAD_CAMERA_FRAME}",
    )
    parser.add_argument(
        "--base-frame",
        default=DEFAULT_BASE_FRAME,
        help=f"TF source/base frame. Default: {DEFAULT_BASE_FRAME}",
    )
    parser.add_argument(
        "--tf-timeout",
        type=float,
        default=1.0,
        help="TF lookup timeout in seconds. Default: 1.0",
    )
    parser.add_argument(
        "--centered-threshold",
        type=float,
        default=DEFAULT_CENTERED_THRESHOLD,
        help=f"Normalized center threshold. Default: {DEFAULT_CENTERED_THRESHOLD}",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run one FOV check, publish markers once, and exit.",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.5,
        help="Publish period in seconds. Default: 0.5",
    )
    parser.add_argument(
        "--fov-marker-topic",
        default=DEFAULT_FOV_MARKER_TOPIC,
        help=f"RViz MarkerArray topic for camera ground FOV. Default: {DEFAULT_FOV_MARKER_TOPIC}",
    )
    parser.add_argument(
        "--box-marker-topic",
        default=DEFAULT_BOX_MARKER_TOPIC,
        help=f"RViz MarkerArray topic for the axis-aligned box. Default: {DEFAULT_BOX_MARKER_TOPIC}",
    )
    parser.add_argument(
        "--box-pose-size-topic",
        default=DEFAULT_BOX_POSE_SIZE_TOPIC,
        help=(
            "Float64MultiArray topic for live box updates: "
            "[x, y, z, length, width, height]. "
            f"Default: {DEFAULT_BOX_POSE_SIZE_TOPIC}"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.tf_timeout < 0.0:
        print("--tf-timeout must be >= 0", file=sys.stderr)
        return 2
    if args.centered_threshold < 0.0:
        print("--centered-threshold must be >= 0", file=sys.stderr)
        return 2
    if args.period <= 0.0:
        print("--period must be > 0", file=sys.stderr)
        return 2

    print("\n" + "=" * 70)
    print(" " * 18 + "Axis-Aligned Box FOV Check")
    print("=" * 70 + "\n")
    print(f"[config] camera_frame={args.camera_frame}")
    print(f"[config] base_frame={args.base_frame}")
    print(f"[config] centered_threshold={args.centered_threshold:+.6f}")
    print(f"[config] tf_timeout={args.tf_timeout:.2f}s")
    print(f"[config] fov_marker_topic={args.fov_marker_topic}")
    print(f"[config] box_marker_topic={args.box_marker_topic}")
    print(f"[config] box_pose_size_topic={args.box_pose_size_topic}")
    print(f"[config] period={args.period:.2f}s  once={args.once}")
    print()

    try:
        center, size = _prompt_box()
    except ValueError as exc:
        print(f"[box input error] {exc}", file=sys.stderr)
        return 2

    corners = _axis_aligned_box_corners(center, size)
    _print_box_corners(corners)
    print()

    box_lock = Lock()
    box_state = {
        "center": center,
        "size": size,
        "corners": corners,
        "version": 0,
    }

    config = ROS2RobotInterfaceConfig(joint_states_topic="/joint_states")
    interface = ROS2RobotInterface(config)

    print("[1] Connecting to ROS 2...")
    try:
        interface.connect()
        print("    Connected\n")
    except Exception as exc:
        print(f"    Failed to connect: {exc}\n", file=sys.stderr)
        return 1

    if interface.robot_node is None:
        print("    Failed to create ROS node\n", file=sys.stderr)
        return 1

    fov_marker_pub = interface.robot_node.create_publisher(MarkerArray, args.fov_marker_topic, 10)
    box_marker_pub = interface.robot_node.create_publisher(MarkerArray, args.box_marker_topic, 10)

    def _box_pose_size_callback(msg: Float64MultiArray) -> None:
        try:
            next_center, next_size = _box_from_pose_size_values(msg.data)
        except ValueError as exc:
            print(f"[box topic ignored] {exc}")
            return

        next_corners = _axis_aligned_box_corners(next_center, next_size)
        with box_lock:
            box_state["center"] = next_center
            box_state["size"] = next_size
            box_state["corners"] = next_corners
            box_state["version"] += 1
        print(
            "[box topic update] "
            f"center=({_format_xyz(next_center)})  "
            f"size=({_format_xyz(next_size)})"
        )

    box_pose_size_sub = interface.robot_node.create_subscription(
        Float64MultiArray,
        args.box_pose_size_topic,
        _box_pose_size_callback,
        10,
    )
    print(f"    Publishing camera FOV MarkerArray on {args.fov_marker_topic}")
    print(f"    Publishing box MarkerArray on {args.box_marker_topic}\n")
    print(
        f"    Listening for live box updates on {args.box_pose_size_topic} "
        "(std_msgs/Float64MultiArray: [x, y, z, length, width, height])\n"
    )

    print("[2] Reading TF, checking box FOV, and publishing markers...")
    print("    Press Ctrl+C to stop.\n")
    printed_box_version = 0
    try:
        while True:
            with box_lock:
                current_center = box_state["center"]
                current_corners = box_state["corners"]
                current_version = box_state["version"]

            if current_version != printed_box_version:
                _print_box_corners(current_corners)
                print()
                printed_box_version = current_version

            transform = interface.lookup_transform(
                args.camera_frame,
                args.base_frame,
                timeout=args.tf_timeout,
            )
            if transform is None:
                print(
                    f"[box fov unavailable] waiting for TF "
                    f"{args.base_frame} -> {args.camera_frame}"
                )
            else:
                box_center_for_marker = current_center
                camera_origin_for_marker = None
                marker_frame_id = args.base_frame

                try:
                    box_estimate = estimate_box_fov_from_transform_stamped(
                        current_corners,
                        transform,
                        centered_threshold=args.centered_threshold,
                    )
                except BoxFovEstimatorError as exc:
                    print(f"[box fov unavailable] {exc}")
                else:
                    center_ray_at_box_z = None
                    try:
                        center_ray_estimate = estimate_ground_fov_from_transform_stamped(
                            transform,
                            ground_z=box_estimate.box_center_base[2],
                        )
                    except GroundFovEstimatorError as exc:
                        print(f"[center ray at box z unavailable] {exc}")
                    else:
                        center_ray_at_box_z = center_ray_estimate.center_intersection

                    _print_box_fov_estimate(box_estimate, center_ray_at_box_z)
                    box_center_for_marker = box_estimate.box_center_base
                    marker_frame_id = box_estimate.source_frame_id

                try:
                    ground_estimate = estimate_ground_fov_from_transform_stamped(transform)
                except GroundFovEstimatorError as exc:
                    print(f"[ground fov visualization unavailable] {exc}")
                else:
                    fov_marker_pub.publish(_build_ground_fov_markers(ground_estimate))
                    camera_origin_for_marker = ground_estimate.camera_origin_base
                    marker_frame_id = ground_estimate.source_frame_id

                box_marker_pub.publish(
                    _build_box_markers(
                        current_corners,
                        box_center_for_marker,
                        camera_origin_for_marker,
                        marker_frame_id,
                    )
                )

                try:
                    com_estimate = interface.get_center_of_mass()
                except ComEstimatorError as exc:
                    print(f"[CoM support unavailable] {exc}")
                else:
                    if com_estimate is None:
                        print("[CoM support unavailable] waiting for cached robot_description and joint_states")
                    else:
                        _print_com_support_estimate(com_estimate)

            if args.once:
                break
            print()
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        del box_pose_size_sub
        print("[3] Disconnecting...")
        interface.disconnect()
        print("    Disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
