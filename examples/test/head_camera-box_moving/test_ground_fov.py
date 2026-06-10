"""
Continuously compute and print the head camera ground footprint.

Run with a robot or simulation already publishing TF. The script queries the
current transform from base_footprint to the head camera frame, projects the
camera image boundary rays to the base_footprint ground plane z=0, and prints
the resulting ground polygon.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

from geometry_msgs.msg import Point
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.dynamics import (
    DEFAULT_BASE_FRAME,
    HEAD_CAMERA_FRAME,
    GroundFovEstimatorError,
    GroundIntersectionStatus,
    estimate_ground_fov_from_transform_stamped,
)
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_MARKER_TOPIC = "/head_camera_ground_fov"


def _format_xyz(values: Iterable[float]) -> str:
    x, y, z = tuple(values)
    return f"x={x:+.4f}  y={y:+.4f}  z={z:+.4f}"


def _format_optional_xyz(values: tuple[float, float, float] | None) -> str:
    if values is None:
        return "None"
    return _format_xyz(values)


def _format_stamp(transform) -> str:
    stamp = transform.header.stamp
    return f"{stamp.sec}.{stamp.nanosec:09d}"


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


def _base_marker(estimate, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = estimate.source_frame_id
    marker.ns = "head_camera_ground_fov"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _delete_all_marker(frame_id: str) -> MarkerArray:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "head_camera_ground_fov"
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def _build_ground_fov_markers(estimate) -> MarkerArray:
    points = list(estimate.polygon_base)
    if not points:
        return _delete_all_marker(estimate.source_frame_id)

    outline = _base_marker(estimate, 0, Marker.LINE_STRIP)
    outline.scale.x = 0.035
    _set_color(outline, (0.05, 0.75, 1.0, 1.0))
    outline.points = [_point(point) for point in points]
    outline.points.append(_point(points[0]))

    rays = _base_marker(estimate, 1, Marker.LINE_LIST)
    rays.scale.x = 0.018
    _set_color(rays, (1.0, 0.65, 0.05, 0.8))
    camera_origin = _point(estimate.camera_origin_base)
    for point in points:
        rays.points.append(camera_origin)
        rays.points.append(_point(point))

    markers = [outline, rays]

    if estimate.center_intersection.point_base is not None:
        center_ray = _base_marker(estimate, 3, Marker.LINE_LIST)
        center_ray.scale.x = 0.018
        _set_color(center_ray, (0.8, 0.0, 1.0, 1.0))
        center_ray.points.append(camera_origin)
        center_ray.points.append(_point(estimate.center_intersection.point_base))
        markers.append(center_ray)

    if len(points) >= 3:
        fill = _base_marker(estimate, 2, Marker.TRIANGLE_LIST)
        fill.scale.x = 1.0
        fill.scale.y = 1.0
        fill.scale.z = 1.0
        _set_color(fill, (0.05, 0.75, 1.0, 0.22))

        # Fan triangulation: P0-P1-P2, P0-P2-P3, ...
        for index in range(1, len(points) - 1):
            fill.points.append(_point(points[0]))
            fill.points.append(_point(points[index]))
            fill.points.append(_point(points[index + 1]))
        markers.append(fill)

    return MarkerArray(markers=markers)


def _print_ground_fov(estimate, transform) -> None:
    print("[Ground FOV]")
    print(f"  stamp       {_format_stamp(transform)}")
    print(f"  transform   {estimate.source_frame_id} -> {estimate.camera_frame_id}")
    print(f"  ground      z={estimate.ground_z:+.4f} in {estimate.source_frame_id}")
    print(f"  camera      {_format_xyz(estimate.camera_origin_base)}")
    print(
        "  corners     "
        f"{len(estimate.polygon_base)}/4 intersect ground "
        f"(all={estimate.all_corners_intersect_ground})"
    )

    if estimate.polygon_base:
        print("  polygon")
        for index, point in enumerate(estimate.polygon_base, start=1):
            print(f"    P{index}: {_format_xyz(point)}")
    else:
        print("  polygon     unavailable")

    print("  rays")
    center = estimate.center_intersection
    center_distance = (
        "None"
        if center.distance_along_ray is None
        else f"{center.distance_along_ray:+.4f}"
    )
    print(
        f"    center: pixel=({center.pixel_uv[0]:.1f}, {center.pixel_uv[1]:.1f})  "
        f"status={center.status.value}  t={center_distance}  "
        f"point={_format_optional_xyz(center.point_base)}"
    )
    for index, intersection in enumerate(estimate.intersections, start=1):
        u, v = intersection.pixel_uv
        status = intersection.status.value
        distance = (
            "None"
            if intersection.distance_along_ray is None
            else f"{intersection.distance_along_ray:+.4f}"
        )
        print(
            f"    C{index}: pixel=({u:.1f}, {v:.1f})  "
            f"status={status}  t={distance}  "
            f"point={_format_optional_xyz(intersection.point_base)}"
        )

    if not estimate.polygon_base:
        print("  note        no image-corner ray intersects the ground in front of the camera")
    elif any(
        intersection.status is not GroundIntersectionStatus.INTERSECTS
        for intersection in estimate.intersections
    ):
        print("  note        partial footprint; at least one image-corner ray misses the ground")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print the current head-camera footprint on base_footprint ground z=0."
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
        "--period",
        type=float,
        default=0.5,
        help="Print period in seconds. Default: 0.5",
    )
    parser.add_argument(
        "--tf-timeout",
        type=float,
        default=1.0,
        help="TF lookup timeout in seconds. Default: 1.0",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=1.0,
        help="Ground plane z in base frame. Default: 0.0",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Print one estimate and exit.",
    )
    parser.add_argument(
        "--marker-topic",
        default=DEFAULT_MARKER_TOPIC,
        help=f"RViz MarkerArray topic. Default: {DEFAULT_MARKER_TOPIC}",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.period <= 0.0:
        print("--period must be > 0", file=sys.stderr)
        return 2
    if args.tf_timeout < 0.0:
        print("--tf-timeout must be >= 0", file=sys.stderr)
        return 2

    print("\n" + "=" * 70)
    print(" " * 18 + "Head Camera Ground FOV Monitor")
    print("=" * 70 + "\n")
    print(f"[config] camera_frame={args.camera_frame}")
    print(f"[config] base_frame={args.base_frame}")
    print(f"[config] ground_z={args.ground_z:+.4f}")
    print(f"[config] marker_topic={args.marker_topic}")
    print(f"[config] period={args.period:.2f}s  tf_timeout={args.tf_timeout:.2f}s\n")

    config = ROS2RobotInterfaceConfig()
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

    marker_pub = interface.robot_node.create_publisher(MarkerArray, args.marker_topic, 10)
    print(f"    Publishing RViz MarkerArray on {args.marker_topic}\n")

    print("[2] Reading TF and computing ground footprint...")
    try:
        while True:
            transform = interface.lookup_transform(
                args.camera_frame,
                args.base_frame,
                timeout=args.tf_timeout,
            )
            if transform is None:
                print(
                    f"[Ground FOV unavailable] waiting for TF "
                    f"{args.base_frame} -> {args.camera_frame}"
                )
            else:
                try:
                    estimate = estimate_ground_fov_from_transform_stamped(
                        transform,
                        ground_z=args.ground_z,
                    )
                except GroundFovEstimatorError as exc:
                    print(f"[Ground FOV unavailable] {exc}")
                else:
                    _print_ground_fov(estimate, transform)
                    marker_pub.publish(_build_ground_fov_markers(estimate))

            if args.once:
                break
            print()
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        print("[3] Disconnecting...")
        interface.disconnect()
        print("    Disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
