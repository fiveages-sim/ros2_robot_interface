#!/usr/bin/env python3
"""
读取 reachability 报告，在 RViz 里用 MarkerArray 显示：按 pos_err_m 着色、各颜色对应的 XYZ 范围说明、yaw（CSV 含 target_yaw_deg 时）。
着色阈值与球体一致：绿 pos_err < 2 mm，黄 2–5 mm，红 > 5 mm。

  cd /path/to/fa-deploy-ws && source install/setup.bash
  python3 ros2_robot_interface/examples/test/visualize_reachability_report.py \\
    path/to/ccs_workspace_eval_*/

可选：--ros-domain-id 11  --launch-rviz  --frame-id arm_base

RViz：Fixed Frame 与终端打印一致；Add → MarkerArray → /reachability_report/markers
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time
from pathlib import Path

TOPIC_POINTS = "/reachability_report/points"
TOPIC_INFO = "/reachability_report/info"
NS = "reachability_report"


def _fev():
    ex = Path(__file__).resolve().parent.parent
    if str(ex) not in sys.path:
        sys.path.insert(0, str(ex))
    import fa_eval_viz  # noqa: PLC0415

    return fa_eval_viz


def _find(root: Path, name: str) -> Path | None:
    p = root / name
    if p.is_file():
        return p
    found = sorted(root.rglob(name), key=lambda x: x.stat().st_mtime, reverse=True)
    return found[0] if found else None


def _load_csv(path: Path) -> tuple[list[tuple[float, float, float]], list[float], list[float] | None]:
    pts, errs, yaws = [], [], []
    with path.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        if not r.fieldnames or "target_yaw_deg" not in r.fieldnames:
            fev = _fev()
            p2, e2 = fev._rviz_load_csv(path)
            return p2, e2, None
        for row in r:
            try:
                x = float(row["target_scan_x"])
                y = float(row["target_scan_y"])
                z = float(row["target_scan_z"])
                pe = float(row["pos_err_m"])
                yaw = float(row["target_yaw_deg"])
            except (KeyError, ValueError):
                continue
            if math.isnan(pe):
                continue
            pts.append((x, y, z))
            errs.append(pe)
            yaws.append(yaw)
    return pts, errs, yaws if pts else None


def _load_log(path: Path) -> tuple[list[tuple[float, float, float]], list[float], str | None]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    frame = None
    for line in lines:
        if line.startswith("# scan="):
            for t in line[2:].strip().split():
                if t.startswith("scan="):
                    frame = t.split("=", 1)[1].strip()
                    break
            break
    pts, errs = [], []
    col: dict[str, int] = {}
    for line in lines:
        if line.startswith("#"):
            continue
        parts = line.split("\t")
        if parts and parts[0] == "id" and "pos_err_m" in line:
            col = {n: i for i, n in enumerate(parts)}
            if not all(k in col for k in ("x", "y", "z", "pos_err_m")):
                raise ValueError(f"{path}: bad header")
            continue
        if not col:
            continue
        try:
            x = float(parts[col["x"]])
            y = float(parts[col["y"]])
            z = float(parts[col["z"]])
            pe = float(parts[col["pos_err_m"]])
        except (ValueError, IndexError, KeyError):
            continue
        if math.isnan(pe):
            continue
        pts.append((x, y, z))
        errs.append(pe)
    return pts, errs, frame


def load_report(path: Path) -> tuple[Path, list[tuple[float, float, float]], list[float], str | None, list[float] | None]:
    path = path.expanduser().resolve()
    if path.is_file():
        if path.name == "ocs2_eval.csv":
            p, e, y = _load_csv(path)
            return path, p, e, None, y
        if path.name == "scan_points.log":
            p, e, fr = _load_log(path)
            return path, p, e, fr, None
        sys.exit(f"need ocs2_eval.csv or scan_points.log, got {path}")
    if not path.is_dir():
        sys.exit(f"not found: {path}")
    if (c := _find(path, "ocs2_eval.csv")):
        p, e, y = _load_csv(c)
        return c, p, e, None, y
    if (lg := _find(path, "scan_points.log")):
        p, e, fr = _load_log(lg)
        return lg, p, e, fr, None
    sys.exit(f"no ocs2_eval.csv or scan_points.log under {path}")


def _extent(points: list[tuple[float, float, float]]) -> tuple[float, float, float, float, float, float, float]:
    xs, ys, zs = zip(*points)
    mn = min(xs), min(ys), min(zs)
    mx = max(xs), max(ys), max(zs)
    ext = max(mx[i] - mn[i] for i in range(3))
    return *mn, *mx, ext


def _bbox_line(label: str, n: int, xmin: float, xmax: float, ymin: float, ymax: float, zmin: float, zmax: float) -> str:
    return (
        f"{label}  n={n}  "
        f"x[{xmin:.3f},{xmax:.3f}]  y[{ymin:.3f},{ymax:.3f}]  z[{zmin:.3f},{zmax:.3f}]"
    )


def _tick_step(ext_m: float) -> float:
    """Pick a simple tick step for axis labels."""
    ext_m = max(0.0, float(ext_m))
    for s in (0.02, 0.05, 0.1, 0.2, 0.5, 1.0):
        if ext_m / s <= 12:
            return s
    return 1.0


def _add_axes_with_ticks(
    fev,
    out: list,
    *,
    frame_id: str,
    stamp,
    ext: float,
    tick_to: tuple[float, float, float],
    line_w: float = 0.004,
    text_h: float = 0.026,
) -> None:
    """Draw XYZ axes from origin with tick labels in arm_base frame."""
    x_to, y_to, z_to = tick_to
    step = _tick_step(ext)

    axes = fev.Marker()
    axes.header.frame_id, axes.header.stamp = frame_id, stamp
    axes.ns, axes.id = f"{NS}_axes", 0
    axes.type, axes.action = fev.Marker.LINE_LIST, fev.Marker.ADD
    axes.pose.orientation.w = 1.0
    axes.scale.x = line_w
    # 3 axis segments
    axes.points = [
        fev.Point(x=0.0, y=0.0, z=0.0),
        fev.Point(x=x_to, y=0.0, z=0.0),
        fev.Point(x=0.0, y=0.0, z=0.0),
        fev.Point(x=0.0, y=y_to, z=0.0),
        fev.Point(x=0.0, y=0.0, z=0.0),
        fev.Point(x=0.0, y=0.0, z=z_to),
    ]
    # White axes to avoid confusion with colored points
    axes.color.r = axes.color.g = axes.color.b = 0.85
    axes.color.a = 1.0
    out.append(axes)

    # Tick labels (positive direction only)
    tid = 10
    for axis, to_val in (("x", x_to), ("y", y_to), ("z", z_to)):
        t = step
        while t <= to_val + 1e-9:
            m = fev.Marker()
            m.header.frame_id, m.header.stamp = frame_id, stamp
            m.ns, m.id = f"{NS}_axes", tid
            tid += 1
            m.type, m.action = fev.Marker.TEXT_VIEW_FACING, fev.Marker.ADD
            if axis == "x":
                m.pose.position.x, m.pose.position.y, m.pose.position.z = t, 0.0, 0.0
            elif axis == "y":
                m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.0, t, 0.0
            else:
                m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.0, 0.0, t
            m.pose.orientation.w = 1.0
            m.scale.z = text_h
            m.color.r = m.color.g = m.color.b = 1.0
            m.color.a = 1.0
            m.text = f"{axis}={t:.2f}m"
            out.append(m)
            t += step


def _err_band_bboxes(
    points: list[tuple[float, float, float]],
    errs: list[float],
    thr_g: float,
    thr_y: float,
) -> str:
    """Human-readable XYZ ranges per color band (same rules as threshold coloring)."""

    def collect(pred) -> list[tuple[float, float, float]]:
        return [p for p, e in zip(points, errs) if pred(e)]

    g_pts = collect(lambda e: e < thr_g)
    y_pts = collect(lambda e: thr_g <= e <= thr_y)
    r_pts = collect(lambda e: e > thr_y)
    lines = [
        f"pos_err: green<{thr_g*1000:.0f}mm  yellow {thr_g*1000:.0f}-{thr_y*1000:.0f}mm  red>{thr_y*1000:.0f}mm",
    ]
    if g_pts:
        a, b, c, d, e_, f = _extent(g_pts)[:6]
        lines.append(_bbox_line("green ", len(g_pts), a, d, b, e_, c, f))
    else:
        lines.append("green   (no points)")
    if y_pts:
        a, b, c, d, e_, f = _extent(y_pts)[:6]
        lines.append(_bbox_line("yellow", len(y_pts), a, d, b, e_, c, f))
    else:
        lines.append("yellow  (no points)")
    if r_pts:
        a, b, c, d, e_, f = _extent(r_pts)[:6]
        lines.append(_bbox_line("red   ", len(r_pts), a, d, b, e_, c, f))
    else:
        lines.append("red     (no points)")
    return "\n".join(lines)


def points_marker(fev, frame_id: str, stamp, points, errs):
    thr_g, thr_y, cap_r = 0.002, 0.005, 0.15
    m = fev._build_marker(
        frame_id, stamp, points, f"{NS}_points", 0.01, "left", errs, "thresholds", False, thr_g, thr_y, cap_r
    )
    m.id = 0
    return m


def info_marker_array(fev, frame_id: str, stamp, points, errs, yaws, legend_text: str):
    from visualization_msgs.msg import MarkerArray

    thr_g, thr_y, cap_r = 0.002, 0.005, 0.15
    out = []

    xmin, ymin, zmin, xmax, ymax, zmax, ext = _extent(points)
    m = max(0.015, ext * 0.04)

    leg = fev.Marker()
    leg.header.frame_id, leg.header.stamp = frame_id, stamp
    leg.ns, leg.id = f"{NS}_legend", 0
    leg.type, leg.action = fev.Marker.TEXT_VIEW_FACING, fev.Marker.ADD
    leg.pose.position.x = xmin
    leg.pose.position.y = ymax + m * 2.4
    leg.pose.position.z = (zmin + zmax) * 0.5
    leg.pose.orientation.w = 1.0
    leg.scale.z = 0.035
    leg.color.r = leg.color.g = leg.color.b = 1.0
    leg.color.a = 1.0
    leg.text = legend_text
    out.append(leg)

    # Arm base axes + tick labels (positive directions).
    # Extend a bit beyond the point cloud extent for readability.
    axis_len = max(0.2, ext + m * 3.0)
    _add_axes_with_ticks(
        fev,
        out,
        frame_id=frame_id,
        stamp=stamp,
        ext=ext,
        tick_to=(axis_len, axis_len, axis_len),
        line_w=0.004,
        text_h=0.028,
    )

    # Yaw direction ticks (blue). Only available when CSV has target_yaw_deg.
    if yaws and len(yaws) == len(points):
        yl = max(0.008, min(ext * 0.06, 0.06))
        ym = fev.Marker()
        ym.header.frame_id, ym.header.stamp = frame_id, stamp
        ym.ns, ym.id = f"{NS}_y", 0
        ym.type, ym.action = fev.Marker.LINE_LIST, fev.Marker.ADD
        ym.pose.orientation.w = 1.0
        ym.scale.x = 0.003
        ym.color.r, ym.color.g, ym.color.b, ym.color.a = 0.15, 0.85, 1.0, 0.85
        pr = []
        for (x, y, z), yaw in zip(points, yaws):
            rad = math.radians(yaw)
            dx, dy = yl * math.cos(rad), yl * math.sin(rad)
            pr.extend([fev.Point(x=x, y=y, z=z), fev.Point(x=x + dx, y=y + dy, z=z)])
        ym.points = pr
        out.append(ym)

    ma = MarkerArray()
    ma.markers = out
    return ma


def main() -> int:
    fev = _fev()
    if fev.rclpy is None:
        sys.exit("need ROS 2 Python (rclpy, visualization_msgs, …)")

    ap = argparse.ArgumentParser(description="Reachability report → RViz MarkerArray")
    ap.add_argument("report", type=Path, help="ccs_workspace_eval dir or ocs2_eval.csv / scan_points.log")
    ap.add_argument("--frame-id", default="", help="default: arm_base or scan= from log")
    ap.add_argument("--ros-domain-id", type=int, default=None)
    ap.add_argument("--launch-rviz", action="store_true")
    args = ap.parse_args()

    src, points, errs, log_fr, yaws = load_report(args.report)
    if not points:
        sys.exit("no points")
    frame = args.frame_id or log_fr or "arm_base"

    thr_g, thr_y = 0.002, 0.005
    legend = _err_band_bboxes(points, errs, thr_g, thr_y)
    print(legend + "\n")

    if args.ros_domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    if args.launch_rviz:
        subprocess.Popen(["rviz2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(0.8)

    from visualization_msgs.msg import MarkerArray

    fev.rclpy.init()
    node = fev.rclpy.node.Node("reachability_report_viz")
    pub_points = node.create_publisher(fev.Marker, TOPIC_POINTS, 10)
    pub_info = node.create_publisher(MarkerArray, TOPIC_INFO, 10)

    print(
        f"{src}\n{len(points)} points  frame={frame!r}\n"
        f"RViz: Fixed Frame = {frame}\n"
        f"  - Marker (points)     : {TOPIC_POINTS}\n"
        f"  - MarkerArray (info)  : {TOPIC_INFO}"
    )

    try:
        while fev.rclpy.ok():
            stamp = node.get_clock().now().to_msg()
            pub_points.publish(points_marker(fev, frame, stamp, points, errs))
            pub_info.publish(info_marker_array(fev, frame, stamp, points, errs, yaws, legend))
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        fev.rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
