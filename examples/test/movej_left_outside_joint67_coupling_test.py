"""
MoveJ 左臂 — 目标落在 joint6/7 耦合允许范围之外（非 URDF 限位）

与 joint67_coupling_test_v2 使用相同的耦合边界模型（|q7|→|q6|max 折线），
在 URDF 关节范围内选取 (q6,q7) 使耦合判据 value=|q6|−limit(|q7|)>0，
用 MoveJ 下发后观察：命令 vs /joint_states 实际、以及 introspection。

默认左臂参考 home 与 joint67_coupling_test_v2 的 case1 左臂一致；**仅下发左臂 MoveJ**，不命令右臂。

用法:
  python movej_left_outside_joint67_coupling_test.py
  python movej_left_outside_joint67_coupling_test.py --no-return-home
  python movej_left_outside_joint67_coupling_test.py --output /tmp/j67.txt
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from pal_statistics_msgs.msg import Statistics
except Exception:  # pragma: no cover
    Statistics = None

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

RESULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

CASE1_HOME_LEFT_DEG = [-16.77, 51.70, 34.29, -130.65, -63.55, -12.70, -12.39]

# 与 joint67_coupling_test_v2 一致（与 OCS2 task 中耦合模型对齐）
J7_KNEE_DEG = 49.0
J7_MAX_DEG = 90.0
J6_AT_KNEE_DEG = 60.0
J6_AT_MAX_DEG = 20.0
SMOOTH_ABS_EPS = 1e-6

# m6_ccs_ag2f90 左臂 URDF，用于裁剪命令（避免误测 URDF 钳位）
URDF_J6_RAD = (-0.8, 0.8)
URDF_J7_RAD = (-1.361356816555577, 1.361356816555577)
# 在耦合边界上再加大 |q6| 的裕量（rad），仍会被 URDF 上限截断
COUPLING_VIOLATION_EXTRA_RAD = math.radians(12.0)


def smooth_abs(x: float) -> float:
    return math.sqrt(x * x + SMOOTH_ABS_EPS)


def compute_limit(abs_j7: float) -> float:
    k_j7_knee = math.radians(J7_KNEE_DEG)
    k_j7_max = math.radians(J7_MAX_DEG)
    k_j6_at_knee = math.radians(J6_AT_KNEE_DEG)
    k_j6_at_max = math.radians(J6_AT_MAX_DEG)
    if abs_j7 <= k_j7_knee:
        return k_j6_at_knee
    slope = (k_j6_at_max - k_j6_at_knee) / (k_j7_max - k_j7_knee)
    lim = k_j6_at_knee + slope * (abs_j7 - k_j7_knee)
    return max(lim, k_j6_at_max)


def eval_coupling(q6: float, q7: float) -> tuple[float, float]:
    """returns (value, limit_rad) where value>0 表示在耦合允许域外。"""
    abs_j6 = smooth_abs(q6)
    abs_j7 = smooth_abs(q7)
    limit = compute_limit(abs_j7)
    return abs_j6 - limit, limit


def build_left_cmd_outside_coupling(home_left_rad: list[float]) -> tuple[list[float], float, float]:
    """在 URDF 内构造左臂 7 轴目标，使 (q6,q7) 违反耦合。"""
    cmd = list(home_left_rad)
    # 较大 |q7| 时耦合允许的 |q6| 变小，便于在 URDF j6 范围内「顶破」耦合
    q7_tgt = -1.12
    q7_tgt = max(URDF_J7_RAD[0], min(URDF_J7_RAD[1], q7_tgt))
    cmd[6] = q7_tgt
    lim = compute_limit(abs(cmd[6]))
    q6_mag = min(URDF_J6_RAD[1], lim + COUPLING_VIOLATION_EXTRA_RAD)
    if q6_mag <= lim + 1e-6:
        q6_mag = min(URDF_J6_RAD[1], lim + math.radians(5.0))
    sign = -1.0 if home_left_rad[5] <= 0 else 1.0
    cmd[5] = sign * q6_mag
    cmd[5] = max(URDF_J6_RAD[0], min(URDF_J6_RAD[1], cmd[5]))
    v, lim2 = eval_coupling(cmd[5], cmd[6])
    return cmd, v, lim2


class ResultLogger:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.lines: list[str] = []

    def log(self, msg: str = ""):
        print(msg)
        self.lines.append(msg)

    def save(self):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        with open(self.filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(self.lines) + "\n")
        print(f"\n结果已保存到: {self.filepath}")


class JointStateCache:
    def __init__(self):
        self.msg: JointState | None = None

    def callback(self, msg: JointState):
        self.msg = msg


class IntrospectionCache:
    def __init__(self):
        self.name_to_value: dict[str, float] = {}

    def callback(self, msg):
        try:
            self.name_to_value = {s.name: float(s.value) for s in msg.statistics}
        except Exception:
            return


def read_all_joints(cache: JointStateCache, node: Node, prefix: str = "left"):
    rclpy.spin_once(node, timeout_sec=0.1)
    if cache.msg is None:
        return None
    name_to_pos = dict(zip(cache.msg.name, cache.msg.position))
    joints = [name_to_pos.get(f"{prefix}_joint{i}") for i in range(1, 8)]
    if None in joints:
        return None
    return joints


def read_left_joint67_introspection(cache: IntrospectionCache) -> dict:
    ntv = cache.name_to_value or {}
    keys = [
        "state_interface.left_joint6/position",
        "command_interface.left_joint6/position",
        "state_interface.left_joint7/position",
        "command_interface.left_joint7/position",
    ]
    out = {k: ntv.get(k) for k in keys}
    if out[keys[0]] is not None and out[keys[1]] is not None:
        out["delta_left6_cmd_minus_state_deg"] = math.degrees(out[keys[1]] - out[keys[0]])
    if out[keys[2]] is not None and out[keys[3]] is not None:
        out["delta_left7_cmd_minus_state_deg"] = math.degrees(out[keys[3]] - out[keys[2]])
    return out


def movej_left_to_home(interface, log, joint_state_cache, rcl_node, home_left_deg) -> bool:
    """仅左臂 MoveJ 到给定 7 轴 home（不发送右臂）。"""
    home_left_rad = [math.radians(d) for d in home_left_deg]
    log.log("  MoveJ 左臂回 home...")
    try:
        interface.send_fsm_command(4)
        time.sleep(1.0)
        timeout_total = 15.0
        start_total = time.time()

        interface.left_arm_handler.send_joint_positions(home_left_rad)
        while time.time() - start_total < timeout_total:
            left_joints = read_all_joints(joint_state_cache, rcl_node, "left")
            if left_joints is not None:
                left_err = max(abs(left_joints[i] - home_left_rad[i]) for i in range(7))
                if left_err < 0.05:
                    log.log(f"    ✓ 左臂到位 (max_err={left_err:.4f} rad)")
                    return True
            time.sleep(0.2)
        log.log("    ⚠ 左臂 MoveJ 超时")
        return False
    except Exception as e:
        log.log(f"    ✗ MoveJ 失败: {e}")
        return False


def _wait_left_joints_settle(
    joint_state_cache, rcl_node, timeout_sec=12.0, interval=0.2, stable_tol_rad=0.002, stable_needed=5
):
    last = None
    stable_count = 0
    start = time.time()
    while time.time() - start < timeout_sec:
        rclpy.spin_once(rcl_node, timeout_sec=0.05)
        joints = read_all_joints(joint_state_cache, rcl_node, "left")
        if joints is None:
            time.sleep(interval)
            continue
        if last is not None:
            if max(abs(joints[i] - last[i]) for i in range(7)) < stable_tol_rad:
                stable_count += 1
                if stable_count >= stable_needed:
                    return True, joints
            else:
                stable_count = 0
        last = joints
        time.sleep(interval)
    joints = read_all_joints(joint_state_cache, rcl_node, "left")
    return False, joints


def run_movej_left_outside_joint67_coupling(
    interface,
    log,
    joint_state_cache,
    rcl_node,
    introspection_cache,
    home_left_deg,
) -> dict:
    home_left_rad = [math.radians(d) for d in home_left_deg]
    cmd_left, cmd_coupling_v, cmd_coupling_lim = build_left_cmd_outside_coupling(home_left_rad)

    rec: dict = {
        "tag": "movej_outside_joint67_coupling|left",
        "cmd_j6_deg": math.degrees(cmd_left[5]),
        "cmd_j7_deg": math.degrees(cmd_left[6]),
        "cmd_coupling_value_deg": math.degrees(cmd_coupling_v),
        "cmd_coupling_limit_deg": math.degrees(cmd_coupling_lim),
        "cmd_outside_coupling": cmd_coupling_v > 0,
        "act_j6_deg": None,
        "act_j7_deg": None,
        "act_coupling_value_deg": None,
        "act_outside_coupling": None,
        "d6_actual_minus_cmd_deg": None,
        "d7_actual_minus_cmd_deg": None,
        "settled": False,
        "intro_left67": {},
    }

    log.log("  说明: MoveJ 目标 (q6,q7) 在 URDF 内、但超出 joint67 耦合允许域（与 v2 相同 limit 模型）")
    log.log(
        f"    命令左 j6,j7 = {rec['cmd_j6_deg']:+.2f}°, {rec['cmd_j7_deg']:+.2f}° | "
        f"耦合: |q6|−limit(|q7|) = {rec['cmd_coupling_value_deg']:+.2f}° "
        f"(limit={rec['cmd_coupling_limit_deg']:.2f}°) → "
        f"{'域外' if rec['cmd_outside_coupling'] else '域内⚠'}"
    )

    try:
        interface.send_fsm_command(4)
        time.sleep(1.0)
        interface.left_arm_handler.send_joint_positions(cmd_left)
    except Exception as e:
        log.log(f"    ✗ 发送 MoveJ 失败: {e}")
        rec["error"] = str(e)
        return rec

    settled, act = _wait_left_joints_settle(joint_state_cache, rcl_node)
    rec["settled"] = settled
    if act is None:
        log.log("    ⚠ 未能读取左臂关节状态")
        return rec

    rec["act_j6_deg"] = math.degrees(act[5])
    rec["act_j7_deg"] = math.degrees(act[6])
    av, alim = eval_coupling(act[5], act[6])
    rec["act_coupling_value_deg"] = math.degrees(av)
    rec["act_outside_coupling"] = av > 0
    rec["d6_actual_minus_cmd_deg"] = rec["act_j6_deg"] - rec["cmd_j6_deg"]
    rec["d7_actual_minus_cmd_deg"] = rec["act_j7_deg"] - rec["cmd_j7_deg"]

    rclpy.spin_once(rcl_node, timeout_sec=0.0)
    rec["intro_left67"] = read_left_joint67_introspection(introspection_cache)

    log.log(
        f"    左臂实际 j6,j7 = {rec['act_j6_deg']:+.2f}°, {rec['act_j7_deg']:+.2f}° | "
        f"耦合余量 = {rec['act_coupling_value_deg']:+.2f}° → "
        f"{'域外' if rec['act_outside_coupling'] else '域内'} | "
        f"Δ(actual−cmd) = {rec['d6_actual_minus_cmd_deg']:+.2f}°, {rec['d7_actual_minus_cmd_deg']:+.2f}°"
    )
    log.log(f"    静止判定={'是' if settled else '否(超时)'}")
    intro = rec["intro_left67"]
    s6, c6 = intro.get("state_interface.left_joint6/position"), intro.get("command_interface.left_joint6/position")
    s7, c7 = intro.get("state_interface.left_joint7/position"), intro.get("command_interface.left_joint7/position")
    if all(v is not None for v in (s6, c6, s7, c7)):
        log.log(
            f"    introspection: L6 s={math.degrees(s6):+.2f}° c={math.degrees(c6):+.2f}° | "
            f"L7 s={math.degrees(s7):+.2f}° c={math.degrees(c7):+.2f}°"
        )
    return rec


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MoveJ left arm to joint6/7 outside coupling chart (inside URDF)"
    )
    parser.add_argument(
        "--output",
        default="",
        help="Log file (default: results/movej_joint67_coupling_outside_<timestamp>.txt)",
    )
    parser.add_argument(
        "--no-return-home",
        action="store_true",
        help="Do not MoveJ back to reference home after the probe",
    )
    args = parser.parse_args()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = args.output or os.path.join(RESULT_DIR, f"movej_joint67_coupling_outside_{ts}.txt")
    log = ResultLogger(out_path)

    if not rclpy.ok():
        rclpy.init()
    rcl_node = Node("movej_left_outside_joint67_coupling_test")
    joint_state_cache = JointStateCache()
    rcl_node.create_subscription(JointState, "/joint_states", joint_state_cache.callback, 10)
    introspection_cache = IntrospectionCache()
    if Statistics is not None:
        rcl_node.create_subscription(
            Statistics, "/controller_manager/introspection_data/full", introspection_cache.callback, 10
        )
    interface = None

    def cleanup():
        if interface:
            interface.disconnect()
        rcl_node.destroy_node()
        rclpy.shutdown()

    log.log("=" * 70)
    log.log("  MoveJ 左臂 — joint6/7 耦合域外探测（URDF 内）")
    log.log(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log.log("=" * 70)

    log.log("\n[1] 连接 ros2_robot_interface...")
    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)
    try:
        interface.connect()
        log.log("    ✓ 连接成功")
    except Exception as e:
        log.log(f"    ✗ 连接失败: {e}")
        log.save()
        cleanup()
        return 1

    log.log("\n[2] 等待 /joint_states（2s）...")
    time.sleep(2.0)

    log.log("\n[3] MoveJ 左臂 → case1 参考 home")
    ok_home = movej_left_to_home(
        interface, log, joint_state_cache, rcl_node, CASE1_HOME_LEFT_DEG
    )
    if not ok_home:
        log.log("  ⚠ home 未完全到位，仍发送耦合域外目标（请自行判断是否安全）")

    log.log("\n[4] MoveJ — 左臂目标在耦合域外")
    br = run_movej_left_outside_joint67_coupling(
        interface,
        log,
        joint_state_cache,
        rcl_node,
        introspection_cache,
        CASE1_HOME_LEFT_DEG,
    )

    if not args.no_return_home:
        log.log("\n[5] MoveJ 左臂 → 回 case1 home")
        movej_left_to_home(
            interface, log, joint_state_cache, rcl_node, CASE1_HOME_LEFT_DEG
        )
    else:
        log.log("\n[5] 跳过回 home (--no-return-home)")

    log.log("\n" + "=" * 70)
    log.log("  Summary")
    log.log("=" * 70)
    if br.get("act_j6_deg") is None:
        log.log(f"  NO_DATA or error={br.get('error', '')}")
    else:
        log.log(
            f"  命令: j6,j7={br['cmd_j6_deg']:+.2f}°, {br['cmd_j7_deg']:+.2f}° | "
            f"耦合 value(deg)={br['cmd_coupling_value_deg']:+.2f} 域外={br['cmd_outside_coupling']}\n"
            f"  实际: j6,j7={br['act_j6_deg']:+.2f}°, {br['act_j7_deg']:+.2f}° | "
            f"耦合 value(deg)={br['act_coupling_value_deg']:+.2f} 域外={br['act_outside_coupling']}\n"
            f"  Δ(actual−cmd): {br['d6_actual_minus_cmd_deg']:+.2f}°, {br['d7_actual_minus_cmd_deg']:+.2f}° | "
            f"settled={br['settled']}"
        )

    exit_code = 0
    if br.get("error") or br.get("act_j6_deg") is None:
        exit_code = 1
    elif not br.get("cmd_outside_coupling"):
        exit_code = 1

    log.save()
    cleanup()
    return exit_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⚠ 用户中断")
        sys.exit(1)
