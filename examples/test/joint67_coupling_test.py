"""
Joint67 Coupling Constraint A/B Test

通过少量（4个）笛卡尔目标点，测试 OCS2 在有/无 joint67Coupling cost 时的行为差异。

流程：MoveJ 回 home_1 → 切到 OCS2 → 依次发送 4 个目标（不再三重组合循环）
每个点记录实际关节角度、耦合边界、笛卡尔误差，并在 Summary 汇总。

用法:
  Run A (有约束):   python joint67_coupling_test.py --label A
  Run B (无约束):   python joint67_coupling_test.py --label B

  运行前需在 task file 中设置 joint67Coupling.activate = true (A) 或 false (B)，
  然后重启 OCS2 控制器。

  结果自动保存到 joint67_coupling_result_A.txt / joint67_coupling_result_B.txt

  若需 MoveJ 将左臂发到 joint6/7 耦合允许域外（URDF 内）并记录误差，请使用
  movej_left_outside_joint67_coupling_test.py。
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.quat_pose import (
    quat_multiply,
    quat_normalize,
    quat_conjugate,
    euler_rpy_to_quat_xyzw,
)

LEFT_HOME_1 = [1.457, -1.063, -1.199, -0.807, -0.695, -0.686, -0.130]

TEST_CASES = [
    # 目标：点位本身远离胸口（更外侧/更靠前/稍抬高），减少 link3/link5 贴胸概率
    # 同时选取不同“基础朝向+手腕旋转”，更容易把 j7 拉到较大角度，从而触发耦合差异。
    {
        "tag": "away1_mid|forward|roll+30",
        # #1: roll+ 容易把关节推到耦合域外，点位再收近/略降低，提升可达与收敛
        # 进一步远离胸口：y 外移；同时减小 roll 幅度降低姿态难度，争取 3mm 收敛
        # Tuning history:
        # - Δ=(+0.0453,+0.0154,-0.0054) -> pos=(0.4347,0.3246,-0.2646)
        # - small step -> pos=(0.4256,0.3215,-0.2635)
        # - Δ=(+0.0228,+0.0067,+0.0108) -> pos=(0.4028,0.3148,-0.2743)
        # - Δ=(+0.0164,+0.0060,+0.0108) -> pos=(0.3864,0.3088,-0.2851)
        # - Δ=(+0.0119,+0.0045,+0.0055) -> pos=(0.3745,0.3043,-0.2906)
        # - Δ=(+0.0087,+0.0036,+0.0047) -> pos=(0.3658,0.3007,-0.2953)
        # - Δ=(+0.0056,+0.0014,+0.0047) -> pos=(0.3602,0.2993,-0.3000)
        # Latest: Δ=(+0.0071,+0.0039,+0.0015) -> pos := pos - Δ = (0.3531,0.2954,-0.3015)
        # Latest: Δ=(+0.0026,-0.0007,+0.0073) -> pos := pos - Δ = (0.3505,0.2961,-0.3088)
        "pos": (0.3505, 0.2961, -0.3088),
        "rpy_deg": (0.0, 90.0, 0.0),
        "wrist_rpy_deg": (30.0, 0.0, 0.0),
    },
    {
        "tag": "away2_far|left|yaw+",
        "pos": (0.54, 0.32, -0.24),
        "rpy_deg": (0.0, 90.0, 90.0),
        "wrist_rpy_deg": (0.0, 0.0, 90.0),
    },
    {
        "tag": "away3_mid_hi|right|roll-30",
        # #3: roll- 同上，收近并略降低
        # Tuning history:
        # - Δ=(+0.0314,+0.0102,+0.0121) -> pos=(0.4486,0.3298,-0.2421)
        # - small step -> pos=(0.4423,0.3278,-0.2445)
        # - Δ=(+0.0195,+0.0068,+0.0122) -> pos=(0.4228,0.3210,-0.2567)
        # - Δ=(+0.0131,+0.0036,+0.0148) -> pos=(0.4097,0.3174,-0.2715)
        # - Δ=(+0.0102,+0.0012,+0.0040) -> pos=(0.3995,0.3162,-0.2755)
        # - Δ=(+0.0121,+0.0041,+0.0090) -> pos=(0.3874,0.3121,-0.2845)
        # - Δ=(+0.0054,+0.0001,+0.0022) -> pos=(0.3820,0.3120,-0.2867)
        # Latest: Δ=(+0.0096,+0.0031,+0.0080) -> pos := pos - Δ = (0.3724,0.3089,-0.2947)
        # Latest: Δ=(+0.0044,-0.0017,+0.0139) -> pos := pos - Δ = (0.3680,0.3106,-0.3086)
        # Latest: Δ=(+0.0084,+0.0050,+0.0030) -> pos := pos - Δ = (0.3596,0.3056,-0.3116)
        # Latest: Δ=(+0.0062,+0.0033,+0.0040) -> pos := pos - Δ = (0.3534,0.3023,-0.3156)
        # Latest: Δ=(+0.0062,+0.0033,+0.0040) -> pos := pos - Δ = (0.3472,0.2990,-0.3196)
        "pos": (0.3472, 0.2990, -0.3196),
        "rpy_deg": (0.0, 90.0, -90.0),
        "wrist_rpy_deg": (-30.0, 0.0, 0.0),
    },
    {
        "tag": "away4_far_hi|left|yaw-60",
        # #4: 已接近 3mm，微调收近/略降低，争取稳定进入 3mm
        # Last run: Δx=+0.0058 Δy=-0.0017 Δz=-0.0005
        # Step1: pos := pos - Δ = (0.5342, 0.4017, -0.2295)
        # Step2 (small): pos := pos - 0.2*Δ = (0.5330, 0.4020, -0.2294)
        # Latest: Δ=(+0.0036,-0.0020,+0.0031) -> pos := pos - Δ = (0.5294,0.4040,-0.2325)
        # - Δ=(+0.0033,-0.0018,+0.0015) -> pos=(0.5261,0.4058,-0.2340)
        "pos": (0.5261, 0.4058, -0.2340),
        "rpy_deg": (0.0, 90.0, 90.0),
        "wrist_rpy_deg": (0.0, 0.0, -60.0),
    },
]

MAX_WAIT = 45.0
WAIT_INTERVAL = 0.2
MOVEJ_SETTLE = 2.0

# We consider the Cartesian position converged only if pos_err <= 3mm.
POS_TOL_M = 0.003

J7_KNEE_DEG = 49.0
J7_MAX_DEG = 90.0
J6_AT_KNEE_DEG = 60.0
J6_AT_MAX_DEG = 20.0
SMOOTH_ABS_EPS = 1e-6

RESULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


class ResultLogger:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.lines = []

    def log(self, msg: str = ""):
        print(msg)
        self.lines.append(msg)

    def save(self):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        with open(self.filepath, "w") as f:
            f.write("\n".join(self.lines) + "\n")
        print(f"\n结果已保存到: {self.filepath}")


def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, qy=qy, qz=qz, w=qw)
    return pose


def pose_has_nan(pose: Pose) -> bool:
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    return any(math.isnan(v) or math.isinf(v) for v in values)


def quat_to_axis_angle(q):
    qw = q[3]
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
    if sin_half < 1e-8:
        return 0.0
    angle = 2.0 * math.atan2(sin_half, qw)
    return angle


def compute_pose_error(target_pose: Pose, actual_pose: Pose):
    dx = target_pose.position.x - actual_pose.position.x
    dy = target_pose.position.y - actual_pose.position.y
    dz = target_pose.position.z - actual_pose.position.z
    pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)

    q_target = (target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w)
    q_actual = (actual_pose.orientation.x, actual_pose.orientation.y,
                actual_pose.orientation.z, actual_pose.orientation.w)
    q_diff = quat_multiply(q_target, quat_conjugate(q_actual))
    q_diff = quat_normalize(q_diff)
    if q_diff[3] < 0:
        q_diff = (-q_diff[0], -q_diff[1], -q_diff[2], -q_diff[3])
    angle_err_rad = quat_to_axis_angle(q_diff)
    angle_err_deg = math.degrees(angle_err_rad)

    return pos_err, angle_err_deg, dx, dy, dz


def compute_limit(abs_j7):
    k_j7_knee = math.radians(J7_KNEE_DEG)
    k_j7_max = math.radians(J7_MAX_DEG)
    k_j6_at_knee = math.radians(J6_AT_KNEE_DEG)
    k_j6_at_max = math.radians(J6_AT_MAX_DEG)
    if abs_j7 <= k_j7_knee:
        return k_j6_at_knee
    slope = (k_j6_at_max - k_j6_at_knee) / (k_j7_max - k_j7_knee)
    limit = k_j6_at_knee + slope * (abs_j7 - k_j7_knee)
    return max(limit, k_j6_at_max)


def smooth_abs(x):
    return math.sqrt(x * x + SMOOTH_ABS_EPS)


def eval_constraint(q6, q7):
    abs_j6 = smooth_abs(q6)
    abs_j7 = smooth_abs(q7)
    limit = compute_limit(abs_j7)
    value = abs_j6 - limit
    return value, limit


class JointStateCache:
    def __init__(self):
        self.msg = None

    def callback(self, msg: JointState):
        self.msg = msg


def read_joint67(cache: JointStateCache, node: Node, timeout_sec: float = 2.0):
    q6_left = q7_left = q6_right = q7_right = None
    start = time.time()
    while time.time() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cache.msg is not None:
            name_to_pos = dict(zip(cache.msg.name, cache.msg.position))
            q6_left = name_to_pos.get("left_joint6")
            q7_left = name_to_pos.get("left_joint7")
            q6_right = name_to_pos.get("right_joint6")
            q7_right = name_to_pos.get("right_joint7")
            if q6_left is not None and q7_left is not None:
                break
    return q6_left, q7_left, q6_right, q7_right


def wait_for_pos_convergence(interface, target_pose: Pose, max_wait=MAX_WAIT, interval=WAIT_INTERVAL):
    """Wait until pos_err <= POS_TOL_M. Returns (arrived, best_snapshot)."""
    best = {
        "pos_err": None,
        "ang_err": None,
        "dx": None,
        "dy": None,
        "dz": None,
    }
    start = time.time()
    while time.time() - start < max_wait:
        actual = interface.left_arm_handler.get_pose()
        if actual is not None:
            pos_err, ang_err, dx, dy, dz = compute_pose_error(target_pose, actual)
            if best["pos_err"] is None or pos_err < best["pos_err"]:
                best = {"pos_err": pos_err, "ang_err": ang_err, "dx": dx, "dy": dy, "dz": dz}
            if pos_err <= POS_TOL_M:
                return True, best
        time.sleep(interval)
    return False, best


def movej_to_home(interface, log, joint_state_cache, rcl_node):
    log.log("  MoveJ 回 home_1...")
    try:
        interface.send_fsm_command(4)
        time.sleep(1.0)
        interface.left_arm_handler.send_joint_positions(LEFT_HOME_1)
        # 等待关节到达 home_1（所有关节误差 < 0.05 rad）
        start = time.time()
        while time.time() - start < 15.0:
            q6, q7, _, _ = read_joint67(joint_state_cache, rcl_node, timeout_sec=0.5)
            if q6 is not None and q7 is not None:
                # 读取全部7个关节
                rclpy.spin_once(rcl_node, timeout_sec=0.1)
                if joint_state_cache.msg is not None:
                    name_to_pos = dict(zip(joint_state_cache.msg.name, joint_state_cache.msg.position))
                    left_joints = [name_to_pos.get(f"left_joint{i}") for i in range(1, 8)]
                    if None not in left_joints:
                        errors = [abs(left_joints[i] - LEFT_HOME_1[i]) for i in range(7)]
                        max_err = max(errors)
                        if max_err < 0.05:
                            log.log(f"    ✓ MoveJ 完成 (max_joint_err={max_err:.4f} rad)")
                            return True
            time.sleep(0.2)
        log.log("    ⚠ MoveJ 超时，关节未完全到达 home_1")
        return False
    except Exception as e:
        log.log(f"    ✗ MoveJ 失败: {e}")
        return False


def switch_to_ocs2(interface, log):
    log.log("  切换到 OCS2 状态...")
    interface.send_fsm_command(2)
    time.sleep(1.0)
    interface.send_fsm_command(3)
    time.sleep(1.0)
    log.log("    ✓ 已切换到 OCS2")


def main():
    parser = argparse.ArgumentParser(description="Joint67 Coupling Constraint A/B Test")
    parser.add_argument("--label", default="A", choices=["A", "B"],
                        help="Run label: A=with constraint, B=without constraint")
    args = parser.parse_args()

    run_label = args.label

    result_file = os.path.join(RESULT_DIR, f"joint67_coupling_result_{run_label}.txt")
    log = ResultLogger(result_file)

    if not rclpy.ok():
        rclpy.init()
    rcl_node = Node("joint67_coupling_test")
    joint_state_cache = JointStateCache()
    rcl_node.create_subscription(JointState, "/joint_states", joint_state_cache.callback, 10)
    interface = None

    def cleanup():
        if interface:
            interface.disconnect()
        rcl_node.destroy_node()
        rclpy.shutdown()

    log.log("")
    log.log("=" * 70)
    log.log(f"  Joint67 Coupling Constraint Test  [Run {run_label}]")
    log.log(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    if run_label == "A":
        log.log("  Mode: WITH joint67Coupling cost (constraint active)")
    else:
        log.log("  Mode: WITHOUT joint67Coupling cost (constraint inactive)")
    log.log(f"  Test cases: {len(TEST_CASES)}")
    log.log("=" * 70)
    log.log("")

    log.log("[1] 创建接口并连接...")
    config = ROS2RobotInterfaceConfig()
    interface = ROS2RobotInterface(config)
    try:
        interface.connect()
        log.log("    ✓ 连接成功")
    except Exception as e:
        log.log(f"    ✗ 连接失败: {e}")
        cleanup()
        return 1

    log.log("[2] 等待数据到达（2秒）...")
    time.sleep(2.0)

    switch_to_ocs2(interface, log)
    log.log("")

    frame_id = "arm_base"

    results = []

    for idx, case in enumerate(TEST_CASES, start=1):
        # 每个点作为一次独立实验：先回 home，再切到 OCS2，再走目标
        movej_to_home(interface, log, joint_state_cache, rcl_node)
        switch_to_ocs2(interface, log)

        tag = case["tag"]
        px, py, pz = case["pos"]
        base_roll, base_pitch, base_yaw = case["rpy_deg"]
        delta_roll, delta_pitch, delta_yaw = case["wrist_rpy_deg"]

        log.log(f"[{idx}] {tag}")

        base_q = euler_rpy_to_quat_xyzw(
            math.radians(base_roll),
            math.radians(base_pitch),
            math.radians(base_yaw),
        )
        delta_q = euler_rpy_to_quat_xyzw(
            math.radians(delta_roll),
            math.radians(delta_pitch),
            math.radians(delta_yaw),
        )
        new_q = quat_normalize(quat_multiply(base_q, delta_q))
        left_target = create_pose(px, py, pz, qx=new_q[0], qy=new_q[1], qz=new_q[2], qw=new_q[3])
        if pose_has_nan(left_target):
            log.log("  ⚠ 目标位姿包含 NaN/Inf，跳过")
            continue

        try:
            interface.left_arm_handler.send_target_stamped(frame_id, left_target)
        except Exception as e:
            log.log(f"  ✗ 发送失败: {e}")
            continue

        arrived, best = wait_for_pos_convergence(interface, left_target)
        pos_err = best["pos_err"]
        angle_err = best["ang_err"]
        dx, dy, dz = best["dx"], best["dy"], best["dz"]

        q6_l, q7_l, _, _ = read_joint67(joint_state_cache, rcl_node)
        if q6_l is None or q7_l is None:
            results.append((idx, "Left", tag, None, None, None, None, "NO_DATA", arrived, pos_err, angle_err, dx, dy, dz))
            log.log(f"  result: NO_DATA arrived={arrived}")
        else:
            value, limit = eval_constraint(q6_l, q7_l)
            status = "OUTSIDE" if value > 0.0 else "INSIDE"
            q6_actual_deg = math.degrees(q6_l)
            q7_actual_deg = math.degrees(q7_l)
            results.append((idx, "Left", tag, q6_l, q7_l, limit, value, status, arrived, pos_err, angle_err, dx, dy, dz))
            pose_str = "pose=N/A" if pos_err is None else f"min_pos={pos_err:.4f}m ang={angle_err:.2f}°"
            log.log(
                f"  result: {status} arrived={arrived} q6={q6_actual_deg:+.1f}° q7={q7_actual_deg:+.1f}° {pose_str}"
            )
        log.log("")

    movej_to_home(interface, log, joint_state_cache, rcl_node)

    log.log("")
    log.log("=" * 70)
    log.log(f"  Summary [Run {run_label}]")
    log.log("=" * 70)

    inside_count = 0
    outside_count = 0
    for item in results:
        step, arm_label, tag, q6, q7, limit, value, status, arrived, pos_err, angle_err, dx, dy, dz = item
        if status == "NO_DATA":
            log.log(f"  #{step} {arm_label:5s} {tag:40s} NO_DATA")
            continue
        arrive_str = "ARRIVED" if arrived else "TIMEOUT"
        q6_deg = math.degrees(q6)
        q7_deg = math.degrees(q7)
        limit_deg = math.degrees(limit)
        if pos_err is not None and angle_err is not None and dx is not None and dy is not None and dz is not None:
            pose_err_str = f"pos={pos_err:.4f}m ang={angle_err:.2f}° Δx={dx:+.4f} Δy={dy:+.4f} Δz={dz:+.4f}"
        else:
            pose_err_str = "pose_err=N/A"
        log.log(
            f"  #{step} {arm_label:5s} {tag:40s} "
            f"q6={q6_deg:+6.1f}° q7={q7_deg:+6.1f}° "
            f"limit={limit_deg:+.1f}° margin={-value:+.3f}rad "
            f"{status:7s} {arrive_str} {pose_err_str}"
        )
        if status == "INSIDE":
            inside_count += 1
        else:
            outside_count += 1

    total = inside_count + outside_count
    log.log("")
    log.log(f"  INSIDE: {inside_count}/{total}  OUTSIDE: {outside_count}/{total}")
    if run_label == "A":
        log.log("  (Run A: joint67Coupling cost 应为 ACTIVE)")
        if outside_count > 0:
            log.log("  ⚠ 约束生效但仍有越界，可能需要增大 barrier mu 或降低激活阈值")
    else:
        log.log("  (Run B: joint67Coupling cost 应为 INACTIVE)")
        if outside_count == 0:
            log.log("  ⚠ 无约束时全部在界内，测试点可能不够极端")
    log.log("")

    log.save()
    cleanup()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断测试")
        sys.exit(1)
