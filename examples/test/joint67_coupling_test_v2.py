"""
Joint67 Coupling Constraint Test V2 - 真实抓取点位测试

使用实际抓取流程中的笛卡尔位姿测试 joint67 耦合约束。
数据来自 TEST_POINT_DATA，包含 3 个 case，每个 case 有不同的
home 姿态和抓取序列位姿（arm_base 坐标系下）。

用法:
  Run A (有约束):   python joint67_coupling_test_v2.py --label A
  Run B (无约束):   python joint67_coupling_test_v2.py --label B

  运行前需在 task file 中设置 joint67Coupling.activate = true (A) 或 false (B)，
  然后重启 OCS2 控制器。

  MoveJ 左臂 joint6/7 耦合域外探测见:
  movej_left_outside_joint67_coupling_test.py
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
try:
    from pal_statistics_msgs.msg import Statistics
except Exception:  # pragma: no cover
    Statistics = None

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.quat_pose import (
    quat_multiply,
    quat_normalize,
    quat_conjugate,
)

MAX_WAIT = 15.0
WAIT_INTERVAL = 0.5

J7_KNEE_DEG = 49.0
J7_MAX_DEG = 90.0
J6_AT_KNEE_DEG = 60.0
J6_AT_MAX_DEG = 20.0
SMOOTH_ABS_EPS = 1e-6

RESULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

TEST_POINT_DATA = {
    "case1": {
        "home_pose": {
            "home_body_deg": [-77.5, -148.91, -72.92, -90],
            "home_left_arm_joints_deg": [-16.77, 51.70, 34.29, -130.65, -63.55, -12.70, -12.39],
            "home_right_arm_joints_deg": [16.77, 51.70, -34.29, -130.65, 63.55, -12.70, 12.39],
        },
        "bad_pose_list_in_arm_base": {
            "中间过渡点": {
                "left": [0.555324930038287, 0.4500695585562205, -0.24677930553994404,
                         -0.6962651900368355, -0.007998702427492232, -0.7167482092021397, 0.03772016049769028],
                "right": [0.5579010606482925, -0.3228652333697543, -0.25993415911756496,
                          -0.7001239693180322, -0.014117618042795654, -0.7115563245344396, 0.05757358302381377],
            },
            "手臂两侧的斜上方位": {
                "left": [0.6960418886570918, 0.40081385214590826, -0.36753393160362274,
                         -0.6962651900368355, -0.007998702427492232, -0.7167482092021397, 0.03772016049769028],
                "right": [0.7011949705056818, -0.14640929384041107, -0.3938438705613251,
                          -0.7001239693180322, -0.014117618042795654, -0.7115563245344396, 0.05757358302381377],
            },
            "抓取点位侧边": {
                "left": [0.6960418886570918, 0.40081385214590826, -0.38753393160362276,
                         -0.6962651900368355, -0.007998702427492232, -0.7167482092021397, 0.03772016049769028],
                "right": [0.7011949705056818, -0.14640929384041107, -0.4138438705613251,
                          -0.7001239693180322, -0.014117618042795654, -0.7115563245344396, 0.05757358302381377],
            },
            "最终抓取": {
                "left": [0.6960418886570918, 0.37081385214590823, -0.38753393160362276,
                         -0.6962651900368355, -0.007998702427492232, -0.7167482092021397, 0.03772016049769028],
                "right": [0.7011949705056818, -0.11640929384041107, -0.4138438705613251,
                          -0.7001239693180322, -0.014117618042795654, -0.7115563245344396, 0.05757358302381377],
            },
        },
    },
    "case2": {
        "home_pose": {
            "home_body_deg": [-77.41, -125.01, -96.42, -180],
            "home_left_arm_joints_deg": [-89.57, 71.04, 61.10, -73.45, -49.25, -42.19, -48.24],
            "home_right_arm_joints_deg": [89.57, 71.04, -61.10, -73.45, 49.25, -42.19, 48.24],
        },
        "bad_pose_list_in_arm_base": {
            "中间过渡点": {
                "left": [0.5522945695684758, 0.4284658499178163, -0.08840533928645031,
                         -0.33150327523790857, 0.01941148321250183, -0.9432489513086809, 0.003191971350714671],
                "right": [0.5465990139251653, -0.516557907458329, -0.10820694786599203,
                          -0.33672145691951016, 0.0029493360074670724, -0.9413940098540963, 0.019679432866400746],
            },
            "手臂两侧的斜上方位": {
                "left": [0.5950748089513619, 0.28397959199087097, -0.062203180867463924,
                         -0.33150327523790857, 0.01941148321250183, -0.9432489513086809, 0.003191971350714671],
                "right": [0.5836895543604468, -0.4601630353691911, -0.10180882379833843,
                          -0.33672145691951016, 0.0029493360074670724, -0.9413940098540963, 0.019679432866400746],
            },
            "抓取点位侧边": {
                "left": [0.6101300028460694, 0.28397959199087097, -0.07536908899386226,
                         -0.33150327523790857, 0.01941148321250183, -0.9432489513086809, 0.003191971350714671],
                "right": [0.5987447482551543, -0.4601630353691911, -0.11497473192473676,
                          -0.33672145691951016, 0.0029493360074670724, -0.9413940098540963, 0.019679432866400746],
            },
            "最终抓取": {
                "left": [0.6101300028460694, 0.253979591990871, -0.07536908899386226,
                         -0.33150327523790857, 0.01941148321250183, -0.9432489513086809, 0.003191971350714671],
                "right": [0.5987447482551543, -0.43016303536919115, -0.11497473192473676,
                          -0.33672145691951016, 0.0029493360074670724, -0.9413940098540963, 0.019679432866400746],
            },
        },
    },
    "case3": {
        "home_pose": {
            "home_body_deg": [-77.41, -125.01, -96.42, -180],
            "home_left_arm_joints_deg": [-89.57, 71.04, 61.10, -73.45, -49.25, -42.19, -48.24],
            "home_right_arm_joints_deg": [89.57, 71.04, -61.10, -73.45, 49.25, -42.19, 48.24],
        },
        "bad_pose_list_in_arm_base": {
            "抬起夹爪": {
                "left": [0.5634142612156462, 0.34922828068452777, -0.054159575975871115,
                         -0.3341377362271211, 0.014210470206948034, -0.9424144304589461, 0.0022532261876227885],
                "right": [0.5469366526692548, -0.33271108720237424, -0.09444973361244274,
                          -0.3391990869130158, -0.0022519641904613216, -0.9405241490533063, 0.018794497657968198],
            },
        },
    },
}


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


def create_pose_from_list(pose_list):
    pose = Pose()
    pose.position = Point(x=pose_list[0], y=pose_list[1], z=pose_list[2])
    pose.orientation = Quaternion(x=pose_list[3], y=pose_list[4], z=pose_list[5], w=pose_list[6])
    return pose


def pose_has_nan(pose: Pose) -> bool:
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    )
    return any(math.isnan(v) or math.isinf(v) for v in values)


def quat_to_axis_angle(q):
    qw = q[3]
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
    if sin_half < 1e-8:
        return 0.0
    return 2.0 * math.atan2(sin_half, qw)


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
    angle_err_deg = math.degrees(quat_to_axis_angle(q_diff))
    return pos_err, angle_err_deg


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


class IntrospectionCache:
    def __init__(self):
        self.name_to_value = {}

    def callback(self, msg):
        try:
            self.name_to_value = {s.name: float(s.value) for s in msg.statistics}
        except Exception:
            return


_INTRO_KEYS = (
    "state_interface.left_joint6/position",
    "command_interface.left_joint6/position",
    "state_interface.right_joint6/position",
    "command_interface.right_joint7/position",
)


def read_selected_introspection(cache: IntrospectionCache) -> dict:
    ntv = cache.name_to_value or {}
    out = {k: ntv.get(k) for k in _INTRO_KEYS}
    s6, c6 = out[_INTRO_KEYS[0]], out[_INTRO_KEYS[1]]
    if s6 is not None and c6 is not None:
        out["delta_left6_cmd_minus_state_deg"] = math.degrees(c6 - s6)
    return out


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


def read_all_joints(cache: JointStateCache, node: Node, prefix="left"):
    rclpy.spin_once(node, timeout_sec=0.1)
    if cache.msg is None:
        return None
    name_to_pos = dict(zip(cache.msg.name, cache.msg.position))
    joints = [name_to_pos.get(f"{prefix}_joint{i}") for i in range(1, 8)]
    if None in joints:
        return None
    return joints


def wait_for_arrival(interface, max_wait=MAX_WAIT, interval=WAIT_INTERVAL, threshold=0.005):
    start = time.time()
    while time.time() - start < max_wait:
        left_ok = interface.left_arm_handler.check_arrival(pose_threshold=threshold)["arrived"]
        right_ok = interface.right_arm_handler.check_arrival(pose_threshold=threshold)["arrived"]
        if left_ok and right_ok:
            return True
        time.sleep(interval)
    return False


def _wait_movej_arm(interface, log, joint_state_cache, rcl_node, prefix, home_rad, start_total, timeout_total):
    handler = interface.left_arm_handler if prefix == "left" else interface.right_arm_handler
    handler.send_joint_positions(home_rad)
    side = "Left" if prefix == "left" else "Right"
    while time.time() - start_total < timeout_total:
        joints = read_all_joints(joint_state_cache, rcl_node, prefix)
        if joints is not None:
            err = max(abs(joints[i] - home_rad[i]) for i in range(7))
            if err < 0.05:
                log.log(f"    ✓ {side} MoveJ 到位 (max_err={err:.4f} rad)")
                return True
        time.sleep(0.2)
    log.log(f"    ⚠ {side} MoveJ 超时")
    return False


def _append_movej_joint67_record(records, label, note, ok, cmd_l, cmd_r, act_l=None, act_r=None):
    if not isinstance(records, list):
        return
    if not ok:
        records.append({"label": label, "note": note, "ok": False})
        return
    q6_l, q7_l = act_l
    q6_r, q7_r = act_r
    records.append({
        "label": label,
        "note": note,
        "ok": True,
        "cmd_q6_l": cmd_l[0], "cmd_q7_l": cmd_l[1],
        "act_q6_l": q6_l, "act_q7_l": q7_l,
        "cmd_q6_r": cmd_r[0], "cmd_q7_r": cmd_r[1],
        "act_q6_r": q6_r, "act_q7_r": q7_r,
        "d_q6_l_deg": math.degrees(q6_l - cmd_l[0]),
        "d_q7_l_deg": math.degrees(q7_l - cmd_l[1]),
        "d_q6_r_deg": math.degrees(q6_r - cmd_r[0]),
        "d_q7_r_deg": math.degrees(q7_r - cmd_r[1]),
    })


def _log_movej_joint67(interface, log, joint_state_cache, rcl_node, records, label, note, cmd_l, cmd_r):
    q6_l, q7_l, q6_r, q7_r = read_joint67(joint_state_cache, rcl_node, timeout_sec=1.0)
    if None in (q6_l, q7_l, q6_r, q7_r):
        log.log(f"    ⚠ MoveJ joint6/7 读取失败（{note}）")
        _append_movej_joint67_record(records, label, note, False, cmd_l, cmd_r)
        return
    for side, c6, c7, a6, a7 in (
        ("Left ", cmd_l[0], cmd_l[1], q6_l, q7_l),
        ("Right", cmd_r[0], cmd_r[1], q6_r, q7_r),
    ):
        log.log(
            f"    {side} j67 cmd q6,q7={math.degrees(c6):+.1f}°,{math.degrees(c7):+.1f}° | "
            f"act={math.degrees(a6):+.1f}°,{math.degrees(a7):+.1f}° | "
            f"Δ={math.degrees(a6 - c6):+.1f}°,{math.degrees(a7 - c7):+.1f}° ({note})"
        )
    _append_movej_joint67_record(records, label, note, True, cmd_l, cmd_r, (q6_l, q7_l), (q6_r, q7_r))


def movej_to_home(interface, log, joint_state_cache, rcl_node, home_left_deg, home_right_deg,
                  movej_joint67_records=None, label: str = ""):
    home_left_rad = [math.radians(d) for d in home_left_deg]
    home_right_rad = [math.radians(d) for d in home_right_deg]
    cmd_l = (home_left_rad[5], home_left_rad[6])
    cmd_r = (home_right_rad[5], home_right_rad[6])
    log.log("  MoveJ 回 home...")
    try:
        interface.send_fsm_command(4)
        time.sleep(1.0)
        t0 = time.time()
        timeout_total = 15.0
        if not _wait_movej_arm(interface, log, joint_state_cache, rcl_node, "left", home_left_rad, t0, timeout_total):
            _log_movej_joint67(interface, log, joint_state_cache, rcl_node, movej_joint67_records, label, "Left 超时", cmd_l, cmd_r)
            return False
        if not _wait_movej_arm(interface, log, joint_state_cache, rcl_node, "right", home_right_rad, t0, timeout_total):
            _log_movej_joint67(interface, log, joint_state_cache, rcl_node, movej_joint67_records, label, "Right 超时", cmd_l, cmd_r)
            return False
        _log_movej_joint67(interface, log, joint_state_cache, rcl_node, movej_joint67_records, label, "完成", cmd_l, cmd_r)
        return True
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


def evaluate_arm(log, arm_label, q6, q7, target_pose, actual_pose):
    result = {
        'q6': q6, 'q7': q7, 'limit': None, 'margin': None,
        'status': 'NO_DATA', 'pos_err': None, 'ang_err': None,
    }
    if q6 is None or q7 is None:
        log.log(f"  {arm_label}: 未读取到 joint6/7")
        return result

    value, limit = eval_constraint(q6, q7)
    status = "OUTSIDE" if value > 0.0 else "INSIDE"
    q6_deg = math.degrees(q6)
    q7_deg = math.degrees(q7)
    margin_deg = math.degrees(-value)

    log.log(f"  {arm_label} 到达关节角度: q6={q6_deg:+6.1f}° q7={q7_deg:+6.1f}°")
    log.log(f"  {arm_label} 约束边界: |q7|={abs(q7_deg):.1f}° → |q6|≤{math.degrees(limit):.1f}°")
    log.log(f"  {arm_label} 距离边界差距: {margin_deg:+.1f}° ({-value:+.3f} rad) -> {status}")

    result['limit'] = limit
    result['margin'] = -value
    result['status'] = status

    if target_pose is not None and actual_pose is not None:
        pos_err, ang_err = compute_pose_error(target_pose, actual_pose)
        tp, ap = target_pose.position, actual_pose.position
        dx, dy, dz = tp.x - ap.x, tp.y - ap.y, tp.z - ap.z
        tq, aq = target_pose.orientation, actual_pose.orientation
        log.log(
            f"  {arm_label} 笛卡尔: |pos|={pos_err:.4f}m (Δx,Δy,Δz)=({dx:+.4f},{dy:+.4f},{dz:+.4f}) ang={ang_err:.2f}°"
        )
        log.log(
            f"  {arm_label}   q_tgt=({tq.x:.4f},{tq.y:.4f},{tq.z:.4f},{tq.w:.4f}) "
            f"q_act=({aq.x:.4f},{aq.y:.4f},{aq.z:.4f},{aq.w:.4f})"
        )
        result['pos_err'] = pos_err
        result['ang_err'] = ang_err
        result['dx'] = dx
        result['dy'] = dy
        result['dz'] = dz

    return result


def _format_summary_intro(intro: dict) -> str:
    s_l6, c_l6, s_r6, c_r7 = (intro.get(k) for k in _INTRO_KEYS)
    if not all(v is not None for v in (s_l6, c_l6, s_r6, c_r7)):
        return ""
    d_l6 = intro.get("delta_left6_cmd_minus_state_deg")
    if d_l6 is None:
        d_l6 = math.degrees(c_l6 - s_l6)
    return (
        f" intro[L6 s={math.degrees(s_l6):+.1f}° c={math.degrees(c_l6):+.1f}° Δc-s={d_l6:+.1f}°"
        f" R6 s={math.degrees(s_r6):+.1f}° R7 c={math.degrees(c_r7):+.1f}°]"
    )


def main():
    parser = argparse.ArgumentParser(description="Joint67 Coupling Constraint Test V2 - 真实抓取点位")
    parser.add_argument("--label", default="A", choices=["A", "B"],
                        help="Run label: A=with constraint, B=without constraint")
    args = parser.parse_args()
    run_label = args.label

    result_file = os.path.join(RESULT_DIR, f"joint67_coupling_result_v2_{run_label}.txt")
    log = ResultLogger(result_file)

    if not rclpy.ok():
        rclpy.init()
    rcl_node = Node("joint67_coupling_test_v2")
    joint_state_cache = JointStateCache()
    rcl_node.create_subscription(JointState, "/joint_states", joint_state_cache.callback, 10)
    introspection_cache = IntrospectionCache()
    if Statistics is not None:
        rcl_node.create_subscription(Statistics, "/controller_manager/introspection_data/full", introspection_cache.callback, 10)
    interface = None

    def cleanup():
        if interface:
            interface.disconnect()
        rcl_node.destroy_node()
        rclpy.shutdown()

    log.log("")
    log.log("=" * 70)
    log.log(f"  Joint67 Coupling Constraint Test V2  [Run {run_label}]")
    log.log(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    if run_label == "A":
        log.log("  Mode: WITH joint67Coupling cost (constraint active)")
    else:
        log.log("  Mode: WITHOUT joint67Coupling cost (constraint inactive)")
    log.log(f"  Cases: {len(TEST_POINT_DATA)}")
    total_poses = sum(len(v['bad_pose_list_in_arm_base']) for v in TEST_POINT_DATA.values())
    log.log(f"  Total poses: {total_poses}")
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
    movej_joint67_records = []
    test_idx = 0

    for case_name, case_data in TEST_POINT_DATA.items():
        home = case_data["home_pose"]
        home_left_deg = home["home_left_arm_joints_deg"]
        home_right_deg = home["home_right_arm_joints_deg"]
        pose_list = case_data["bad_pose_list_in_arm_base"]

        log.log(f"===== Case: {case_name} =====")
        log.log(f"  Home 左臂: {[f'{d:.1f}°' for d in home_left_deg]}")
        log.log(f"  Home 右臂: {[f'{d:.1f}°' for d in home_right_deg]}")
        log.log(f"  测试点位数: {len(pose_list)}")
        log.log("")

        movej_to_home(
            interface, log, joint_state_cache, rcl_node, home_left_deg, home_right_deg,
            movej_joint67_records=movej_joint67_records, label=f"{case_name}|home"
        )
        switch_to_ocs2(interface, log)

        for pose_name, pose_data in pose_list.items():
            test_idx += 1
            tag = f"{case_name}|{pose_name}"
            log.log(f"[{test_idx}] 测试点: {tag}")
            log.log("-" * 60)

            left_pose_list = pose_data["left"]
            right_pose_list = pose_data["right"]

            left_target = create_pose_from_list(left_pose_list)
            right_target = create_pose_from_list(right_pose_list)

            if pose_has_nan(left_target) or pose_has_nan(right_target):
                log.log("  ⚠ 目标位姿包含 NaN/Inf，跳过")
                continue

            log.log(f"  左臂目标: x={left_pose_list[0]:.3f} y={left_pose_list[1]:.3f} z={left_pose_list[2]:.3f}")
            log.log(f"  右臂目标: x={right_pose_list[0]:.3f} y={right_pose_list[1]:.3f} z={right_pose_list[2]:.3f}")

            try:
                interface.left_arm_handler.send_target_stamped(frame_id, left_target)
                interface.right_arm_handler.send_target_stamped(frame_id, right_target)
            except Exception as e:
                log.log(f"  ✗ 发送失败: {e}")
                continue

            log.log("  等待 OCS2 规划并稳定...")
            arrived = wait_for_arrival(interface)
            log.log(f"  {'✓ 已到达' if arrived else '⚠ 超时'}")

            time.sleep(1.0)

            left_actual = interface.left_arm_handler.get_pose()
            right_actual = interface.right_arm_handler.get_pose()

            q6_l, q7_l, q6_r, q7_r = read_joint67(joint_state_cache, rcl_node)
            rclpy.spin_once(rcl_node, timeout_sec=0.0)
            selected_intro = read_selected_introspection(introspection_cache)

            for prefix, joints in (("左臂", read_all_joints(joint_state_cache, rcl_node, "left")),
                                   ("右臂", read_all_joints(joint_state_cache, rcl_node, "right"))):
                if joints:
                    jstr = ", ".join(f"j{i}={math.degrees(joints[i - 1]):+.1f}°" for i in range(1, 8))
                    log.log(f"  {prefix}关节: {jstr}")

            left_result = evaluate_arm(log, "Left", q6_l, q7_l, left_target, left_actual)
            right_result = evaluate_arm(log, "Right", q6_r, q7_r, right_target, right_actual)
            left_result["intro"] = selected_intro
            right_result["intro"] = selected_intro

            results.append((test_idx, tag, "Left", left_result))
            results.append((test_idx, tag, "Right", right_result))
            log.log("")

    movej_to_home(
        interface, log, joint_state_cache, rcl_node,
        TEST_POINT_DATA["case1"]["home_pose"]["home_left_arm_joints_deg"],
        TEST_POINT_DATA["case1"]["home_pose"]["home_right_arm_joints_deg"],
        movej_joint67_records=movej_joint67_records, label="final|home"
    )

    log.log("")
    log.log("=" * 70)
    log.log(f"  Summary [Run {run_label}]")
    log.log("=" * 70)

    inside_count = 0
    outside_count = 0
    for item in results:
        step, tag, arm_label, result = item
        status = result['status']
        if status == "NO_DATA":
            log.log(f"  #{step} {arm_label:5s} {tag:40s} NO_DATA")
            continue
        q6_deg = math.degrees(result['q6'])
        q7_deg = math.degrees(result['q7'])
        limit_deg = math.degrees(result['limit'])
        margin_deg = math.degrees(result['margin'])
        if result['pos_err'] is not None:
            pose_err_str = f"pos_err={result['pos_err']:.4f}m ang_err={result['ang_err']:.2f}°"
            if 'dx' in result:
                pose_err_str += f" Δx={result['dx']:+.4f} Δy={result['dy']:+.4f} Δz={result['dz']:+.4f}"
        else:
            pose_err_str = "pose=N/A"
        intro_str = _format_summary_intro(result.get("intro") or {})
        log.log(
            f"  #{step} {arm_label:5s} {tag:40s} "
            f"q6={q6_deg:+6.1f}° q7={q7_deg:+6.1f}° "
            f"limit={limit_deg:+.1f}° margin={margin_deg:+.1f}° "
            f"{status:7s}{intro_str} {pose_err_str}"
        )
        if status == "INSIDE":
            inside_count += 1
        else:
            outside_count += 1

    total = inside_count + outside_count
    log.log("")
    log.log(f"  INSIDE: {inside_count}/{total}  OUTSIDE: {outside_count}/{total}")

    if movej_joint67_records:
        log.log("")
        log.log("  MoveJ joint67 cmd vs actual (home)")
        for rec in movej_joint67_records:
            tag = rec.get("label", "")
            note = rec.get("note", "")
            if not rec.get("ok", False):
                log.log(f"    {tag:20s} {note:16s}  NO_DATA")
                continue
            log.log(
                f"    {tag:20s} {note:16s}  "
                f"L: cmd(q6,q7)=({math.degrees(rec['cmd_q6_l']):+.1f}°, {math.degrees(rec['cmd_q7_l']):+.1f}°) "
                f"act=({math.degrees(rec['act_q6_l']):+.1f}°, {math.degrees(rec['act_q7_l']):+.1f}°) "
                f"Δ=({rec['d_q6_l_deg']:+.1f}°, {rec['d_q7_l_deg']:+.1f}°) | "
                f"R: cmd(q6,q7)=({math.degrees(rec['cmd_q6_r']):+.1f}°, {math.degrees(rec['cmd_q7_r']):+.1f}°) "
                f"act=({math.degrees(rec['act_q6_r']):+.1f}°, {math.degrees(rec['act_q7_r']):+.1f}°) "
                f"Δ=({rec['d_q6_r_deg']:+.1f}°, {rec['d_q7_r_deg']:+.1f}°)"
            )
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
