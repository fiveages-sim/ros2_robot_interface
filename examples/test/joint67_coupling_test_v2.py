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


def movej_to_home(interface, log, joint_state_cache, rcl_node, home_left_deg, home_right_deg):
    home_left_rad = [math.radians(d) for d in home_left_deg]
    home_right_rad = [math.radians(d) for d in home_right_deg]
    log.log("  MoveJ 回 home...")
    try:
        interface.send_fsm_command(4)
        time.sleep(1.0)
        interface.left_arm_handler.send_joint_positions(home_left_rad)
        interface.right_arm_handler.send_joint_positions(home_right_rad)
        start = time.time()
        while time.time() - start < 15.0:
            left_joints = read_all_joints(joint_state_cache, rcl_node, "left")
            right_joints = read_all_joints(joint_state_cache, rcl_node, "right")
            if left_joints is not None and right_joints is not None:
                left_err = max(abs(left_joints[i] - home_left_rad[i]) for i in range(7))
                right_err = max(abs(right_joints[i] - home_right_rad[i]) for i in range(7))
                if left_err < 0.05 and right_err < 0.05:
                    log.log(f"    ✓ MoveJ 完成 (L_max={left_err:.4f}, R_max={right_err:.4f} rad)")
                    return True
            time.sleep(0.2)
        log.log("    ⚠ MoveJ 超时")
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
        dx = target_pose.position.x - actual_pose.position.x
        dy = target_pose.position.y - actual_pose.position.y
        dz = target_pose.position.z - actual_pose.position.z

        tq = target_pose.orientation
        aq = actual_pose.orientation

        log.log(f"  {arm_label} === 笛卡尔空间误差 ===")
        log.log(f"  {arm_label}   目标位置: x={target_pose.position.x:.4f} y={target_pose.position.y:.4f} z={target_pose.position.z:.4f}")
        log.log(f"  {arm_label}   实际位置: x={actual_pose.position.x:.4f} y={actual_pose.position.y:.4f} z={actual_pose.position.z:.4f}")
        log.log(f"  {arm_label}   位置误差: Δx={dx:+.4f}m Δy={dy:+.4f}m Δz={dz:+.4f}m |总|={pos_err:.4f}m")
        log.log(f"  {arm_label}   目标姿态: qx={tq.x:.4f} qy={tq.y:.4f} qz={tq.z:.4f} qw={tq.w:.4f}")
        log.log(f"  {arm_label}   实际姿态: qx={aq.x:.4f} qy={aq.y:.4f} qz={aq.z:.4f} qw={aq.w:.4f}")
        log.log(f"  {arm_label}   姿态误差(角度): {ang_err:.2f}°")
        result['pos_err'] = pos_err
        result['ang_err'] = ang_err
        result['dx'] = dx
        result['dy'] = dy
        result['dz'] = dz

    return result


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

        movej_to_home(interface, log, joint_state_cache, rcl_node, home_left_deg, home_right_deg)
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

            left_joints = read_all_joints(joint_state_cache, rcl_node, "left")
            right_joints = read_all_joints(joint_state_cache, rcl_node, "right")
            if left_joints:
                log.log(f"  左臂关节: {', '.join(f'j{i}={math.degrees(left_joints[i-1]):+.1f}°' for i in range(1, 8))}")
            if right_joints:
                log.log(f"  右臂关节: {', '.join(f'j{i}={math.degrees(right_joints[i-1]):+.1f}°' for i in range(1, 8))}")

            left_result = evaluate_arm(log, "Left", q6_l, q7_l, left_target, left_actual)
            right_result = evaluate_arm(log, "Right", q6_r, q7_r, right_target, right_actual)

            results.append((test_idx, tag, "Left", left_result))
            results.append((test_idx, tag, "Right", right_result))
            log.log("")

    movej_to_home(interface, log, joint_state_cache, rcl_node,
                  TEST_POINT_DATA["case1"]["home_pose"]["home_left_arm_joints_deg"],
                  TEST_POINT_DATA["case1"]["home_pose"]["home_right_arm_joints_deg"])

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
        log.log(
            f"  #{step} {arm_label:5s} {tag:40s} "
            f"q6={q6_deg:+6.1f}° q7={q7_deg:+6.1f}° "
            f"limit={limit_deg:+.1f}° margin={margin_deg:+.1f}° "
            f"{status:7s} {pose_err_str}"
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
