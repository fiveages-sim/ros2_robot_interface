"""
Joint67 Coupling Constraint A/B Test

通过在不同空间位置上施加手腕旋转，构造使 joint6/7 违反耦合约束的
笛卡尔目标，测试 OCS2 在有/无 joint67Coupling cost 时的行为差异。

每个测试点：先 MoveJ 回 home_1，再 OCS2 移动到目标位置+姿态，
记录实际关节角度和位姿误差。

用法:
  Run A (有约束):   python joint67_coupling_test.py --label A
  Run B (无约束):   python joint67_coupling_test.py --label B

  运行前需在 task file 中设置 joint67Coupling.activate = true (A) 或 false (B)，
  然后重启 OCS2 控制器。

  结果自动保存到 joint67_coupling_result_A.txt / joint67_coupling_result_B.txt
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

TEST_POSITIONS = [
    ("front_left_low",  0.50, 0.45, -0.30),
    ("front_left_mid",  0.60, 0.50, -0.30),
    ("front_left_far",  0.70, 0.55, -0.30),
]

# 基础朝向定义 (label, roll, pitch, yaw) in degrees
# 这些是绝对朝向，不是基于 home_1 的相对旋转
BASE_ORIENTATIONS = [
    ("forward", 0.0, 90.0, 0.0),   # 末端朝前
    ("left",    0.0, 90.0, 90.0),  # 末端朝左
]

# 手腕旋转叠加 (label, delta_roll, delta_pitch, delta_yaw)
WRIST_ROTATIONS_DEG = [
    ("none",  0.0, 0.0, 0.0),
    ("roll+", 60.0, 0.0, 0.0),
]

MAX_WAIT = 10.0
WAIT_INTERVAL = 0.5
MOVEJ_SETTLE = 2.0

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


def wait_for_arrival(interface, max_wait=MAX_WAIT, interval=WAIT_INTERVAL, threshold=0.005):
    start = time.time()
    while time.time() - start < max_wait:
        left_ok = interface.left_arm_handler.check_arrival(pose_threshold=threshold)["arrived"]
        if left_ok:
            return True
        time.sleep(interval)
    return False


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


def save_point_analysis(log, test_points, results):
    """保存点位分析和约束边界信息"""
    analysis_file = os.path.join(RESULT_DIR, "joint67_coupling_point_analysis.txt")
    
    with open(analysis_file, 'w', encoding='utf-8') as f:
        f.write("Joint67 Coupling 约束点位分析报告\n")
        f.write("=" * 60 + "\n\n")
        
        f.write("约束边界定义：\n")
        f.write(f"  |q7| ≤ {J7_KNEE_DEG}°  →  |q6| ≤ {J6_AT_KNEE_DEG}°\n")
        f.write(f"  |q7| > {J7_KNEE_DEG}°  →  |q6| ≤ {J6_AT_KNEE_DEG}° - 0.8×(|q7| - {J7_KNEE_DEG}°)\n")
        f.write(f"  |q7| ≥ {J7_MAX_DEG}°  →  |q6| ≤ {J6_AT_MAX_DEG}°\n\n")
        
        f.write("测试点位详细分析：\n")
        f.write("-" * 60 + "\n\n")
        
        for i, (label, pos_label, ori_label, wrist_label) in enumerate(test_points):
            if i < len(results):
                result = results[i]
                f.write(f"点位 #{i+1}: {label}\n")
                f.write(f"  位置: {pos_label}\n")
                f.write(f"  朝向: {ori_label}\n")
                f.write(f"  手腕: {wrist_label}\n")
                
                if result['q6'] is not None and result['q7'] is not None:
                    q6_deg = math.degrees(result['q6'])
                    q7_deg = math.degrees(result['q7'])
                    abs_q7 = abs(q7_deg)
                    limit = math.degrees(compute_limit(abs(math.radians(abs_q7))))
                    margin = math.degrees(result['margin'])
                    
                    f.write(f"  实际关节角度: q6={q6_deg:+.1f}°, q7={q7_deg:+.1f}°\n")
                    f.write(f"  约束边界: |q7|={abs_q7:.1f}° → |q6|≤{limit:.1f}°\n")
                    f.write(f"  越界量: {margin:+.1f}° ({result['margin']:+.3f} rad)\n")
                    f.write(f"  状态: {'INSIDE' if margin >= 0 else 'OUTSIDE'}\n")
                    f.write(f"  位姿误差: 位置={result['pos_err']:.4f}m, 朝向={result['ang_err']:.2f}°\n")
                else:
                    f.write("  未获取到关节数据\n")
                f.write("\n")
        
        # 统计信息
        inside_count = sum(1 for r in results if r.get('margin', -1) >= 0)
        outside_count = len(results) - inside_count
        
        f.write("统计信息：\n")
        f.write("-" * 60 + "\n")
        f.write(f"总测试点数: {len(results)}\n")
        f.write(f"INSIDE 约束: {inside_count}\n")
        f.write(f"OUTSIDE 约束: {outside_count}\n")
        
        if results:
            avg_q6 = sum(math.degrees(r['q6']) for r in results if r['q6'] is not None) / len(results)
            avg_q7 = sum(math.degrees(r['q7']) for r in results if r['q7'] is not None) / len(results)
            avg_margin = sum(math.degrees(r['margin']) for r in results if r['margin'] is not None) / len(results)
            
            f.write(f"平均 q6: {avg_q6:.1f}°\n")
            f.write(f"平均 q7: {avg_q7:.1f}°\n")
            f.write(f"平均越界量: {avg_margin:.1f}°\n")
    
    log.log(f"点位分析已保存到: {analysis_file}")
    return analysis_file


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
    log.log(f"  Positions: {len(TEST_POSITIONS)}  Orientations: {len(BASE_ORIENTATIONS)}  WristRots: {len(WRIST_ROTATIONS_DEG)}")
    log.log(f"  Total test cases: {len(TEST_POSITIONS) * len(BASE_ORIENTATIONS) * len(WRIST_ROTATIONS_DEG)}")
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
    test_points = []  # 新增：存储测试点信息
    test_idx = 0

    for pos_label, px, py, pz in TEST_POSITIONS:
        for orient_label, base_roll, base_pitch, base_yaw in BASE_ORIENTATIONS:
            for rot_label, delta_roll, delta_pitch, delta_yaw in WRIST_ROTATIONS_DEG:
                test_idx += 1
                tag = f"{pos_label}|{orient_label}|{rot_label}"
                
                # 添加到 test_points
                test_points.append((tag, pos_label, orient_label, rot_label))
                
                log.log(f"[4.{test_idx}] 测试点: {tag}")
                log.log("-" * 60)

                movej_to_home(interface, log, joint_state_cache, rcl_node)
                switch_to_ocs2(interface, log)

                # 计算基础朝向四元数
                base_q = euler_rpy_to_quat_xyzw(
                    math.radians(base_roll),
                    math.radians(base_pitch),
                    math.radians(base_yaw),
                )
                # 计算手腕旋转四元数
                delta_q = euler_rpy_to_quat_xyzw(
                    math.radians(delta_roll),
                    math.radians(delta_pitch),
                    math.radians(delta_yaw),
                )
                # 叠加：基础朝向 * 手腕旋转
                new_q = quat_normalize(quat_multiply(base_q, delta_q))

                left_target = create_pose(
                    px, py, pz,
                    qx=new_q[0], qy=new_q[1], qz=new_q[2], qw=new_q[3],
                )

                if pose_has_nan(left_target):
                    log.log("  ⚠ 目标位姿包含 NaN/Inf，跳过")
                    continue

                log.log(f"  发送目标位姿 (位置={pos_label}[{px:.2f},{py:.2f},{pz:.2f}], 朝向={orient_label}, 手腕={rot_label})...")
                
                # 打印详细的笛卡尔空间信息
                log.log(f"    笛卡尔坐标: x={px:.3f}m, y={py:.3f}m, z={pz:.3f}m")
                log.log(f"    基础朝向: roll={base_roll:.1f}°, pitch={base_pitch:.1f}°, yaw={base_yaw:.1f}°")
                log.log(f"    手腕旋转: Δroll={delta_roll:.1f}°, Δpitch={delta_pitch:.1f}°, Δyaw={delta_yaw:.1f}°")
                
                try:
                    interface.left_arm_handler.send_target_stamped(frame_id, left_target)
                except Exception as e:
                    log.log(f"  ✗ 发送失败: {e}")
                    continue

            log.log("  等待 OCS2 规划并稳定...")
            arrived = wait_for_arrival(interface)
            log.log(f"  {'✓ 已到达' if arrived else '⚠ 超时'}")

            time.sleep(1.0)

            # 获取实际到达的笛卡尔位置
            left_actual = interface.left_arm_handler.get_pose()
            if left_actual:
                pos_err, angle_err = compute_pose_error(left_target, left_actual)
                
                # 详细记录笛卡尔空间信息
                log.log(f"  目标笛卡尔位置: x={left_target.position.x:.3f}m, y={left_target.position.y:.3f}m, z={left_target.position.z:.3f}m")
                log.log(f"  实际到达位置: x={left_actual.position.x:.3f}m, y={left_actual.position.y:.3f}m, z={left_actual.position.z:.3f}m")
                log.log(f"  位姿误差: 位置={pos_err:.4f}m  朝向={angle_err:.2f}°")
            else:
                pos_err, angle_err = None, None
                log.log("  ⚠ 无法获取实际位姿")

            q6_l, q7_l, _, _ = read_joint67(joint_state_cache, rcl_node)

            # 读取全部7个关节角度
            rclpy.spin_once(rcl_node, timeout_sec=0.1)
            all_joints_str = ""
            if joint_state_cache.msg is not None:
                name_to_pos = dict(zip(joint_state_cache.msg.name, joint_state_cache.msg.position))
                left_joints = [name_to_pos.get(f"left_joint{i}") for i in range(1, 8)]
                if None not in left_joints:
                    all_joints_str = ", ".join(f"j{i}={math.degrees(left_joints[i-1]):+.1f}°" for i in range(1, 8))
                    log.log(f"  实际关节角度: {all_joints_str}")

            if q6_l is None or q7_l is None:
                log.log("  Left: 未读取到 joint6/7")
                results.append((test_idx, "Left", tag, None, None, None, None, "NO_DATA", arrived, pos_err, angle_err))
            else:
                value, limit = eval_constraint(q6_l, q7_l)
                status = "OUTSIDE" if value > 0.0 else "INSIDE"
                q6_actual_deg = math.degrees(q6_l)
                q7_actual_deg = math.degrees(q7_l)
                margin_deg = math.degrees(-value)
                
                # 详细记录关节约束信息
                log.log(f"  到达关节角度: q6={q6_actual_deg:+6.1f}° q7={q7_actual_deg:+6.1f}°")
                log.log(f"  约束边界: |q7|={abs(q7_actual_deg):.1f}° → |q6|≤{math.degrees(limit):.1f}°")
                log.log(f"  距离边界差距: {margin_deg:+.1f}° ({-value:+.3f} rad) -> {status}")
                
                results.append((test_idx, "Left", tag, q6_l, q7_l, limit, value, status, arrived, pos_err, angle_err))

            log.log("")

    movej_to_home(interface, log, joint_state_cache, rcl_node)

    log.log("")
    log.log("=" * 70)
    log.log(f"  Summary [Run {run_label}]")
    log.log("=" * 70)

    inside_count = 0
    outside_count = 0
    for item in results:
        step, arm_label, tag, q6, q7, limit, value, status, arrived, pos_err, angle_err = item
        if status == "NO_DATA":
            log.log(f"  #{step} {arm_label:5s} {tag:40s} NO_DATA")
            continue
        arrive_str = "ARRIVED" if arrived else "TIMEOUT"
        q6_deg = math.degrees(q6)
        q7_deg = math.degrees(q7)
        limit_deg = math.degrees(limit)
        pose_err_str = f"pos={pos_err:.4f}m ang={angle_err:.2f}°" if pos_err is not None else "pose_err=N/A"
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

    # 转换为适合分析的格式
    analysis_results = []
    for item in results:
        step, arm_label, tag, q6, q7, limit, value, status, arrived, pos_err, angle_err = item
        analysis_results.append({
            'q6': q6,
            'q7': q7,
            'limit': limit,
            'margin': -value if value is not None else None,
            'status': status,
            'pos_err': pos_err,
            'ang_err': angle_err
        })
    
    # 保存详细点位分析
    save_point_analysis(log, test_points, analysis_results)
    
    log.save()
    cleanup()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠ 用户中断测试")
        sys.exit(1)
