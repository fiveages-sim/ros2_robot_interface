"""沿固定左臂路点验证分体双臂 JointTrajectory Action。

把模块里的 LEFT_ARM_WAYPOINTS（三个左臂关节位置，弧度）作为路点依次下发，
打印运动前后的全部关节位置、目标与误差，并验证右臂等未指定关节保持。
Action accepted 不视为运动完成；同时检查成功结果和实际反馈到点。

注意：GetParameters / SetParameters 的服务调用已注释，只保留 Action 通路；
因此不再切换插值模式，沿用控制器当前的 movej_interpolation_type。
逐点累计时间按 --duration 的段时长累加；默认三段 2、4、10 秒，
对应累计 2、6、16 秒，不修改控制器总时长参数。

在已激活 ROS 工作空间、可导入 rclpy 和 arms_ros2_control_msgs 的环境运行：
    python examples/test/05_arm_jointspace/check_split_joint_trajectory_action.py

会实际运动且幅度较大；优先在仿真验证。
结束或失败时切 HOLD。不会自动回到初始姿态。
"""

from __future__ import annotations

from itertools import accumulate

import argparse
import math
import time
import xml.etree.ElementTree as ET

import rclpy
# NOTE: 服务调用已按要求注释，只保留 Action 通路，需要恢复时取消下面两行注释。
# from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
# from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

from arms_ros2_control_msgs.msg import JointWaypoint
from arms_ros2_control_msgs.action import JointTrajectory

# 目标关节：左臂 7 个关节
LEFT_ARM_JOINT_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
]

# 左臂路点（弧度），每点顺序与 LEFT_ARM_JOINT_NAMES 一一对应
LEFT_ARM_WAYPOINTS = [
    [1.87641669974313, -0.5687477016984709, -1.730039779714491,
     -1.9825839235531273, -1.0040110954577952, 0.07270804751898144, 0.2687643399448287],
    [0.9644474437188942, -0.5046140506501708, -1.5921780158023264,
     -0.47424058673172265, -0.7691065252830858, -0.0748338879732527, -0.8967314065858463],
    [1.5177486656402732, -1.2694007275883998, -0.89838601010572,
     -1.6746200861523974, -0.27997303648008287, -0.02395193698124802, 0.678350940785112],
]


class Check(Node):
    def __init__(self, controller: str):
        super().__init__("check_wbc_joint_trajectory_service")
        self.positions: dict[str, float] = {}
        self.feedback_time = 0.0
        self.fsm = None
        self.description = ""
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Int32, "/fsm_state", self.on_fsm, latched)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.fsm_pub = self.create_publisher(Int32, "/fsm_command", 10)
        self.motion = ActionClient(self, JointTrajectory, controller + "/joint_trajectory_with_para")
        self.active_goal = None
        self.last_progress_print = 0.0
        # NOTE: 参数服务客户端已注释，只保留 Action。
        # self.get_params = self.create_client(GetParameters, controller + "/get_parameters")
        # self.set_params = self.create_client(SetParameters, controller + "/set_parameters")

    def on_progress(self, msg):
        now = time.monotonic()
        if now - self.last_progress_print >= 0.5:
            feedback = msg.feedback
            print(f"  progress={feedback.progress:.3f}, "
                  f"elapsed={feedback.elapsed_time:.2f}s, "
                  f"remaining={feedback.remaining_time:.2f}s", flush=True)
            self.last_progress_print = now

    def on_joints(self, msg):
        self.positions = dict(zip(msg.name, msg.position))
        self.feedback_time = time.monotonic()

    def on_fsm(self, msg):
        self.fsm = msg.data

    def on_description(self, msg):
        self.description = msg.data

    def wait(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError("等待状态、反馈或服务响应超时")

    # NOTE: 服务辅助方法已注释（只保留 Action）。
    # def call(self, client, request):
    #     if not client.wait_for_service(timeout_sec=5.0):
    #         raise RuntimeError(f"Service 不可用：{client.srv_name}")
    #     future = client.call_async(request)
    #     self.wait(future.done)
    #     return future.result()
    #
    # def set_parameters_remote(self, values):
    #     request = SetParameters.Request()
    #     request.parameters = [Parameter(name, value=value).to_parameter_msg()
    #                           for name, value in values.items()]
    #     response = self.call(self.set_params, request)
    #     for name, result in zip(values, response.results):
    #         if not result.successful:
    #             raise RuntimeError(f"设置 {name} 失败：{result.reason}")

    def switch(self, state):
        self.wait(lambda: self.fsm_pub.get_subscription_count() > 0)
        self.fsm_pub.publish(Int32(data=state))
        self.wait(lambda: self.fsm == state)

    def snapshot(self, names):
        received = self.feedback_time
        self.wait(lambda: self.feedback_time > received and all(n in self.positions for n in names))
        result = {n: self.positions[n] for n in names}
        if not all(math.isfinite(v) for v in result.values()):
            raise RuntimeError("关节反馈包含 NaN/Inf")
        return result


def run_case(node, label, requested, waypoints, controlled, limits, args):
    before = node.snapshot(controlled)
    for i, positions in enumerate(waypoints):
        if len(positions) != len(requested):
            raise RuntimeError(f"路点 {i} 长度 {len(positions)} 与关节数 {len(requested)} 不一致")
        for name, value in zip(requested, positions):
            lower, upper = limits[name]
            if not lower <= value <= upper:
                raise RuntimeError(
                    f"路点 {i} 的 {name}={value:.6f} 超出限位 [{lower:.6f}, {upper:.6f}]"
                )
    request = JointTrajectory.Goal(
        joint_names=requested,
        time_mode=True,
        use_waypoint_blend_ratio=True,
    )
    arrival_times = list(accumulate(args.duration))
    print(f"段时长：{args.duration} 秒；累计到达时间：{arrival_times} 秒", flush=True)

    for i, positions in enumerate(waypoints):
        waypoint = JointWaypoint(
            position=list(positions),
            time_from_start=Duration(seconds=arrival_times[i]).to_msg(),
            blend_ratio_percent=0.0,
        )
        request.waypoints.append(waypoint)

    target = dict(zip(requested, request.waypoints[-1].position))
    print(f"\n[{label}] 关节顺序：{requested}", flush=True)
    print(f"路点数：{len(request.waypoints)}，末路点（按请求顺序）："
          f"{request.waypoints[-1].position}", flush=True)
    if not node.motion.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("分体双臂 JointTrajectory Action 不可用")
    sent = node.motion.send_goal_async(
        request,
        feedback_callback=node.on_progress,
    )
    node.wait(sent.done)
    node.active_goal = sent.result()
    if not node.active_goal.accepted:
        node.active_goal = None
        raise RuntimeError("Action 目标被拒绝")
    result_future = node.active_goal.get_result_async()
    started = time.monotonic()
    # 按最后一个累计到达时间加余量等待。
    timeout = arrival_times[-1] + args.margin
    held_drift = {n: 0.0 for n in controlled if n not in target}
    stable_since = None
    failure = None
    while time.monotonic() - started < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if node.fsm != 4:
            failure = f"运动期间离开 MOVEJ：FSM={node.fsm}"
            break
        if now - node.feedback_time > 1.0 or any(n not in node.positions for n in controlled):
            failure = "关节反馈缺失或超过 1 秒未更新"
            break
        if not all(math.isfinite(node.positions[n]) for n in controlled):
            failure = "关节反馈包含 NaN/Inf"
            break
        for n in held_drift:
            held_drift[n] = max(held_drift[n], abs(node.positions[n] - before[n]))
        arrived = all(abs(node.positions[n] - value) <= args.tolerance for n, value in target.items())
        stable_since = (stable_since or now) if arrived else None
        if result_future.done():
            outcome = result_future.result()
            if outcome.status != GoalStatus.STATUS_SUCCEEDED or not outcome.result.success:
                failure = f"Action 失败：status={outcome.status}, {outcome.result.message}"
                break
            if stable_since is not None and now - stable_since >= 0.3:
                break
    else:
        failure = "等待 Action 完成或反馈稳定到点超时"

    if result_future.done():
        outcome = result_future.result()
        print(f"Action status={outcome.status}, result={outcome.result}", flush=True)
    if failure is None:
        node.active_goal = None

    print("关节名称                 运动前       最终目标     当前反馈     绝对误差   状态")
    for name in controlled:
        expected = target.get(name, before[name])
        actual = node.positions.get(name, float("nan"))
        print(f"{name:22s} {before[name]: .6f}  {expected: .6f}  {actual: .6f}  "
              f"{abs(actual - expected):.6f}  {'运动' if name in target else '保持'}")
    print("实际位置（按请求顺序）：", [node.positions.get(n) for n in requested])
    drift = max(held_drift.values(), default=0.0)
    print(f"未指定关节（右臂等）运动期间最大偏移：{drift:.6f} rad", flush=True)
    if failure or drift > args.hold_tolerance:
        raise RuntimeError(failure or f"未指定关节偏移超限：{drift:.6f}")
    print(f"PASS: {label}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="/ocs2_arm_controller")
    parser.add_argument("--duration", type=float, nargs="+", default=[2.0, 4.0, 10.0],
                        help="各段秒数，默认 2 4 10；只传一个数则所有段同一时长")
    parser.add_argument("--tolerance", type=float, default=0.01)
    parser.add_argument("--hold-tolerance", type=float, default=0.01)
    parser.add_argument("--margin", type=float, default=5.0,
                        help="累计到达总时间以外的等待余量（秒）")
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in
           (*args.duration, args.tolerance, args.hold_tolerance, args.margin)):
        parser.error("时长、容差和等待余量必须为有限正数")
    if len(args.duration) == 1:
        args.duration *= len(LEFT_ARM_WAYPOINTS)
    if len(args.duration) != len(LEFT_ARM_WAYPOINTS):
        parser.error("段时长数量必须等于路点数量，或只提供一个统一时长")
    rclpy.init()
    node = Check(args.controller.rstrip("/"))
    # original = {}  # NOTE: 不再读写控制器参数，无需保存原始值
    motion_started = False
    try:
        # NOTE: GetParameters 调用已注释；改用 /joint_states 中的左右臂关节作为受控集合。
        # names = ["joints", "movej_interpolation_type", "movej_trajectory_duration"]
        # response = node.call(node.get_params, GetParameters.Request(names=names))
        # controlled = list(response.values[0].string_array_value)
        # original = {names[1]: response.values[1].string_value,
        #             names[2]: response.values[2].double_value}
        node.wait(lambda: bool(node.positions))
        controlled = sorted(
            n for n in node.positions if n.startswith(("left_joint", "right_joint"))
        )
        if not controlled:
            raise RuntimeError("/joint_states 中没有 left_joint*/right_joint*，无法确定受控关节")
        missing = [n for n in LEFT_ARM_JOINT_NAMES if n not in controlled]
        if missing:
            raise RuntimeError(f"/joint_states 中缺少左臂关节：{missing}")
        node.wait(lambda: bool(node.description))
        limits = {}
        for joint in ET.fromstring(node.description).findall("joint"):
            limit = joint.find("limit")
            if joint.get("type") == "continuous":
                limits[joint.get("name")] = (-math.inf, math.inf)
            elif limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
                limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        missing_limits = [n for n in LEFT_ARM_JOINT_NAMES if n not in limits]
        if missing_limits:
            raise RuntimeError(f"robot_description 中缺少关节限位：{missing_limits}")
        print("受控关节（来自 /joint_states）：", controlled)
        motion_started = True
        node.switch(2)
        node.switch(4)
        # NOTE: 插值模式切换依赖 SetParameters，已注释，沿用控制器当前配置。
        run_case(node, "左臂固定路点", LEFT_ARM_JOINT_NAMES, LEFT_ARM_WAYPOINTS,
                 controlled, limits, args)
        print("\nPASS: 左臂路点 Action 检查通过")
    finally:
        try:
            if node.active_goal is not None:
                try:
                    cancel = node.active_goal.cancel_goal_async()
                    node.wait(cancel.done, timeout=2.0)
                except Exception as exc:
                    print(f"取消 Action 未确认：{exc}；继续切 HOLD", flush=True)
            if motion_started:
                node.switch(2)
            # NOTE: 参数恢复依赖 SetParameters，已注释。
            # if original:
            #     node.set_parameters_remote(original)
        finally:
            node.motion.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f"FAIL: {exc}", flush=True)
        raise SystemExit(1)
