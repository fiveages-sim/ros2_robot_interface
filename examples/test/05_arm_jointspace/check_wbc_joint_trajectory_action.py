"""验证 ROS2RobotInterface 的 WBC 轨迹 Action 接口。

运动经 execute_joint_trajectory_action() 下发，验证自动发现、FSM 切换、
反馈回调和结果返回。独立观察节点仅查询参数和状态，不直接发送 Action。

依次测试 doubles / linear / tanh，每种类型包含单点左臂子集、
多点左右臂混合子集、腰部单点及腰部+双臂多点；默认从控制器 joints
识别腰部（可用 --body-joints 显式指定），未识别时停止；请求名称故意打乱，绝不发送完整关节列表。
打印每次运动后的全部关节位置、目标和误差，并验证未指定关节保持。
Goal accepted 不视为运动完成；等待规划时长及反馈到点后才判定。

在已激活 ROS 工作空间、可导入 rclpy 和 arms_ros2_control_msgs 的环境运行：
    python examples/test/05_arm_jointspace/check_wbc_joint_trajectory_action.py

会实际运动，默认偏移 0.04 rad；优先在仿真验证。
结束或失败时切 HOLD，恢复原插值类型和多点时长。不会自动回到初始姿态。
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor

import argparse
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.constants import is_body_joint_name


class Check(Node):
    def __init__(self, controller: str):
        super().__init__("check_wbc_joint_trajectory_action")
        self.positions: dict[str, float] = {}
        self.feedback_time = 0.0
        self.fsm = None
        self.description = ""
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Int32, "/fsm_state", self.on_fsm, latched)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.fsm_pub = self.create_publisher(Int32, "/fsm_command", 10)
        self.get_params = self.create_client(GetParameters, controller + "/get_parameters")
        self.set_params = self.create_client(SetParameters, controller + "/set_parameters")

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

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Service 不可用：{client.srv_name}")
        future = client.call_async(request)
        self.wait(future.done)
        return future.result()

    def set_parameters_remote(self, values):
        request = SetParameters.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in values.items()]
        response = self.call(self.set_params, request)
        for name, result in zip(values, response.results):
            if not result.successful:
                raise RuntimeError(f"设置 {name} 失败：{result.reason}")

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


def run_case(interface, node, mode, requested, multi, controlled, limits, args):
    before = node.snapshot(controlled)
    offsets = {}
    for i, name in enumerate(requested):
        # 不同偏移让乱序映射错误更容易识别；根据限位选择可行方向。
        offset = args.delta * (1.0 + i * 0.15)
        lower, upper = limits[name]
        if before[name] + offset > upper:
            offset = -offset
        if not lower <= before[name] + offset <= upper:
            raise RuntimeError(f"{name} 没有足够空间生成测试目标")
        offsets[name] = offset
    waypoints = [
        [before[n] + offsets[n] * fraction for n in requested]
        for fraction in ([0.5, 1.0] if multi else [1.0])
    ]
    arrival_times = [args.duration * (i + 1) for i in range(len(waypoints))]
    if args.dwell:
        waypoints.insert(1, list(waypoints[0]))
        arrival_times.insert(1, arrival_times[0] + args.dwell)
        for i in range(2, len(arrival_times)):
            arrival_times[i] += args.dwell
    target = dict(zip(requested, waypoints[-1]))
    label = f"{mode} / {'多点' if multi else '单点'} / {', '.join(requested)}"
    print(f"\n[{label}] 请求顺序：{requested}", flush=True)
    print(f"目标（按请求顺序）：{waypoints[-1]}", flush=True)
    feedback = []
    held_drift = {n: 0.0 for n in controlled if n not in target}

    entered_movej = False

    def monitor_result():
        nonlocal entered_movej
        entered_movej = entered_movej or node.fsm == 4
        for name in held_drift:
            held_drift[name] = max(held_drift[name], abs(node.positions[name] - before[name]))
        if (entered_movej and node.fsm != 4) or time.monotonic() - node.feedback_time > 1.0:
            raise RuntimeError("执行期间离开 MOVEJ 或关节反馈超时")
        return result_future.done()

    # The public API blocks; keep this independent observer spinning on the
    # calling thread. The interface owns its own background executor.
    with ThreadPoolExecutor(max_workers=1) as worker:
        result_future = worker.submit(
            interface.execute_joint_trajectory_action,
            joint_names=requested,
            waypoints=waypoints,
            time_mode=True,
            movej_trajectory_duration=args.duration if args.timing == "total" else None,
            time_from_start=arrival_times if args.timing == "waypoints" else None,
            feedback_callback=feedback.append,
            timeout=args.duration * (len(waypoints) + 1) + args.dwell + args.margin,
        )
        try:
            node.wait(monitor_result, timeout=args.duration * (len(waypoints) + 1) + args.dwell + args.margin + 15.0)
        except (Exception, KeyboardInterrupt):
            interface.send_fsm_command(2)
            raise
        response = result_future.result()
    if response is None or not feedback:
        raise RuntimeError("接口未返回 Action 结果或未收到反馈")
    print(f"response: {response}", flush=True)
    if not response.success or response.planned_duration < 0:
        raise RuntimeError(f"Action 规划或执行未成功：{response.message}")
    started = time.monotonic()
    stable_since = None
    failure = None
    while time.monotonic() - started < response.planned_duration + args.margin:
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
        if (now - started >= response.planned_duration and stable_since is not None
                and now - stable_since >= 0.3):
            break
    else:
        failure = "运动结束等待超时，未稳定到点"

    print("关节名称                 运动前       最终目标     当前反馈     绝对误差   状态")
    for name in controlled:
        expected = target.get(name, before[name])
        actual = node.positions.get(name, float("nan"))
        print(f"{name:22s} {before[name]: .6f}  {expected: .6f}  {actual: .6f}  "
              f"{abs(actual - expected):.6f}  {'运动' if name in target else '保持'}")
    print("实际位置（按乱序请求顺序）：", [node.positions.get(n) for n in requested])
    drift = max(held_drift.values(), default=0.0)
    print(f"未指定关节运动期间最大偏移：{drift:.6f} rad", flush=True)
    if failure or drift > args.hold_tolerance:
        raise RuntimeError(failure or f"未指定关节偏移超限：{drift:.6f}")
    print(f"PASS: {label}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="/ocs2_wbc_controller")
    parser.add_argument("--body-joints", nargs="+", help="覆盖默认自动识别的腰部关节名")
    parser.add_argument("--delta", type=float, default=0.04)
    parser.add_argument("--dwell", type=float, default=0.0, help="首目标后显式停留秒数，仅 waypoints 模式")
    parser.add_argument("--timing", choices=["waypoints", "total", "default"], default="waypoints",
                        help="累计时间、持久总时长或控制器默认时长")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--tolerance", type=float, default=0.01)
    parser.add_argument("--hold-tolerance", type=float, default=0.01)
    parser.add_argument("--margin", type=float, default=5.0)
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in
           (args.delta, args.duration, args.tolerance, args.hold_tolerance, args.margin)):
        parser.error("偏移、时长、容差和等待余量必须为有限正数")
    if args.delta <= args.tolerance:
        parser.error("偏移应大于到点容差，避免不运动也通过")
    if not math.isfinite(args.dwell) or args.dwell < 0 or (args.dwell and args.timing != "waypoints"):
        parser.error("dwell 必须非负有限，且只用于 waypoints 模式")
    rclpy.init()
    node = Check(args.controller.rstrip("/"))
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    original = {}
    motion_started = False
    try:
        interface.connect()
        expected_action = args.controller.rstrip("/") + "/joint_trajectory_with_para"
        if not interface.is_wbc or interface.config.joint_trajectory_action_name != expected_action:
            raise RuntimeError(
                f"接口未自动选择预期 WBC Action：{interface.config.joint_trajectory_action_name}"
            )
        if not interface.wait_for_joint_trajectory_action_server():
            raise RuntimeError("接口等待 WBC Action 服务端超时")
        names = ["joints", "movej_interpolation_type", "movej_trajectory_duration"]
        response = node.call(node.get_params, GetParameters.Request(names=names))
        controlled = list(response.values[0].string_array_value)
        original = {names[1]: response.values[1].string_value,
                    names[2]: response.values[2].double_value}
        cases = [(["left_joint5", "left_joint1", "left_joint3", "left_joint2", "left_joint4"], False),
                 (["right_joint3", "left_joint2", "right_joint1", "left_joint3", "right_joint2", "left_joint1"], True)]
        body_names = args.body_joints if args.body_joints is not None else [
            name for name in controlled if is_body_joint_name(name)
        ]
        if not body_names:
            raise RuntimeError("未识别到腰部关节，请通过 --body-joints 指定控制器中的实际关节名")
        if len(set(body_names)) != len(body_names):
            raise RuntimeError("腰部关节名不能重复")
        body = list(reversed(body_names))
        print("腰部测试关节：", body)
        cases.extend([(body, False), (body + ["left_joint1", "right_joint1"], True)])
        if not controlled or any(n not in controlled for ns, _ in cases for n in ns):
            raise RuntimeError("控制器关节配置不支持本测试使用的腰部或左右臂关节名称")
        if any(len(ns) >= len(controlled) for ns, _ in cases):
            raise RuntimeError("本测试要求请求关节为控制器关节的严格子集")
        node.wait(lambda: bool(node.description))
        limits = {}
        for joint in ET.fromstring(node.description).findall("joint"):
            limit = joint.find("limit")
            if joint.get("type") == "continuous":
                limits[joint.get("name")] = (-math.inf, math.inf)
            elif limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
                limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        if any(n not in limits for ns, _ in cases for n in ns):
            raise RuntimeError("robot_description 中缺少测试关节限位")
        print("控制器完整顺序：", controlled)
        motion_started = True
        node.switch(2)
        for mode in ("doubles", "linear", "tanh"):
            node.set_parameters_remote({"movej_interpolation_type": mode})
            for requested, multi in cases:
                node.switch(2)
                run_case(interface, node, mode, requested, multi, controlled, limits, args)
        print(f"\nPASS: {3 * len(cases)} 个 Action 运动检查全部通过")
    finally:
        try:
            if motion_started:
                node.switch(2)
            if original:
                node.set_parameters_remote(original)
        finally:
            try:
                if interface.is_connected:
                    interface.disconnect()
            finally:
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f"FAIL: {exc}", flush=True)
        raise SystemExit(1)
