#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""左右手/夹爪开关指令压力测试。

按设定频率向左右手 ``target_command`` 话题交替发送关(0)/开(1)指令，
从 ``/joint_states`` 读取实际位置，在每个发送周期内判断是否到达目标或已朝
目标方向产生有效运动，最后
生成 CSV 明细、JSON 和 Markdown 汇总报告。``hand`` 模式测试前通过全局
``/fsm_command`` 执行 HOLD→MOVEJ，测试结束或异常退出时切回 HOLD；``gripper``
模式不依赖 FSM。

``hand`` 模式从 BasicJointController 参数读取 ``joints``、``home_N`` 以及开/关
构型索引；``gripper`` 模式从 AdaptiveGripperController 读取 ``joint``，并按该
控制器相同的规则从 ``/robot_description`` 计算开/关限位。也可以通过命令行
显式给出关节和目标位置。

示例：
  # 先做 10 次低频验证
  python3 scripts/hand_stress_test.py -n 10 -f 0.5

  # 确认安全后做 1000 次，并跳过交互确认
  python3 scripts/hand_stress_test.py -n 1000 -f 0.5 --yes

  # 测试左右 AdaptiveGripperController
  python3 scripts/hand_stress_test.py --controller-type gripper -n 100 --yes

注意：脚本统计的是“位置反馈未在时限内显示有效响应”的次数。到达目标或至少
一个参与判定的关节朝目标方向移动超过阈值都算响应。仅凭位置反馈无法把
通信丢包、控制器状态不对、机械运动超时和硬件故障完全区分开。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String


COMMAND_NAMES = {0: "close", 1: "open"}
FSM_HOLD = 2
FSM_MOVEJ = 4
FSM_NAMES = {FSM_HOLD: "HOLD", FSM_MOVEJ: "MOVEJ"}


@dataclass
class SideConfig:
    side: str
    controller: str
    controller_type: str
    joints: List[str]
    close_target: List[float]
    open_target: List[float]
    active_indices: List[int]
    tolerance: float
    movement_threshold: float

    @property
    def topic(self) -> str:
        return f"/{self.controller.strip('/')}/target_command"

    def target_for(self, command: int) -> List[float]:
        return self.open_target if command == 1 else self.close_target


@dataclass
class SideResult:
    status: str
    response_reason: Optional[str]
    latency_ms: Optional[float]
    max_error: Optional[float]
    max_expected_movement: Optional[float]
    feedback_samples: int
    feedback_age_ms: Optional[float]
    positions: Dict[str, float]


@dataclass
class TrialResult:
    trial: int
    command: int
    command_name: str
    scheduled_elapsed_s: float
    sent_elapsed_s: float
    send_jitter_ms: float
    sides: Dict[str, SideResult]


def parse_csv_strings(value: Optional[str]) -> Optional[List[str]]:
    if value is None:
        return None
    result = [item.strip() for item in value.split(",") if item.strip()]
    if not result:
        raise argparse.ArgumentTypeError("列表不能为空")
    return result


def parse_csv_floats(value: Optional[str]) -> Optional[List[float]]:
    if value is None:
        return None
    try:
        result = [float(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"位置列表必须是逗号分隔的数字: {exc}") from exc
    if not result or not all(math.isfinite(item) for item in result):
        raise argparse.ArgumentTypeError("位置列表不能为空，且必须全部为有限数字")
    return result


def parameter_to_python(value):
    try:
        return parameter_value_to_python(value)
    except Exception:
        return None


class HandStressNode(Node):
    def __init__(
        self,
        joint_states_topic: str,
        fsm_command_topic: str,
        fsm_state_topic: str,
        robot_description_topic: str,
    ):
        super().__init__("hand_stress_test")
        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {}
        self._feedback_seq = 0
        self._feedback_time: Optional[float] = None
        self._fsm_state: Optional[int] = None
        self._fsm_state_seq = 0
        self._robot_description: Optional[str] = None
        # Do not use Node's reserved ``_publishers`` member: rclpy maintains
        # that as an internal list and create_publisher() appends to it.
        self._command_publishers: Dict[str, object] = {}
        self._fsm_command_publisher = self.create_publisher(
            Int32, fsm_command_topic, 10)
        fsm_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._fsm_state_subscription = self.create_subscription(
            Int32, fsm_state_topic, self._fsm_state_callback, fsm_state_qos)
        self.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback,
            qos_profile_sensor_data)
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, robot_description_topic, self._robot_description_callback,
            robot_description_qos)

    def _robot_description_callback(self, msg: String) -> None:
        with self._lock:
            self._robot_description = msg.data

    def _fsm_state_callback(self, msg: Int32) -> None:
        with self._lock:
            self._fsm_state = int(msg.data)
            self._fsm_state_seq += 1

    def _joint_state_callback(self, msg: JointState) -> None:
        received = time.monotonic()
        positions = {
            name: float(value)
            for name, value in zip(msg.name, msg.position)
            if math.isfinite(value)
        }
        with self._lock:
            self._positions.update(positions)
            self._feedback_seq += 1
            self._feedback_time = received

    def snapshot(self) -> Tuple[Dict[str, float], int, Optional[float]]:
        with self._lock:
            return dict(self._positions), self._feedback_seq, self._feedback_time

    def fsm_snapshot(self) -> Tuple[Optional[int], int]:
        with self._lock:
            return self._fsm_state, self._fsm_state_seq

    def robot_description_snapshot(self) -> Optional[str]:
        with self._lock:
            return self._robot_description

    def fsm_command_subscription_count(self) -> int:
        return self._fsm_command_publisher.get_subscription_count()

    def fsm_state_publisher_count(self) -> int:
        return self._fsm_state_subscription.get_publisher_count()

    def publish_fsm_command(self, command: int) -> None:
        msg = Int32()
        msg.data = command
        self._fsm_command_publisher.publish(msg)

    def add_side(self, config: SideConfig) -> None:
        self._command_publishers[config.side] = self.create_publisher(
            Int32, config.topic, 10)

    def subscription_count(self, side: str) -> int:
        return self._command_publishers[side].get_subscription_count()

    def publish(self, side: str, command: int) -> None:
        msg = Int32()
        msg.data = command
        self._command_publishers[side].publish(msg)

    def get_parameters(
        self, controller: str, names: Sequence[str], timeout: float
    ) -> Optional[List[object]]:
        client = AsyncParameterClient(self, controller.strip("/"))
        # AsyncParameterClient uses the plural API because a ROS parameter
        # server exposes a group of services (get/list/set/describe/...).
        if not client.wait_for_services(timeout_sec=timeout):
            return None
        future = client.get_parameters(list(names))
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or future.exception() is not None:
            return None
        return [parameter_to_python(value) for value in future.result().values]


def validate_explicit_side(
    side: str,
    controller: str,
    controller_type: str,
    joints: Optional[List[str]],
    close_target: Optional[List[float]],
    open_target: Optional[List[float]],
    active_delta: float,
    tolerance: float,
    movement_threshold: float,
) -> Optional[SideConfig]:
    supplied = (joints is not None, close_target is not None, open_target is not None)
    if not any(supplied):
        return None
    if not all(supplied):
        raise RuntimeError(
            f"{side}: 显式配置必须同时提供 --{side}-joints、"
            f"--{side}-close-target 和 --{side}-open-target")
    assert joints is not None and close_target is not None and open_target is not None
    if len(set(joints)) != len(joints):
        raise RuntimeError(f"{side}: 关节名有重复")
    if len(joints) != len(close_target) or len(joints) != len(open_target):
        raise RuntimeError(
            f"{side}: joints/close-target/open-target 长度必须相同，当前为 "
            f"{len(joints)}/{len(close_target)}/{len(open_target)}")
    active = [
        i for i, (closed, opened) in enumerate(zip(close_target, open_target))
        if abs(opened - closed) >= active_delta
    ]
    if not active:
        raise RuntimeError(f"{side}: 开关目标没有任何位置差达到 {active_delta} 的关节")
    return SideConfig(
        side, controller.strip("/"), controller_type, joints,
        close_target, open_target, active, tolerance, movement_threshold)


def local_name(tag: str) -> str:
    """Return an XML tag name without an optional namespace."""
    return tag.rsplit("}", 1)[-1]


def gripper_targets_from_robot_description(
    robot_description: str,
    joint: str,
) -> Tuple[float, float]:
    """Match AdaptiveGripperController's limit/initial-value convention."""
    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as exc:
        raise RuntimeError(f"robot_description XML 解析失败: {exc}") from exc

    lower: Optional[float] = None
    upper: Optional[float] = None
    for element in root.iter():
        if local_name(element.tag) != "joint" or element.get("name") != joint:
            continue
        for child in element:
            if local_name(child.tag) == "limit":
                try:
                    lower = float(child.attrib["lower"])
                    upper = float(child.attrib["upper"])
                except (KeyError, ValueError) as exc:
                    raise RuntimeError(f"关节 {joint} 的 limit lower/upper 无效") from exc
                break
        if lower is not None:
            break
    if lower is None or upper is None:
        raise RuntimeError(f"robot_description 中找不到关节 {joint} 的位置限位")
    if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
        raise RuntimeError(f"关节 {joint} 的位置限位无效: [{lower}, {upper}]")

    # The controller defaults config_initial_position_ to zero when the
    # ros2_control state interface does not provide an initial value.
    initial = 0.0
    for ros2_control in root.iter():
        if local_name(ros2_control.tag) != "ros2_control":
            continue
        for joint_element in ros2_control.iter():
            if (
                local_name(joint_element.tag) != "joint"
                or joint_element.get("name") != joint
            ):
                continue
            for state_interface in joint_element:
                if (
                    local_name(state_interface.tag) != "state_interface"
                    or state_interface.get("name") != "position"
                ):
                    continue
                for parameter in state_interface:
                    if (
                        local_name(parameter.tag) == "param"
                        and parameter.get("name") == "initial_value"
                        and parameter.text
                    ):
                        try:
                            initial = float(parameter.text.strip())
                        except ValueError as exc:
                            raise RuntimeError(
                                f"关节 {joint} 的 position initial_value 无效"
                            ) from exc

    if not math.isfinite(initial):
        raise RuntimeError(f"关节 {joint} 的 position initial_value 不是有限数字")
    if abs(upper - initial) < abs(lower - initial):
        return upper, lower
    return lower, upper


def wait_for_robot_description(
    node: HandStressNode,
    timeout: float,
    topic: str,
) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        description = node.robot_description_snapshot()
        if description:
            return description
        time.sleep(0.02)
    raise RuntimeError(f"等待 {topic} 超时；gripper 模式需要从中读取关节限位")


def load_side_from_controller(
    node: HandStressNode,
    side: str,
    controller: str,
    controller_type: str,
    timeout: float,
    active_delta: float,
    tolerance: float,
    movement_threshold: float,
    robot_description_topic: str,
) -> SideConfig:
    if controller_type == "gripper":
        values = node.get_parameters(controller, ["joint"], timeout)
        if values is None:
            raise RuntimeError(
                f"{side}: 无法连接控制器 /{controller.strip('/')} 的参数服务；"
                "请确认控制器已启动，或显式传入关节及开/关目标")
        joint = values[0]
        if not isinstance(joint, str) or not joint.strip():
            raise RuntimeError(f"{side}: gripper 控制器参数 joint 不存在或为空")
        description = wait_for_robot_description(
            node, timeout, robot_description_topic)
        close_position, open_position = gripper_targets_from_robot_description(
            description, joint)
        if abs(open_position - close_position) < active_delta:
            raise RuntimeError(
                f"{side}: gripper 开关限位差 {abs(open_position - close_position):.6g} "
                f"小于 --gripper-active-joint-delta ({active_delta})")
        return SideConfig(
            side=side,
            controller=controller.strip("/"),
            controller_type=controller_type,
            joints=[joint],
            close_target=[close_position],
            open_target=[open_position],
            active_indices=[0],
            tolerance=tolerance,
            movement_threshold=movement_threshold,
        )

    base_names = ["joints", "target_command_close_config", "target_command_open_config"]
    values = node.get_parameters(controller, base_names, timeout)
    if values is None:
        raise RuntimeError(
            f"{side}: 无法连接控制器 /{controller.strip('/')} 的参数服务；"
            "请确认控制器已启动，或显式传入关节及开/关目标")
    joints, close_index, open_index = values
    if not isinstance(joints, (list, tuple)) or not joints:
        raise RuntimeError(f"{side}: 控制器参数 joints 不存在或为空")
    if not isinstance(close_index, int) or not isinstance(open_index, int):
        raise RuntimeError(f"{side}: 控制器开/关构型索引参数无效")
    if close_index < 0 or open_index < 0:
        raise RuntimeError(f"{side}: 控制器开/关构型索引不能为负数")

    home_names = [f"home_{close_index + 1}", f"home_{open_index + 1}"]
    homes = node.get_parameters(controller, home_names, timeout)
    if homes is None or any(not isinstance(value, (list, tuple)) for value in homes):
        raise RuntimeError(f"{side}: 无法读取目标构型参数 {home_names}")
    try:
        close_target = [float(value) for value in homes[0]]
        open_target = [float(value) for value in homes[1]]
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{side}: 控制器开/关目标包含无效位置值") from exc
    if not all(math.isfinite(value) for value in close_target + open_target):
        raise RuntimeError(f"{side}: 控制器开/关目标必须全部为有限数字")
    joints = [str(value) for value in joints]
    if len(joints) != len(close_target) or len(joints) != len(open_target):
        raise RuntimeError(
            f"{side}: 控制器 joints/关目标/开目标长度不一致: "
            f"{len(joints)}/{len(close_target)}/{len(open_target)}")
    active = [
        i for i, (closed, opened) in enumerate(zip(close_target, open_target))
        if abs(opened - closed) >= active_delta
    ]
    if not active:
        raise RuntimeError(f"{side}: 开关构型没有任何位置差达到 {active_delta} rad 的关节")
    return SideConfig(
        side, controller.strip("/"), controller_type, joints,
        close_target, open_target, active, tolerance, movement_threshold)


def evaluate_position(
    config: SideConfig,
    command: int,
    positions: Dict[str, float],
) -> Tuple[bool, Optional[float], Dict[str, float]]:
    target = config.target_for(command)
    observed = {name: positions[name] for name in config.joints if name in positions}
    if any(config.joints[index] not in observed for index in config.active_indices):
        return False, None, observed
    max_error = max(
        abs(observed[config.joints[index]] - target[index])
        for index in config.active_indices
    )
    return max_error <= config.tolerance, max_error, observed


def evaluate_movement(
    config: SideConfig,
    command: int,
    baseline: Dict[str, float],
    positions: Dict[str, float],
) -> Tuple[bool, Optional[float]]:
    """Detect meaningful movement toward the commanded target.

    A controller-level response only needs one judged joint to move in the
    expected direction. Requiring every hand joint to arrive would conflate a
    slow mechanism with a dropped/ignored command.
    """
    target = config.target_for(command)
    expected_movements: List[float] = []
    for index in config.active_indices:
        joint = config.joints[index]
        if joint not in baseline or joint not in positions:
            continue
        remaining = target[index] - baseline[joint]
        if abs(remaining) <= config.tolerance:
            continue
        direction = 1.0 if remaining > 0.0 else -1.0
        expected_movements.append((positions[joint] - baseline[joint]) * direction)
    if not expected_movements:
        return False, None
    max_movement = max(expected_movements)
    return max_movement >= config.movement_threshold, max_movement


def wait_for_feedback_joints(
    node: HandStressNode,
    configs: Sequence[SideConfig],
    timeout: float,
    topic: str,
) -> None:
    required = {joint for config in configs for joint in config.joints}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        positions, _, _ = node.snapshot()
        missing = required.difference(positions)
        if not missing:
            return
        time.sleep(0.02)
    positions, _, _ = node.snapshot()
    missing = sorted(required.difference(positions))
    raise RuntimeError(
        f"位置反馈等待超时，{topic} 缺少 {len(missing)} 个关节: {missing}")


def wait_for_subscribers(
    node: HandStressNode,
    configs: Sequence[SideConfig],
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if all(node.subscription_count(config.side) > 0 for config in configs):
            return
        time.sleep(0.05)
    missing = [config.topic for config in configs if node.subscription_count(config.side) == 0]
    raise RuntimeError(f"指令话题没有订阅者: {missing}；请确认目标控制器已激活")


def wait_for_fsm_command_subscriber(node: HandStressNode, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.fsm_command_subscription_count() > 0:
            return
        time.sleep(0.05)
    raise RuntimeError("FSM 指令话题没有订阅者；请确认控制器已激活")


def publish_and_wait_for_fsm_state(
    node: HandStressNode,
    command: int,
    timeout: float,
    fallback_delay: float,
) -> bool:
    """Publish one FSM command and verify it when /fsm_state is available.

    BasicJointController subscribes to the global FSM command but does not publish
    its own state. In a full robot launch, the active arm/WBC controller publishes
    the shared, transient-local /fsm_state. For hand-only launches we use a delay
    covering several controller cycles after reliable command delivery.
    """
    state_name = FSM_NAMES[command]
    print(f"FSM：发送 {state_name}({command})")
    node.publish_fsm_command(command)

    discovery_deadline = time.monotonic() + min(timeout, 0.5)
    while (
        node.fsm_state_publisher_count() == 0
        and time.monotonic() < discovery_deadline
    ):
        time.sleep(0.02)

    if node.fsm_state_publisher_count() == 0:
        time.sleep(fallback_delay)
        print(
            f"FSM：未发现状态发布器，已等待 {fallback_delay:.3f}s，"
            f"按控制器转换规则视为 {state_name}")
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state, _ = node.fsm_snapshot()
        if state == command:
            print(f"FSM：已确认进入 {state_name}")
            return True
        time.sleep(0.02)
    state, _ = node.fsm_snapshot()
    raise RuntimeError(
        f"等待 FSM 进入 {state_name} 超时，当前状态="
        f"{state if state is not None else '无反馈'}")


def enter_movej(node: HandStressNode, timeout: float, fallback_delay: float) -> None:
    """Normalize any valid starting state through HOLD, then enter MOVEJ."""
    wait_for_fsm_command_subscriber(node, timeout)
    publish_and_wait_for_fsm_state(
        node, FSM_HOLD, timeout, fallback_delay)
    publish_and_wait_for_fsm_state(
        node, FSM_MOVEJ, timeout, fallback_delay)


def exit_to_hold(node: HandStressNode, timeout: float, fallback_delay: float) -> None:
    """Return all controllers listening to the shared FSM topic to HOLD."""
    wait_for_fsm_command_subscriber(node, timeout)
    publish_and_wait_for_fsm_state(
        node, FSM_HOLD, timeout, fallback_delay)


def sleep_until(deadline: float) -> None:
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(remaining, 0.02))


def run_trials(
    node: HandStressNode,
    configs: Sequence[SideConfig],
    count: int,
    frequency: float,
    response_timeout: float,
    first_command: int,
    progress_every: int,
    stop_event: threading.Event,
) -> Tuple[List[TrialResult], float, float]:
    results: List[TrialResult] = []
    period = 1.0 / frequency
    start = time.monotonic() + 0.2

    for index in range(count):
        if stop_event.is_set():
            break
        scheduled = start + index * period
        sleep_until(scheduled)
        if stop_event.is_set():
            break

        command = first_command if index % 2 == 0 else 1 - first_command
        baseline_positions, seq_at_send, _ = node.snapshot()
        sent = time.monotonic()
        for config in configs:
            node.publish(config.side, command)

        pending = {config.side for config in configs}
        side_results: Dict[str, SideResult] = {}
        sample_counts = {config.side: 0 for config in configs}
        last_seq = seq_at_send
        deadline = min(scheduled + period, sent + response_timeout)

        while pending and time.monotonic() < deadline and not stop_event.is_set():
            positions, seq, feedback_time = node.snapshot()
            if seq > last_seq:
                delta = seq - last_seq
                for side in pending:
                    sample_counts[side] += delta
                last_seq = seq
                now = time.monotonic()
                for config in configs:
                    if config.side not in pending:
                        continue
                    reached, max_error, observed = evaluate_position(
                        config, command, positions)
                    moved, max_movement = evaluate_movement(
                        config, command, baseline_positions, positions)
                    if reached or moved:
                        side_results[config.side] = SideResult(
                            status="success",
                            response_reason=(
                                "target_reached" if reached else "movement_detected"),
                            latency_ms=(now - sent) * 1000.0,
                            max_error=max_error,
                            max_expected_movement=max_movement,
                            feedback_samples=sample_counts[config.side],
                            feedback_age_ms=(now - feedback_time) * 1000.0
                            if feedback_time is not None else None,
                            positions=observed,
                        )
                        pending.remove(config.side)
            time.sleep(0.002)

        positions, final_seq, feedback_time = node.snapshot()
        now = time.monotonic()
        for config in configs:
            if config.side not in pending:
                continue
            reached, max_error, observed = evaluate_position(
                config, command, positions)
            moved, max_movement = evaluate_movement(
                config, command, baseline_positions, positions)
            # deadline 边界刚好到达时仍计为成功，但必须有发送后的新反馈。
            success = (reached or moved) and final_seq > seq_at_send
            side_results[config.side] = SideResult(
                status="success" if success else (
                    "no_feedback" if final_seq <= seq_at_send else "position_timeout"),
                response_reason=(
                    "target_reached" if success and reached
                    else "movement_detected" if success else None),
                latency_ms=(now - sent) * 1000.0 if success else None,
                max_error=max_error,
                max_expected_movement=max_movement,
                feedback_samples=max(sample_counts[config.side], final_seq - seq_at_send),
                feedback_age_ms=(now - feedback_time) * 1000.0
                if feedback_time is not None else None,
                positions=observed,
            )

        results.append(TrialResult(
            trial=index + 1,
            command=command,
            command_name=COMMAND_NAMES[command],
            scheduled_elapsed_s=scheduled - start,
            sent_elapsed_s=sent - start,
            send_jitter_ms=(sent - scheduled) * 1000.0,
            sides=side_results,
        ))

        if progress_every > 0 and (
            (index + 1) % progress_every == 0 or index + 1 == count
        ):
            summary = []
            for config in configs:
                losses = sum(
                    result.sides[config.side].status != "success" for result in results)
                summary.append(f"{config.side}未响应={losses}")
            print(
                f"[{index + 1}/{count}] {COMMAND_NAMES[command]}  " + "  ".join(summary),
                flush=True)

    end = time.monotonic()
    return results, start, end


def safe_mean(values: Iterable[Optional[float]]) -> Optional[float]:
    numbers = [value for value in values if value is not None]
    return sum(numbers) / len(numbers) if numbers else None


def safe_max(values: Iterable[Optional[float]]) -> Optional[float]:
    numbers = [value for value in values if value is not None]
    return max(numbers) if numbers else None


def build_summary(
    args: argparse.Namespace,
    configs: Sequence[SideConfig],
    results: Sequence[TrialResult],
    started_at: str,
    elapsed_s: float,
    interrupted: bool,
) -> Dict[str, object]:
    sides: Dict[str, object] = {}
    for config in configs:
        side_results = [result.sides[config.side] for result in results]
        success = sum(result.status == "success" for result in side_results)
        target_reached = sum(
            result.response_reason == "target_reached" for result in side_results)
        movement_detected = sum(
            result.response_reason == "movement_detected" for result in side_results)
        position_timeout = sum(result.status == "position_timeout" for result in side_results)
        no_feedback = sum(result.status == "no_feedback" for result in side_results)
        attempted = len(side_results)
        sides[config.side] = {
            "controller": config.controller,
            "controller_type": config.controller_type,
            "command_topic": config.topic,
            "joints": config.joints,
            "judged_joints": [config.joints[index] for index in config.active_indices],
            "close_target": config.close_target,
            "open_target": config.open_target,
            "position_tolerance": config.tolerance,
            "movement_threshold": config.movement_threshold,
            "attempted": attempted,
            "success": success,
            "target_reached": target_reached,
            "movement_detected": movement_detected,
            "inferred_lost": attempted - success,
            "inferred_loss_rate_percent":
                (attempted - success) * 100.0 / attempted if attempted else 0.0,
            "position_timeout": position_timeout,
            "no_feedback": no_feedback,
            "average_success_latency_ms": safe_mean(
                result.latency_ms for result in side_results if result.status == "success"),
            "max_success_latency_ms": safe_max(
                result.latency_ms for result in side_results if result.status == "success"),
        }
    total_attempted = len(results) * len(configs)
    total_success = sum(
        result.status == "success"
        for trial in results for result in trial.sides.values())
    return {
        "test": "left_right_end_effector_target_command_stress_test",
        "started_at": started_at,
        "completed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "interrupted": interrupted,
        "requested_cycles_per_side": args.count,
        "completed_cycles_per_side": len(results),
        "frequency_hz": args.frequency,
        "period_s": 1.0 / args.frequency,
        "response_timeout_s": args.response_timeout,
        "controller_type": args.controller_type,
        "hand_position_tolerance_rad": args.tolerance,
        "hand_active_joint_delta_rad": args.active_joint_delta,
        "gripper_position_tolerance": args.gripper_tolerance,
        "gripper_active_joint_delta": args.gripper_active_joint_delta,
        "hand_movement_threshold_rad": args.movement_threshold,
        "gripper_movement_threshold": args.gripper_movement_threshold,
        "joint_states_topic": args.joint_states_topic,
        "robot_description_topic": args.robot_description_topic,
        "elapsed_s": elapsed_s,
        "total_commands": total_attempted,
        "total_success": total_success,
        "total_inferred_lost": total_attempted - total_success,
        "total_inferred_loss_rate_percent":
            (total_attempted - total_success) * 100.0 / total_attempted
            if total_attempted else 0.0,
        "sides": sides,
        "definition": (
            "inferred_lost means no post-send joint-state feedback reached the configured "
            "target tolerance and no judged joint moved toward the target by the configured "
            "movement threshold before the response deadline; it is not a transport-layer "
            "packet capture"
        ),
    }


def write_reports(
    base_dir: Path,
    summary: Dict[str, object],
    results: Sequence[TrialResult],
    configs: Sequence[SideConfig],
) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = base_dir / f"hand_stress_{stamp}"
    suffix = 1
    while report_dir.exists():
        report_dir = base_dir / f"hand_stress_{stamp}_{suffix}"
        suffix += 1
    report_dir.mkdir(parents=True)

    json_path = report_dir / "summary.json"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    csv_path = report_dir / "trials.csv"
    fieldnames = [
        "trial", "command", "command_name", "scheduled_elapsed_s", "sent_elapsed_s",
        "send_jitter_ms",
    ]
    for config in configs:
        prefix = config.side
        fieldnames.extend([
            f"{prefix}_status", f"{prefix}_response_reason",
            f"{prefix}_latency_ms", f"{prefix}_max_error",
            f"{prefix}_max_expected_movement",
            f"{prefix}_feedback_samples", f"{prefix}_feedback_age_ms", f"{prefix}_positions",
        ])
    with csv_path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for trial in results:
            row = {
                "trial": trial.trial,
                "command": trial.command,
                "command_name": trial.command_name,
                "scheduled_elapsed_s": f"{trial.scheduled_elapsed_s:.6f}",
                "sent_elapsed_s": f"{trial.sent_elapsed_s:.6f}",
                "send_jitter_ms": f"{trial.send_jitter_ms:.3f}",
            }
            for config in configs:
                result = trial.sides[config.side]
                prefix = config.side
                row.update({
                    f"{prefix}_status": result.status,
                    f"{prefix}_response_reason": result.response_reason or "",
                    f"{prefix}_latency_ms": "" if result.latency_ms is None
                    else f"{result.latency_ms:.3f}",
                    f"{prefix}_max_error": "" if result.max_error is None
                    else f"{result.max_error:.6f}",
                    f"{prefix}_max_expected_movement": ""
                    if result.max_expected_movement is None
                    else f"{result.max_expected_movement:.6f}",
                    f"{prefix}_feedback_samples": result.feedback_samples,
                    f"{prefix}_feedback_age_ms": "" if result.feedback_age_ms is None
                    else f"{result.feedback_age_ms:.3f}",
                    f"{prefix}_positions": json.dumps(result.positions, ensure_ascii=False),
                })
            writer.writerow(row)

    markdown_path = report_dir / "report.md"
    lines = [
        "# 左右手/夹爪开关指令压力测试报告",
        "",
        f"- 开始时间：{summary['started_at']}",
        f"- 完成时间：{summary['completed_at']}",
        f"- 测试频率：{summary['frequency_hz']} Hz",
        f"- 每侧完成次数：{summary['completed_cycles_per_side']} / {summary['requested_cycles_per_side']}",
        f"- 控制器类型：{summary['controller_type']}",
        f"- 响应时限：{summary['response_timeout_s']} s",
        f"- 总指令数：{summary['total_commands']}",
        f"- 位置反馈推断未响应数：{summary['total_inferred_lost']}",
        f"- 推断未响应率：{summary['total_inferred_loss_rate_percent']:.4f}%",
        f"- 是否中断：{'是' if summary['interrupted'] else '否'}",
        "",
        "## 分侧统计",
        "",
        "| 侧 | 类型 | 到位容差 | 运动阈值 | 指令数 | 响应 | 到位 | 检测到运动 | 推断未响应 | 未响应率 | 无反馈 | 平均响应延迟(ms) |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for config in configs:
        side = summary["sides"][config.side]
        average = side["average_success_latency_ms"]
        lines.append(
            f"| {config.side} | {config.controller_type} | {config.tolerance} | "
            f"{config.movement_threshold} | "
            f"{side['attempted']} | {side['success']} | "
            f"{side['target_reached']} | {side['movement_detected']} | "
            f"{side['inferred_lost']} | {side['inferred_loss_rate_percent']:.4f}% | "
            f"{side['no_feedback']} | "
            f"{'-' if average is None else f'{average:.3f}'} |")
    lines.extend([
        "",
        "## 判定说明",
        "",
        "到达目标容差，或至少一个参与判定的关节朝目标方向移动达到运动阈值，均计为响应。"
        "“推断未响应”表示两项条件都未满足，或没有收到新的位置反馈。"
        "它不是总线抓包结果，因此还可能包含控制器状态异常、"
        "机械运动超时或硬件故障。逐次位置、误差和反馈状态见 `trials.csv`。",
        "",
    ])
    markdown_path.write_text("\n".join(lines), encoding="utf-8")
    return report_dir


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按指定频率交替发送左右手/夹爪开关指令，并通过位置反馈统计未响应次数",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-n", "--count", type=int, default=1000, help="每侧发送的指令次数")
    parser.add_argument("-f", "--frequency", type=float, default=0.5, help="指令发送频率 (Hz)")
    parser.add_argument("--hands", choices=("both", "left", "right"), default="both")
    parser.add_argument("--first", choices=("close", "open"), default="close", help="第一条指令")
    parser.add_argument(
        "--controller-type", choices=("hand", "gripper"), default="hand",
        help="hand=BasicJointController，gripper=AdaptiveGripperController")
    parser.add_argument(
        "--left-controller", default=None,
        help="左控制器名；默认由 --controller-type 生成")
    parser.add_argument(
        "--right-controller", default=None,
        help="右控制器名；默认由 --controller-type 生成")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--robot-description-topic", default="/robot_description")
    parser.add_argument("--fsm-command-topic", default="/fsm_command")
    parser.add_argument("--fsm-state-topic", default="/fsm_state")
    parser.add_argument(
        "--fsm-timeout", type=float, default=5.0,
        help="等待 FSM 状态转换确认的最长时间 (s)")
    parser.add_argument(
        "--fsm-fallback-delay", type=float, default=0.3,
        help="没有 /fsm_state 发布器时，每次 FSM 指令后的等待时间 (s)")
    parser.add_argument("--tolerance", type=float, default=0.12, help="目标位置最大允许误差 (rad)")
    parser.add_argument(
        "--active-joint-delta", type=float, default=0.05,
        help="开关目标差小于该值的关节不参与判定 (rad)")
    parser.add_argument(
        "--movement-threshold", type=float, default=0.01,
        help="hand 朝目标方向的最小有效位移 (rad)")
    parser.add_argument(
        "--gripper-tolerance", type=float, default=0.002,
        help="gripper 目标位置最大允许误差（关节位置单位，移动夹爪通常为 m）")
    parser.add_argument(
        "--gripper-active-joint-delta", type=float, default=0.001,
        help="gripper 开关限位差必须达到的最小值（关节位置单位）")
    parser.add_argument(
        "--gripper-movement-threshold", type=float, default=0.0002,
        help="gripper 朝目标方向的最小有效位移（关节位置单位）")
    parser.add_argument(
        "--response-timeout", type=float, default=None,
        help="每条指令的位置响应时限 (s)，默认取发送周期的 90%%")
    parser.add_argument("--startup-timeout", type=float, default=5.0, help="等待控制器/话题时间 (s)")
    parser.add_argument("--feedback-timeout", type=float, default=5.0, help="等待所需关节反馈时间 (s)")
    parser.add_argument("--progress-every", type=int, default=10, help="每多少次打印一次进度，0 为关闭")
    parser.add_argument("--report-dir", type=Path, default=Path("hand_stress_reports"))
    parser.add_argument("--yes", action="store_true", help="跳过真实硬件运动确认")

    for side in ("left", "right"):
        parser.add_argument(
            f"--{side}-joints", type=parse_csv_strings,
            help=f"显式指定{side}侧关节名，逗号分隔")
        parser.add_argument(
            f"--{side}-close-target", type=parse_csv_floats,
            help=f"显式指定{side}侧关闭目标，逗号分隔")
        parser.add_argument(
            f"--{side}-open-target", type=parse_csv_floats,
            help=f"显式指定{side}侧打开目标，逗号分隔")
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    for side in ("left", "right"):
        if getattr(args, f"{side}_controller") is None:
            setattr(args, f"{side}_controller", f"{side}_{args.controller_type}_controller")
    if args.count <= 0:
        parser.error("--count 必须大于 0")
    if not math.isfinite(args.frequency) or args.frequency <= 0:
        parser.error("--frequency 必须是大于 0 的有限数字")
    if not math.isfinite(args.tolerance) or args.tolerance <= 0:
        parser.error("--tolerance 必须大于 0")
    if not math.isfinite(args.active_joint_delta) or args.active_joint_delta < 0:
        parser.error("--active-joint-delta 必须大于等于 0")
    if not math.isfinite(args.movement_threshold) or args.movement_threshold <= 0:
        parser.error("--movement-threshold 必须大于 0")
    if not math.isfinite(args.gripper_tolerance) or args.gripper_tolerance <= 0:
        parser.error("--gripper-tolerance 必须大于 0")
    if (
        not math.isfinite(args.gripper_active_joint_delta)
        or args.gripper_active_joint_delta < 0
    ):
        parser.error("--gripper-active-joint-delta 必须大于等于 0")
    if (
        not math.isfinite(args.gripper_movement_threshold)
        or args.gripper_movement_threshold <= 0
    ):
        parser.error("--gripper-movement-threshold 必须大于 0")
    if args.startup_timeout <= 0 or args.feedback_timeout <= 0 or args.fsm_timeout <= 0:
        parser.error("启动和反馈等待时间必须大于 0")
    if args.fsm_fallback_delay <= 0:
        parser.error("--fsm-fallback-delay 必须大于 0")
    period = 1.0 / args.frequency
    if args.response_timeout is None:
        args.response_timeout = period * 0.9
    if not math.isfinite(args.response_timeout) or args.response_timeout <= 0:
        parser.error("--response-timeout 必须大于 0")
    if args.response_timeout > period:
        parser.error(
            f"--response-timeout ({args.response_timeout}s) 不能大于发送周期 ({period}s)，"
            "否则下一条反向指令会使位置判定失效")
    if args.progress_every < 0:
        parser.error("--progress-every 不能为负数")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = create_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    selected = ["left", "right"] if args.hands == "both" else [args.hands]
    estimated = args.count / args.frequency
    if not args.yes:
        device_name = "灵巧手" if args.controller_type == "hand" else "夹爪"
        print(f"警告：该测试会让选定的{device_name}持续交替开合。")
        print(
            f"侧={','.join(selected)}，每侧={args.count} 条，频率={args.frequency} Hz，"
            f"预计至少 {estimated:.1f} 秒。请确保人员和物体远离机构。")
        if args.controller_type == "hand":
            print("测试会通过全局 FSM 指令先切换到 MOVEJ，结束后切回 HOLD。")
        try:
            answer = input("输入 YES 开始测试：").strip()
        except EOFError:
            print("非交互环境请在确认安全后加 --yes", file=sys.stderr)
            return 1
        if answer != "YES":
            print("已取消")
            return 1

    try:
        rclpy.init(args=None)
    except Exception as exc:
        print(f"测试失败：ROS 2 初始化失败: {exc}", file=sys.stderr)
        return 1
    node = HandStressNode(
        args.joint_states_topic,
        args.fsm_command_topic,
        args.fsm_state_topic,
        args.robot_description_topic,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="ros2-spin", daemon=True)
    spin_thread.start()
    stop_event = threading.Event()
    interrupted = False
    results: List[TrialResult] = []
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    run_start = time.monotonic()
    exit_code = 1
    fsm_control_started = False

    old_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, lambda _signum, _frame: stop_event.set())

    try:
        configs: List[SideConfig] = []
        for side in selected:
            controller = getattr(args, f"{side}_controller")
            if args.controller_type == "gripper":
                active_delta = args.gripper_active_joint_delta
                tolerance = args.gripper_tolerance
                movement_threshold = args.gripper_movement_threshold
            else:
                active_delta = args.active_joint_delta
                tolerance = args.tolerance
                movement_threshold = args.movement_threshold
            explicit = validate_explicit_side(
                side,
                controller,
                args.controller_type,
                getattr(args, f"{side}_joints"),
                getattr(args, f"{side}_close_target"),
                getattr(args, f"{side}_open_target"),
                active_delta,
                tolerance,
                movement_threshold,
            )
            config = explicit or load_side_from_controller(
                node=node,
                side=side,
                controller=controller,
                controller_type=args.controller_type,
                timeout=args.startup_timeout,
                active_delta=active_delta,
                tolerance=tolerance,
                movement_threshold=movement_threshold,
                robot_description_topic=args.robot_description_topic,
            )
            configs.append(config)
            node.add_side(config)
            judged = [config.joints[index] for index in config.active_indices]
            print(
                f"{side}: 类型={config.controller_type}，控制器=/{config.controller}，"
                f"关节={len(config.joints)}，参与判定={len(judged)}，"
                f"到位容差={config.tolerance}，运动阈值={config.movement_threshold}")

        wait_for_subscribers(node, configs, args.startup_timeout)
        wait_for_feedback_joints(
            node, configs, args.feedback_timeout, args.joint_states_topic)
        if args.controller_type == "hand":
            # AdaptiveGripperController is not FSM-driven; only hand controllers
            # need the shared HOLD -> MOVEJ transition.
            wait_for_fsm_command_subscriber(node, args.fsm_timeout)
            fsm_control_started = True
            enter_movej(node, args.fsm_timeout, args.fsm_fallback_delay)
        print(
            f"开始：{args.frequency} Hz，每侧 {args.count} 条，"
            f"响应时限 {args.response_timeout:.3f}s，类型={args.controller_type}")

        results, _, run_end = run_trials(
            node=node,
            configs=configs,
            count=args.count,
            frequency=args.frequency,
            response_timeout=args.response_timeout,
            first_command=0 if args.first == "close" else 1,
            progress_every=args.progress_every,
            stop_event=stop_event,
        )
        interrupted = stop_event.is_set() or len(results) < args.count
        # Normal completion and Ctrl+C both return to HOLD before report output.
        # The outer finally remains as a second safety net for exceptions.
        if fsm_control_started:
            exit_to_hold(node, args.fsm_timeout, args.fsm_fallback_delay)
            fsm_control_started = False
        summary = build_summary(
            args, configs, results, started_at, run_end - run_start, interrupted)
        report_dir = write_reports(args.report_dir, summary, results, configs)
        print(f"报告目录：{report_dir.resolve()}")
        print(
            f"完成：总指令={summary['total_commands']}，"
            f"位置反馈推断未响应={summary['total_inferred_lost']}，"
            f"未响应率={summary['total_inferred_loss_rate_percent']:.4f}%")
        if interrupted:
            exit_code = 130
        else:
            exit_code = 2 if summary["total_inferred_lost"] else 0
    except Exception as exc:
        print(f"测试失败：{exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if fsm_control_started:
            try:
                exit_to_hold(node, args.fsm_timeout, args.fsm_fallback_delay)
            except Exception as exc:
                print(f"安全退出失败：未能确认进入 HOLD: {exc}", file=sys.stderr)
                exit_code = 1
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

