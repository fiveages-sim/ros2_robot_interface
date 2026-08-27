#!/usr/bin/env python3
"""按原始采样频率回放录制的左臂、右臂或双臂 position command interface。

脚本启动后扫描同目录 ``record_data/*/joint_interfaces.csv``，按修改时间倒序
列出所有录制会话并询问要回放哪一个，再选择双臂（默认）、左臂或右臂。
双臂优先通过统一话题一次发布 14 个关节；统一话题不存在时回退到左右分话题。
发布频率由 CSV ``stamp`` 的相邻正时间差中位数推算，失败时回退到
``elapsed_s``，不再固定假设为 500 Hz。

本脚本流程：
    1. 选择模式并完整校验 CSV，同时推算整数发布频率。
    2. 连接 ROS 2 并确认 HOLD；无条件使用 5 秒 linear MoveJ 发送录制首点，
       随后固定等待 5 秒，不读取当前位置或判断是否实际到位。
    3. 在 HOLD 中将插值设为 ``none`` 并完成安全倒计时，然后切换 MOVEJ。
    4. 以 ``--plot`` 启动 recorder，按原始采样频率从 CSV 第一行逐行发布。
    5. 最后一行发布后等待实际关节到位，再停止 recorder。
    6. finally 中切换 HOLD，恢复原有插值参数和 MoveJ 时长并销毁节点。

前置条件：
    - ROS 2 已 source，``ocs2_arm_controller`` 与真实机器人/仿真正在运行。
    - ``record_data`` 的会话子目录中存在 ``joint_interfaces.csv``。
    - CSV 包含所选手臂的 position command interface 列，单位为弧度。
    - 脚本不检查目标话题是否存在多个 publisher；使用者必须自行避免 teleop、
      RViz 或其他节点与回放脚本同时发送手臂控制指令。
    - 回放期间不得从其他节点修改控制器的 MoveJ 插值或时长参数。
    - 仿真环境必须追加 ``--ros-args -p use_sim_time:=true``，确保本节点与
      ``/joint_states`` 使用同一个 ROS 时钟。

运行（进入本脚本目录且 uv 环境已激活后）：
    python replay_joint_commands.py

安全说明：
    本脚本会让所选手臂高速回放录制动作。运行前确认机器人工作空间无人、
    无障碍物，并准备好急停。脚本保证逐行发布，但不能保证控制器或硬件
    执行每一条 DDS 消息。
"""

from __future__ import annotations

import csv
import math
import re
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

from _recording_process import RecorderProcess, finish_recorder, start_recorder


SCRIPT_DIR = Path(__file__).resolve().parent
RECORD_DATA_DIR = SCRIPT_DIR / "record_data"
CSV_FILENAME = "joint_interfaces.csv"

UNIFIED_TARGET_TOPIC = "/ocs2_arm_controller/target_joint_position"
LEFT_TARGET_TOPIC = f"{UNIFIED_TARGET_TOPIC}/left"
RIGHT_TARGET_TOPIC = f"{UNIFIED_TARGET_TOPIC}/right"
FSM_COMMAND_TOPIC = "/fsm_command"
FSM_STATE_TOPIC = "/fsm_state"
CONTROLLER_NODE = "/ocs2_arm_controller"
INTERPOLATION_PARAMETER = "movej_interpolation_type"
DURATION_PARAMETER = "movej_duration"
PASSTHROUGH_INTERPOLATION = "none"
POSITIONING_INTERPOLATION = "linear"
POSITIONING_DURATION_SEC = 5.0

FSM_HOLD = 2
FSM_MOVEJ = 4
DISCOVERY_TIMEOUT_SEC = 5.0
GRAPH_SETTLE_SEC = 0.5
FSM_TIMEOUT_SEC = 5.0
FSM_COMMAND_RETRY_SEC = 0.2
COUNTDOWN_SEC = 3
BUSY_WAIT_WINDOW_NS = 200_000
FINAL_POSITION_TOLERANCE_RAD = 0.03
FINAL_POSITION_TIMEOUT_SEC = 10.0
JOINT_TARGET_CONSECUTIVE_SAMPLES = 5
JOINT_STATE_MAX_CLOCK_LEAD_SEC = 0.005
JOINT_STATE_MAX_CLOCK_LAG_SEC = 1.0

MODE_DUAL = "dual"
MODE_LEFT = "left"
MODE_RIGHT = "right"
MODE_LABELS = {
    MODE_DUAL: "双臂",
    MODE_LEFT: "左臂",
    MODE_RIGHT: "右臂",
}

INTERFACE_COLUMN_RE = re.compile(
    r"^(?:state|command)_interface\.[^/]+/(?:position|velocity|effort)$"
)

LEFT_COMMAND_COLUMNS = tuple(
    f"command_interface.left_joint{i}/position" for i in range(1, 8)
)
RIGHT_COMMAND_COLUMNS = tuple(
    f"command_interface.right_joint{i}/position" for i in range(1, 8)
)
LEFT_JOINT_NAMES = tuple(f"left_joint{i}" for i in range(1, 8))
RIGHT_JOINT_NAMES = tuple(f"right_joint{i}" for i in range(1, 8))


@dataclass(frozen=True)
class ReplayData:
    commands: list[tuple[float, ...]]
    publish_rate_hz: int
    median_period_sec: float
    time_source: str


def discover_recordings(record_data_dir: Path = RECORD_DATA_DIR) -> list[Path]:
    """返回直接位于 record_data 会话子目录中的 CSV，最新的排在前面。"""
    if not record_data_dir.is_dir():
        return []
    recordings = [
        child / CSV_FILENAME
        for child in record_data_dir.iterdir()
        if child.is_dir() and (child / CSV_FILENAME).is_file()
    ]
    return sorted(recordings, key=lambda path: path.stat().st_mtime_ns, reverse=True)


def choose_recording(recordings: Sequence[Path]) -> Path:
    """打印编号列表并交互选择 CSV；直接回车选择最新会话。"""
    if not recordings:
        raise FileNotFoundError(
            f"未在 {RECORD_DATA_DIR} 的会话子目录中找到 {CSV_FILENAME}"
        )

    print("可用的关节接口录制：")
    for index, path in enumerate(recordings, start=1):
        modified = time.strftime(
            "%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime)
        )
        print(f"  [{index}] {path.parent.name}  ({modified})")

    while True:
        answer = input(f"请选择回放会话 [1-{len(recordings)}]（回车选择 1）: ").strip()
        if not answer:
            return recordings[0]
        try:
            selected = int(answer)
        except ValueError:
            print("请输入列表中的数字。")
            continue
        if 1 <= selected <= len(recordings):
            return recordings[selected - 1]
        print(f"请输入 1 到 {len(recordings)}。")


def choose_replay_mode() -> str:
    """交互选择双臂、左臂或右臂；直接回车默认双臂。"""
    print("回放模式：")
    print("  [1] 双臂（默认）")
    print("  [2] 左臂")
    print("  [3] 右臂")
    choices = {"1": MODE_DUAL, "2": MODE_LEFT, "3": MODE_RIGHT}
    while True:
        answer = input("请选择回放模式 [1-3]（回车选择 1）: ").strip()
        if not answer:
            return MODE_DUAL
        if answer in choices:
            return choices[answer]
        print("请输入 1、2 或 3。")


def command_columns_for_mode(mode: str) -> tuple[str, ...]:
    if mode == MODE_LEFT:
        return LEFT_COMMAND_COLUMNS
    if mode == MODE_RIGHT:
        return RIGHT_COMMAND_COLUMNS
    if mode == MODE_DUAL:
        return LEFT_COMMAND_COLUMNS + RIGHT_COMMAND_COLUMNS
    raise ValueError(f"未知回放模式: {mode}")


def joint_names_for_mode(mode: str) -> tuple[str, ...]:
    if mode == MODE_LEFT:
        return LEFT_JOINT_NAMES
    if mode == MODE_RIGHT:
        return RIGHT_JOINT_NAMES
    if mode == MODE_DUAL:
        return LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
    raise ValueError(f"未知回放模式: {mode}")


def select_target_route(
    mode: str,
    *,
    unified_subscribers: int,
    left_subscribers: int,
    right_subscribers: int,
) -> tuple[tuple[str, ...], str] | None:
    """按统一双臂优先策略选择目标话题；未发现可用路由时返回 ``None``。"""
    if mode == MODE_LEFT and left_subscribers > 0:
        return (LEFT_TARGET_TOPIC,), "左臂分话题"
    if mode == MODE_RIGHT and right_subscribers > 0:
        return (RIGHT_TARGET_TOPIC,), "右臂分话题"
    if mode == MODE_DUAL and unified_subscribers > 0:
        return (UNIFIED_TARGET_TOPIC,), "统一双臂话题（14关节）"
    if mode == MODE_DUAL and left_subscribers > 0 and right_subscribers > 0:
        return (LEFT_TARGET_TOPIC, RIGHT_TARGET_TOPIC), "左右分话题回退（7+7关节）"
    return None


def _positive_adjacent_intervals(values: Sequence[float]) -> list[float]:
    return [
        current - previous
        for previous, current in zip(values, values[1:])
        if math.isfinite(previous)
        and math.isfinite(current)
        and current > previous
    ]


def detect_publish_rate(
    stamp_values: Sequence[float],
    elapsed_values: Sequence[float],
) -> tuple[int, float, str]:
    """用相邻正时间差中位数推算整数 Hz，优先 ``stamp``。"""
    for source, values in (("stamp", stamp_values), ("elapsed_s", elapsed_values)):
        intervals = _positive_adjacent_intervals(values)
        if not intervals:
            continue
        median_period_sec = statistics.median(intervals)
        raw_rate_hz = 1.0 / median_period_sec
        publish_rate_hz = int(math.floor(raw_rate_hz + 0.5))
        if publish_rate_hz < 1:
            continue
        publish_period_ns = round(1_000_000_000 / publish_rate_hz)
        if publish_period_ns < 1:
            continue
        return publish_rate_hz, median_period_sec, source
    raise ValueError(
        "CSV 的 stamp 与 elapsed_s 都没有足够的递增时间数据，无法推算发布频率"
    )


def _parse_optional_time(row: dict[str, str], column: str) -> float:
    try:
        value = float(row[column])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def load_replay_data(csv_path: Path, mode: str) -> ReplayData:
    """在运动前读取所选关节指令并从 CSV 时间列推算发布频率。"""
    command_columns = command_columns_for_mode(mode)
    commands: list[tuple[float, ...]] = []
    stamp_values: list[float] = []
    elapsed_values: list[float] = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        missing = [name for name in command_columns if name not in fieldnames]
        if missing:
            raise ValueError(f"CSV 缺少{MODE_LABELS[mode]}指令列: {missing}")

        for line_number, row in enumerate(reader, start=2):
            # recorder 在接口集合动态增加时可能再次写入表头，该行不是数据。
            extra_columns = row.get(None) or []
            if (
                all(row.get(name) == name for name in fieldnames)
                and all(
                    isinstance(value, str) and INTERFACE_COLUMN_RE.fullmatch(value)
                    for value in extra_columns
                )
            ):
                continue
            try:
                command = tuple(float(row[name]) for name in command_columns)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"CSV 第 {line_number} 行{MODE_LABELS[mode]}指令无效: {exc}"
                ) from exc
            if not all(math.isfinite(value) for value in command):
                raise ValueError(f"CSV 第 {line_number} 行包含 NaN 或 Inf: {command}")
            commands.append(command)
            stamp_values.append(_parse_optional_time(row, "stamp"))
            elapsed_values.append(_parse_optional_time(row, "elapsed_s"))

    if not commands:
        raise ValueError(f"CSV 中没有可回放的{MODE_LABELS[mode]}关节指令")
    publish_rate_hz, median_period_sec, time_source = detect_publish_rate(
        stamp_values,
        elapsed_values,
    )
    return ReplayData(
        commands=commands,
        publish_rate_hz=publish_rate_hz,
        median_period_sec=median_period_sec,
        time_source=time_source,
    )


class ReplayNode(Node):
    """回放所需的 publisher、FSM 状态订阅与参数客户端。"""

    def __init__(self, mode: str) -> None:
        super().__init__("joint_command_replayer")
        self.mode = mode
        self.mode_label = MODE_LABELS[mode]
        self.joint_names = joint_names_for_mode(mode)
        self.latest_fsm_state: int | None = None
        self.latest_joint_positions: tuple[float, ...] | None = None
        self.latest_joint_received_at: float | None = None
        self.latest_joint_stamp_ns: int | None = None
        self.latest_joint_sequence = 0
        self.target_topics: tuple[str, ...] = ()
        self.target_pubs = []
        self.target_route = ""
        self.fsm_command_pub = self.create_publisher(Int32, FSM_COMMAND_TOPIC, 10)
        fsm_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.fsm_state_sub = self.create_subscription(
            Int32, FSM_STATE_TOPIC, self._fsm_state_callback, fsm_qos
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.parameter_client = AsyncParameterClient(self, CONTROLLER_NODE)

    def _fsm_state_callback(self, msg: Int32) -> None:
        self.latest_fsm_state = int(msg.data)

    def _joint_state_callback(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        if all(name in positions_by_name for name in self.joint_names):
            self.latest_joint_positions = tuple(
                float(positions_by_name[name]) for name in self.joint_names
            )
            self.latest_joint_received_at = time.monotonic()
            self.latest_joint_stamp_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )
            self.latest_joint_sequence += 1

    def configure_target_publishers(self) -> None:
        """根据模式和 ROS 图选择单臂、统一双臂或分话题双臂路由。"""
        settle_deadline = time.monotonic() + GRAPH_SETTLE_SEC
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node=self, timeout_sec=0.05)

        deadline = time.monotonic() + DISCOVERY_TIMEOUT_SEC
        split_fallback: tuple[tuple[str, ...], str] | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            unified_subscribers = self.count_subscribers(UNIFIED_TARGET_TOPIC)
            left_subscribers = self.count_subscribers(LEFT_TARGET_TOPIC)
            right_subscribers = self.count_subscribers(RIGHT_TARGET_TOPIC)
            route = select_target_route(
                self.mode,
                unified_subscribers=unified_subscribers,
                left_subscribers=left_subscribers,
                right_subscribers=right_subscribers,
            )
            if self.mode != MODE_DUAL and route is not None:
                self.target_topics, self.target_route = route
                break
            if self.mode == MODE_DUAL and unified_subscribers > 0:
                self.target_topics, self.target_route = route
                break
            if self.mode == MODE_DUAL and route is not None:
                split_fallback = route
            rclpy.spin_once(node=self, timeout_sec=0.05)

        if not rclpy.ok():
            raise RuntimeError("ROS 2 已停止，无法配置回放目标话题")
        if not self.target_topics and split_fallback is not None:
            self.target_topics, self.target_route = split_fallback
        if not self.target_topics:
            raise RuntimeError(
                "未发现所选模式需要的目标订阅者: "
                f"unified={self.count_subscribers(UNIFIED_TARGET_TOPIC)}, "
                f"left={self.count_subscribers(LEFT_TARGET_TOPIC)}, "
                f"right={self.count_subscribers(RIGHT_TARGET_TOPIC)}"
            )

        self.target_pubs = [
            self.create_publisher(Float64MultiArray, topic, 10)
            for topic in self.target_topics
        ]
        print(
            f"目标路由: {self.target_route} -> {', '.join(self.target_topics)}"
        )

    def build_target_messages(
        self, target: Sequence[float]
    ) -> tuple[Float64MultiArray, ...]:
        if len(target) != len(self.joint_names):
            raise ValueError(
                f"{self.mode_label}目标维度错误: "
                f"got={len(target)}, expected={len(self.joint_names)}"
            )
        target_parts: tuple[Sequence[float], ...]
        if self.mode == MODE_DUAL and len(self.target_topics) == 2:
            target_parts = (target[:7], target[7:])
        else:
            target_parts = (target,)
        messages = []
        for part in target_parts:
            msg = Float64MultiArray()
            msg.data = list(part)
            messages.append(msg)
        return tuple(messages)

    def publish_target_messages(
        self, messages: Sequence[Float64MultiArray]
    ) -> None:
        if len(messages) != len(self.target_pubs):
            raise RuntimeError(
                f"目标消息与 publisher 数量不一致: "
                f"messages={len(messages)}, publishers={len(self.target_pubs)}"
            )
        for publisher, message in zip(self.target_pubs, messages):
            publisher.publish(message)


def wait_for_future(node: Node, future, timeout_sec: float, operation: str):
    """在这个独立节点上等待异步参数请求完成并返回结果。"""
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(f"{operation} 超时（{timeout_sec:.1f}s）")
    exception = future.exception()
    if exception is not None:
        raise RuntimeError(f"{operation} 失败: {exception}") from exception
    result = future.result()
    if result is None:
        raise RuntimeError(f"{operation} 未返回结果")
    return result


def get_interpolation_parameter(node: ReplayNode) -> str:
    if not node.parameter_client.wait_for_services(timeout_sec=DISCOVERY_TIMEOUT_SEC):
        raise RuntimeError(f"参数服务不可用: {CONTROLLER_NODE}")
    response = wait_for_future(
        node,
        node.parameter_client.get_parameters([INTERPOLATION_PARAMETER]),
        DISCOVERY_TIMEOUT_SEC,
        f"读取 {CONTROLLER_NODE}.{INTERPOLATION_PARAMETER}",
    )
    if not response.values:
        raise RuntimeError(f"控制器未返回参数: {INTERPOLATION_PARAMETER}")
    value = response.values[0]
    if value.type != Parameter.Type.STRING.value:
        raise RuntimeError(
            f"参数 {INTERPOLATION_PARAMETER} 不是字符串类型，type={value.type}"
        )
    return value.string_value


def set_interpolation_parameter(node: ReplayNode, value: str) -> None:
    response = wait_for_future(
        node,
        node.parameter_client.set_parameters(
            [Parameter(INTERPOLATION_PARAMETER, value=value)]
        ),
        DISCOVERY_TIMEOUT_SEC,
        f"设置 {CONTROLLER_NODE}.{INTERPOLATION_PARAMETER}={value}",
    )
    if not response.results or not response.results[0].successful:
        reason = response.results[0].reason if response.results else "无返回结果"
        raise RuntimeError(f"设置 {INTERPOLATION_PARAMETER}={value} 失败: {reason}")


def get_duration_parameter(node: ReplayNode) -> float:
    response = wait_for_future(
        node,
        node.parameter_client.get_parameters([DURATION_PARAMETER]),
        DISCOVERY_TIMEOUT_SEC,
        f"读取 {CONTROLLER_NODE}.{DURATION_PARAMETER}",
    )
    if not response.values:
        raise RuntimeError(f"控制器未返回参数: {DURATION_PARAMETER}")
    value = response.values[0]
    if value.type != Parameter.Type.DOUBLE.value:
        raise RuntimeError(
            f"参数 {DURATION_PARAMETER} 不是浮点类型，type={value.type}"
        )
    return float(value.double_value)


def set_duration_parameter(node: ReplayNode, value: float) -> None:
    response = wait_for_future(
        node,
        node.parameter_client.set_parameters(
            [Parameter(DURATION_PARAMETER, value=float(value))]
        ),
        DISCOVERY_TIMEOUT_SEC,
        f"设置 {CONTROLLER_NODE}.{DURATION_PARAMETER}={value}",
    )
    if not response.results or not response.results[0].successful:
        reason = response.results[0].reason if response.results else "无返回结果"
        raise RuntimeError(f"设置 {DURATION_PARAMETER}={value} 失败: {reason}")


def wait_for_required_subscribers(node: ReplayNode) -> None:
    deadline = time.monotonic() + DISCOVERY_TIMEOUT_SEC
    while rclpy.ok() and time.monotonic() < deadline:
        if (
            node.target_pubs
            and all(pub.get_subscription_count() > 0 for pub in node.target_pubs)
            and node.fsm_command_pub.get_subscription_count() > 0
        ):
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError(
        "未发现必要订阅者: "
        f"targets={[pub.get_subscription_count() for pub in node.target_pubs]}, "
        f"fsm_command={node.fsm_command_pub.get_subscription_count()}"
    )


def command_fsm_state(node: ReplayNode, expected: int) -> None:
    """重复发送 FSM 命令并等待状态确认，避免 VOLATILE 首帧丢失。"""
    deadline = time.monotonic() + FSM_TIMEOUT_SEC
    next_publish = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        if node.latest_fsm_state == expected:
            return
        now = time.monotonic()
        if now >= next_publish:
            publish_fsm_command(node, expected)
            next_publish = now + FSM_COMMAND_RETRY_SEC
        rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError(
        f"FSM 未进入状态 {expected}，当前状态: {node.latest_fsm_state}"
    )


def publish_fsm_command(node: ReplayNode, command: int) -> None:
    msg = Int32()
    msg.data = command
    node.fsm_command_pub.publish(msg)


def enter_hold(node: ReplayNode) -> tuple[float, int]:
    """读取并确认 HOLD，返回单调时钟和 ROS 时钟确认边界。"""
    # 先 spin 一次以读取 TRANSIENT_LOCAL 的当前 FSM 状态。
    rclpy.spin_once(node, timeout_sec=0.5)
    if node.latest_fsm_state == FSM_HOLD:
        return time.monotonic(), node.get_clock().now().nanoseconds
    print(f"切换 FSM: {node.latest_fsm_state} -> HOLD")
    command_fsm_state(node, FSM_HOLD)
    return time.monotonic(), node.get_clock().now().nanoseconds


def enter_movej(node: ReplayNode) -> None:
    """从已确认的 HOLD 切换到 MOVEJ，并确认最终状态。"""
    if node.latest_fsm_state != FSM_HOLD:
        raise RuntimeError(
            f"进入 MOVEJ 前 FSM 不是 HOLD，当前状态: {node.latest_fsm_state}"
        )
    print("切换 FSM: HOLD -> MOVEJ")
    command_fsm_state(node, FSM_MOVEJ)


def validate_joint_state_clock(
    node: ReplayNode,
    stamp_ns: int,
    *,
    label: str,
) -> None:
    """确认状态消息时间戳与本节点 ROS 时钟同域且偏差足够小。"""
    if stamp_ns <= 0:
        raise RuntimeError(
            f"{label}: /joint_states header.stamp 无效，无法建立新鲜数据屏障"
        )
    now_ns = node.get_clock().now().nanoseconds
    clock_delta_sec = (stamp_ns - now_ns) / 1_000_000_000
    if (
        clock_delta_sec > JOINT_STATE_MAX_CLOCK_LEAD_SEC
        or clock_delta_sec < -JOINT_STATE_MAX_CLOCK_LAG_SEC
    ):
        raise RuntimeError(
            f"{label}: /joint_states 与回放节点 ROS 时钟不同步，"
            f"joint_stamp-node_now={clock_delta_sec:+.6f}s；"
            "仿真请使用 --ros-args -p use_sim_time:=true，"
            "真机请检查消息源时钟同步"
        )


def maximum_joint_error(
    current: Sequence[float], target: Sequence[float]
) -> float:
    if len(current) != len(target):
        raise ValueError(
            f"关节维度不一致: current={len(current)}, target={len(target)}"
        )
    return max(abs(actual - expected) for actual, expected in zip(current, target))


def wait_for_joint_target(
    node: ReplayNode,
    target: Sequence[float],
    *,
    label: str,
    not_before: float,
    after_sequence: int,
    after_stamp_ns: int,
    timeout_sec: float,
    tolerance_rad: float,
) -> None:
    """等待新鲜关节状态连续多帧处于目标容差内。"""
    deadline = time.monotonic() + timeout_sec
    consecutive = 0
    last_error: float | None = None
    last_processed_sequence = after_sequence
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        received_at = node.latest_joint_received_at
        current = node.latest_joint_positions
        sequence = node.latest_joint_sequence
        stamp_ns = node.latest_joint_stamp_ns
        if (
            current is None
            or received_at is None
            or stamp_ns is None
            or received_at < not_before
            or sequence <= last_processed_sequence
        ):
            continue
        last_processed_sequence = sequence
        validate_joint_state_clock(node, stamp_ns, label=label)
        if stamp_ns <= after_stamp_ns:
            continue
        if not all(math.isfinite(value) for value in current):
            raise RuntimeError(f"{label}期间 /joint_states 包含 NaN 或 Inf: {current}")
        last_error = maximum_joint_error(current, target)
        if last_error <= tolerance_rad:
            consecutive += 1
            if consecutive >= JOINT_TARGET_CONSECUTIVE_SAMPLES:
                print(f"{label}已到位，最大关节误差: {last_error:.6f} rad")
                return
        else:
            consecutive = 0
    raise RuntimeError(
        f"{label}等待到位超时（{timeout_sec:.1f}s），"
        f"最大关节误差: {last_error} rad"
    )


def move_to_start_position(
    node: ReplayNode,
    first_command: Sequence[float],
) -> None:
    """无条件发送录制首点，固定等待预定位时长后重新进入 HOLD。"""
    print(f"录制首点: {[f'{value:.6f}' for value in first_command]}")
    print(
        "设置首点预定位参数: "
        f"{INTERPOLATION_PARAMETER}={POSITIONING_INTERPOLATION}, "
        f"{DURATION_PARAMETER}={POSITIONING_DURATION_SEC:.1f}s"
    )
    set_duration_parameter(node, POSITIONING_DURATION_SEC)
    set_interpolation_parameter(node, POSITIONING_INTERPOLATION)
    confirmed = get_interpolation_parameter(node)
    confirmed_duration = get_duration_parameter(node)
    if confirmed.lower() != POSITIONING_INTERPOLATION:
        raise RuntimeError(
            "首点预定位插值参数确认失败: "
            f"expected={POSITIONING_INTERPOLATION}, actual={confirmed}"
        )
    if not math.isclose(
        confirmed_duration, POSITIONING_DURATION_SEC, rel_tol=0.0, abs_tol=1e-6
    ):
        raise RuntimeError(
            "首点预定位时长参数确认失败: "
            f"expected={POSITIONING_DURATION_SEC}, actual={confirmed_duration}"
        )
    countdown(
        node,
        f"即将使用 {POSITIONING_DURATION_SEC:.1f} 秒 linear MoveJ "
        "移动到录制首点，请准备好急停。",
    )
    enter_movej(node)
    node.publish_target_messages(node.build_target_messages(first_command))
    print(
        "已发送录制首点指令，固定等待 "
        f"{POSITIONING_DURATION_SEC:.1f}s（不检查实际到位）"
    )
    time.sleep(POSITIONING_DURATION_SEC)
    enter_hold(node)


def countdown(node: ReplayNode, message: str) -> None:
    """倒计时期间持续处理 ROS 回调，并要求 FSM 始终保持 HOLD。"""
    print(message)
    for remaining in range(COUNTDOWN_SEC, 0, -1):
        if not rclpy.ok():
            raise RuntimeError("ROS 2 已停止，取消安全倒计时")
        print(f"  {remaining}...")
        second_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < second_deadline:
            remaining_sec = second_deadline - time.monotonic()
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining_sec))
            if node.latest_fsm_state != FSM_HOLD:
                raise RuntimeError(
                    "安全倒计时期间 FSM 离开 HOLD，"
                    f"当前状态: {node.latest_fsm_state}"
                )
        if not rclpy.ok():
            raise RuntimeError("ROS 2 已停止，取消安全倒计时")


def ensure_replay_health(node: ReplayNode) -> None:
    """recorder 就绪等待期间持续处理回调并验证回放控制状态。"""
    if not rclpy.ok():
        raise RuntimeError("ROS 2 已停止，取消回放")
    rclpy.spin_once(node, timeout_sec=0.0)
    if node.latest_fsm_state != FSM_MOVEJ:
        raise RuntimeError(
            "recorder 就绪等待期间 FSM 离开 MOVEJ，"
            f"当前状态: {node.latest_fsm_state}"
        )


def hold_after_recorder_start_failure(node: ReplayNode) -> None:
    """recorder 启动失败时先确认 HOLD，再等待录制子进程绘图退出。"""
    if not rclpy.ok():
        return
    print("[safety] recorder 启动失败，立即切换 FSM 到 HOLD")
    command_fsm_state(node, FSM_HOLD)


def wait_until_ns(deadline_ns: int) -> None:
    """绝对时间等待；末尾短暂忙等以减小发布周期的 sleep 抖动。"""
    while True:
        remaining_ns = deadline_ns - time.monotonic_ns()
        if remaining_ns <= 0:
            return
        if remaining_ns > BUSY_WAIT_WINDOW_NS:
            time.sleep((remaining_ns - BUSY_WAIT_WINDOW_NS) / 1_000_000_000)


def replay_commands(
    node: ReplayNode,
    commands: Sequence[Sequence[float]],
    publish_rate_hz: int,
) -> tuple[float, int, int]:
    """按推算频率的绝对时间轴发布；严重迟到时重基线而不突发补发。"""
    publish_period_ns = round(1_000_000_000 / publish_rate_hz)
    messages = [node.build_target_messages(command) for command in commands]

    start_ns = time.monotonic_ns()
    schedule_origin_ns = start_ns
    maximum_lateness_ns = 0
    overrun_count = 0
    rebase_count = 0
    schedule_shift_ns = 0
    last_publish_at = time.monotonic()
    sequence_before_last_publish = node.latest_joint_sequence
    last_stamp_boundary_ns = node.get_clock().now().nanoseconds

    for index, msg in enumerate(messages):
        if not rclpy.ok():
            raise RuntimeError("ROS 2 已停止，终止回放")
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.latest_fsm_state != FSM_MOVEJ:
            raise RuntimeError(
                f"回放期间 FSM 离开 MOVEJ，当前状态: {node.latest_fsm_state}"
            )
        deadline_ns = schedule_origin_ns + index * publish_period_ns
        wait_until_ns(deadline_ns)
        now_ns = time.monotonic_ns()
        lateness_ns = max(0, now_ns - deadline_ns)
        maximum_lateness_ns = max(maximum_lateness_ns, lateness_ns)
        if lateness_ns >= publish_period_ns:
            overrun_count += 1
        sequence_before_last_publish = node.latest_joint_sequence
        last_stamp_boundary_ns = node.get_clock().now().nanoseconds
        node.publish_target_messages(msg)
        last_publish_at = time.monotonic()

        # 正常时保持 start + index * period 的绝对时间轴，不累计 publish 耗时。
        # 如果 publish 返回时下一截止点已过，则把时间轴移到“当前 + 1周期”，
        # 避免下一条立即突发补发。
        after_publish_ns = time.monotonic_ns()
        next_deadline_ns = schedule_origin_ns + (index + 1) * publish_period_ns
        if after_publish_ns >= next_deadline_ns:
            shift_ns = after_publish_ns + publish_period_ns - next_deadline_ns
            schedule_origin_ns += shift_ns
            schedule_shift_ns += shift_ns
            rebase_count += 1

    end_ns = time.monotonic_ns()
    elapsed_sec = (end_ns - start_ns) / 1_000_000_000
    achieved_hz = (len(messages) - 1) / elapsed_sec if elapsed_sec > 0 else 0.0
    print(
        f"回放完成: rate={publish_rate_hz}Hz, rows={len(messages)}, "
        f"messages={len(messages) * len(node.target_pubs)}, "
        f"elapsed={elapsed_sec:.6f}s, "
        f"effective_rate={achieved_hz:.2f}Hz"
    )
    print(
        f"调度统计: overruns(>={publish_period_ns / 1_000_000:.3f}ms)="
        f"{overrun_count}, "
        f"max_lateness={maximum_lateness_ns / 1_000_000:.3f}ms, "
        f"rebases={rebase_count}, "
        f"schedule_shift={schedule_shift_ns / 1_000_000:.3f}ms"
    )
    return (
        last_publish_at,
        sequence_before_last_publish,
        last_stamp_boundary_ns,
    )


def cleanup(
    node: ReplayNode | None,
    original_interpolation: str | None,
    original_duration: float | None,
) -> list[str]:
    """尽力完成 HOLD、参数恢复、节点销毁，并返回所有清理失败。"""
    if node is None:
        return []
    failures: list[str] = []
    try:
        print("\n[cleanup] 切换 FSM 到 HOLD")
        try:
            command_fsm_state(node, FSM_HOLD)
        except BaseException as exc:
            message = f"HOLD 确认失败: {exc}"
            failures.append(message)
            print(f"[cleanup] 警告: {message}")

        if original_interpolation is not None:
            print(
                f"[cleanup] 恢复 {INTERPOLATION_PARAMETER}="
                f"{original_interpolation}"
            )
            try:
                set_interpolation_parameter(node, original_interpolation)
                restored = get_interpolation_parameter(node)
                if restored != original_interpolation:
                    raise RuntimeError(
                        f"恢复后参数不一致: expected={original_interpolation}, actual={restored}"
                    )
            except BaseException as exc:
                message = f"恢复插值参数失败: {exc}"
                failures.append(message)
                print(f"[cleanup] 警告: {message}")
        if original_duration is not None:
            print(f"[cleanup] 恢复 {DURATION_PARAMETER}={original_duration}")
            try:
                set_duration_parameter(node, original_duration)
                restored_duration = get_duration_parameter(node)
                if not math.isclose(
                    restored_duration,
                    original_duration,
                    rel_tol=0.0,
                    abs_tol=1e-6,
                ):
                    raise RuntimeError(
                        "恢复后参数不一致: "
                        f"expected={original_duration}, actual={restored_duration}"
                    )
            except BaseException as exc:
                message = f"恢复 MoveJ 时长参数失败: {exc}"
                failures.append(message)
                print(f"[cleanup] 警告: {message}")
    finally:
        try:
            node.destroy_node()
        except BaseException as exc:
            message = f"销毁 ROS 节点失败: {exc}"
            failures.append(message)
            print(f"[cleanup] 警告: {message}")
    return failures


def main() -> int:
    recordings = discover_recordings()
    selected_csv = choose_recording(recordings)
    print(f"已选择: {selected_csv}")

    mode = choose_replay_mode()
    replay_data = load_replay_data(selected_csv, mode)
    commands = replay_data.commands
    expected_duration = (len(commands) - 1) / replay_data.publish_rate_hz
    print(f"已选择回放模式: {MODE_LABELS[mode]}")
    print(
        f"频率检测: source={replay_data.time_source}, "
        f"median_dt={replay_data.median_period_sec:.9f}s, "
        f"publish_rate={replay_data.publish_rate_hz}Hz"
    )
    print(
        f"已加载 {len(commands)} 行{MODE_LABELS[mode]}指令，"
        f"预计回放 {expected_duration:.3f}s"
    )
    print(f"首点: {[f'{value:.6f}' for value in commands[0]]}")
    print(f"末点: {[f'{value:.6f}' for value in commands[-1]]}")

    node: ReplayNode | None = None
    recorder: RecorderProcess | None = None
    original_interpolation: str | None = None
    original_duration: float | None = None
    rclpy.init()
    try:
        node = ReplayNode(mode)
        node.configure_target_publishers()
        wait_for_required_subscribers(node)
        enter_hold(node)

        original_interpolation = get_interpolation_parameter(node)
        original_duration = get_duration_parameter(node)
        print(f"原插值参数: {INTERPOLATION_PARAMETER}={original_interpolation}")
        print(f"原 MoveJ 时长: {DURATION_PARAMETER}={original_duration:.3f}s")
        move_to_start_position(node, commands[0])

        set_interpolation_parameter(node, PASSTHROUGH_INTERPOLATION)
        confirmed = get_interpolation_parameter(node)
        if confirmed.lower() != PASSTHROUGH_INTERPOLATION:
            raise RuntimeError(
                f"插值参数确认失败: expected={PASSTHROUGH_INTERPOLATION}, actual={confirmed}"
            )
        print(f"已设置: {INTERPOLATION_PARAMETER}={confirmed}")

        countdown(
            node,
            f"即将开始{node.mode_label} {replay_data.publish_rate_hz} Hz "
            "指令回放，请确认工作空间安全并准备好急停。",
        )
        enter_movej(node)
        print("启动回放过程的关节接口录制（--plot）...")
        recorder = start_recorder(
            f"replay_{mode}",
            health_check=lambda: ensure_replay_health(node),
            failure_cleanup=lambda: hold_after_recorder_start_failure(node),
        )
        ensure_replay_health(node)
        (
            last_publish_at,
            sequence_before_last_publish,
            last_stamp_boundary_ns,
        ) = replay_commands(node, commands, replay_data.publish_rate_hz)
        wait_for_joint_target(
            node,
            commands[-1],
            label="回放终点",
            not_before=last_publish_at,
            after_sequence=sequence_before_last_publish,
            after_stamp_ns=last_stamp_boundary_ns,
            timeout_sec=FINAL_POSITION_TIMEOUT_SEC,
            tolerance_rad=FINAL_POSITION_TOLERANCE_RAD,
        )
        return 0
    finally:
        active_exception = sys.exc_info()[0] is not None
        failures: list[str] = []
        if recorder is not None:
            try:
                recorder.request_stop()
            except BaseException as exc:
                failures.append(f"请求 recorder 停止失败: {exc}")

        failures.extend(cleanup(node, original_interpolation, original_duration))
        recorder_interrupt: KeyboardInterrupt | None = None
        try:
            finish_recorder(recorder)
        except KeyboardInterrupt as exc:
            recorder_interrupt = exc
        except BaseException as exc:
            failures.append(f"recorder/plot 清理失败: {exc}")
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException as exc:
                failures.append(f"ROS 2 shutdown 失败: {exc}")

        if recorder_interrupt is not None:
            raise recorder_interrupt
        if failures:
            print(f"[cleanup] 未完整成功: {'; '.join(failures)}")
            if not active_exception:
                raise RuntimeError("清理未完整成功: " + "; ".join(failures))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n用户中断")
        sys.exit(0)
    except Exception as exc:  # noqa: BLE001
        print(f"执行失败: {exc}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
