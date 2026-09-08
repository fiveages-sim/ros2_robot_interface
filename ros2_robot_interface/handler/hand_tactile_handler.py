"""
Hand Tactile Handler - 单手触觉处理器

封装左手或右手灵巧手五指触觉阵列的订阅与缓存。

数据源为 can-ros2-control 的 LinkerHand 硬件插件（O6 / L6 / O7），
话题形如 /<o6|l6|o7>_hand/<left|right>/tactile/<finger>，
消息类型 std_msgs/msg/UInt8MultiArray，layout.dim 为 [row, column]，data 行优先。
仅当硬件以 read_tactile:=true 启动时这些话题才存在。

另提供 get_rate() 用于统计每根手指的缓存更新频率（即回调触发频率），
供测试期核对是否与 `ros2 topic hz` 的读数一致。
"""

import logging
import time
from collections import deque
from enum import Enum
from typing import Callable, Deque, Dict, Optional, TYPE_CHECKING

from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import UInt8MultiArray

from ..utils.exceptions import ROS2InterfaceError

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from ..config import ROS2RobotInterfaceConfig


# 与 can-ros2-control 的 LinkerHandCanHardware::finger_name() 一一对应
TACTILE_FINGERS = ("thumb", "index", "middle", "ring", "pinky")

# finger 传这个值表示一次取回五根手指
TACTILE_ALL = "all"

# 频率统计窗口：保留最近这么多个"帧间隔"，与 ros2 topic hz 的
# DEFAULT_WINDOW_SIZE 取值一致，便于两边读数直接对照
RATE_WINDOW_SIZE = 10000

# 距最后一帧超过这么久就认为话题已停，get_rate() 返回 0.0。
# 对应 ros2 topic hz 里 "msg_tn < last_printed_tn + 1e9" 那道 1 秒判据
RATE_STALE_SEC = 1.0


class HandType(Enum):
    """灵巧手侧别枚举"""
    LEFT = "left"
    RIGHT = "right"


class HandTactileHandler:
    """单手触觉处理器 - 管理一侧灵巧手五指触觉话题的订阅与缓存

    负责管理单侧灵巧手的：
    - 五个触觉话题的订阅创建与销毁
    - 每根手指最新一条消息的缓存
    - 按手指名读取缓存
    """

    def __init__(
        self,
        node: Node,
        hand_type: HandType,
        config: "ROS2RobotInterfaceConfig"
    ):
        """
        初始化单手触觉处理器

        Args:
            node: ROS 2 节点
            hand_type: 灵巧手侧别（LEFT 或 RIGHT）
            config: ROS2RobotInterfaceConfig 配置对象
        """
        self.node = node
        self.hand_type = hand_type
        self.config = config
        self.label = f"{hand_type.value.upper()}_HAND_TACTILE"

        # 根据侧别确定触觉话题前缀
        if hand_type == HandType.LEFT:
            self.topic_prefix = config.left_hand_tactile_topic_prefix
        else:  # RIGHT
            self.topic_prefix = config.right_hand_tactile_topic_prefix

        # 订阅句柄与缓存：键为手指名
        self.tactile_subs: Dict[str, Optional[Subscription]] = {
            finger: None for finger in TACTILE_FINGERS
        }
        self._latest: Dict[str, Optional[UInt8MultiArray]] = {
            finger: None for finger in TACTILE_FINGERS
        }
        # 帧间隔（秒）与最后一帧时刻，仅用于 get_rate() 统计频率。
        # 存间隔而非时刻，与 ros2 topic hz 的 self.times 一致；
        # deque 的 maxlen 天然实现它那句 "len > window_size -> pop(0)"
        self._intervals: Dict[str, Deque[float]] = {
            finger: deque(maxlen=RATE_WINDOW_SIZE) for finger in TACTILE_FINGERS
        }
        self._last_stamp: Dict[str, Optional[float]] = {
            finger: None for finger in TACTILE_FINGERS
        }

    def initialize(self) -> None:
        """创建五个手指的触觉订阅"""
        if not self.topic_prefix:
            logger.debug(f"{self.label}: no tactile topic prefix configured, skip subscriptions")
            return

        for finger in TACTILE_FINGERS:
            topic = f"{self.topic_prefix}/{finger}"
            self.tactile_subs[finger] = self.node.create_subscription(
                UInt8MultiArray, topic,
                self._make_callback(finger), 10
            )
            logger.debug(f"{self.label}: Created tactile subscription for {topic}")

    def _make_callback(self, finger: str) -> Callable[[UInt8MultiArray], None]:
        """为指定手指生成回调（只写缓存与收包时刻，不做任何数据转换）"""
        def _callback(msg: UInt8MultiArray) -> None:
            try:
                self._latest[finger] = msg
                # 记录帧间隔：首帧只落时刻、不产生间隔（对应 ros2 topic hz 里
                # msg_t0 < 0 的分支）。deque 的 maxlen 自动丢弃最老的间隔。
                # get_rate() 只读不写，写入全部在这里做。
                now = time.monotonic()
                previous = self._last_stamp[finger]
                if previous is not None:
                    self._intervals[finger].append(now - previous)
                self._last_stamp[finger] = now
            except Exception as e:
                logger.error(
                    f"{self.label}: error in {finger} tactile callback: {e}",
                    exc_info=True
                )
        return _callback

    def get(self, finger: str) -> "UInt8MultiArray | Dict[str, UInt8MultiArray]":
        """返回触觉消息对象（不做拷贝，也不做 reshape）

        Args:
            finger: 手指名，取 thumb / index / middle / ring / pinky，
                或 ``"all"`` 表示一次取回五根手指（大小写不敏感）

        Returns:
            单指时返回 std_msgs/msg/UInt8MultiArray（layout.dim 为 [row, column]，
            data 行优先）；``finger="all"`` 时返回 ``{手指名: 消息}`` 字典。

        Raises:
            ValueError: 手指名非法
            ROS2InterfaceError: 该侧未配置触觉话题前缀，或尚未收到对应手指的消息
        """
        finger_key = str(finger).strip().lower()

        if finger_key == TACTILE_ALL:
            return {name: self._get_one(name) for name in TACTILE_FINGERS}

        if finger_key not in TACTILE_FINGERS:
            raise ValueError(
                f"finger must be one of {TACTILE_FINGERS} or "
                f"{TACTILE_ALL!r}, got: {finger!r}"
            )
        return self._get_one(finger_key)

    def _get_one(self, finger_key: str) -> UInt8MultiArray:
        """读取单根手指的缓存；finger_key 必须已归一化且合法"""
        if not self.topic_prefix:
            raise ROS2InterfaceError(
                f"No hand tactile topic prefix configured for side="
                f"{self.hand_type.value!r}; start can-ros2-control with "
                f"read_tactile:=true, or set config."
                f"{self.hand_type.value}_hand_tactile_topic_prefix before connect()"
            )

        msg = self._latest[finger_key]
        if msg is None:
            raise ROS2InterfaceError(
                f"No tactile message received yet on "
                f"{self.topic_prefix}/{finger_key} (side={self.hand_type.value})"
            )
        return msg

    def get_rate(self, finger: str) -> "float | Dict[str, float]":
        """统计缓存更新频率（Hz），即回调触发频率

        衡量的是本进程实际收到帧的速率。缓存在回调里写，因此该值就是缓存更新
        频率；若与 ``ros2 topic hz`` 的读数对不上，说明 QoS 队列溢出丢帧或
        executor 被拖慢，而非话题本身变慢。

        统计口径与 ``ros2 topic hz`` 一致：保留最近 ``RATE_WINDOW_SIZE`` 个帧
        间隔（计数窗，非时间窗），取 ``1 / 间隔均值``。因此两边的默认读数可以
        直接对照；缩小对照窗口时用 ``ros2 topic hz --window <N>``。

        ``ros2 topic hz`` 在距上次打印 1 秒内没有新消息时干脆不输出；函数调用
        没法"不输出"，因此这里改为：距最后一帧超过 ``RATE_STALE_SEC`` 返回
        ``0.0``。**与 get() 不同，无数据时不抛异常而是返回 0.0** —— 频率查询是
        诊断手段，链路断掉时「频率为 0」正是最有用的结论。

        Args:
            finger: 手指名，或 ``"all"``（大小写不敏感）

        Returns:
            单指时为 float（Hz）；``finger="all"`` 时为 ``{手指名: Hz}`` 字典。
            尚无间隔样本、或距最后一帧超过 ``RATE_STALE_SEC`` 时为 ``0.0``。

        Raises:
            ValueError: 手指名非法（唯一会抛的异常）
        """
        finger_key = str(finger).strip().lower()

        if finger_key == TACTILE_ALL:
            now = time.monotonic()
            return {name: self._rate_of(name, now) for name in TACTILE_FINGERS}

        if finger_key not in TACTILE_FINGERS:
            raise ValueError(
                f"finger must be one of {TACTILE_FINGERS} or "
                f"{TACTILE_ALL!r}, got: {finger!r}"
            )
        return self._rate_of(finger_key, time.monotonic())

    def _rate_of(self, finger_key: str, now: float) -> float:
        """算频率；只读快照，不修改 deque"""
        # 停发判据：计数窗不会自己老化，话题停了 deque 里的旧间隔会一直留着，
        # 频率就会停在最后的读数上。必须显式比对最后一帧时刻才会归零。
        last = self._last_stamp[finger_key]
        if last is None or now - last > RATE_STALE_SEC:
            return 0.0

        intervals = list(self._intervals[finger_key])
        if not intervals:
            return 0.0
        mean = sum(intervals) / len(intervals)
        if mean <= 0.0:
            return 0.0
        return 1.0 / mean

    def cleanup(self) -> None:
        """清理资源"""
        for finger in TACTILE_FINGERS:
            sub = self.tactile_subs[finger]
            if sub is not None:
                sub.destroy()
                self.tactile_subs[finger] = None
            self._latest[finger] = None
            self._intervals[finger].clear()
            self._last_stamp[finger] = None
