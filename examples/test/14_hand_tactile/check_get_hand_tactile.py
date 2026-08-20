"""
验证 ROS2RobotInterface.get_hand_tactile() 的灵巧手触觉读取。

get_hand_tactile() 行为:
    - 读缓存返回原始 std_msgs/msg/UInt8MultiArray，不 reshape、不拷贝。
    - finger 传具体手指名返回一条消息；传 "all" 返回 {手指名: 消息} 字典。
      读取只有这一个方法，没有单独的 get_hand_tactile_all()。
    - connect() 时用正则扫描 ROS 图匹配
      /<o6|l6|o7>_hand/<left|right>/tactile/<finger>，反推型号前缀写入
      config.left_hand_tactile_topic_prefix / right_hand_tactile_topic_prefix，
      并为该侧五根手指各挂一个订阅。
    - 未连接抛 ROS2NotConnectedError；side/finger 非法抛 ValueError；
      该侧无触觉话题或对应手指尚无消息抛 ROS2InterfaceError
      （finger="all" 时任一手指无数据即抛出，不做部分返回）。

本脚本流程（默认单次模式）:
    1. connect()，检查两侧 *_hand_tactile_topic_prefix。
    2. 两侧都未检测到则打印 skip 并正常退出。
    3. 对每个检测到的侧，轮询等待首帧，再静置 SETTLE_SEC 秒攒够间隔样本。
    4. 用 finger="all" 一次取回五指；该调用因某指无数据而抛异常时退回逐指读取，
       以便指明缺哪根手指。
    5. 逐指打印形状、数值摘要与缓存更新频率（Hz）。
    6. finally 中 disconnect()。

--watch 持续监视模式:
    完成上面第 1-5 步后不退出，改为每 WATCH_INTERVAL_SEC 秒打印一行各指频率，
    直到 Ctrl-C。间隔默认 1.0 秒，与 ros2 topic hz 一致 —— 后者的
    get_hz() 里有 "msg_tn < last_printed_tn + 1e9" 这道闸，把输出限成每秒
    最多一次（见 ros2topic/verb/hz.py）。连接与统计窗口全程连续，
    因此能看出频率的长期趋势和周期性掉帧，这是反复重跑单次模式做不到的。
    频率为 0.00 时标 STALE：距最后一帧已超过 handler 的 RATE_STALE_SEC。

成功判据:
    - finger="all" 与逐指读取至少一条路径拿到数据。
    - 每根手指的 rows * cols 等于 len(data)。
    - 未检测到触觉话题、或超时无消息时打印 skip 并正常退出。
    - --watch 模式由 Ctrl-C 结束，退出码 0。

频率读数怎么看:
    打印的 Hz 是缓存更新频率，也就是本进程回调的触发频率。拿它和
        ros2 topic hz <prefix>/<finger>
    对比：两者接近说明订阅链路没丢帧；本脚本明显偏低说明 QoS 队列
    （depth 10）溢出丢帧或 executor 被拖慢，而不是话题本身变慢。
    统计口径与 ros2 topic hz 一致（10000 个帧间隔的计数窗，1 / 间隔均值），
    因此可直接与默认参数的 ros2 topic hz 对照。

前置条件:
    ROS 2 已 source；灵巧手（O6 / L6 / O7）驱动在运行，且启动时传入
    read_tactile:=true；CAN 总线正常。

运行:
    # 单次检查
    .venv/bin/python examples/test/14_hand_tactile/check_get_hand_tactile.py

    # 持续监视频率，Ctrl-C 结束
    .venv/bin/python examples/test/14_hand_tactile/check_get_hand_tactile.py --watch

    # 对照（另开终端）
    ros2 topic hz /o7_hand/left/tactile/thumb

安全说明:
    只读，不发送任何运动指令。
"""

from __future__ import annotations

import argparse
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.exceptions import ROS2InterfaceError

SIDES = ("left", "right")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
WAIT_SEC = 5.0             # 等待首帧的最长秒数
POLL_SEC = 0.1             # 轮询间隔（秒）
SETTLE_SEC = 2.5           # 首帧后静置多久再读频率（20 Hz 下约攒 50 个间隔样本）
WATCH_INTERVAL_SEC = 1.0   # --watch 的打印间隔；与 ros2 topic hz 的 1 秒闸一致


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            interface.disconnect()
            print("[cleanup] disconnected")
    except Exception as exc:
        print(f"[cleanup] warn: {exc}")


def wait_first_frame(interface: ROS2RobotInterface, side: str) -> bool:
    """轮询等待该侧拇指首帧；超时返回 False。"""
    deadline = time.time() + WAIT_SEC
    while time.time() < deadline:
        try:
            interface.get_hand_tactile(side, "thumb")
            return True
        except ROS2InterfaceError:
            time.sleep(POLL_SEC)
    return False


def report_side(interface: ROS2RobotInterface, side: str) -> int:
    print("-" * 70)
    prefix = getattr(interface.config, f"{side}_hand_tactile_topic_prefix")
    print(f"{side.upper()} hand tactile prefix: {prefix}")

    if not wait_first_frame(interface, side):
        print(f"skip: no {side} tactile message received within {WAIT_SEC:.1f}s")
        return 0


    # 首帧刚到时只有个位数间隔样本，均值噪声大；静置几秒攒够样本再读。
    print(f"  settling {SETTLE_SEC:.1f}s to accumulate rate samples...")
    time.sleep(SETTLE_SEC)

    # 优先走 finger="all" 一次取回五指；任一手指无数据时它会抛异常，
    # 此时退回逐指读取，好在输出里指明是哪根手指缺数据。
    try:
        messages = interface.get_hand_tactile(side, "all")
        print(f"  get_hand_tactile({side!r}, 'all') -> {len(messages)} fingers")
    except ROS2InterfaceError as exc:
        print(f"  get_hand_tactile({side!r}, 'all') failed: {exc}")
        print("  falling back to per-finger read")
        messages = {}

    # 频率只做到 handler 层，没有主类方法，因此直接取 handler 句柄。
    # get_rate() 无数据时返回 0.0 而不抛异常，所以不用包 try。
    handler = getattr(interface, f"{side}_hand_tactile_handler")
    rates = handler.get_rate("all")

    failures = 0
    for finger in FINGERS:
        msg = messages.get(finger)
        if msg is None:
            try:
                msg = interface.get_hand_tactile(side, finger)
            except ROS2InterfaceError as exc:
                print(f"  {finger:<7}: no data ({exc}), rate={rates[finger]:.2f} Hz")
                failures += 1
                continue

        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        length = len(msg.data)
        ok = rows * cols == length
        print(
            f"  {finger:<7}: {rows}x{cols}, len(data)={length}, "
            f"max={max(msg.data) if length else 0}, "
            f"rate={rates[finger]:6.2f} Hz, "
            f"{'ok' if ok else 'MISMATCH'}"
        )
        if not ok:
            failures += 1

    print(
        f"  compare with: ros2 topic hz {prefix}/thumb"
    )
    return failures


def watch_loop(interface: ROS2RobotInterface, sides: list[str], interval: float) -> int:
    """每 interval 秒打印一行各指频率，直到 Ctrl-C。

    连接与统计窗口全程连续，因此能看出频率的长期趋势和周期性掉帧。
    get_rate() 无数据返回 0.0 而不抛异常，所以循环体里不需要 try。
    """
    print("-" * 70)
    print(
        f"watching every {interval:.1f}s (same cadence as ros2 topic hz); "
        "Ctrl-C to stop"
    )
    header = "  ".join(f"{finger:>6}" for finger in FINGERS)
    print(f"{'elapsed':>9}  {'side':<5}  {header}")

    started = time.monotonic()
    while True:
        elapsed = time.monotonic() - started
        for side in sides:
            handler = getattr(interface, f"{side}_hand_tactile_handler")
            if handler is None:
                continue
            rates = handler.get_rate("all")
            cells = "  ".join(f"{rates[finger]:6.2f}" for finger in FINGERS)
            stale = " STALE" if all(rates[f] == 0.0 for f in FINGERS) else ""
            print(f"{elapsed:8.1f}s  {side:<5}  {cells}{stale}")
        time.sleep(interval)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check ROS2RobotInterface.get_hand_tactile()."
    )
    parser.add_argument(
        "--watch", action="store_true",
        help="持续监视各指频率，直到 Ctrl-C（默认只检查一次就退出）",
    )
    parser.add_argument(
        "--interval", type=float, default=WATCH_INTERVAL_SEC,
        help=f"--watch 的打印间隔，单位秒（默认 {WATCH_INTERVAL_SEC}，"
             "与 ros2 topic hz 一致）",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=" * 70)
    print("get_hand_tactile() check")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        detected = [
            side for side in SIDES
            if getattr(interface.config, f"{side}_hand_tactile_topic_prefix")
        ]
        if not detected:
            print(
                "skip: no hand tactile topic detected; start can-ros2-control "
                "with read_tactile:=true"
            )
            return 0

        failures = 0
        for side in detected:
            failures += report_side(interface, side)

        print("-" * 70)
        if failures:
            print(f"failed: {failures} finger(s) missing data or shape mismatch")
            return 1

        # 单次检查通过后，--watch 才进入持续监视；由 Ctrl-C 结束
        if args.watch:
            return watch_loop(interface, detected, args.interval)

        print("done")
        print("tip: add --watch to keep printing rates every second")
        return 0
    finally:
        cleanup(interface)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        sys.exit(0)
    except Exception as exc:
        print(f"failed: {exc}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)
