"""
用实时热力图观察 ROS2RobotInterface.get_hand_tactile() 读到的灵巧手触觉阵列。

与 can-ros2-control 的 linkerhand_tactile_visualizer 的区别:
    那个工具是独立 ROS 节点，会另起一个节点、再订阅一遍五个话题。
    本脚本复用 interface 已经建好的订阅，只读 handler 缓存，
    不新建节点、不重复订阅，因此显示的频率就是本进程的真实收帧率。

数据来源:
    - 画面: get_hand_tactile(side, "all") 返回的原始 UInt8MultiArray，
      按 layout.dim 现读 rows/cols（O6 为 10x4，L6 / O7 为 12x6），
      因此无需事先知道型号。
    - max: 在重绘时从刚取到的那条消息现算，与画面同源，必然自洽。
    - Hz: handler.get_rate()，由订阅回调在后台线程攒出来，
      反映收帧速率，与本脚本的重绘频率无关。

为什么 Hz 是存活指示器:
    get_hand_tactile() 不检查数据年龄，某根手指的话题断了它会一直返回
    最后那一帧，不报错。此时画面只是静止不动，看不出异常。
    get_rate() 有 RATE_STALE_SEC 判据，超时返回 0.0 —— 所以频率归零、
    标题变红才是"这根手指断了"的信号，而不是画面本身。

重绘频率与收帧频率不同:
    重绘固定为 --refresh-rate（默认 20 Hz），收帧通常更快（实测约 45 Hz）。
    每次重绘取当时最新的一帧，中间的帧不会被画出来。持续一两帧的瞬时
    尖峰可能被漏掉；要抓瞬时接触事件需另做峰值保持，本脚本不做。

五指不同龄:
    驱动逐指轮询 CAN（0xB1~0xB5）、五个话题独立发布，因此同一次
    get_hand_tactile(side, "all") 拿到的五条消息各是各自的最新值，
    但不属于同一次采样，彼此可能差几十毫秒。

本脚本流程:
    1. connect()，探测触觉话题；未检测到则打印 skip 并正常退出。
    2. 轮询等待首帧，从首帧的 layout.dim 确定矩阵形状。
    3. 建图，FuncAnimation 按 --refresh-rate 重绘，plt.show() 阻塞。
    4. 关闭窗口或 Ctrl-C 退出，finally 中 disconnect()。

成功判据:
    - 窗口正常显示五指热力图，标题的 Hz 与 ros2 topic hz 接近。
    - 按压指腹时对应格子变亮、max 跟着跳。
    - matplotlib 缺失、未检测到触觉话题、首帧超时均打印 skip 并退出码 0。

前置条件:
    ROS 2 已 source；灵巧手（O6 / L6 / O7）驱动在运行，且启动时传入
    read_tactile:=true；CAN 总线正常；已安装 matplotlib。

运行:
    # 自动选探测到的第一侧
    .venv/bin/python examples/test/14_hand_tactile/check_get_hand_tactile_heatmap.py

    # 指定右手，重绘 30 Hz
    .venv/bin/python examples/test/14_hand_tactile/check_get_hand_tactile_heatmap.py \
        --side right --refresh-rate 30

安全说明:
    只读，不发送任何运动指令。
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import numpy as np

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from ros2_robot_interface.utils.exceptions import ROS2InterfaceError

SIDES = ("left", "right")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
FINGER_LABELS = {
    "thumb": "Thumb",
    "index": "Index",
    "middle": "Middle",
    "ring": "Ring",
    "pinky": "Pinky",
}

WAIT_SEC = 5.0            # 等待首帧的最长秒数
POLL_SEC = 0.1            # 等待首帧的轮询间隔（秒）
REFRESH_RATE_HZ = 20.0    # 默认重绘频率，与 linkerhand_tactile_visualizer 一致
COLOR_MAX = 255.0         # imshow 的 vmax，触觉为无符号 8 位
STALE_COLOR = "tab:red"   # 频率归零时的标题颜色
LIVE_COLOR = "black"


def cleanup(interface: ROS2RobotInterface) -> None:
    try:
        if interface.is_connected:
            interface.disconnect()
            print("[cleanup] disconnected")
    except Exception as exc:
        print(f"[cleanup] warn: {exc}")


def resolve_side(interface: ROS2RobotInterface, requested: str | None) -> str | None:
    """选定要显示的一侧；无可用侧时打印 skip 并返回 None。"""
    detected = [
        side for side in SIDES
        if getattr(interface.config, f"{side}_hand_tactile_topic_prefix")
    ]
    if not detected:
        print(
            "skip: no hand tactile topic detected; start can-ros2-control "
            "with read_tactile:=true"
        )
        return None

    if requested is None:
        side = detected[0]
        if len(detected) > 1:
            print(f"note: both hands detected, showing {side} (use --side to switch)")
        return side

    if requested not in detected:
        print(f"skip: {requested} hand tactile not detected (available: {detected})")
        return None
    return requested


def wait_first_frame(interface: ROS2RobotInterface, side: str):
    """轮询等待该侧拇指首帧；超时返回 None。"""
    deadline = time.time() + WAIT_SEC
    while time.time() < deadline:
        try:
            return interface.get_hand_tactile(side, "thumb")
        except ROS2InterfaceError:
            time.sleep(POLL_SEC)
    return None


def matrix_shape(msg) -> tuple[int, int]:
    """从消息自带的 layout 读矩阵形状；layout 异常时退回按 data 长度推断。"""
    dims = msg.layout.dim
    if len(dims) >= 2 and dims[0].size > 0 and dims[1].size > 0:
        return dims[0].size, dims[1].size
    return len(msg.data), 1


def to_matrix(msg, rows: int, cols: int) -> np.ndarray:
    """把行优先的 data 还原成二维矩阵；长度对不上时补零，避免 reshape 抛错。"""
    data = np.asarray(msg.data, dtype=np.uint8)
    expected = rows * cols
    if data.size != expected:
        padded = np.zeros(expected, dtype=np.uint8)
        padded[: min(data.size, expected)] = data[:expected]
        data = padded
    return data.reshape(rows, cols)


def model_from_prefix(prefix: str) -> str:
    """/l6_hand/left/tactile -> L6；解析不出就返回 HAND。"""
    match = re.match(r"^/([a-z0-9_-]+)_hand/", prefix or "")
    return match.group(1).upper() if match else "HAND"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live heatmap of ROS2RobotInterface.get_hand_tactile()."
    )
    parser.add_argument(
        "--side", choices=SIDES, default=None,
        help="显示哪只手（默认自动选探测到的第一侧）",
    )
    parser.add_argument(
        "--refresh-rate", type=float, default=REFRESH_RATE_HZ,
        help=f"窗口重绘频率，单位 Hz（默认 {REFRESH_RATE_HZ}）。"
             "与收帧频率无关，仅决定画面刷新快慢",
    )
    parser.add_argument(
        "--color-max", type=float, default=COLOR_MAX,
        help=f"热力图色标上限（默认 {COLOR_MAX}）",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        print("skip: matplotlib not installed (pip install matplotlib)")
        return 0

    if args.refresh_rate <= 0.0:
        print(f"skip: --refresh-rate must be positive, got {args.refresh_rate}")
        return 0

    print("=" * 70)
    print("get_hand_tactile() live heatmap")
    print("=" * 70)

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    interface.connect()
    time.sleep(1.0)

    try:
        side = resolve_side(interface, args.side)
        if side is None:
            return 0

        prefix = getattr(interface.config, f"{side}_hand_tactile_topic_prefix")
        model = model_from_prefix(prefix)
        print(f"{side.upper()} hand tactile prefix: {prefix}")

        first = wait_first_frame(interface, side)
        if first is None:
            print(f"skip: no {side} tactile message received within {WAIT_SEC:.1f}s")
            return 0

        rows, cols = matrix_shape(first)
        print(f"matrix: {rows}x{cols} ({rows * cols} values per finger)")
        print(f"redraw: {args.refresh_rate:.1f} Hz (independent of receive rate)")
        print(f"compare with: ros2 topic hz {prefix}/thumb")
        print("close the window or press Ctrl-C to exit")

        handler = getattr(interface, f"{side}_hand_tactile_handler")

        # 视觉沿用 can-ros2-control 的 linkerhand_tactile_visualizer，
        # 免得两个工具看起来像两回事
        figure, axes = plt.subplots(1, len(FINGERS), figsize=(13, 6), sharey=True)
        figure.canvas.manager.set_window_title(
            f"{model} {side.capitalize()} Hand Tactile"
        )
        figure.suptitle(f"{model} {side.capitalize()} Hand Tactile Matrices")

        images = {}
        for axis, finger in zip(axes, FINGERS):
            images[finger] = axis.imshow(
                np.zeros((rows, cols), dtype=np.uint8),
                cmap="inferno",
                vmin=0,
                vmax=args.color_max,
                origin="upper",
                interpolation="nearest",
                aspect="equal",
            )
            axis.set_title(f"{FINGER_LABELS[finger]}\nwaiting")
            axis.set_xlabel("Column")
            axis.set_xticks(range(cols))
            axis.set_yticks(range(rows))
        axes[0].set_ylabel("Row")
        figure.colorbar(
            images[FINGERS[-1]],
            ax=axes,
            shrink=0.8,
            label="Normal force (raw)",
        )

        def update(_frame):
            """只读缓存与频率，绝不做 ROS 操作，也绝不让异常冒进事件循环。

            get_hand_tactile(side, "all") 是全有或全无：任一手指尚无数据就抛
            ROS2InterfaceError。这里捕获后保留上一帧画面，仅更新标题，
            让 STALE 标记来说明情况 —— 异常若逃逸出去会卡死 matplotlib 事件循环。
            """
            try:
                messages = interface.get_hand_tactile(side, "all")
            except ROS2InterfaceError:
                messages = None
            except Exception as exc:  # 兜底：任何异常都不能杀掉窗口
                print(f"warn: update failed: {exc}", file=sys.stderr)
                messages = None

            rates = handler.get_rate("all")

            for axis, finger in zip(axes, FINGERS):
                peak = None
                if messages is not None:
                    msg = messages[finger]
                    images[finger].set_data(to_matrix(msg, rows, cols))
                    peak = int(max(msg.data)) if len(msg.data) else 0

                rate = rates[finger]
                stale = rate == 0.0
                peak_text = "--" if peak is None else str(peak)
                status = "STALE" if stale else f"{rate:.1f} Hz"
                axis.set_title(
                    f"{FINGER_LABELS[finger]}\n{status}  max={peak_text}",
                    color=STALE_COLOR if stale else LIVE_COLOR,
                )

            return list(images.values())

        # 必须保留引用，否则 FuncAnimation 会被 GC 掉、动画不动
        animation = FuncAnimation(
            figure,
            update,
            interval=1000.0 / args.refresh_rate,
            blit=False,          # 标题也要更新，blit 覆盖不到
            cache_frame_data=False,
        )
        _ = animation

        plt.show()   # 阻塞主线程；ROS 回调在 interface 的后台 executor 线程继续跑
        print("done")
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
