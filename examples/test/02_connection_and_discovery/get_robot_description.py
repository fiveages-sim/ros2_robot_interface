"""
验证 ROS2RobotInterface 的 /robot_description 订阅：has 与 get 两个接口。

实际使用情况（本仓库）:
    get_robot_description()  【常用】
        - ros2-viser/ros2_viser/visualizer.py：轮询 URDF，驱动机器人模型加载与重载。
        - ros2-viser/ros2_viser/panels/joint_panel.py：无预解析 URDF 时，回退解析关节限位。
        - 典型写法：description = interface.get_robot_description(); if not description: ...

    has_robot_description()  【基本未用】
        - 当前仓库内无业务代码调用；仅本示例脚本与 API 文档出现。
        - 多数场景用 get_robot_description() 判空即可替代。
        - 细微差别：收到空字符串时 has 为 True，get 返回 ""（falsy），
          ros2-viser 将空串视为“尚未收到有效 URDF”。

has_robot_description() 行为:
    - 返回 bool：connect() 后是否至少收到一条 /robot_description（std_msgs/String）。
    - 订阅使用 TRANSIENT_LOCAL QoS，可接收已 latched 的历史消息。
    - 只判断“是否收到”，不返回 URDF 内容。

get_robot_description() 行为:
    - 返回最近一次收到的 URDF 字符串，尚未收到时返回 None。
    - 内容在 connect() 后由后台订阅回调缓存，不会主动轮询或阻塞等待。

本脚本流程:
    1. connect()，等待 --wait-sec 秒（默认 2.0）。
    2. 调用 has_robot_description() 与 get_robot_description()，校验两者结论一致。
    3. 若收到内容，打印长度、是否像 URDF XML 及 --preview-chars 前缀预览。

成功判据:
    has_robot_description() 与 get_robot_description() is not None 结果一致。
    默认要求已收到 robot description；--allow-missing 时允许缺失仍返回 0。

前置条件:
    若需收到 URDF，机器人或仿真栈须发布 /robot_description。

运行:
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/get_robot_description.py
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/get_robot_description.py --allow-missing
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/get_robot_description.py --wait-sec 5 --preview-chars 800

安全说明:
    本脚本只订阅 /robot_description，不发送控制命令，不打印完整 URDF。
"""

from __future__ import annotations

import argparse
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check robot_description has/get APIs.")
    parser.add_argument("--wait-sec", type=float, default=2.0, help="seconds to wait after connect")
    parser.add_argument("--preview-chars", type=int, default=500, help="number of URDF characters to preview")
    parser.add_argument("--allow-missing", action="store_true", help="return 0 when robot_description is missing")
    return parser.parse_args()


def compact_preview(text: str, max_chars: int) -> str:
    compact = " ".join(text.split())
    if len(compact) <= max_chars:
        return compact
    return compact[: max_chars - 3] + "..."


def main() -> int:
    args = parse_args()
    if args.wait_sec < 0 or args.preview_chars < 20:
        print("error: --wait-sec must be >= 0 and --preview-chars must be >= 20", file=sys.stderr)
        return 2

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        interface.connect()
        if args.wait_sec:
            print(f"waiting {args.wait_sec:.2f}s for /robot_description...")
            time.sleep(args.wait_sec)

        received = interface.has_robot_description()
        description = interface.get_robot_description()
        print(f"has_robot_description() -> {received}")
        print(f"get_robot_description() -> {'<received>' if description else None}")

        has_content = description is not None
        if received != has_content:
            print(
                "requirement failed: has_robot_description() disagrees with get_robot_description()",
                file=sys.stderr,
            )
            return 1

        if not has_content:
            print("robot description was not received")
            return 0 if args.allow_missing else 1

        stripped = description.lstrip()
        looks_like_xml = stripped.startswith("<robot") or stripped.startswith("<?xml")
        print(f"robot_description_length={len(description)}")
        print(f"looks_like_urdf_xml={looks_like_xml}")
        print("preview:")
        print(compact_preview(description, args.preview_chars))
        return 0
    finally:
        if interface.is_connected:
            interface.disconnect()


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
