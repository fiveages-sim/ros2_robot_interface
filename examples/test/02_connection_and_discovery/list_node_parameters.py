"""
验证 ROS2RobotInterface.list_node_parameters() 查询指定节点的 ROS 2 参数。

list_node_parameters(full_node_name) 行为（本脚本直接调用该接口）:
    - 通过目标节点的 list_parameters / describe_parameters / get_parameters
      服务读取可配置参数及其当前值。
    - 返回列表，每项含 name、type、value、description、read_only、dynamic_typing；
      自动过滤 use_sim_time 等系统内部参数。
    - 无需 connect()：未连接时创建临时节点查询；--connect 时复用接口节点。
    - 目标节点不存在或未暴露参数服务时返回空列表，不抛异常。

本脚本流程:
    默认（交互）:
        1. 调用 list_nodes() 列出当前 graph 中所有节点。
        2. 用户输入序号或节点全名，选择要查看的节点。
        3. 调用 list_node_parameters() 打印该节点的参数。

    非交互（--node）:
        1. 直接查询 --node 指定节点，适合脚本化调用。

前置条件:
    目标节点须提供标准参数服务。

运行:
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/list_node_parameters.py
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/list_node_parameters.py --node /controller_manager
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/list_node_parameters.py --node controller_manager --connect

安全说明:
    本脚本只读查询参数，不设置参数，不发送控制命令。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="List parameters from one ROS 2 node.")
    parser.add_argument(
        "--node",
        help="full node name to query non-interactively, e.g. /controller_manager",
    )
    parser.add_argument("--connect", action="store_true", help="connect interface before querying")
    parser.add_argument("--limit-params", type=int, default=50, help="maximum params to print; 0 means unlimited")
    parser.add_argument("--value-width", type=int, default=80, help="maximum printed width for parameter values")
    return parser.parse_args()


def format_parameter_value(value: Any, max_length: int = 80) -> str:
    if value is None:
        return "None"
    if isinstance(value, (list, tuple)):
        if not value:
            return "[]"
        if len(value) <= 4:
            text = repr(list(value))
        else:
            text = f"[{value[0]!r}, {value[1]!r}, ..., {value[-1]!r}] (len={len(value)})"
    else:
        text = str(value)
    if len(text) > max_length:
        return text[: max_length - 3] + "..."
    return text


def normalize_node_name(name: str) -> str:
    stripped = name.strip()
    if not stripped:
        return ""
    if stripped.startswith("/"):
        return stripped
    return f"/{stripped}"


def print_nodes_menu(nodes: list[dict[str, str]]) -> None:
    print(f"total nodes: {len(nodes)}")
    print("-" * 70)
    for index, node in enumerate(nodes, 1):
        print(f"{index:3d}. {node['full_name']}")
        print(f"     name={node['name']} namespace={node['namespace']}")


def select_node_interactive(nodes: list[dict[str, str]]) -> str | None:
    if not nodes:
        print("no ROS 2 nodes found in the current graph", file=sys.stderr)
        return None

    print_nodes_menu(nodes)
    print("-" * 70)

    full_names = [node["full_name"] for node in nodes]
    while True:
        try:
            choice = input(
                f"select node [1-{len(nodes)}], enter full name, or q to quit: "
            ).strip()
        except EOFError:
            print("\nno input received", file=sys.stderr)
            return None

        if not choice:
            continue
        if choice.lower() in {"q", "quit", "exit"}:
            return None

        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(nodes):
                return full_names[index - 1]
            print(f"invalid index: enter a number between 1 and {len(nodes)}", file=sys.stderr)
            continue

        normalized = normalize_node_name(choice)
        if normalized in full_names:
            return normalized

        suffix_matches = [
            full_name
            for full_name in full_names
            if full_name.endswith(normalized) or full_name.endswith(f"/{normalized.lstrip('/')}")
        ]
        if len(suffix_matches) == 1:
            return suffix_matches[0]
        if len(suffix_matches) > 1:
            print("ambiguous node name, matches:", file=sys.stderr)
            for match in suffix_matches:
                print(f"  {match}", file=sys.stderr)
            continue

        print(f"warning: '{normalized}' is not in the listed nodes; querying anyway", file=sys.stderr)
        return normalized


def print_parameters(node_name: str, parameters: list[dict[str, Any]], limit: int, value_width: int) -> None:
    print("-" * 70)
    print(f"node: {node_name}")
    print(f"parameters: {len(parameters)}")
    shown = parameters if limit <= 0 else parameters[:limit]
    for param in shown:
        value = format_parameter_value(param.get("value"), value_width)
        flags = []
        if param.get("read_only"):
            flags.append("read_only")
        if param.get("dynamic_typing"):
            flags.append("dynamic_typing")
        suffix = f" [{' '.join(flags)}]" if flags else ""
        print(f"  - {param.get('name')} ({param.get('type')}) = {value}{suffix}")
        description = param.get("description") or ""
        if description:
            print(f"    description: {format_parameter_value(description, value_width)}")
    if len(shown) < len(parameters):
        print(f"  ... {len(parameters) - len(shown)} more parameters hidden by --limit-params")


def query_node_parameters(
    interface: ROS2RobotInterface,
    node_name: str,
    limit_params: int,
    value_width: int,
) -> int:
    parameters = interface.list_node_parameters(node_name)
    if not parameters:
        print(f"node: {node_name}")
        print("parameters: 0 (node may not expose parameter services)")
        return 0

    print_parameters(node_name, parameters, limit_params, value_width)
    return 0


def resolve_node_name(args: argparse.Namespace, interface: ROS2RobotInterface) -> str | None:
    if args.node:
        node_name = normalize_node_name(args.node)
        if not node_name:
            print("error: --node cannot be empty", file=sys.stderr)
            return None
        return node_name

    if not sys.stdin.isatty():
        print("error: --node is required when not running in an interactive terminal", file=sys.stderr)
        return None

    print("discovering ROS 2 nodes...")
    nodes = sorted(interface.list_nodes(), key=lambda node: node["full_name"])
    return select_node_interactive(nodes)


def main() -> int:
    args = parse_args()
    if args.limit_params < 0 or args.value_width < 10:
        print("error: --limit-params must be >= 0 and --value-width must be >= 10", file=sys.stderr)
        return 2

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        if args.connect:
            interface.connect()
            time.sleep(1.0)

        node_name = resolve_node_name(args, interface)
        if node_name is None:
            return 2 if args.node is not None or not sys.stdin.isatty() else 0

        return query_node_parameters(interface, node_name, args.limit_params, args.value_width)
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
