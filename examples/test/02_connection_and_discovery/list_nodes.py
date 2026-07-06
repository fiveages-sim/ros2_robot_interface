"""
验证 ROS2RobotInterface.list_nodes() 发现当前 ROS 2 graph 中的节点。

list_nodes() 行为（本脚本直接调用该接口）:
    - 查询当前运行的 ROS 2 节点，返回列表，每项含 name、namespace、full_name。
    - 无需 connect()：未连接时会创建临时节点完成查询后销毁。
    - 若已 connect()，则复用接口内部的 robot_node，避免重复创建节点。
    - graph 为空时返回空列表，不抛异常。

本脚本流程:
    1. 创建 ROS2RobotInterface（不 connect）。
    2. 调用 list_nodes()，按序号打印每个节点的 full_name、name、namespace 及总数。

前置条件:
    ROS 2 daemon / graph 可用；没有任何节点运行时脚本正常输出空结果。

运行:
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/list_nodes.py

安全说明:
    本脚本只读查询 ROS graph，不发送控制命令。
"""

from __future__ import annotations

import sys

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def print_nodes(nodes: list[dict[str, str]]) -> None:
    print(f"total nodes: {len(nodes)}")
    print("-" * 70)
    for index, node in enumerate(nodes, 1):
        print(f"{index:3d}. {node['full_name']}")
        print(f"     name={node['name']} namespace={node['namespace']}")


def main() -> int:
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    nodes = interface.list_nodes()
    print_nodes(nodes)
    return 0


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
