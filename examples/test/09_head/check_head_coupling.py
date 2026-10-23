"""头部耦合避让实机/仿真检查：[yaw, pitch]，单位 rad。

在 ros2_robot_interface 根目录、已加载 ROS 2 的 Python 环境运行：
    python examples/test/09_head/check_head_coupling.py
    python examples/test/09_head/check_head_coupling.py --interpolation linear --case 2

默认 doubles、请求时长 4 秒，依次执行七组。每组先到起点，再发目标。
设置目标控制器的插值方式、时长并开启自动延长；这些参数在退出后保留。
使用现有 send_head_joint_positions() 自动切 MOVEJ，不使用多路点入口。
结束、异常或 Ctrl+C 后发送全局 HOLD 并断开连接。
到位检查通过不代表轨迹平滑或耦合保护已全部验证，需观察实机和控制器日志。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


# 名称、起点、目标、预期行为；顺序为 yaw、pitch。
CASES = [
    ("普通中心运动", [0.0, 0.0], [0.349066, 0.174533], "正常同时运动"),
    ("向左跨区", [0.0, 0.488692], [1.047198, 0.174533], "必要时延迟 yaw，启动后连续运动"),
    ("向右跨区", [0.0, 0.488692], [-1.047198, 0.174533], "必要时延迟 yaw，启动后连续运动"),
    ("左侧回中心", [1.047198, 0.174533], [0.0, 0.488692], "必要时延迟 pitch"),
    ("右侧回中心", [-1.047198, 0.174533], [0.0, 0.488692], "必要时延迟 pitch"),
    ("向外但全程安全", [0.0, 0.174533], [1.047198, 0.174533], "pitch 保持 10°，yaw 不应额外等待"),
    ("两侧互穿", [1.047198, 0.174533], [-1.047198, 0.174533], "pitch 保持 10°，yaw 连续经过中心"),
]


def move_and_wait(interface, positions, label, timeout):
    print(f"  {label}：{positions} rad", flush=True)
    interface.send_head_joint_positions(positions)
    result = interface.wait_until_arrive(
        part="head", timeout=timeout, poll_period=0.1, position_threshold=0.001
    )
    if not result["arrived"]:
        raise RuntimeError(f"{label} 到位超时：{result}；停止后续测试，请检查 HOLD/拒绝日志")
    print(f"  已到位，等待耗时 {result['elapsed']:.2f} 秒", flush=True)
    # 留出最终到位检查的控制周期，避免下一目标被当作运动中抢占。
    time.sleep(0.3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interpolation", choices=("doubles", "linear", "tanh"), default="doubles")
    parser.add_argument("--duration", type=float, default=4.0, help="请求运动时长，默认 4 秒")
    parser.add_argument("--timeout", type=float, default=30.0, help="每次起点/目标到位超时，默认 30 秒")
    parser.add_argument("--case", type=int, choices=range(1, len(CASES) + 1), help="只执行指定组，默认全部")
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in (args.duration, args.timeout)):
        parser.error("duration 和 timeout 必须为有限正数")

    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    motion_started = False
    try:
        interface.connect()
        time.sleep(1.0)
        topic = interface.config.head_joint_controller_topic
        if not topic or interface.head_joint_controller_pub is None:
            raise RuntimeError("未检测到头部位置目标话题")
        head = (interface.get_joint_state(categorized=True) or {}).get("head") or {}
        if head.get("names") != ["head_joint1", "head_joint2"] or len(head.get("positions", [])) != 2:
            raise RuntimeError(f"需要按 head_joint1、head_joint2 排列的反馈，当前为：{head}")
        controller, separator, _ = topic.partition("/target_joint_position")
        if not separator or not controller:
            raise RuntimeError(f"无法从目标话题解析控制器节点：{topic}")
        parameters = {
            "movej_interpolation_type": args.interpolation,
            "movej_duration": args.duration,
            "movej_auto_extend_duration": True,
        }
        if not interface.set_node_parameters(controller, parameters):
            raise RuntimeError(f"设置 {controller} 参数失败")
        print(f"话题：{topic}\n参数：{parameters}", flush=True)
        print("本脚本会驱动头部；结束时发送全局 HOLD。", flush=True)
        selected = enumerate(CASES, 1)
        for number, (name, start, target, expected) in selected:
            if args.case is not None and number != args.case:
                continue
            print(f"\n[{number}/{len(CASES)}] {name}：{expected}", flush=True)
            motion_started = True
            move_and_wait(interface, start, "起点", args.timeout)
            move_and_wait(interface, target, "目标", args.timeout)
        print("\n所选点位均已到位；请结合运动过程确认延迟启动及无中途停顿。", flush=True)
        return 0
    finally:
        try:
            if motion_started and interface.is_connected:
                interface.send_fsm_command(2)
        finally:
            if interface.is_connected:
                interface.disconnect()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n测试中断。", file=sys.stderr)
        sys.exit(130)
    except Exception as error:
        print(f"测试失败：{error}", file=sys.stderr)
        sys.exit(1)
