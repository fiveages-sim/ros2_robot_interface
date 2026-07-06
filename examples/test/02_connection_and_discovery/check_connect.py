"""
验证 ROS2RobotInterface.connect() / disconnect() 的连接建立与释放。

connect() 行为:
    - 若 rclpy 未初始化则自动 init；已连接时抛出 ROS2AlreadyConnectedError。
    - 扫描当前话题并自动检测单臂/双臂、WBC 等配置，据此创建 ROS 2 节点。
    - 创建 joint_states、/fsm_state、/robot_description 等订阅，以及控制所需的
      publisher / action client / TF listener，并启动后台 executor 线程处理回调。
    - 成功后 is_connected 变为 True。

disconnect() 行为:
    - 将 _connected 置为 False，关闭 executor 线程并销毁订阅、publisher、
      action client 及 arm handler 等资源；调用后 is_connected 变为 False。
    - 重复调用是安全的；不会 shutdown rclpy。

is_connected 属性:
    - 只读；当 _connected 为 True 且 robot_node 仍存在时返回 True，否则 False。
    - 若配置了 joint_state_timeout 且长时间未收到 joint_states，get_joint_state()
      也可能将 _connected 置为 False（本脚本不触发该超时逻辑）。

本脚本流程:
    1. 打印初始 is_connected（预期 False）。
    2. 调用 connect()，打印自动检测到的配置摘要。
    3. 调用 disconnect()，验证断开后的实际副作用（见下方成功判据）。

成功判据:
    connect() 后 is_connected 为 True。
    disconnect() 后:
        - 需连接的 API（如 send_fsm_command）抛出 ROS2NotConnectedError（publish 前即失败，不发送命令）；
        - 再次 disconnect() 不抛异常；
        - 可重新 connect() 且 is_connected 恢复为 True（说明资源已真正释放）。
    配置摘要用于佐证话题扫描已完成，默认值仅表示未扫到对应话题。

前置条件:
    ROS 2 环境已 source；机器人或仿真栈在运行时可收到更多订阅数据，但非必须。

运行:
    conda run -n fa-ros2 python examples/test/02_connection_and_discovery/check_connect.py

安全说明:
    本脚本不发送有效控制命令；disconnect 验证阶段调用 send_fsm_command() 仅用于
    确认其在 publish 前抛出 ROS2NotConnectedError。
"""

from __future__ import annotations

import sys

from ros2_robot_interface import ROS2NotConnectedError, ROS2RobotInterface, ROS2RobotInterfaceConfig


def print_detected_configuration(interface: ROS2RobotInterface) -> None:
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    print(f"is_connected: {interface.is_connected}")
    print(f"arm_mode: {'dual-arm' if is_dual_arm else 'single-arm'}")
    print(f"is_wbc: {interface.is_wbc}")

    detected_topics = [
        ("right_end_effector_pose_topic", interface.config.right_end_effector_pose_topic),
        ("left_arm_joint_controller_topic", interface.config.left_arm_joint_controller_topic),
        ("unified_arm_joint_controller_topic", interface.config.unified_arm_joint_controller_topic),
        ("body_joint_controller_topic", interface.config.body_joint_controller_topic),
        ("head_joint_controller_topic", interface.config.head_joint_controller_topic),
    ]
    for name, topic in detected_topics:
        if topic:
            print(f"  {name}: {topic}")


def verify_disconnect_effects(interface: ROS2RobotInterface) -> int:
    try:
        interface.send_fsm_command(2)
    except ROS2NotConnectedError:
        print("verified: send_fsm_command() raises ROS2NotConnectedError when disconnected")
    else:
        print(
            "requirement failed: send_fsm_command() did not raise when disconnected",
            file=sys.stderr,
        )
        return 1

    interface.disconnect()
    print("verified: second disconnect() is safe")

    print("calling connect() again after disconnect...")
    interface.connect()
    if not interface.is_connected:
        print("requirement failed: reconnect did not set is_connected to True", file=sys.stderr)
        return 1
    print("reconnect succeeded")
    return 0


def main() -> int:
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        print(f"initial: is_connected={interface.is_connected}")

        print("calling connect()...")
        interface.connect()
        print("connect() succeeded")
        print_detected_configuration(interface)
        if not interface.is_connected:
            print("requirement failed: is_connected is False after connect()", file=sys.stderr)
            return 1

        print("calling disconnect()...")
        interface.disconnect()
        print("disconnect() succeeded")

        result = verify_disconnect_effects(interface)
        if result != 0:
            return result
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
