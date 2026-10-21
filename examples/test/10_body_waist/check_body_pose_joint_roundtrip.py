"""两次 Enter 验证 WBC 腰部 pose 回放前后的实际关节偏差。

运行（先 source ROS 2 环境并启动 WBC）：
    conda run --no-capture-output -n fa-ros2 python examples/test/10_body_waist/check_body_pose_joint_roundtrip.py

首次 Enter 仅采样；第二次 Enter 发送运动目标，观察 5 秒后比较。
腰部反馈须为 base_footprint 下的 pose（当前 W2 配置）；getter 不保留 header。
采样来自各反馈缓存，不保证关节、腰部和双臂反馈严格时间同步。
"""

import copy
import sys
import time

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def body_joints(robot):
    state = robot.get_joint_state(categorized=True)
    body = (state or {}).get("body", {})
    names, positions = body.get("names", []), body.get("positions", [])
    if not names or len(names) != len(positions):
        raise RuntimeError("腰部关节反馈缺失或不完整")
    return dict(zip(names, positions))


def body_pose(robot):
    pose = robot.get_body_current_pose()
    if pose is None:
        raise RuntimeError("未收到腰部实际 pose，请检查 /body_current_pose")
    return copy.deepcopy(pose)


def arm_pose(robot, handler):
    if handler is None:
        raise RuntimeError("需要左右臂 handler")
    pose, frame = handler.get_pose(), handler.get_frame_id()
    if pose is None or not frame:
        raise RuntimeError("手臂实际 pose 或反馈坐标系缺失")
    pose = copy.deepcopy(pose)
    if frame != "arm_base":
        pose = robot.transform_pose(pose, source_frame=frame, target_frame="arm_base")
        if pose is None:
            raise RuntimeError(f"手臂 pose 从 {frame} 转换到 arm_base 失败")
    return pose


def print_pose(label, pose):
    p, q = pose.position, pose.orientation
    print(f"{label}: xyz=({p.x:.6f}, {p.y:.6f}, {p.z:.6f}) m; "
          f"xyzw=({q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f})")


def main():
    robot = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        robot.connect()
        time.sleep(2.0)
        if not robot.is_wbc:
            raise RuntimeError("本脚本要求 WBC 全身控制模式")
        print("腰部参考系：base_footprint；双臂参考系：arm_base。")
        print("第二次 Enter 将发送运动目标；Ctrl+C 退出。")
        input("[1/2] 在待记录位置按 Enter，记录腰部关节和 pose：")
        first = body_joints(robot)
        target = body_pose(robot)
        left = arm_pose(robot, robot.left_arm_handler)
        right = arm_pose(robot, robot.right_arm_handler)
        print("首次腰部实际关节位置（原始反馈单位，转动关节 rad / 移动关节 m）：")
        for name, value in first.items():
            print(f"  {name}: {value:.6f}")
        print_pose("已记录腰部 pose", target)

        input("[2/2] 按 Enter 回放已记录的双臂和腰部 pose，并比较关节位置：")
        before_send = body_joints(robot)
        robot.send_dual_arm_target_stamped(
            left, right,
            frame_id="arm_base",
            body_pose=target,
            body_frame_id="base_footprint",
            body_mode="BODY_TRACKING",
        )
        print("已下发；观察 5 秒后采样（固定观察时间，不代表已经到位）……", flush=True)
        time.sleep(5.0)
        final = body_joints(robot)
        actual = body_pose(robot)
        if first.keys() != before_send.keys() or first.keys() != final.keys():
            raise RuntimeError("采样期间腰部关节集合发生变化，无法逐关节比较")
        print("\n腰部关节比较（差值 = 执行后 - 首次；单位同原始反馈）：")
        print(f"{'关节':<24} {'首次':>12} {'下发前':>12} {'执行后':>12} {'差值':>12}")
        for name, initial in first.items():
            print(f"{name:<24} {initial:12.6f} {before_send[name]:12.6f} "
                  f"{final[name]:12.6f} {final[name] - initial:+12.6f}")
        print_pose("目标腰部 pose", target)
        print_pose("执行后腰部 pose", actual)
        dx = actual.position.x - target.position.x
        dy = actual.position.y - target.position.y
        dz = actual.position.z - target.position.z
        print(f"腰部位置偏差（实际 - 首次目标）：dx={dx:+.6f}, dy={dy:+.6f}, "
              f"dz={dz:+.6f} m；距离={(dx*dx + dy*dy + dz*dz)**0.5:.6f} m")
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print("\n已退出。")
        sys.exit(130)
    except Exception as exc:
        print(f"\n测试失败：{exc}", file=sys.stderr)
        sys.exit(1)
