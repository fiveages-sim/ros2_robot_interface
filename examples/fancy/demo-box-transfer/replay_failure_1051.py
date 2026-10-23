"""2026-10-23 10:51 故障轮次定时回放；无逐步输入，不等待到位。"""

import argparse
import json
from pathlib import Path
import time

from run_box_transfer import current_pair, make_pose

# pose 为 base_link / 米；四元数按原日志顺序保存。
OUTSIDE = [
    [0.723609, 0.297029, 0.853158, 0.703282, -0.079447, 0.699322, 0.100153],
    [0.693585, -0.288692, 0.848396, -0.708990, -0.039243, -0.703219, 0.035718],
]
GRASP = [
    [0.723609, 0.247029, 0.853158, 0.703282, -0.079447, 0.699322, 0.100153],
    [0.693585, -0.238692, 0.848396, -0.708990, -0.039243, -0.703219, 0.035718],
]
LOWER = [
    [0.630922, 0.249810, 0.604542, 0.698896, -0.057788, 0.708123, 0.082265],
    [0.629987, -0.236855, 0.607987, -0.701113, -0.061565, -0.707949, 0.058801],
]
HOME = dict(
    body_positions=[-1.252797, -2.294235, -1.040216, 0.0],
    left_arm_positions=[-0.441917, 0.945445, 0.945794, -2.165605, -0.852419, -0.350986, -0.156905],
    right_arm_positions=[0.441917, 0.945445, -0.945794, -2.165605, 0.852419, -0.350986, 0.156905],
)


def build_steps(config):
    back = config['grasp']['A']['back']
    # 时刻按日志；sleep 是本动作完成发送到下一项操作的间隔。
    return [
        ('10:50:43.181', 'joint', 3.0, HOME, 15.297),
        ('10:50:58.478', 'outside', 1.5, OUTSIDE, 2.422),
        ('10:51:00.900', 'grasp', 1.5, GRASP, 2.311),
        ('10:51:03.211', 'lift', 2.5, 0.4, 2.521),
        ('10:51:05.732', 'lock', None, None, 0.715),
        ('10:51:06.447', 'back', 2.5,
         [back['back_left_arm_pose'], back['back_right_arm_pose']], 2.810),
        ('10:51:09.257', 'lock', None, None, 15.415),
        ('10:51:24.672', 'lower', 2.5, LOWER, 20.095),
    ]


def execute(steps, order):
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        interface.connect()
        time.sleep(1.0)
        if not interface.is_wbc or not interface.right_arm_handler:
            raise RuntimeError('要求 WBC 双臂模式')
        current_pair(interface)
        for stamp, kind, duration, target, delay in steps:
            print(f'{stamp} {kind}: duration={duration}, sleep={delay}s', flush=True)
            if kind == 'joint':
                interface.auto_switch_fsm_for_control('arm_joint')
                if not interface.set_node_parameters(interface.arm_controller, {'movej_duration': duration}):
                    raise RuntimeError('设置 movej_duration 失败')
                interface.send_coordinated_joint_positions(**target)
            elif kind == 'lock':
                interface.auto_switch_fsm_for_control('arm_joint')
            else:
                # 仅每段开头使能，避免抓取、抬升之间重复切换模式。
                if kind in ('outside', 'back', 'lower'):
                    interface.auto_switch_fsm_for_control('arm_pose')
                    interface.send_mode_command('LEFT_ARM_ENABLE')
                    interface.send_mode_command('RIGHT_ARM_ENABLE')
                if kind == 'lift':
                    # 原日志只记录相对位移，没有绝对目标；使用本次实时反馈。
                    poses = current_pair(interface)
                    for pose in poses:
                        pose.position.z += target
                else:
                    poses = [make_pose(values, order) for values in target]
                interface.send_dual_arm_target_stamped(
                    *poses, frame_id='base_link', movel_duration=duration)
            time.sleep(delay)
        print('已回放至原日志超时点；结束并请求 HOLD，不执行撤离或回 home。', flush=True)
    finally:
        if interface.is_connected:
            try:
                interface.send_fsm_command(2)
            finally:
                interface.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('task_config.json'))
    parser.add_argument('--quaternion-order', choices=['xyzw', 'wxyz'], default='xyzw')
    parser.add_argument('--dry-run', action='store_true', help='只打印计划，不连接机器人或 sleep')
    args = parser.parse_args()
    steps = build_steps(json.loads(args.config.read_text()))
    print('关节目标单位 rad；pose 为 base_link / m；四元数顺序：', args.quaternion_order)
    print('back 来自当前配置；lift 基于实时反馈；不接入视觉、夹爪或底盘。')
    print(json.dumps(steps, ensure_ascii=False, indent=2))
    print(f'累计 sleep={sum(step[4] for step in steps):.3f}s；接口调用耗时会额外累加。')
    if not args.dry_run:
        execute(steps, args.quaternion_order)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        raise SystemExit(f'回放停止：{exc}')
