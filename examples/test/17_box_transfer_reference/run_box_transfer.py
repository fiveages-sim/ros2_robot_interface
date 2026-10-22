"""搬箱平均目标流程测试；默认执行三档流程；--dry-run 仅离线预览。"""

import argparse
import json
import math
from pathlib import Path
import time

# base_link；四元数分量顺序保留原日志，执行时必须显式指定。
REFERENCE_POSES = {'outside': [[0.71794, 0.291189, 0.848538, 0.701574, -0.07667, 0.702321, 0.093068],
             [0.693566, -0.294651, 0.84093, -0.70607, -0.042533, -0.705589, 0.042415]],
 'grasp': [[0.717941, 0.241189, 0.848538, 0.701574, -0.07667, 0.702321, 0.093068],
           [0.693566, -0.244651, 0.84093, -0.70607, -0.042533, -0.705589, 0.042415]],
 'lower_low': [[0.630756, 0.250375, 0.605546, 0.698509, -0.058087, 0.708476, 0.082299],
               [0.63046, -0.236489, 0.608727, -0.70039, -0.062271, -0.708613, 0.05868]],
 'withdraw_low': [[0.630756, 0.300375, 0.605546, 0.698509, -0.058087, 0.708476, 0.082299],
                  [0.63046, -0.286489, 0.608727, -0.70039, -0.062271, -0.708613, 0.05868]],
 'place_middle': [[0.634176, 0.255588, 1.31341, 0.707012, -0.052901, 0.701579, 0.071569],
                  [0.636553, -0.230175, 1.315841, -0.707835, -0.065723, -0.700347, 0.064527]],
 'lower_middle': [[0.635072, 0.256372, 0.952833, 0.706312, -0.052962, 0.702219, 0.072158],
                  [0.637732, -0.230727, 0.955089, -0.706984, -0.066007, -0.701108, 0.065298]],
 'withdraw_middle': [[0.635072, 0.306372, 0.952833, 0.706312, -0.052962, 0.702219, 0.072158],
                     [0.637732, -0.280727, 0.955089, -0.706984, -0.066007, -0.701108, 0.065298]],
 'place_high': [[0.586664, 0.270981, 1.686266, 0.682399, -0.043115, 0.726976, 0.063076],
                [0.610261, -0.22063, 1.689442, -0.681363, -0.081528, -0.723703, 0.073152]],
 'lower_high': [[0.591298, 0.272864, 1.297893, 0.680957, -0.043659, 0.728193, 0.064246],
                [0.615044, -0.220746, 1.3008, -0.679494, -0.081957, -0.725248, 0.074733]],
 'withdraw_high': [[0.591763, 0.323208, 1.297741, 0.680363, -0.046621, 0.728236, 0.067858],
                   [0.611344, -0.270975, 1.300568, -0.679155, -0.0787, -0.726283, 0.071186]]}


def build_sequence(config, levels, station, place, body_layers):
    pick = config['grasp'][station]
    box = pick['box']
    home = dict(body=box['home_body_list'], left_arm=box['home_left_arm_joints'],
                right_arm=box['home_right_arm_joints'])
    back = [pick['back']['back_left_arm_pose'], pick['back']['back_right_arm_pose']]
    steps = []
    for level in levels:
        def add(name, kind, target):
            steps.append(dict(level=level, name=name, kind=kind, target=target))
        add('头部观察', 'joint', dict(head=[0.0, math.degrees(0.26)]))
        add('全身准备', 'joint', home)
        add('外侧预备（代替视觉）', 'pose', REFERENCE_POSES['outside'])
        add('向内抓箱（无夹爪指令）', 'pose', REFERENCE_POSES['grasp'])
        add('基于当前反馈同步抬升', 'lift', 0.4)
        add('返回持箱位姿', 'pose', back)
        if level in body_layers:
            body = config['grasp']['place'][place]['layer_num'][body_layers[level]]
            add('码放腰部姿态', 'joint', dict(body=body))
        if level != 'low':
            add('码放对位（代替视觉）', 'pose', REFERENCE_POSES['place_'+level])
        # 低档没有独立可靠的码放预备点，沿用持箱 back 后下放的日志路径。
        add('放箱下移', 'pose', REFERENCE_POSES['lower_'+level])
        add('向外移开', 'pose', REFERENCE_POSES['withdraw_'+level])
        add('抬起撤离（原日志与外移相同）', 'pose', REFERENCE_POSES['withdraw_'+level])
        add('全身复位', 'joint', home)
    return steps


def make_pose(values, order):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[:3]
    q = values[3:]
    if order == 'wxyz':
        q = [q[1], q[2], q[3], q[0]]
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-9:
        raise ValueError('无效四元数')
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [x/norm for x in q]
    return pose


def current_pair(interface):
    pair = []
    for handler in (interface.left_arm_handler, interface.right_arm_handler):
        pose, frame = handler.get_pose(), handler.get_frame_id()
        if pose is None or not frame:
            raise RuntimeError('无法获取双臂反馈或坐标系')
        pose = interface.transform_pose(pose, frame, 'base_link', timeout=1.0)
        if pose is None:
            raise RuntimeError('反馈转换到 base_link 失败')
        pair.append(pose)
    return pair


def pose_arrived(current, target, pos_tol, angle_tol):
    distance = math.sqrt(sum((getattr(current.position, k)-getattr(target.position, k))**2
                             for k in ('x', 'y', 'z')))
    a = [getattr(current.orientation, k) for k in ('x', 'y', 'z', 'w')]
    b = [getattr(target.orientation, k) for k in ('x', 'y', 'z', 'w')]
    norm = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    if norm < 1e-9:
        return False
    angle = math.degrees(2*math.acos(min(1.0, abs(sum(x*y for x,y in zip(a,b))/norm))))
    return distance <= pos_tol and angle <= angle_tol


def wait_until(check, timeout):
    deadline = time.monotonic()+timeout
    stable = 0
    while time.monotonic() < deadline:
        stable = stable+1 if check() else 0
        if stable >= 3:
            return
        time.sleep(0.1)
    raise TimeoutError('到位超时，停止后续流程')


def execute(steps, args):
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    try:
        interface.connect()
        time.sleep(1.0)
        if not interface.is_wbc or not interface.right_arm_handler:
            raise RuntimeError('本流程要求 WBC 双臂模式')
        current_pair(interface)
        for index, step in enumerate(steps, 1):
            print(f"[{index}/{len(steps)}] {step['level']} {step['name']}", flush=True)
            if not args.auto:
                if input('回车执行；输入 q 结束：').strip().lower() == 'q':
                    break
            if step['kind'] == 'joint':
                target = {k: [math.radians(x) for x in v] for k,v in step['target'].items()}
                if not interface.set_node_parameters(interface.arm_controller,
                                                     {'movej_duration': args.duration}):
                    raise RuntimeError('设置 movej_duration 失败')
                interface.send_coordinated_joint_positions(**{k+'_positions':v for k,v in target.items()})
                def joints_arrived():
                    state = interface.get_joint_state(categorized=True) or {}
                    for key, values in target.items():
                        actual = state.get(key, {}).get('positions')
                        if actual is None or len(actual) != len(values):
                            return False
                        if max(abs(x-y) for x,y in zip(actual,values)) > args.joint_tolerance:
                            return False
                    return True
                wait_until(joints_arrived, args.timeout)
            else:
                if step['kind'] == 'lift':
                    poses = current_pair(interface)
                    for pose in poses:
                        pose.position.z += step['target']
                else:
                    poses = [make_pose(v, args.quaternion_order) for v in step['target']]
                interface.auto_switch_fsm_for_control('arm_pose')
                interface.send_mode_command('LEFT_ARM_ENABLE')
                interface.send_mode_command('RIGHT_ARM_ENABLE')
                interface.send_dual_arm_target_stamped(*poses, frame_id='base_link',
                                                       movel_duration=args.duration)
                wait_until(lambda: all(pose_arrived(c,t,args.position_tolerance,args.angle_tolerance)
                                        for c,t in zip(current_pair(interface),poses)), args.timeout)
    finally:
        if interface.is_connected:
            try:
                interface.send_fsm_command(2)  # HOLD，不自动返回 home。
            finally:
                interface.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[5]/'shunfeng/task_config.json')
    parser.add_argument('--level', choices=['low','middle','high','all'], default='all')
    parser.add_argument('--station', default='A')
    parser.add_argument('--place', default='G')
    parser.add_argument('--body-layers', default='1,2,0', help='低、中、高对应的 config 层索引，默认 1,2,0')
    parser.add_argument('--quaternion-order', choices=['xyzw','wxyz'], default='xyzw', help='经原始发布代码确认的日志四元数顺序')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', dest='execute', action='store_true', help='执行运动（默认）')
    mode.add_argument('--dry-run', dest='execute', action='store_false', help='仅预览，不连接机器人')
    parser.set_defaults(execute=True)
    parser.add_argument('--auto', action='store_true', help='执行时省略每步回车')
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--position-tolerance', type=float, default=0.01)
    parser.add_argument('--angle-tolerance', type=float, default=3.0, help='度')
    parser.add_argument('--joint-tolerance', type=float, default=0.03, help='弧度')
    args = parser.parse_args()
    if any(v <= 0 or not math.isfinite(v) for v in (args.duration,args.timeout,args.position_tolerance,
                                                   args.angle_tolerance,args.joint_tolerance)):
        parser.error('时长和容差必须为有限正数')
    if args.execute and (not args.quaternion_order or not args.body_layers):
        parser.error('执行前必须指定 --quaternion-order 和 --body-layers')
    mapping = {}
    if args.body_layers:
        keys = args.body_layers.split(',')
        if len(keys) != 3:
            parser.error('--body-layers 必须为低、中、高三个逗号分隔的层索引')
        mapping = dict(zip(['low','middle','high'],keys))
    config = json.loads(args.config.read_text())
    levels = ['low','middle','high'] if args.level == 'all' else [args.level]
    steps = build_sequence(config, levels, args.station, args.place, mapping)
    print(json.dumps(steps, ensure_ascii=False, indent=2))
    if args.execute:
        execute(steps,args)
    else:
        print('仅预览，没有连接 ROS 或下发运动；未指定 body-layers 时预览不含码放腰部动作。')


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        raise SystemExit(f'流程停止：{exc}')
