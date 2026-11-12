"""通过 Python 公开接口验证分体腰部单点/多点 Action。

已激活 ROS/Python 环境后运行本文件。会实际运动：旋转默认 0.02 rad，
升降默认 0.005 m。按控制器 joints 和 URDF 生成目标，不自动返回初始姿态。
结束尝试 HOLD、断开连接；建议先在仿真验证。--constraint 测试约束模式。
"""
import argparse
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delta-rad', type=float, default=0.02)
    parser.add_argument('--delta-m', type=float, default=0.005)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--dwell', type=float, default=0.0, help='首目标后停留秒数，仅累计时间模式')
    parser.add_argument('--constraint', action='store_true')
    parser.add_argument('--timing', choices=['waypoints', 'total', 'default'], default='waypoints',
                        help='累计时间、持久总时长或控制器默认时长；约束模式不填写时间')
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in (args.delta_rad, args.delta_m, args.duration)):
        parser.error('偏移和时长必须为有限正数')
    if not math.isfinite(args.dwell) or args.dwell < 0 or (args.dwell and (args.constraint or args.timing != 'waypoints')):
        parser.error('dwell 必须非负有限且只用于累计时间模式')
    interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
    query = None
    try:
        interface.connect()
        if interface.is_wbc:
            raise RuntimeError('本脚本仅验证分体腰部')
        if not interface.wait_for_body_joint_trajectory_action_server():
            raise RuntimeError('未发现分体腰部 Action')
        query = rclpy.create_node('check_body_action_parameters')
        client = query.create_client(GetParameters, interface.body_controller + '/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('body 参数服务不可用')
        future = client.call_async(GetParameters.Request(names=['joints', 'movej_interpolation_type']))
        rclpy.spin_until_future_complete(query, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError('读取 body 参数超时')
        params = future.result().values
        names = list(reversed(params[0].string_array_value))
        if not names or params[1].string_value != 'doubles':
            raise RuntimeError('需要非空 body joints 和 doubles 插值模式')
        joints = {j.get('name'): j for j in ET.fromstring(interface.get_robot_description()).findall('joint')}
        tolerances = {n: 0.001 if joints[n].get('type') == 'prismatic' else 0.005 for n in names}

        def positions():
            state = interface.get_joint_state()
            if state is None:
                raise RuntimeError('没有有效关节反馈')
            result = dict(zip(state['names'], state['positions']))
            if any(n not in result or not math.isfinite(result[n]) for n in names):
                raise RuntimeError('腰部反馈缺失或非法')
            return result

        all_names = names
        for multi in (False, True):
            names = all_names if multi else all_names[::2]
            before = positions()
            targets = []
            velocity, acceleration, jerk = [], [], []
            for n in names:
                joint = joints[n]
                linear = joint.get('type') == 'prismatic'
                delta = args.delta_m if linear else args.delta_rad
                limit = joint.find('limit')
                if joint.get('type') == 'continuous':
                    lower, upper = -math.inf, math.inf
                elif limit is not None:
                    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                else:
                    raise RuntimeError(f'{n} 缺少关节限位')
                if before[n] + delta > upper:
                    delta = -delta
                target = before[n] + delta
                if not lower <= target <= upper:
                    raise RuntimeError(f'{n} 没有足够运动空间')
                targets.append(target)
                print(n, 'm' if linear else 'rad', before[n], '->', target)
                velocity.append(0.02 if linear else 0.1)
                acceleration.append(0.04 if linear else 0.2)
                jerk.append(0.2 if linear else 1.0)
            points = [[before[n] + (t - before[n]) * f for n, t in zip(names, targets)]
                      for f in ([0.5, 1.0] if multi else [1.0])]
            arrival_times = [args.duration * (i + 1) for i in range(len(points))]
            if args.dwell:
                points.insert(1, list(points[0]))
                arrival_times.insert(1, arrival_times[0] + args.dwell)
                for i in range(2, len(arrival_times)):
                    arrival_times[i] += args.dwell
            feedback = []
            drift = {}
            observer_errors = []

            def observe(msg):
                feedback.append(msg)
                try:
                    current = positions()
                except Exception as exc:
                    observer_errors.append(str(exc))
                    return
                for n in before.keys() - set(names):
                    if n in current:
                        drift[n] = max(drift.get(n, 0.0), abs(current[n] - before[n]))

            interface.send_fsm_command(2)
            result = interface.execute_body_joint_trajectory_action(
                names, points, time_mode=not args.constraint,
                movej_trajectory_duration=args.duration if not args.constraint and args.timing == 'total' else None,
                time_from_start=arrival_times if not args.constraint and args.timing == 'waypoints' else None,
                max_velocity=velocity, max_acceleration=acceleration, max_jerk=jerk,
                feedback_callback=observe,
                blend_ratio_percent=0.0, timeout=60.0,
            )
            if observer_errors:
                raise RuntimeError(f"反馈观察失败：{observer_errors}")
            if result is None or not result.success or not feedback:
                raise RuntimeError(f'Action 执行失败或没有反馈：{result}')
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                current = positions()
                if all(abs(current[n] - target) <= tolerances[n] for n, target in zip(names, targets)):
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError('Action 完成后腰部未到位')
            print('PASS', '多点' if multi else '单点', result, '其他关节最大偏移', max(drift.values(), default=0.0))
            if max(drift.values(), default=0.0) > 0.01:
                raise RuntimeError('未指定关节偏移超过 0.01（按各关节自身单位）')
    finally:
        try:
            if interface.is_connected:
                interface.send_fsm_command(2)
        finally:
            if query is not None:
                query.destroy_node()
            interface.disconnect()


if __name__ == '__main__':
    main()
