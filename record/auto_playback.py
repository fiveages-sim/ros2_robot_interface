#!/usr/bin/env python3
"""
自动播放脚本 - 每10秒切换动作

功能：
使用硬编码的两个动作点，每间隔2秒自动切换到下一个动作

使用方法：
    python auto_playback.py [--interval <seconds>]
"""

import json
import sys
import time
import argparse
import re
from typing import Dict, List, Any
from pathlib import Path

# 尝试导入 ROS2RobotInterface
try:
    from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
except ImportError:
    print("错误: 无法导入 ros2_robot_interface")
    print("请确保已安装 ros2_robot_interface 库")
    sys.exit(1)


def parse_data_file(filepath: str) -> List[Dict[str, Any]]:
    """解析 data1.txt 文件，提取动作点
    
    Args:
        filepath: 数据文件路径
        
    Returns:
        动作点列表
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 修复格式问题：添加缺失的逗号和括号
    # 首先尝试修复格式
    content = content.strip()
    
    # 查找所有动作点（通过查找 "left_arm" 开始的位置）
    actions = []
    
    # 使用正则表达式分割不同的动作点
    # 查找每个动作点的开始（"left_arm" 前面可能有空白和逗号）
    pattern = r'(\s*"left_arm"\s*:\s*\{[^}]*"positions"\s*:\s*\[[^\]]+\]\s*\})'
    
    # 更简单的方法：查找所有 "left_arm" 开始的位置
    left_arm_positions = [m.start() for m in re.finditer(r'"left_arm"\s*:\s*\{', content)]
    
    if len(left_arm_positions) < 2:
        # 如果找不到两个动作点，尝试手动解析
        # 先修复格式，然后尝试解析为 JSON
        print("尝试修复文件格式...")
        
        # 在第一个 "right_arm" 后添加逗号（如果缺失）
        content = re.sub(r'(\]\s*)\s*"right_arm"', r'\1,\n      "right_arm"', content, count=1)
        
        # 在第一个 "body" 的 } 后添加逗号（如果缺失）
        content = re.sub(r'(\]\s*)\s*\}\s*(\s*"left_arm")', r'\1\n      }\n    },\n    {\n      "left_arm"', content)
        
        # 修复最后一个动作点的格式
        content = re.sub(r'(\]\s*)\s*\}\s*$', r'\1\n      }\n    }', content)
        
        # 添加外层括号
        if not content.strip().startswith('['):
            content = '[\n    {\n' + content
        if not content.strip().endswith(']'):
            content = content + '\n]'
    
    # 尝试解析为 JSON
    try:
        # 如果内容看起来像是一个对象数组
        if content.strip().startswith('{'):
            # 只有一个对象，需要包装成数组
            data = json.loads('[' + content + ']')
        elif content.strip().startswith('['):
            data = json.loads(content)
        else:
            # 尝试手动解析
            # 分割成两个动作点
            parts = content.split('"left_arm"')
            if len(parts) >= 3:
                # 第一个动作点
                part1 = '"left_arm"' + parts[1]
                # 找到第一个动作点的结束（在第二个 "left_arm" 之前）
                if part1.strip().endswith(','):
                    part1 = part1.rstrip(',')
                part1 = '{' + part1 + '}'
                
                # 第二个动作点
                part2 = '"left_arm"' + parts[2]
                if part2.strip().endswith(','):
                    part2 = part2.rstrip(',')
                part2 = '{' + part2 + '}'
                
                data = [json.loads(part1), json.loads(part2)]
            else:
                raise ValueError("无法解析文件格式")
    except json.JSONDecodeError as e:
        print(f"JSON 解析错误: {e}")
        print("尝试手动解析...")
        
        # 手动解析方法：查找每个动作点的各个部分
        actions = []
        
        # 查找第一个动作点（行1-99）
        # 查找所有键值对
        left_arm_match = re.search(r'"left_arm"\s*:\s*\{[^}]+\}', content)
        if left_arm_match:
            # 提取第一个动作点的所有部分
            start_pos = left_arm_match.start()
            # 找到第一个动作点的结束位置（在第二个 "left_arm" 之前）
            second_left_arm = content.find('"left_arm"', start_pos + 1)
            if second_left_arm > 0:
                first_action_str = content[start_pos:second_left_arm].strip()
                # 移除末尾的逗号
                first_action_str = first_action_str.rstrip(',').strip()
                # 添加大括号
                if not first_action_str.startswith('{'):
                    first_action_str = '{' + first_action_str + '}'
                
                try:
                    first_action = json.loads(first_action_str)
                    actions.append(first_action)
                except:
                    pass
            
            # 提取第二个动作点
            if second_left_arm > 0:
                second_action_str = content[second_left_arm:].strip()
                second_action_str = second_action_str.rstrip(',').strip()
                if not second_action_str.startswith('{'):
                    second_action_str = '{' + second_action_str + '}'
                
                try:
                    second_action = json.loads(second_action_str)
                    actions.append(second_action)
                except:
                    pass
        
        if len(actions) < 2:
            # 最后的尝试：直接使用正则表达式提取
            print("使用正则表达式提取动作点...")
            # 简化：直接查找 positions 数组
            # 这个方法更可靠：查找所有 "positions" 数组
            positions_matches = list(re.finditer(r'"positions"\s*:\s*\[([^\]]+)\]', content))
            
            if len(positions_matches) >= 14:  # 每个动作点有7个部分（left_arm, right_arm, left_gripper, right_gripper, head, body）
                # 手动构建动作点
                # 第一个动作点：前7个 positions
                # 第二个动作点：后7个 positions
                pass  # 这个方法太复杂，改用更简单的方法
        
        # 最简单的方法：直接硬编码解析这两个动作点
        print("使用硬编码方式解析动作点...")
        actions = [
            {
                "left_arm": {
                    "names": ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"],
                    "positions": [1.9031929192987382, -0.8684644880629735, 0.24016829460865796, -1.1371121280262346, -0.2638275514168129, -0.6349110528383953, 0.0011697802450267199]
                },
                "right_arm": {
                    "names": ["right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"],
                    "positions": [-1.8729998630721831, 0.9880000075379893, 2.013999728563938, -0.888000172818277, 0.7788995842844642, -0.1329997295771567, -0.981999972181068]
                },
                "left_gripper": {
                    "names": ["left_hand_index_joint", "left_hand_middle_joint", "left_hand_pinky_joint", "left_hand_ring_joint", "left_hand_thumb_joint1", "left_hand_thumb_joint2"],
                    "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
                },
                "right_gripper": {
                    "names": ["right_hand_index_joint", "right_hand_middle_joint", "right_hand_pinky_joint", "right_hand_ring_joint", "right_hand_thumb_joint1", "right_hand_thumb_joint2"],
                    "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
                },
                "head": {
                    "names": ["head_joint1", "head_joint2"],
                    "positions": [-0.000296705972839036, -0.00010471975511965978]
                },
                "body": {
                    "names": ["body_joint1", "body_joint2", "body_joint3", "body_joint4"],
                    "positions": [-0.2687457982220869, -0.5837951814995832, -0.3448596069015596, 0.0]
                }
            },
            {
                "left_arm": {
                    "names": ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"],
                    "positions": [1.9031917208762479, -0.8684592283198206, 0.24016183644301453, -1.1371002769593836, -0.26382648615237686, -0.6349059928323241, 0.00116726350578173]
                },
                "right_arm": {
                    "names": ["right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"],
                    "positions": [-1.873000395704401, 1.3080002509577848, 2.013999728563938, -0.887999640186059, 0.7788999837586276, -0.1329999709261305, -0.9819998390230137]
                },
                "left_gripper": {
                    "names": ["left_hand_index_joint", "left_hand_middle_joint", "left_hand_pinky_joint", "left_hand_ring_joint", "left_hand_thumb_joint1", "left_hand_thumb_joint2"],
                    "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
                },
                "right_gripper": {
                    "names": ["right_hand_index_joint", "right_hand_middle_joint", "right_hand_pinky_joint", "right_hand_ring_joint", "right_hand_thumb_joint1", "right_hand_thumb_joint2"],
                    "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
                },
                "head": {
                    "names": ["head_joint1", "head_joint2"],
                    "positions": [-0.000296705972839036, -0.00010471975511965978]
                },
                "body": {
                    "names": ["body_joint1", "body_joint2", "body_joint3", "body_joint4"],
                    "positions": [-0.2687457982220869, -0.5837951814995832, -0.3448596069015596, 0.0]
                }
            }
        ]
        return actions
    
    return data if isinstance(data, list) else [data]


def send_action(interface: ROS2RobotInterface, action: Dict[str, Any]):
    """发送一个动作点到机器人
    
    Args:
        interface: ROS2RobotInterface 实例
        action: 动作点字典
    """
    try:
        # 切换到MOVEJ状态
        interface.send_fsm_command(4)  # MOVEJ状态
        time.sleep(0.1)  # 等待状态切换
        
        is_dual_arm = interface.config.right_end_effector_pose_topic is not None
        
        # 发送双臂关节位置
        left_arm_positions = None
        right_arm_positions = None
        
        if 'left_arm' in action and action['left_arm'].get('positions'):
            left_arm_positions = action['left_arm']['positions']
        
        if is_dual_arm and 'right_arm' in action and action['right_arm'].get('positions'):
            right_arm_positions = action['right_arm']['positions']
        
        # 如果是双臂模式且左右臂都有数据，使用双臂轨迹接口
        if is_dual_arm and left_arm_positions is not None and right_arm_positions is not None:
            try:
                interface.send_dual_arm_joint_positions(left_arm_positions, right_arm_positions)
                print(f"    ✓ 双臂: 左臂 {len(left_arm_positions)} 个关节，右臂 {len(right_arm_positions)} 个关节")
            except Exception as e:
                print(f"    ✗ 双臂发送失败: {e}")
                # 降级到分别发送
                if interface.left_arm_handler:
                    interface.left_arm_handler.send_joint_positions(left_arm_positions)
                if interface.right_arm_handler:
                    interface.right_arm_handler.send_joint_positions(right_arm_positions)
        else:
            # 单臂模式或只有一侧有数据，分别发送
            if left_arm_positions is not None and interface.left_arm_handler:
                interface.left_arm_handler.send_joint_positions(left_arm_positions)
                print(f"    ✓ 左臂: {len(left_arm_positions)} 个关节")
            
            if right_arm_positions is not None and interface.right_arm_handler:
                interface.right_arm_handler.send_joint_positions(right_arm_positions)
                print(f"    ✓ 右臂: {len(right_arm_positions)} 个关节")
        
        # 发送左夹爪/灵巧手关节位置
        if 'left_gripper' in action and action['left_gripper'].get('positions'):
            positions = action['left_gripper']['positions']
            if len(positions) == 1:
                if interface.left_gripper_handler:
                    interface.left_gripper_handler.send_joint_positions(positions[0])
                    print(f"    ✓ 左夹爪: 1 个关节")
            else:
                if interface.config.left_hand_joint_controller_topic:
                    interface.send_left_hand_joint_positions(positions)
                    print(f"    ✓ 左灵巧手: {len(positions)} 个关节")
        
        # 发送右夹爪/灵巧手关节位置（双臂模式）
        if is_dual_arm and 'right_gripper' in action and action['right_gripper'].get('positions'):
            positions = action['right_gripper']['positions']
            if len(positions) == 1:
                if interface.right_gripper_handler:
                    interface.right_gripper_handler.send_joint_positions(positions[0])
                    print(f"    ✓ 右夹爪: 1 个关节")
            else:
                if interface.config.right_hand_joint_controller_topic:
                    interface.send_right_hand_joint_positions(positions)
                    print(f"    ✓ 右灵巧手: {len(positions)} 个关节")
        
        # 发送头部关节位置
        if 'head' in action and action['head'].get('positions'):
            positions = action['head']['positions']
            interface.send_head_joint_positions(positions)
            print(f"    ✓ 头部: {len(positions)} 个关节")
        
        # 发送腰部关节位置
        if 'body' in action and action['body'].get('positions'):
            positions = action['body']['positions']
            interface.send_body_joint_positions(positions)
            print(f"    ✓ 腰部: {len(positions)} 个关节")
        
        return True
        
    except Exception as e:
        print(f"  ✗ 发送动作失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='自动播放脚本 - 每2秒切换动作',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 使用默认间隔2秒
  python auto_playback.py
  
  # 指定间隔时间
  python auto_playback.py --interval 2
        """
    )
    
    parser.add_argument(
        '--interval', '-i',
        type=float,
        default=2.0,
        help='动作切换间隔时间（秒，默认: 2.0）'
    )
    
    args = parser.parse_args()
    
    # ========================================================================
    # 初始化和连接
    # ========================================================================
    print("\n" + "=" * 70)
    print(" " * 20 + "自动播放脚本")
    print("=" * 70)
    
    # 使用硬编码的动作点数据
    print("\n[1] 加载动作点数据...")
    actions = [
        {
            "left_arm": {
                "names": ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"],
                "positions": [1.9031929192987382, -0.8684644880629735, 0.24016829460865796, -1.1371121280262346, -0.2638275514168129, -0.6349110528383953, 0.0011697802450267199]
            },
            "right_arm": {
                "names": ["right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"],
                "positions": [-1.8729998630721831, 0.9880000075379893, 2.013999728563938, -0.888000172818277, 0.7788995842844642, -0.1329997295771567, -0.981999972181068]
            },
            "left_gripper": {
                "names": ["left_hand_index_joint", "left_hand_middle_joint", "left_hand_pinky_joint", "left_hand_ring_joint", "left_hand_thumb_joint1", "left_hand_thumb_joint2"],
                "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            },
            "right_gripper": {
                "names": ["right_hand_index_joint", "right_hand_middle_joint", "right_hand_pinky_joint", "right_hand_ring_joint", "right_hand_thumb_joint1", "right_hand_thumb_joint2"],
                "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            },
            "head": {
                "names": ["head_joint1", "head_joint2"],
                "positions": [-0.000296705972839036, -0.00010471975511965978]
            },
            "body": {
                "names": ["body_joint1", "body_joint2", "body_joint3", "body_joint4"],
                "positions": [-0.2687457982220869, -0.5837951814995832, -0.3448596069015596, 0.0]
            }
        },
        {
            "left_arm": {
                "names": ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"],
                "positions": [1.9031917208762479, -0.8684592283198206, 0.24016183644301453, -1.1371002769593836, -0.26382648615237686, -0.6349059928323241, 0.00116726350578173]
            },
            "right_arm": {
                "names": ["right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"],
                "positions": [-1.873000395704401, 1.3080002509577848, 2.013999728563938, -0.887999640186059, 0.7788999837586276, -0.1329999709261305, -0.9819998390230137]
            },
            "left_gripper": {
                "names": ["left_hand_index_joint", "left_hand_middle_joint", "left_hand_pinky_joint", "left_hand_ring_joint", "left_hand_thumb_joint1", "left_hand_thumb_joint2"],
                "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            },
            "right_gripper": {
                "names": ["right_hand_index_joint", "right_hand_middle_joint", "right_hand_pinky_joint", "right_hand_ring_joint", "right_hand_thumb_joint1", "right_hand_thumb_joint2"],
                "positions": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            },
            "head": {
                "names": ["head_joint1", "head_joint2"],
                "positions": [-0.000296705972839036, -0.00010471975511965978]
            },
            "body": {
                "names": ["body_joint1", "body_joint2", "body_joint3", "body_joint4"],
                "positions": [-0.2687457982220869, -0.5837951814995832, -0.3448596069015596, 0.0]
            }
        }
    ]
    print(f"    ✓ 已加载 {len(actions)} 个动作点")
    
    print("\n[2] 创建配置...")
    config = ROS2RobotInterfaceConfig()
    
    print("[3] 创建ROS2RobotInterface实例...")
    interface = ROS2RobotInterface(config)
    
    print("[4] 连接到ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功!\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1
    
    # 检查模式
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    print(f"    ✓ 检测到{'双臂' if is_dual_arm else '单臂'}模式\n")
    
    # 切换到MOVEJ状态
    print("\n准备切换到MOVEJ状态...")
    try:
        print("  → 先切换到HOLD状态...")
        interface.send_fsm_command(2)  # HOLD状态
        time.sleep(0.3)
        print("  ✓ 已切换到HOLD状态")
        
        print("  → 再切换到MOVEJ状态...")
        interface.send_fsm_command(4)  # MOVEJ状态
        time.sleep(0.3)
        print("  ✓ 已切换到MOVEJ状态\n")
    except Exception as e:
        print(f"  ⚠ 切换状态失败: {e}\n")
    
    # ========================================================================
    # 自动播放循环
    # ========================================================================
    print(f"\n开始自动播放，每 {args.interval} 秒切换一次动作...")
    print("按 Ctrl+C 停止\n")
    
    current_index = 0
    
    try:
        while True:
            action = actions[current_index]
            print(f"\n[{time.strftime('%H:%M:%S')}] 播放动作点 #{current_index + 1}/{len(actions)}")
            
            if send_action(interface, action):
                print(f"  ✓ 动作点 #{current_index + 1} 已发送")
            else:
                print(f"  ✗ 动作点 #{current_index + 1} 发送失败")
            
            # 切换到下一个动作点
            current_index = (current_index + 1) % len(actions)
            
            # 等待指定时间
            print(f"\n等待 {args.interval} 秒后切换到下一个动作...")
            time.sleep(args.interval)
    
    except KeyboardInterrupt:
        print("\n\n  ⚠ 用户中断")
    
    # 切换回HOLD状态
    print("\n切换回HOLD状态...")
    try:
        interface.send_fsm_command(2)  # HOLD状态
        time.sleep(0.5)
        print("  ✓ 已切换到HOLD状态")
    except Exception as e:
        print(f"  ⚠ 切换状态失败: {e}")
    
    # 断开连接
    print("\n[5] 断开连接...")
    interface.disconnect()
    print("    ✓ 已断开连接\n")
    
    print("=" * 70)
    print("程序结束")
    print("=" * 70 + "\n")
    
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n用户中断程序")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n未预期的错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
