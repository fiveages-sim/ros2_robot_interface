"""
关节录制和回放脚本

功能：
1. 录制模式：录制当前所有关节状态和末端pose（左右灵巧手/夹爪、左右臂、腰部、头部），按Enter记录一次节点，可随时保存为JSON
2. 回放模式：加载JSON文件，可选择MOVEJ模式（关节控制）或OCS2模式（pose控制），按Enter发送一次指令

使用方法：
    python joint_record_playback.py [record|playback] [--file <json_file>]
"""

import json
import sys
import time
import argparse
import os
import logging
from typing import Dict, List, Optional, Any
from pathlib import Path

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig

logger = logging.getLogger(__name__)

# 默认保存目录
DEFAULT_RECORD_DIR = "joint_records"


def get_record_directory() -> Path:
    """获取录制文件保存目录，如果不存在则创建
    
    Returns:
        Path对象，指向保存目录
    """
    record_dir = Path.cwd() / DEFAULT_RECORD_DIR
    record_dir.mkdir(exist_ok=True)
    return record_dir


def list_record_files() -> List[Path]:
    """列出保存目录中的所有JSON文件
    
    Returns:
        JSON文件路径列表，按修改时间倒序排列
    """
    record_dir = get_record_directory()
    json_files = sorted(
        record_dir.glob("*.json"),
        key=lambda p: p.stat().st_mtime,
        reverse=True
    )
    return json_files


class JointRecorder:
    """关节录制器"""
    
    def __init__(self, interface: ROS2RobotInterface):
        self.interface = interface
        self.recorded_nodes: List[Dict[str, Any]] = []
        self.is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    
    def record_node(self) -> bool:
        """录制一个节点（当前所有关节状态）
        
        Returns:
            True if successfully recorded, False otherwise
        """
        # 获取分类后的关节状态
        categorized_state = self.interface.get_joint_state(categorized=True)
        if not categorized_state:
            print("  ✗ 无法获取关节状态，请确保机器人正在运行")
            return False
        
        # 构建节点数据
        node_data: Dict[str, Any] = {
            'timestamp': time.time(),
            'node_index': len(self.recorded_nodes) + 1
        }
        
        # 提取左臂关节
        left_arm_data = categorized_state.get('left_arm', {})
        if left_arm_data.get('positions'):
            node_data['left_arm'] = {
                'names': left_arm_data.get('names', []),
                'positions': left_arm_data.get('positions', [])
            }
        
        # 提取右臂关节（双臂模式）
        if self.is_dual_arm:
            right_arm_data = categorized_state.get('right_arm', {})
            if right_arm_data.get('positions'):
                node_data['right_arm'] = {
                    'names': right_arm_data.get('names', []),
                    'positions': right_arm_data.get('positions', [])
                }
        
        # 提取左夹爪/灵巧手关节
        left_gripper_data = categorized_state.get('left_gripper' if self.is_dual_arm else 'gripper', {})
        if left_gripper_data.get('positions'):
            node_data['left_gripper'] = {
                'names': left_gripper_data.get('names', []),
                'positions': left_gripper_data.get('positions', [])
            }
        
        # 提取右夹爪/灵巧手关节（双臂模式）
        if self.is_dual_arm:
            right_gripper_data = categorized_state.get('right_gripper', {})
            if right_gripper_data.get('positions'):
                node_data['right_gripper'] = {
                    'names': right_gripper_data.get('names', []),
                    'positions': right_gripper_data.get('positions', [])
                }
        
        # 提取头部关节
        head_data = categorized_state.get('head', {})
        if head_data.get('positions'):
            node_data['head'] = {
                'names': head_data.get('names', []),
                'positions': head_data.get('positions', [])
            }
        
        # 提取腰部关节
        body_data = categorized_state.get('body', {})
        if body_data.get('positions'):
            node_data['body'] = {
                'names': body_data.get('names', []),
                'positions': body_data.get('positions', [])
            }
        
        # 获取并保存末端执行器pose
        left_pose = self.interface.get_end_effector_pose()
        if left_pose and self.interface.left_arm_handler:
            left_frame_id = self.interface.left_arm_handler.get_frame_id()
            node_data['left_end_effector_pose'] = {
                'position': {
                    'x': left_pose.position.x,
                    'y': left_pose.position.y,
                    'z': left_pose.position.z
                },
                'orientation': {
                    'x': left_pose.orientation.x,
                    'y': left_pose.orientation.y,
                    'z': left_pose.orientation.z,
                    'w': left_pose.orientation.w
                },
                'frame_id': left_frame_id if left_frame_id else 'arm_base'  # 如果没有frame_id，使用默认值
            }
        
        if self.is_dual_arm:
            right_pose = self.interface.get_right_end_effector_pose()
            if right_pose and self.interface.right_arm_handler:
                right_frame_id = self.interface.right_arm_handler.get_frame_id()
                node_data['right_end_effector_pose'] = {
                    'position': {
                        'x': right_pose.position.x,
                        'y': right_pose.position.y,
                        'z': right_pose.position.z
                    },
                    'orientation': {
                        'x': right_pose.orientation.x,
                        'y': right_pose.orientation.y,
                        'z': right_pose.orientation.z,
                        'w': right_pose.orientation.w
                    },
                    'frame_id': right_frame_id if right_frame_id else 'arm_base'  # 如果没有frame_id，使用默认值
                }
        
        # 保存节点
        self.recorded_nodes.append(node_data)
        
        # 打印节点信息
        print(f"\n  ✓ 已记录节点 #{len(self.recorded_nodes)}")
        if 'left_arm' in node_data:
            print(f"    左臂: {len(node_data['left_arm']['positions'])} 个关节")
        if 'right_arm' in node_data:
            print(f"    右臂: {len(node_data['right_arm']['positions'])} 个关节")
        if 'left_gripper' in node_data:
            print(f"    左夹爪/手: {len(node_data['left_gripper']['positions'])} 个关节")
        if 'right_gripper' in node_data:
            print(f"    右夹爪/手: {len(node_data['right_gripper']['positions'])} 个关节")
        if 'head' in node_data:
            print(f"    头部: {len(node_data['head']['positions'])} 个关节")
        if 'body' in node_data:
            print(f"    腰部: {len(node_data['body']['positions'])} 个关节")
        if 'left_end_effector_pose' in node_data:
            print(f"    左臂末端pose: 已记录")
        if 'right_end_effector_pose' in node_data:
            print(f"    右臂末端pose: 已记录")
        
        return True
    
    def save_to_json(self, filename: str) -> bool:
        """保存录制的节点到JSON文件
        
        Args:
            filename: 文件名（不含路径，会自动保存到记录目录）
            
        Returns:
            True if successfully saved, False otherwise
        """
        if not self.recorded_nodes:
            print("  ✗ 没有录制的节点可保存")
            return False
        
        try:
            # 确保文件名以.json结尾
            if not filename.endswith('.json'):
                filename += '.json'
            
            # 获取保存目录并构建完整路径
            record_dir = get_record_directory()
            filepath = record_dir / filename
            
            # 如果文件已存在，询问是否覆盖
            if filepath.exists():
                overwrite = input(f"  文件 {filename} 已存在，是否覆盖？(y/n): ").strip().lower()
                if overwrite not in ['y', 'yes']:
                    print("  ✗ 取消保存")
                    return False
            
            output_data = {
                'metadata': {
                    'total_nodes': len(self.recorded_nodes),
                    'is_dual_arm': self.is_dual_arm,
                    'recorded_at': time.strftime('%Y-%m-%d %H:%M:%S')
                },
                'nodes': self.recorded_nodes
            }
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(output_data, f, indent=2, ensure_ascii=False)
            
            print(f"\n  ✓ 已保存 {len(self.recorded_nodes)} 个节点到: {filepath}")
            return True
        except Exception as e:
            print(f"  ✗ 保存失败: {e}")
            return False


class JointPlayer:
    """关节回放器"""
    
    def __init__(self, interface: ROS2RobotInterface):
        self.interface = interface
        self.nodes: List[Dict[str, Any]] = []
        self.current_index = 0
        self.is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    
    def load_from_json(self, filepath: str) -> bool:
        """从JSON文件加载节点数据
        
        Args:
            filepath: JSON文件路径
            
        Returns:
            True if successfully loaded, False otherwise
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            if 'nodes' not in data:
                print("  ✗ JSON文件格式错误：缺少 'nodes' 字段")
                return False
            
            self.nodes = data['nodes']
            metadata = data.get('metadata', {})
            
            print(f"\n  ✓ 已加载 {len(self.nodes)} 个节点")
            if metadata:
                print(f"    录制时间: {metadata.get('recorded_at', '未知')}")
                print(f"    双臂模式: {metadata.get('is_dual_arm', False)}")
            
            # 检查模式是否匹配
            recorded_dual_arm = metadata.get('is_dual_arm', False)
            if recorded_dual_arm != self.is_dual_arm:
                print(f"  ⚠ 警告: 录制时是{'双臂' if recorded_dual_arm else '单臂'}模式，当前是{'双臂' if self.is_dual_arm else '单臂'}模式")
            
            return True
        except FileNotFoundError:
            print(f"  ✗ 文件不存在: {filepath}")
            return False
        except json.JSONDecodeError as e:
            print(f"  ✗ JSON解析错误: {e}")
            return False
        except Exception as e:
            print(f"  ✗ 加载失败: {e}")
            return False
    
    def play_next_node(self) -> bool:
        """播放下一个节点
        
        Returns:
            True if successfully played, False if no more nodes
        """
        if self.current_index >= len(self.nodes):
            print("  ✗ 已到达最后一个节点")
            return False
        
        node = self.nodes[self.current_index]
        self.current_index += 1
        
        print(f"\n  → 播放节点 #{self.current_index}/{len(self.nodes)}")
        
        try:
            # 切换到MOVEJ状态（如果还没有切换）
            # 注意：send_joint_positions 会自动切换，但为了确保，我们先切换一次
            self.interface.send_fsm_command(4)  # MOVEJ状态
            time.sleep(0.1)  # 等待状态切换
            
            # 发送双臂关节位置（使用双臂轨迹接口）
            left_arm_positions = None
            right_arm_positions = None
            
            if 'left_arm' in node and node['left_arm'].get('positions'):
                left_arm_positions = node['left_arm']['positions']
            
            if self.is_dual_arm and 'right_arm' in node and node['right_arm'].get('positions'):
                right_arm_positions = node['right_arm']['positions']
            
            # 如果是双臂模式且左右臂都有数据，使用双臂轨迹接口
            if self.is_dual_arm and left_arm_positions is not None and right_arm_positions is not None:
                try:
                    self.interface.send_dual_arm_joint_positions(left_arm_positions, right_arm_positions)
                    print(f"    ✓ 双臂: 左臂 {len(left_arm_positions)} 个关节，右臂 {len(right_arm_positions)} 个关节")
                except Exception as e:
                    print(f"    ✗ 双臂发送失败: {e}")
                    # 降级到分别发送
                    if self.interface.left_arm_handler:
                        self.interface.left_arm_handler.send_joint_positions(left_arm_positions)
                        print(f"    ✓ 左臂: {len(left_arm_positions)} 个关节（降级模式）")
                    if self.interface.right_arm_handler:
                        self.interface.right_arm_handler.send_joint_positions(right_arm_positions)
                        print(f"    ✓ 右臂: {len(right_arm_positions)} 个关节（降级模式）")
            else:
                # 单臂模式或只有一侧有数据，分别发送
                if left_arm_positions is not None and self.interface.left_arm_handler:
                    self.interface.left_arm_handler.send_joint_positions(left_arm_positions)
                    print(f"    ✓ 左臂: {len(left_arm_positions)} 个关节")
                
                if right_arm_positions is not None and self.interface.right_arm_handler:
                    self.interface.right_arm_handler.send_joint_positions(right_arm_positions)
                    print(f"    ✓ 右臂: {len(right_arm_positions)} 个关节")
            
            # 发送左夹爪/灵巧手关节位置
            if 'left_gripper' in node and node['left_gripper'].get('positions'):
                positions = node['left_gripper']['positions']
                # 判断是单关节夹爪还是多关节灵巧手
                if len(positions) == 1:
                    # 单关节夹爪，使用 gripper_handler
                    if self.interface.left_gripper_handler:
                        self.interface.left_gripper_handler.send_joint_positions(positions[0])
                        print(f"    ✓ 左夹爪: 1 个关节")
                else:
                    # 多关节灵巧手，使用关节控制器
                    if self.interface.config.left_hand_joint_controller_topic:
                        self.interface.send_left_hand_joint_positions(positions)
                        print(f"    ✓ 左灵巧手: {len(positions)} 个关节")
                    elif self.interface.left_gripper_handler:
                        # 如果没有关节控制器，尝试使用 gripper_handler（可能会失败）
                        logger.warning("Left hand joint controller not available, trying gripper handler (may fail)")
                        try:
                            self.interface.left_gripper_handler.send_joint_positions(positions[0])
                            print(f"    ⚠ 左灵巧手: 仅发送第一个关节（{len(positions)} 个关节可用）")
                        except Exception as e:
                            print(f"    ✗ 左灵巧手发送失败: {e}")
            
            # 发送右夹爪/灵巧手关节位置（双臂模式）
            if self.is_dual_arm and 'right_gripper' in node and node['right_gripper'].get('positions'):
                positions = node['right_gripper']['positions']
                # 判断是单关节夹爪还是多关节灵巧手
                if len(positions) == 1:
                    # 单关节夹爪，使用 gripper_handler
                    if self.interface.right_gripper_handler:
                        self.interface.right_gripper_handler.send_joint_positions(positions[0])
                        print(f"    ✓ 右夹爪: 1 个关节")
                else:
                    # 多关节灵巧手，使用关节控制器
                    if self.interface.config.right_hand_joint_controller_topic:
                        self.interface.send_right_hand_joint_positions(positions)
                        print(f"    ✓ 右灵巧手: {len(positions)} 个关节")
                    elif self.interface.right_gripper_handler:
                        # 如果没有关节控制器，尝试使用 gripper_handler（可能会失败）
                        logger.warning("Right hand joint controller not available, trying gripper handler (may fail)")
                        try:
                            self.interface.right_gripper_handler.send_joint_positions(positions[0])
                            print(f"    ⚠ 右灵巧手: 仅发送第一个关节（{len(positions)} 个关节可用）")
                        except Exception as e:
                            print(f"    ✗ 右灵巧手发送失败: {e}")
            
            # 发送头部关节位置
            if 'head' in node and node['head'].get('positions'):
                positions = node['head']['positions']
                self.interface.send_head_joint_positions(positions)
                print(f"    ✓ 头部: {len(positions)} 个关节")
            
            # 发送腰部关节位置
            if 'body' in node and node['body'].get('positions'):
                positions = node['body']['positions']
                self.interface.send_body_joint_positions(positions)
                print(f"    ✓ 腰部: {len(positions)} 个关节")
            
            print(f"  ✓ 节点 #{self.current_index} 已发送")
            return True
            
        except Exception as e:
            print(f"  ✗ 播放节点失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def play_next_node_ocs2(self) -> bool:
        """使用OCS2模式播放下一个节点（通过pose控制）
        
        Returns:
            True if successfully played, False if no more nodes
        """
        if self.current_index >= len(self.nodes):
            print("  ✗ 已到达最后一个节点")
            return False
        
        node = self.nodes[self.current_index]
        self.current_index += 1
        
        print(f"\n  → 播放节点 #{self.current_index}/{len(self.nodes)} (OCS2模式)")
        
        try:
            # 切换到OCS2状态
            self.interface.send_fsm_command(3)  # OCS2状态
            time.sleep(0.3)  # 等待状态切换
            
            # 检查是否有pose数据
            has_left_pose = 'left_end_effector_pose' in node
            has_right_pose = self.is_dual_arm and 'right_end_effector_pose' in node
            
            if not has_left_pose and not has_right_pose:
                print("  ⚠ 警告: 节点中没有末端pose数据，无法使用OCS2模式")
                print("  请使用MOVEJ模式回放，或重新录制包含pose数据的节点")
                return False
            
            from geometry_msgs.msg import Pose
            
            # 如果是双臂模式且左右臂都有pose数据，使用双臂接口一次发送
            if self.is_dual_arm and has_left_pose and has_right_pose:
                # 构建左臂pose
                left_pose = Pose()
                left_pose_data = node['left_end_effector_pose']
                left_pose.position.x = left_pose_data['position']['x']
                left_pose.position.y = left_pose_data['position']['y']
                left_pose.position.z = left_pose_data['position']['z']
                left_pose.orientation.x = left_pose_data['orientation']['x']
                left_pose.orientation.y = left_pose_data['orientation']['y']
                left_pose.orientation.z = left_pose_data['orientation']['z']
                left_pose.orientation.w = left_pose_data['orientation']['w']
                
                # 构建右臂pose
                right_pose = Pose()
                right_pose_data = node['right_end_effector_pose']
                right_pose.position.x = right_pose_data['position']['x']
                right_pose.position.y = right_pose_data['position']['y']
                right_pose.position.z = right_pose_data['position']['z']
                right_pose.orientation.x = right_pose_data['orientation']['x']
                right_pose.orientation.y = right_pose_data['orientation']['y']
                right_pose.orientation.z = right_pose_data['orientation']['z']
                right_pose.orientation.w = right_pose_data['orientation']['w']
                
                # 获取frame_id（优先使用左臂的frame_id，如果不同则警告）
                left_frame_id = left_pose_data.get('frame_id', 'arm_base')
                right_frame_id = right_pose_data.get('frame_id', 'arm_base')
                frame_id = left_frame_id
                
                if left_frame_id != right_frame_id:
                    print(f"    ⚠ 警告: 左右臂frame_id不同 (左: {left_frame_id}, 右: {right_frame_id})，使用左臂的frame_id")
                
                # 使用send_dual_arm_target_stamped一次发送双臂pose到/dual_target/stamped topic
                try:
                    self.interface.send_dual_arm_target_stamped(left_pose, right_pose, frame_id)
                    print(f"    ✓ 双臂: 已发送目标pose到/dual_target/stamped (frame: {frame_id})")
                except Exception as e:
                    print(f"    ✗ 双臂发送失败: {e}")
                    # 降级到分别发送
                    if self.interface.left_arm_handler:
                        self.interface.left_arm_handler.send_target_stamped(left_frame_id, left_pose)
                        print(f"    ✓ 左臂: 已发送目标pose (frame: {left_frame_id})（降级模式）")
                    if self.interface.right_arm_handler:
                        self.interface.right_arm_handler.send_target_stamped(right_frame_id, right_pose)
                        print(f"    ✓ 右臂: 已发送目标pose (frame: {right_frame_id})（降级模式）")
            else:
                # 单臂模式或只有一侧有数据，分别发送
                # 发送左臂目标pose（使用stamped版本）
                if has_left_pose and self.interface.left_arm_handler:
                    left_pose = Pose()
                    pose_data = node['left_end_effector_pose']
                    left_pose.position.x = pose_data['position']['x']
                    left_pose.position.y = pose_data['position']['y']
                    left_pose.position.z = pose_data['position']['z']
                    left_pose.orientation.x = pose_data['orientation']['x']
                    left_pose.orientation.y = pose_data['orientation']['y']
                    left_pose.orientation.z = pose_data['orientation']['z']
                    left_pose.orientation.w = pose_data['orientation']['w']
                    
                    # 获取frame_id，如果没有则使用默认值
                    frame_id = pose_data.get('frame_id', 'arm_base')
                    
                    # 使用send_target_stamped发送到/left_target/stamped topic
                    self.interface.left_arm_handler.send_target_stamped(frame_id, left_pose)
                    print(f"    ✓ 左臂: 已发送目标pose (frame: {frame_id})")
                
                # 发送右臂目标pose（使用stamped版本）
                if has_right_pose and self.interface.right_arm_handler:
                    right_pose = Pose()
                    pose_data = node['right_end_effector_pose']
                    right_pose.position.x = pose_data['position']['x']
                    right_pose.position.y = pose_data['position']['y']
                    right_pose.position.z = pose_data['position']['z']
                    right_pose.orientation.x = pose_data['orientation']['x']
                    right_pose.orientation.y = pose_data['orientation']['y']
                    right_pose.orientation.z = pose_data['orientation']['z']
                    right_pose.orientation.w = pose_data['orientation']['w']
                    
                    # 获取frame_id，如果没有则使用默认值
                    frame_id = pose_data.get('frame_id', 'arm_base')
                    
                    # 使用send_target_stamped发送到/right_target/stamped topic
                    self.interface.right_arm_handler.send_target_stamped(frame_id, right_pose)
                    print(f"    ✓ 右臂: 已发送目标pose (frame: {frame_id})")
            
            # 发送左夹爪/灵巧手关节位置
            if 'left_gripper' in node and node['left_gripper'].get('positions'):
                positions = node['left_gripper']['positions']
                # 判断是单关节夹爪还是多关节灵巧手
                if len(positions) == 1:
                    # 单关节夹爪，使用 gripper_handler
                    if self.interface.left_gripper_handler:
                        self.interface.left_gripper_handler.send_joint_positions(positions[0])
                        print(f"    ✓ 左夹爪: 1 个关节")
                else:
                    # 多关节灵巧手，使用关节控制器
                    if self.interface.config.left_hand_joint_controller_topic:
                        self.interface.send_left_hand_joint_positions(positions)
                        print(f"    ✓ 左灵巧手: {len(positions)} 个关节")
                    elif self.interface.left_gripper_handler:
                        # 如果没有关节控制器，尝试使用 gripper_handler（可能会失败）
                        logger.warning("Left hand joint controller not available, trying gripper handler (may fail)")
                        try:
                            self.interface.left_gripper_handler.send_joint_positions(positions[0])
                            print(f"    ⚠ 左灵巧手: 仅发送第一个关节（{len(positions)} 个关节可用）")
                        except Exception as e:
                            print(f"    ✗ 左灵巧手发送失败: {e}")
            
            # 发送右夹爪/灵巧手关节位置（双臂模式）
            if self.is_dual_arm and 'right_gripper' in node and node['right_gripper'].get('positions'):
                positions = node['right_gripper']['positions']
                # 判断是单关节夹爪还是多关节灵巧手
                if len(positions) == 1:
                    # 单关节夹爪，使用 gripper_handler
                    if self.interface.right_gripper_handler:
                        self.interface.right_gripper_handler.send_joint_positions(positions[0])
                        print(f"    ✓ 右夹爪: 1 个关节")
                else:
                    # 多关节灵巧手，使用关节控制器
                    if self.interface.config.right_hand_joint_controller_topic:
                        self.interface.send_right_hand_joint_positions(positions)
                        print(f"    ✓ 右灵巧手: {len(positions)} 个关节")
                    elif self.interface.right_gripper_handler:
                        # 如果没有关节控制器，尝试使用 gripper_handler（可能会失败）
                        logger.warning("Right hand joint controller not available, trying gripper handler (may fail)")
                        try:
                            self.interface.right_gripper_handler.send_joint_positions(positions[0])
                            print(f"    ⚠ 右灵巧手: 仅发送第一个关节（{len(positions)} 个关节可用）")
                        except Exception as e:
                            print(f"    ✗ 右灵巧手发送失败: {e}")
            
            # 发送头部关节位置
            if 'head' in node and node['head'].get('positions'):
                positions = node['head']['positions']
                self.interface.send_head_joint_positions(positions)
                print(f"    ✓ 头部: {len(positions)} 个关节")
            
            # 发送腰部关节位置
            if 'body' in node and node['body'].get('positions'):
                positions = node['body']['positions']
                self.interface.send_body_joint_positions(positions)
                print(f"    ✓ 腰部: {len(positions)} 个关节")
            
            print(f"  ✓ 节点 #{self.current_index} 已发送 (OCS2模式)")
            return True
            
        except Exception as e:
            print(f"  ✗ 播放节点失败: {e}")
            import traceback
            traceback.print_exc()
            return False


def record_mode(interface: ROS2RobotInterface):
    """录制模式"""
    print("\n" + "=" * 70)
    print(" " * 25 + "关节录制模式")
    print("=" * 70)
    
    # 显示保存目录
    record_dir = get_record_directory()
    print(f"\n保存目录: {record_dir.absolute()}")
    
    print("\n操作说明:")
    print("  - 按 Enter 键：录制当前关节状态（一个节点）")
    print("  - 输入 's' 或 'save' 后按 Enter：保存录制的节点到JSON文件（可指定文件名）")
    print("  - 输入 'q' 或 'quit' 后按 Enter：退出（会提示是否保存）")
    print("  - 按 Ctrl+C：强制退出\n")
    
    recorder = JointRecorder(interface)
    
    # 等待数据到达
    print("等待关节数据到达...")
    time.sleep(2.0)
    
    # 检查是否能获取关节状态
    test_state = interface.get_joint_state(categorized=True)
    if not test_state:
        print("  ✗ 无法获取关节状态，请确保机器人正在运行并发布关节状态数据")
        interface.disconnect()
        return 1
    
    print("  ✓ 关节数据已就绪\n")
    
    try:
        while True:
            user_input = input("按 Enter 录制节点，输入 's' 保存，输入 'q' 退出: ").strip().lower()
            
            if user_input == '':
                # 按Enter录制节点
                recorder.record_node()
            elif user_input in ['s', 'save']:
                # 保存
                filename = input("请输入文件名（不含路径，直接按Enter使用时间戳命名）: ").strip()
                if not filename:
                    timestamp = time.strftime('%Y%m%d_%H%M%S')
                    filename = f"joint_record_{timestamp}"
                
                recorder.save_to_json(filename)
            elif user_input in ['q', 'quit']:
                # 退出
                if recorder.recorded_nodes:
                    save_choice = input(f"\n已录制 {len(recorder.recorded_nodes)} 个节点，是否保存？(y/n): ").strip().lower()
                    if save_choice in ['y', 'yes']:
                        filename = input("请输入文件名（不含路径，直接按Enter使用时间戳命名）: ").strip()
                        if not filename:
                            timestamp = time.strftime('%Y%m%d_%H%M%S')
                            filename = f"joint_record_{timestamp}"
                        recorder.save_to_json(filename)
                break
            else:
                print("  ⚠ 无效输入，请重试")
    
    except KeyboardInterrupt:
        print("\n\n  ⚠ 用户中断")
        if recorder.recorded_nodes:
            save_choice = input(f"\n已录制 {len(recorder.recorded_nodes)} 个节点，是否保存？(y/n): ").strip().lower()
            if save_choice in ['y', 'yes']:
                filename = input("请输入文件名（不含路径，直接按Enter使用时间戳命名）: ").strip()
                if not filename:
                    timestamp = time.strftime('%Y%m%d_%H%M%S')
                    filename = f"joint_record_{timestamp}"
                recorder.save_to_json(filename)
    
    print("\n录制模式结束")
    return 0


def playback_mode(interface: ROS2RobotInterface, input_file: Optional[str] = None):
    """回放模式"""
    print("\n" + "=" * 70)
    print(" " * 25 + "关节回放模式")
    print("=" * 70)
    
    player = JointPlayer(interface)
    
    # 如果没有指定文件，列出所有可用文件供选择
    if input_file is None:
        json_files = list_record_files()
        
        if not json_files:
            print(f"\n  ✗ 在 {get_record_directory().absolute()} 目录中未找到JSON文件")
            print("  请先使用录制模式创建一些记录文件")
            interface.disconnect()
            return 1
        
        print(f"\n找到 {len(json_files)} 个记录文件:")
        print("-" * 70)
        for i, file_path in enumerate(json_files, 1):
            # 尝试读取文件元数据以显示信息
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    metadata = data.get('metadata', {})
                    total_nodes = metadata.get('total_nodes', '未知')
                    recorded_at = metadata.get('recorded_at', '未知')
                    is_dual_arm = metadata.get('is_dual_arm', False)
                    mode_str = '双臂' if is_dual_arm else '单臂'
                    file_size = file_path.stat().st_size / 1024  # KB
                    print(f"  [{i}] {file_path.name}")
                    print(f"      节点数: {total_nodes} | 模式: {mode_str} | 录制时间: {recorded_at} | 大小: {file_size:.1f} KB")
            except Exception:
                file_size = file_path.stat().st_size / 1024  # KB
                print(f"  [{i}] {file_path.name} (大小: {file_size:.1f} KB)")
        
        print("-" * 70)
        
        while True:
            try:
                choice = input(f"\n请选择要回放的文件 (1-{len(json_files)})，或输入 'q' 退出: ").strip()
                
                if choice.lower() in ['q', 'quit']:
                    interface.disconnect()
                    return 0
                
                file_index = int(choice) - 1
                if 0 <= file_index < len(json_files):
                    input_file = str(json_files[file_index])
                    break
                else:
                    print(f"  ⚠ 无效选择，请输入 1-{len(json_files)} 之间的数字")
            except ValueError:
                print("  ⚠ 无效输入，请输入数字或 'q'")
    
    # 加载JSON文件
    print(f"\n加载JSON文件: {input_file}")
    if not player.load_from_json(input_file):
        interface.disconnect()
        return 1
    
    if not player.nodes:
        print("  ✗ JSON文件中没有节点数据")
        interface.disconnect()
        return 1
    
    # 选择回放模式
    print("\n请选择回放模式:")
    print("  [1] MOVEJ模式 - 使用关节角度控制（默认）")
    print("  [2] OCS2模式 - 使用末端pose控制（需要节点中包含pose数据）")
    
    while True:
        mode_choice = input("\n请选择模式 (1/2，默认1): ").strip()
        if not mode_choice:
            mode_choice = '1'
        if mode_choice in ['1', '2']:
            use_ocs2 = (mode_choice == '2')
            break
        else:
            print("  ⚠ 无效选择，请输入 1 或 2")
    
    if use_ocs2:
        print("\n  → 使用OCS2模式回放")
        print("\n操作说明:")
        print("  - 按 Enter 键：发送下一个节点的OCS2 pose指令")
        print("  - 输入 'q' 或 'quit' 后按 Enter：退出")
        print("  - 按 Ctrl+C：强制退出\n")
        
        # 切换到OCS2状态
        print("\n准备切换到OCS2状态...")
        try:
            # 先切换到HOLD状态
            print("  → 先切换到HOLD状态...")
            interface.send_fsm_command(2)  # HOLD状态
            time.sleep(0.3)  # 等待状态切换完成
            print("  ✓ 已切换到HOLD状态")
            
            # 再切换到OCS2状态
            print("  → 再切换到OCS2状态...")
            interface.send_fsm_command(3)  # OCS2状态
            time.sleep(0.3)  # 等待状态切换完成
            print("  ✓ 已切换到OCS2状态\n")
        except Exception as e:
            print(f"  ⚠ 切换状态失败: {e}\n")
        
        play_func = player.play_next_node_ocs2
    else:
        print("\n  → 使用MOVEJ模式回放")
        print("\n操作说明:")
        print("  - 按 Enter 键：发送下一个节点的movej指令")
        print("  - 输入 'q' 或 'quit' 后按 Enter：退出")
        print("  - 按 Ctrl+C：强制退出\n")
        
        # 先切换到HOLD状态，然后再切换到MOVEJ状态
        print("\n准备切换到MOVEJ状态...")
        try:
            # 先切换到HOLD状态
            print("  → 先切换到HOLD状态...")
            interface.send_fsm_command(2)  # HOLD状态
            time.sleep(0.3)  # 等待状态切换完成
            print("  ✓ 已切换到HOLD状态")
            
            # 再切换到MOVEJ状态
            print("  → 再切换到MOVEJ状态...")
            interface.send_fsm_command(4)  # MOVEJ状态
            time.sleep(0.3)  # 等待状态切换完成
            print("  ✓ 已切换到MOVEJ状态\n")
        except Exception as e:
            print(f"  ⚠ 切换状态失败: {e}")
            print("  将继续尝试发送关节位置（send_joint_positions会自动切换状态）\n")
        
        play_func = player.play_next_node
    
    try:
        while True:
            user_input = input("按 Enter 发送下一个节点，输入 'q' 退出: ").strip().lower()
            
            if user_input == '':
                # 按Enter播放下一个节点
                if not play_func():
                    print("\n  ✓ 所有节点已播放完成")
                    break
            elif user_input in ['q', 'quit']:
                break
            else:
                print("  ⚠ 无效输入，请重试")
    
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
    
    print("\n回放模式结束")
    return 0


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='关节录制和回放脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 录制模式（文件自动保存到 joint_records/ 目录）
  python joint_record_playback.py record
  
  # 回放模式（列出所有文件供选择）
  python joint_record_playback.py playback
  
  # 回放模式（直接指定文件）
  python joint_record_playback.py playback --file joint_records/my_joints.json
        """
    )
    
    parser.add_argument(
        'mode',
        choices=['record', 'playback'],
        help='运行模式：record（录制）或 playback（回放）'
    )
    
    parser.add_argument(
        '--file', '-f',
        type=str,
        help='JSON文件路径（可选，回放模式：如果指定则直接使用该文件，否则列出所有文件供选择）'
    )
    
    args = parser.parse_args()
    
    # ========================================================================
    # 初始化和连接
    # ========================================================================
    print("\n" + "=" * 70)
    print(" " * 20 + "关节录制和回放脚本")
    print("=" * 70)
    
    print("\n[1] 创建配置...")
    config = ROS2RobotInterfaceConfig()
    
    print("[2] 创建ROS2RobotInterface实例...")
    interface = ROS2RobotInterface(config)
    
    print("[3] 连接到ROS 2...")
    try:
        interface.connect()
        print("    ✓ 接口连接成功!\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}\n")
        return 1
    
    # 检查模式
    is_dual_arm = interface.config.right_end_effector_pose_topic is not None
    print(f"    ✓ 检测到{'双臂' if is_dual_arm else '单臂'}模式\n")
    
    # ========================================================================
    # 根据模式执行相应功能
    # ========================================================================
    try:
        if args.mode == 'record':
            result = record_mode(interface)
        else:  # playback
            result = playback_mode(interface, args.file)
    finally:
        # 断开连接
        print("\n[4] 断开连接...")
        interface.disconnect()
        print("    ✓ 已断开连接\n")
    
    print("=" * 70)
    print("程序结束")
    print("=" * 70 + "\n")
    
    return result


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
