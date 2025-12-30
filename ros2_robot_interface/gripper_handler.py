"""
Gripper Handler - 单夹爪处理器

封装左夹爪或右夹爪的命令发布、状态管理等共同逻辑。
类似于 arms_target_manager 中的 ArmMarker 模式。
"""

import logging
import threading
from enum import Enum
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float64

from .exceptions import ROS2NotConnectedError

logger = logging.getLogger(__name__)

# 默认阈值常量
DEFAULT_GRIPPER_POSITION_THRESHOLD: float = 0.01  # 位置距离阈值
DEFAULT_GRIPPER_STABILITY_THRESHOLD: float = 0.0001  # 稳定性阈值
DEFAULT_GRIPPER_STABILITY_HISTORY_SIZE: int = 15  # 历史记录大小


class GripperType(Enum):
    """夹爪类型枚举"""
    LEFT = "left"
    RIGHT = "right"


class GripperHandler:
    """单夹爪处理器 - 封装左夹爪或右夹爪的所有共同逻辑
    
    负责管理单夹爪的：
    - 命令发布
    - 目标位置管理
    - 位置历史记录
    - 到达检查
    """
    
    def __init__(
        self,
        node: Node,
        gripper_type: GripperType,
        config: 'ROS2RobotInterfaceConfig',
        data_lock: threading.Lock
    ):
        """
        初始化单夹爪处理器
        
        Args:
            node: ROS 2 节点
            gripper_type: 夹爪类型（LEFT 或 RIGHT）
            config: ROS2RobotInterfaceConfig 配置对象
            data_lock: 数据锁（用于线程安全）
        """
        self.node = node
        self.gripper_type = gripper_type
        self.config = config
        self.data_lock = data_lock
        
        # 根据夹爪类型确定 topic 名称和标签
        if gripper_type == GripperType.LEFT:
            self.command_topic = config.gripper_command_topic
            # 判断是否为单臂模式
            is_dual_arm = config.right_end_effector_pose_topic is not None
            self.label = "LEFT_GRIPPER" if is_dual_arm else "GRIPPER"
        else:  # RIGHT
            self.command_topic = config.right_gripper_command_topic
            self.label = "RIGHT_GRIPPER"
        
        # 状态变量
        self.target_position: Optional[float] = None
        self.position_history: list[float] = []
        
        # Publisher
        self.command_pub: Optional[Publisher] = None
    
    def initialize(self) -> None:
        """初始化发布器"""
        # 创建命令发布器
        if self.command_topic:
            self.command_pub = self.node.create_publisher(
                Float64, self.command_topic, 10
            )
            logger.debug(f"{self.label}: Created command publisher for {self.command_topic}")
        else:
            logger.debug(f"{self.label}: Command topic not configured, skipping publisher")
    
    def send_joint_positions(self, position: float) -> None:
        """发送夹爪关节位置命令
        
        为了与手臂的 API 保持一致，使用相同的命名 `send_joint_positions()`。
        夹爪通常只有一个关节，所以直接传入位置值即可。
        
        Args:
            position: 目标关节位置（夹爪只有一个关节）
            
        Raises:
            ROS2NotConnectedError: 如果发布器未初始化或夹爪未启用
        """
        if not self.config.gripper_enabled:
            logger.warning(f"{self.label} is not enabled in configuration")
            return
        
        if self.command_pub is None:
            logger.warning(f"{self.label} command publisher not initialized")
            return
        
        # 位置限制
        clamped_position = max(
            self.config.gripper_min_position,
            min(position, self.config.gripper_max_position)
        )
        
        # 更新目标位置并清空历史记录
        with self.data_lock:
            self.target_position = clamped_position
            self.position_history.clear()
        
        # 发布消息
        gripper_msg = Float64()
        gripper_msg.data = clamped_position
        self.command_pub.publish(gripper_msg)
        logger.debug(f"Published {self.label.lower()} joint positions: {clamped_position}")
    
    def update_position_history(self, position: float) -> None:
        """更新位置历史记录
        
        从 joint_state 回调中调用，用于维护位置历史记录。
        注意：调用者必须已经持有 data_lock，此方法不会再次获取锁。
        
        Args:
            position: 当前位置
        """
        # 注意：调用者已经持有 data_lock，这里不再获取锁以避免死锁
        self.position_history.append(position)
        if len(self.position_history) > DEFAULT_GRIPPER_STABILITY_HISTORY_SIZE:
            self.position_history.pop(0)
    
    def check_arrival(
        self,
        current_position: Optional[float],
        threshold: float = DEFAULT_GRIPPER_POSITION_THRESHOLD
    ) -> Dict[str, Any]:
        """检查夹爪是否到达目标位置
        
        Args:
            current_position: 当前位置（从 get_joint_state() 获取）
            threshold: 位置距离阈值，默认 DEFAULT_GRIPPER_POSITION_THRESHOLD
            
        Returns:
            包含到达状态、距离等信息的字典
        """
        arrived = False
        distance = float('inf')
        
        if current_position is not None and self.target_position is not None:
            distance = abs(current_position - self.target_position)
            
            # 计算稳定性
            is_stable = False
            position_variance = float('inf')
            with self.data_lock:
                if len(self.position_history) == DEFAULT_GRIPPER_STABILITY_HISTORY_SIZE:
                    recent_positions = self.position_history[-DEFAULT_GRIPPER_STABILITY_HISTORY_SIZE:]
                    position_variance = max(recent_positions) - min(recent_positions)
                    is_stable = position_variance < DEFAULT_GRIPPER_STABILITY_THRESHOLD
            
            # 判断是否在关闭过程中
            is_closing = current_position > self.target_position
            
            # 到达判断：关闭时考虑稳定性，打开时只考虑距离
            if is_closing:
                arrived = (distance < threshold) or is_stable
            else:
                arrived = distance < threshold
            
            # 打印检查信息
            print(f"  [位置检查-{self.label}] 当前位置: {current_position:.4f}")
            print(f"  [位置检查-{self.label}] 目标位置: {self.target_position:.4f}")
            print(f"  [位置检查-{self.label}] 距离: {distance:.4f} (阈值: {threshold:.4f})")
            
            with self.data_lock:
                if len(self.position_history) > 0:
                    history_str = ", ".join([f"{p:.4f}" for p in self.position_history])
                    print(f"  [位置检查-{self.label}] 位置历史 ({len(self.position_history)}个值): [{history_str}]")
                if len(self.position_history) == DEFAULT_GRIPPER_STABILITY_HISTORY_SIZE:
                    print(f"  [位置检查-{self.label}] 位置稳定性: {is_stable} (变化: {position_variance:.4f}, 阈值: {DEFAULT_GRIPPER_STABILITY_THRESHOLD:.4f})")
            
            if arrived:
                if is_stable and is_closing and distance >= threshold:
                    print(f"  [位置检查-{self.label}] ✓ 已到达关闭状态（位置稳定，可能已夹住物体）")
                else:
                    print(f"  [位置检查-{self.label}] ✓ 已到达目标位置")
            else:
                print(f"  [位置检查-{self.label}] ✗ 未到达目标位置")
            print()
        
        return {'arrived': arrived, 'distance': distance}
    
    def get_target_position(self) -> Optional[float]:
        """获取当前的目标位置
        
        Returns:
            目标位置，如果未设置则返回 None
        """
        with self.data_lock:
            return self.target_position
    
    def cleanup(self) -> None:
        """清理资源"""
        if self.command_pub:
            self.command_pub.destroy()
            self.command_pub = None

