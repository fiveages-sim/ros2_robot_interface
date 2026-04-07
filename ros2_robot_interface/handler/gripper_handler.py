"""
Gripper Handler - 单夹爪处理器

封装左夹爪或右夹爪的命令发布、状态管理等共同逻辑。
类似于 arms_target_manager 中的 ArmMarker 模式。
"""

import logging
from enum import Enum
from typing import Optional, Dict, Any, TYPE_CHECKING

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Float64, Int32

from ..utils.exceptions import ROS2NotConnectedError

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from ..config import ROS2RobotInterfaceConfig




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
        config: "ROS2RobotInterfaceConfig"
    ):
        """
        初始化单夹爪处理器
        
        Args:
            node: ROS 2 节点
            gripper_type: 夹爪类型（LEFT 或 RIGHT）
            config: ROS2RobotInterfaceConfig 配置对象
        """
        self.node = node
        self.gripper_type = gripper_type
        self.config = config
        
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
        self.is_open: bool = False  # 开关状态：True=打开, False=关闭（用于target_command）
        
        # 根据夹爪类型确定 target_percent 话题名称
        if gripper_type == GripperType.LEFT:
            self.target_percent_topic = config.left_gripper_target_percent_topic
        else:  # RIGHT
            self.target_percent_topic = config.right_gripper_target_percent_topic

        # Publisher（位置控制 - 保留原有功能）
        self.command_pub: Optional[Publisher] = None

        # Publisher（百分比控制）
        self.target_percent_pub: Optional[Publisher] = None

        # Publisher 和 Subscription（开关控制 - 新功能）
        self.target_command_pub: Optional[Publisher] = None
        self.target_command_sub: Optional[Subscription] = None
    
    def initialize(self) -> None:
        """初始化发布器和订阅器"""
        # 创建位置控制发布器（保留原有功能）
        if self.command_topic:
            self.command_pub = self.node.create_publisher(
                Float64, self.command_topic, 10
            )
            logger.debug(f"{self.label}: Created position command publisher for {self.command_topic}")
        
        # 注意：只有在 controller_name 存在时才会创建 GripperHandler，所以这里一定不为 None
        if self.gripper_type == GripperType.LEFT:
            controller_name = self.config.left_gripper_controller_name
            target_command_topic = f"/{controller_name}/target_command"
        else:  # RIGHT
            controller_name = self.config.right_gripper_controller_name
            target_command_topic = f"/{controller_name}/target_command"
        
        self.target_command_pub = self.node.create_publisher(
            Int32, target_command_topic, 10
        )
        logger.debug(f"{self.label}: Created target_command publisher for {target_command_topic}")
        
        # 创建订阅器用于状态同步（像VR和joystick一样）
        self.target_command_sub = self.node.create_subscription(
            Int32, target_command_topic,
            self._target_command_callback, 10
        )
        logger.debug(f"{self.label}: Created target_command subscription for state sync")

        # 创建百分比控制发布器（如果话题存在）
        if self.target_percent_topic:
            self.target_percent_pub = self.node.create_publisher(
                Float64, self.target_percent_topic, 10
            )
            logger.debug(f"{self.label}: Created target_percent publisher for {self.target_percent_topic}")
    
    def send_position_percent(self, percent: float) -> None:
        """发送夹爪百分比控制命令

        Args:
            percent: 目标百分比，范围 0.0（完全关闭）~ 1.0（完全打开）

        Raises:
            ValueError: 如果 percent 不在 [0.0, 1.0] 范围内
            ROS2NotConnectedError: 如果发布器未初始化（话题未检测到）
        """
        percent = float(percent)
        if not (0.0 <= percent <= 1.0):
            raise ValueError(f"{self.label}: Invalid percent {percent}, must be in [0.0, 1.0]")

        if self.target_percent_pub is None:
            raise ROS2NotConnectedError(
                f"{self.label} target_percent publisher not initialized "
                f"(topic not detected: {self.target_percent_topic})"
            )

        # 换算为实际行程位置并更新目标，清空历史记录（与 send_joint_positions 一致）
        actual_position = (
            self.config.gripper_min_position
            + percent * (self.config.gripper_max_position - self.config.gripper_min_position)
        )
        self.target_position = actual_position
        self.position_history.clear()

        msg = Float64()
        msg.data = percent
        self.target_percent_pub.publish(msg)
        logger.debug(f"Published {self.label.lower()} target_percent: {percent} (actual position: {actual_position:.4f})")

    def send_joint_positions(self, position: float) -> None:
        """发送夹爪关节位置命令
        
        为了与手臂的 API 保持一致，使用相同的命名 `send_joint_positions()`。
        夹爪通常只有一个关节，所以直接传入位置值即可。
        
        Args:
            position: 目标关节位置（夹爪只有一个关节）
            
        Raises:
            ROS2NotConnectedError: 如果发布器未初始化
        """
        if self.command_pub is None:
            raise ROS2NotConnectedError(f"{self.label} command publisher not initialized (command_topic not configured)")
        
        # 位置限制
        clamped_position = max(
            self.config.gripper_min_position,
            min(position, self.config.gripper_max_position)
        )
        
        # 更新目标位置并清空历史记录
        self.target_position = clamped_position
        self.position_history.clear()
        
        # 发布消息（位置控制方式）
        gripper_msg = Float64()
        gripper_msg.data = clamped_position
        self.command_pub.publish(gripper_msg)
        logger.debug(f"Published {self.label.lower()} joint positions: {clamped_position}")
    
    def send_target_command(self, target_value: int) -> None:
        """发送夹爪开关控制命令（新方式）
        
        使用 target_command 话题进行开关控制，与 VR、RViz、Joystick 保持一致。
        
        Args:
            target_value: 目标值，0=关闭, 1=打开
            
        Raises:
            ValueError: 如果 target_value 不是 0 或 1
        """
        if target_value not in [0, 1]:
            raise ValueError(f"{self.label}: Invalid target_value {target_value}, must be 0 or 1")
        
        # 创建 Int32 消息
        target_msg = Int32()
        target_msg.data = target_value
        
        # 发布到 target_command 话题
        # 注意：target_command_pub 在 initialize() 中总是会被创建（因为 controller_name 一定存在）
        self.target_command_pub.publish(target_msg)
        
        # 不立即更新状态，等待订阅器回调来更新（与VR/joystick逻辑一致）
        action = "打开" if target_value == 1 else "关闭"
        logger.debug(f"Published {self.label.lower()} target_command: {action} (value={target_value})")
    
    def _target_command_callback(self, msg: Int32) -> None:
        """target_command 话题回调 - 同步夹爪状态
        
        接收其他节点发送的 target_command 消息，更新本地状态。
        与 VR 和 joystick 的逻辑保持一致。
        
        Args:
            msg: Int32 消息，data=1 表示打开，data=0 表示关闭
        """
        self.is_open = (msg.data == 1)
        # 同时更新 target_position 用于兼容性
        if msg.data == 1:
            self.target_position = self.config.gripper_max_position
        else:
            self.target_position = self.config.gripper_min_position
        
        action = "打开" if self.is_open else "关闭"
        logger.debug(f"{self.label}: 状态同步更新 (target_command): {action}")
    
    def update_position_history(self, position: float) -> None:
        """更新位置历史记录
        
        从 joint_state 回调中调用，用于维护位置历史记录。
        
        Args:
            position: 当前位置
        """
        self.position_history.append(position)
        if len(self.position_history) > self.config.gripper_stability_history_size:
            self.position_history.pop(0)
    
    def check_arrival(
        self,
        current_position: Optional[float],
        threshold: float | None = None
    ) -> Dict[str, Any]:
        """检查夹爪是否到达目标位置
        
        Args:
            current_position: 当前位置（从 get_joint_state() 获取）
            threshold: 位置距离阈值，如果为 None 则使用 config.gripper_position_threshold
            
        Returns:
            包含到达状态、距离等信息的字典
        """
        # 使用 config 中的默认值
        threshold = threshold if threshold is not None else self.config.gripper_position_threshold
        
        arrived = False
        distance = float('inf')
        
        if current_position is not None and self.target_position is not None:
            distance = abs(current_position - self.target_position)
            
            # 计算稳定性
            is_stable = False
            position_variance = float('inf')
            history_size = self.config.gripper_stability_history_size
            if len(self.position_history) == history_size:
                recent_positions = self.position_history[-history_size:]
                position_variance = max(recent_positions) - min(recent_positions)
                is_stable = position_variance < self.config.gripper_stability_threshold
            
            # 判断是否在关闭过程中
            is_closing = current_position > self.target_position
            
            # 到达判断：关闭时考虑稳定性，打开时只考虑距离
            if is_closing:
                arrived = (distance < threshold) or is_stable
            else:
                arrived = distance < threshold
        
        return {'arrived': arrived, 'distance': distance}
    
    def get_target_position(self) -> Optional[float]:
        """获取当前的目标位置
        
        Returns:
            目标位置，如果未设置则返回 None
        """
        return self.target_position
    
    def cleanup(self) -> None:
        """清理资源"""
        if self.command_pub:
            self.command_pub.destroy()
            self.command_pub = None
        if self.target_percent_pub:
            self.target_percent_pub.destroy()
            self.target_percent_pub = None
        if self.target_command_pub:
            self.target_command_pub.destroy()
            self.target_command_pub = None
        if self.target_command_sub:
            self.target_command_sub.destroy()
            self.target_command_sub = None

