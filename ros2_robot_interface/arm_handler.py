"""
Arm Handler - 单臂处理器

封装左臂或右臂的 pose 订阅、target 发布等共同逻辑。
类似于 arms_target_manager 中的 ArmMarker 模式。
"""

import logging
from enum import Enum
from typing import Optional, Dict, Any, List, Callable

from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Float64MultiArray

from .exceptions import ROS2NotConnectedError

logger = logging.getLogger(__name__)


class ArmType(Enum):
    """手臂类型枚举"""
    LEFT = "left"
    RIGHT = "right"


class ArmHandler:
    """单臂处理器 - 封装左臂或右臂的所有共同逻辑
    
    负责管理单臂的：
    - Pose 订阅和状态管理
    - Target pose 发布（普通和 stamped）
    """
    
    def __init__(
        self,
        node: Node,
        arm_type: ArmType,
        config: 'ROS2RobotInterfaceConfig',
        fsm_command_callback: Optional[Callable[[int], None]] = None
    ):
        """
        初始化单臂处理器
        
        Args:
            node: ROS 2 节点
            arm_type: 手臂类型（LEFT 或 RIGHT）
            config: ROS2RobotInterfaceConfig 配置对象
            fsm_command_callback: 可选的 FSM 命令回调函数，用于切换状态（传入命令值）
        """
        self.node = node
        self.arm_type = arm_type
        self.config = config
        self.fsm_command_callback = fsm_command_callback
        
        # 根据手臂类型确定 topic 名称和标签
        if arm_type == ArmType.LEFT:
            self.pose_topic = config.end_effector_pose_topic
            self.target_topic = config.end_effector_target_topic
            self.current_target_topic = config.end_effector_current_target_topic
            # 判断是否为单臂模式
            is_dual_arm = config.right_end_effector_pose_topic is not None
            self.label = "LEFT_ARM" if is_dual_arm else "ARM"
        else:  # RIGHT
            self.pose_topic = config.right_end_effector_pose_topic
            self.target_topic = config.right_end_effector_target_topic
            self.current_target_topic = config.right_end_effector_current_target_topic
            self.label = "RIGHT_ARM"
        
        # 状态变量
        self.latest_pose: Optional[Pose] = None
        self.latest_target_pose: Optional[Pose] = None  # 从话题订阅获取的目标位置
        self.frame_id: Optional[str] = None  # 从 pose 订阅获取的 frame_id（只在第一次收到时更新）
        
        # 状态标志
        self._had_pose = False
        self._had_target = False
        
        # Publishers 和 Subscriptions
        self.pose_sub: Optional[Subscription] = None
        self.target_sub: Optional[Subscription] = None  # 目标位置订阅器
        self.target_pub: Optional[Publisher] = None
        self.target_stamped_pub: Optional[Publisher] = None
        self.joint_controller_pub: Optional[Publisher] = None
    
    def initialize(self) -> None:
        """初始化订阅器和发布器"""
        # 创建 pose 订阅器
        if self.pose_topic:
            self.pose_sub = self.node.create_subscription(
                PoseStamped,
                self.pose_topic,
                self._pose_callback,
                10
            )
            logger.debug(f"{self.label}: Created pose subscription to {self.pose_topic}")
        else:
            logger.warning(f"{self.label}: Pose topic not configured, skipping subscription")
        
        # 创建目标位置订阅器（用于到达判断）
        if self.current_target_topic:
            self.target_sub = self.node.create_subscription(
                PoseStamped,
                self.current_target_topic,
                self._target_callback,
                10
            )
            logger.debug(f"{self.label}: Created current target subscription to {self.current_target_topic}")
        else:
            logger.debug(f"{self.label}: Current target topic not configured, arrival checking will return None")
        
        # 创建 target 发布器
        if self.target_topic:
            self.target_pub = self.node.create_publisher(
                Pose, self.target_topic, 10
            )
            self.target_stamped_pub = self.node.create_publisher(
                PoseStamped, f"{self.target_topic}/stamped", 10
            )
            logger.debug(f"{self.label}: Created target publishers for {self.target_topic}")
        else:
            logger.warning(f"{self.label}: Target topic not configured, skipping publishers")
        
        # 创建关节控制器发布器
        if self.arm_type == ArmType.LEFT:
            joint_topic = self.config.left_arm_joint_controller_topic
        else:  # RIGHT
            joint_topic = self.config.right_arm_joint_controller_topic
        
        if joint_topic:
            self.joint_controller_pub = self.node.create_publisher(
                Float64MultiArray, joint_topic, 10
            )
            logger.debug(f"{self.label}: Created joint controller publisher for {joint_topic}")
        else:
            logger.debug(f"{self.label}: Joint controller topic not configured, skipping publisher")
    
    def _pose_callback(self, msg: PoseStamped) -> None:
        """Pose 回调函数 - 处理接收到的 pose 消息"""
        # 更新状态
        self.latest_pose = msg.pose
        if self.frame_id is None:
            self.frame_id = msg.header.frame_id
        self._had_pose = True
    
    def _target_callback(self, msg: PoseStamped) -> None:
        """目标位置回调函数 - 处理接收到的目标位置消息"""
        # 更新状态
        self.latest_target_pose = msg.pose
        self._had_target = True
    
    def get_pose(self) -> Optional[Pose]:
        return self.latest_pose
    
    def send_target(self, pose: Pose) -> None:
        """发送目标 pose（不带坐标系）
        
        Args:
            pose: 目标 pose
            
        Raises:
            ROS2NotConnectedError: 如果发布器未初始化
        """
        if self.target_pub is None:
            raise ROS2NotConnectedError(f"{self.label} target publisher not initialized")
        
        # 清除旧的 current target，避免在收到新目标前误判为已到达
        self.latest_target_pose = None
        
        self.target_pub.publish(pose)
        logger.debug(f"Published {self.label.lower()} target: {pose}")
    
    def send_target_stamped(self, frame_id: str, pose: Pose) -> None:
        """发送带坐标系的目标 pose
        
        Args:
            frame_id: 坐标系 ID
            pose: 目标 pose
            
        Raises:
            ROS2NotConnectedError: 如果发布器未初始化
        """
        if self.target_stamped_pub is None:
            raise ROS2NotConnectedError(f"{self.label} target stamped publisher not initialized")
        
        # 清除旧的 current target，避免在收到新目标前误判为已到达
        self.latest_target_pose = None
        
        # 创建 PoseStamped 消息
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        # 发布
        self.target_stamped_pub.publish(pose_stamped)
        logger.debug(f"Published {self.label.lower()} target (stamped) in frame '{frame_id}': {pose}")
    
    def get_target_pose(self) -> Optional[Pose]:
        if not self.current_target_topic:
            return None
        
        # 直接返回 latest_target_pose，在 Python 中单个对象引用读取是原子的
        return self.latest_target_pose
    
    def get_frame_id(self) -> Optional[str]:
        # 如果未配置话题订阅，返回 None
        if not self.current_target_topic:
            return None
        
        # 直接返回 frame_id，在 Python 中单个对象引用读取是原子的
        return self.frame_id
    
    def send_joint_positions(self, positions: List[float]) -> None:
        """发送关节位置命令（MoveJ 模式）
        
        Args:
            positions: 目标关节位置列表
            
        Raises:
            ROS2NotConnectedError: 如果发布器未初始化
            ValueError: 如果初始化时未提供 FSM 命令回调函数
        """
        if self.joint_controller_pub is None:
            raise ROS2NotConnectedError(
                f"{self.label} joint controller publisher not initialized. "
                f"Joint controller topic not found."
            )
        
        # 如果没有 FSM 回调，抛出异常
        if self.fsm_command_callback is None:
            raise ValueError(
                f"{self.label}: FSM command callback is required for send_joint_positions(). "
                f"Please initialize ArmHandler with fsm_command_callback."
            )
        
        # 发送 FSM 命令切换到 MOVEJ 状态
        try:
            self.fsm_command_callback(4)
            logger.debug(f"{self.label}: Automatically switched to MOVEJ state for arm joint control")
            # 注意：状态切换是异步的，如果需要确保状态切换完成后再发送关节位置，
            # 可以在调用此函数后自行等待，或确保当前已在 MOVEJ 状态
        except Exception as e:
            logger.warning(f"{self.label}: Failed to switch to MOVEJ state: {e}")
        
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_controller_pub.publish(msg)
        logger.info(f"Published {self.label.lower()} joint positions: {positions}")
    
    def check_arrival(self, pose_threshold: float | None = None, 
                     orient_threshold: float | None = None) -> Dict[str, Any]:
        """检查手臂是否到达目标位置
        
        Args:
            pose_threshold: 位置距离阈值（米），如果为 None 则使用 config.pose_position_threshold
            orient_threshold: 姿态距离阈值，如果为 None 则使用 config.pose_orientation_threshold
            
        Returns:
            包含到达状态、距离等信息的字典
        """
        # 使用 config 中的默认值
        pose_threshold = pose_threshold if pose_threshold is not None else self.config.pose_position_threshold
        orient_threshold = orient_threshold if orient_threshold is not None else self.config.pose_orientation_threshold
        
        arrived = False
        pos_dist = float('inf')
        orient_dist = float('inf')
        total_dist = float('inf')
        status_msg = None
        
        current_pose = self.get_pose()
        target_pose = self.get_target_pose()
        
        if target_pose is not None and current_pose is not None:
            pos_dist = ((current_pose.position.x - target_pose.position.x) ** 2 +
                       (current_pose.position.y - target_pose.position.y) ** 2 +
                       (current_pose.position.z - target_pose.position.z) ** 2) ** 0.5
            
            dot_product = (current_pose.orientation.w * target_pose.orientation.w +
                          current_pose.orientation.x * target_pose.orientation.x +
                          current_pose.orientation.y * target_pose.orientation.y +
                          current_pose.orientation.z * target_pose.orientation.z)
            dot_product = max(-1.0, min(1.0, dot_product))
            orient_dist = 1.0 - abs(dot_product)
            
            total_dist = pos_dist + orient_dist * 0.1
            arrived = (pos_dist < pose_threshold and orient_dist < orient_threshold)
            
            status_msg = f"{self.label}已到达目标位置" if arrived else f"{self.label}未到达目标位置"
            print(f"  [位置检查-{self.label}] 当前位置: ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
            print(f"  [位置检查-{self.label}] 目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
            print(f"  [位置检查-{self.label}] 位置距离: {pos_dist:.4f} 米 (阈值: {pose_threshold:.4f})")
            print(f"  [位置检查-{self.label}] 姿态距离: {orient_dist:.4f} (阈值: {orient_threshold:.4f})")
            print(f"  [位置检查-{self.label}] {'✓ 已到达目标位置' if arrived else '✗ 未到达目标位置'}")
            if arrived:
                print(f"  [OCS2] → {status_msg}，等待中...")
            print()
        
        return {
            'arrived': arrived,
            'distance': total_dist,
            'position_distance': pos_dist,
            'orientation_distance': orient_dist,
            'status_message': status_msg
        }
    
    def _copy_pose(self, src: Pose, dst: Pose) -> None:
        """复制 pose 数据从 src 到 dst"""
        dst.position.x = src.position.x
        dst.position.y = src.position.y
        dst.position.z = src.position.z
        dst.orientation.x = src.orientation.x
        dst.orientation.y = src.orientation.y
        dst.orientation.z = src.orientation.z
        dst.orientation.w = src.orientation.w
    
    
    def cleanup(self) -> None:
        """清理资源"""
        if self.pose_sub:
            self.pose_sub.destroy()
            self.pose_sub = None
        
        if self.target_sub:
            self.target_sub.destroy()
            self.target_sub = None
        
        if self.target_pub:
            self.target_pub.destroy()
            self.target_pub = None
        
        if self.target_stamped_pub:
            self.target_stamped_pub.destroy()
            self.target_stamped_pub = None
        
        if self.joint_controller_pub:
            self.joint_controller_pub.destroy()
            self.joint_controller_pub = None

