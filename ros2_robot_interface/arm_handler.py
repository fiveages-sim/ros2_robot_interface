"""
Arm Handler - 单臂处理器

封装左臂或右臂的 pose 订阅、target 发布等共同逻辑。
类似于 arms_target_manager 中的 ArmMarker 模式。
"""

import logging
import threading
import time
from enum import Enum
from typing import Optional, Dict, Any, List, Callable

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Float64MultiArray
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

from .exceptions import ROS2NotConnectedError

logger = logging.getLogger(__name__)

# 默认阈值常量
DEFAULT_POSE_POSITION_THRESHOLD: float = 0.06  # 位置距离阈值（米）
DEFAULT_POSE_ORIENTATION_THRESHOLD: float = 0.1  # 姿态距离阈值


class ArmType(Enum):
    """手臂类型枚举"""
    LEFT = "left"
    RIGHT = "right"


class ArmHandler:
    """单臂处理器 - 封装左臂或右臂的所有共同逻辑
    
    负责管理单臂的：
    - Pose 订阅和状态管理
    - Target pose 发布（普通和 stamped）
    - TF 坐标转换
    - 时间戳和超时检查
    """
    
    def __init__(
        self,
        node: Node,
        arm_type: ArmType,
        config: 'ROS2RobotInterfaceConfig',
        data_lock: threading.Lock,
        tf_buffer: Optional[tf2_ros.Buffer] = None,
        base_frame: str = "arm_base",
        fsm_command_callback: Optional[Callable[[int], None]] = None
    ):
        """
        初始化单臂处理器
        
        Args:
            node: ROS 2 节点
            arm_type: 手臂类型（LEFT 或 RIGHT）
            config: ROS2RobotInterfaceConfig 配置对象
            data_lock: 数据锁（用于线程安全）
            tf_buffer: TF 缓冲区（可选，用于坐标转换）
            base_frame: 基坐标系名称
            fsm_command_callback: 可选的 FSM 命令回调函数，用于切换状态（传入命令值）
        """
        self.node = node
        self.arm_type = arm_type
        self.config = config
        self.data_lock = data_lock
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.fsm_command_callback = fsm_command_callback
        
        # 根据手臂类型确定 topic 名称和标签
        if arm_type == ArmType.LEFT:
            self.pose_topic = config.end_effector_pose_topic
            self.target_topic = config.end_effector_target_topic
            # 判断是否为单臂模式
            is_dual_arm = config.right_end_effector_pose_topic is not None
            self.label = "LEFT_ARM" if is_dual_arm else "ARM"
        else:  # RIGHT
            self.pose_topic = config.right_end_effector_pose_topic
            self.target_topic = config.right_end_effector_target_topic
            self.label = "RIGHT_ARM"
        
        # 状态变量
        self.latest_pose: Optional[Pose] = None
        self.target_pose: Optional[Pose] = None
        
        # 时间戳和状态标志
        self.last_pose_time = 0.0
        self._had_pose = False
        
        # Publishers 和 Subscriptions
        self.pose_sub: Optional[Subscription] = None
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
        with self.data_lock:
            current_time = time.time()
            was_recovering = False
            
            # 检查是否是数据恢复（首次接收或超时后恢复）
            if not self._had_pose:
                was_recovering = True
            elif self.latest_pose is None:
                was_recovering = True
            elif self.config.end_effector_pose_timeout > 0:
                time_since_last = current_time - self.last_pose_time
                if time_since_last > self.config.end_effector_pose_timeout:
                    was_recovering = True
            
            # 更新状态
            self.latest_pose = msg.pose
            self.last_pose_time = current_time
            self._had_pose = True
            
            # 记录恢复信息
            if was_recovering:
                logger.info(f"{self.label} pose data recovery: Started receiving pose messages again")
    
    def get_pose(self) -> Optional[Pose]:
        """获取最新的 end effector pose
        
        Returns:
            最新的 pose，如果数据过期或不存在则返回 None
        """
        with self.data_lock:
            # 检查超时
            if self.config.end_effector_pose_timeout > 0:
                if (time.time() - self.last_pose_time) > self.config.end_effector_pose_timeout:
                    logger.warning(f"{self.label} pose data is stale")
                    return None
            
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
        
        self.target_pub.publish(pose)
        
        # 更新内部 target pose 状态
        self.target_pose = Pose()
        self._copy_pose(pose, self.target_pose)
        
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
        
        # 创建 PoseStamped 消息
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        # 发布
        self.target_stamped_pub.publish(pose_stamped)
        logger.debug(f"Published {self.label.lower()} target (stamped) in frame '{frame_id}': {pose}")
        
        # 使用 TF 转换并更新内部 target pose
        self._update_target_pose_with_tf(pose, frame_id)
    
    def get_target_pose(self) -> Optional[Pose]:
        """获取当前的目标 pose
        
        Returns:
            目标 pose，如果未设置则返回 None
        """
        return self.target_pose
    
    def set_target_pose_internal(self, frame_id: str, pose: Pose) -> None:
        """仅更新内部目标 pose（不发布到话题），用于到达判断
        
        此方法只更新内部的 target_pose 变量，不发布任何消息到 ROS 话题。
        主要用于在需要到达判断但不想发布目标到话题的场景。
        
        Args:
            frame_id: 坐标系 ID
            pose: 目标 pose
        """
        # 使用 TF 转换并更新内部 target pose（不发布）
        self._update_target_pose_with_tf(pose, frame_id)
        logger.debug(f"Updated {self.label.lower()} internal target pose (not published) in frame '{frame_id}'")
    
    def send_joint_positions(self, positions: List[float], 
                            fsm_command_callback: Optional[Callable[[int], None]] = None) -> None:
        """发送关节位置命令（MoveJ 模式）
        
        Args:
            positions: 目标关节位置列表
            fsm_command_callback: 可选的 FSM 命令回调函数，用于切换状态（传入命令值）。
                                 如果未提供，则使用初始化时传入的回调（如果存在）
            
        Raises:
            ROS2NotConnectedError: 如果发布器未初始化
        """
        if self.joint_controller_pub is None:
            raise ROS2NotConnectedError(
                f"{self.label} joint controller publisher not initialized. "
                f"Joint controller topic not found."
            )
        
        # 使用传入的回调，如果没有则使用初始化时保存的回调
        callback = fsm_command_callback if fsm_command_callback is not None else self.fsm_command_callback
        
        # 发送 FSM 命令切换到 MOVEJ 状态（如果提供了回调）
        if callback:
            try:
                callback(4)
                logger.debug(f"{self.label}: Automatically switched to MOVEJ state for arm joint control")
                time.sleep(0.5)
            except Exception as e:
                logger.warning(f"{self.label}: Failed to switch to MOVEJ state: {e}")
        
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_controller_pub.publish(msg)
        logger.info(f"Published {self.label.lower()} joint positions: {positions}")
    
    def check_arrival(self, pose_threshold: float = DEFAULT_POSE_POSITION_THRESHOLD, 
                     orient_threshold: float = DEFAULT_POSE_ORIENTATION_THRESHOLD) -> Dict[str, Any]:
        """检查手臂是否到达目标位置
        
        Args:
            pose_threshold: 位置距离阈值（米），默认 DEFAULT_POSE_POSITION_THRESHOLD
            orient_threshold: 姿态距离阈值，默认 DEFAULT_POSE_ORIENTATION_THRESHOLD
            
        Returns:
            包含到达状态、距离等信息的字典
        """
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
    
    def _update_target_pose_with_tf(self, pose: Pose, frame_id: str) -> None:
        """使用 TF 转换更新目标 pose
        
        如果 frame_id 与 base_frame 不同，则进行坐标转换。
        
        Args:
            pose: 源 pose
            frame_id: 源坐标系 ID
        """
        if self.tf_buffer is None:
            # 如果没有 TF buffer，直接使用原始 pose
            target_pose = Pose()
            self._copy_pose(pose, target_pose)
            self.target_pose = target_pose
            return
        
        try:
            if frame_id == self.base_frame:
                # 如果已经在目标坐标系，直接复制
                target_pose = Pose()
                self._copy_pose(pose, target_pose)
                self.target_pose = target_pose
            else:
                # 需要坐标转换
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, frame_id, rclpy.time.Time()
                )
                transformed_pose = do_transform_pose(pose, transform)
                target_pose = Pose()
                self._copy_pose(transformed_pose, target_pose)
                self.target_pose = target_pose
        except TransformException as ex:
            logger.warning(f"Failed to transform pose from '{frame_id}' to '{self.base_frame}': {ex}")
            self.target_pose = None
    
    def cleanup(self) -> None:
        """清理资源"""
        if self.pose_sub:
            self.pose_sub.destroy()
            self.pose_sub = None
        
        if self.target_pub:
            self.target_pub.destroy()
            self.target_pub = None
        
        if self.target_stamped_pub:
            self.target_stamped_pub.destroy()
            self.target_stamped_pub = None
        
        if self.joint_controller_pub:
            self.joint_controller_pub.destroy()
            self.joint_controller_pub = None

