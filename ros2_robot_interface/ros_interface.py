"""
ROS 2 Robot Interface

Interface class for communicating with ROS 2 robots through topics.
This is a standalone implementation independent of LeRobot.
"""

import logging
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from .config import ControlType, ROS2RobotInterfaceConfig
from .exceptions import ROS2AlreadyConnectedError, ROS2NotConnectedError
from .arm_handler import ArmHandler, ArmType
from .gripper_handler import GripperHandler, GripperType

logger = logging.getLogger(__name__)


class ROS2RobotInterface:
    """Interface for communicating with ROS 2 robots."""
    
    def __init__(self, config: ROS2RobotInterfaceConfig):
        """Initialize the ROS 2 robot interface."""
        self.config = config
        self.robot_node: Node | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.executor_thread: threading.Thread | None = None
        
        self.joint_state_sub: Subscription | None = None
        self.target_path_pub: Publisher | None = None
        self.dual_target_stamped_pub: Publisher | None = None
        self.fsm_command_pub: Publisher | None = None
        self.head_joint_controller_pub: Publisher | None = None
        self.body_joint_controller_pub: Publisher | None = None
        self.left_hand_joint_controller_pub: Publisher | None = None
        self.right_hand_joint_controller_pub: Publisher | None = None
        self.arm_trajectory_pub: Publisher | None = None  # Unified trajectory publisher for both arms
        self.unified_arm_joint_controller_pub: Publisher | None = None  # Unified joint position publisher for both arms
        
        self.latest_joint_state: Dict[str, Any] | None = None
        
        self.data_lock = threading.Lock()
        self._connected = False
        
        self.last_joint_state_time = 0.0
        
        self._had_joint_state = False
        
        self.head_target_positions: Optional[List[float]] = None
        self.body_target_positions: Optional[List[float]] = None
        
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        
        # Arm handlers
        self.left_arm_handler: Optional[ArmHandler] = None
        self.right_arm_handler: Optional[ArmHandler] = None
        
        # Gripper handlers
        self.left_gripper_handler: Optional[GripperHandler] = None
        self.right_gripper_handler: Optional[GripperHandler] = None
    
    @property
    def is_connected(self) -> bool:
        """Check if the interface is connected."""
        return self._connected and self.robot_node is not None
    
    def _discover_topics(self) -> List[str]:
        """Discover available ROS 2 topics."""
        temp_node = Node(
            "ros2_robot_interface_temp",
            namespace=self.config.namespace if self.config.namespace else ""
        )
        temp_executor = SingleThreadedExecutor()
        temp_executor.add_node(temp_node)
        
        max_attempts = 30
        topic_names = []
        stable_count = 0
        last_count = 0
        
        for attempt in range(max_attempts):
            for _ in range(5):
                temp_executor.spin_once(timeout_sec=0.05)
            time.sleep(0.3)
            
            topic_names_and_types = temp_node.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_names_and_types]
            current_count = len(topic_names)
            
            if current_count == last_count and current_count > 2:
                stable_count += 1
                if stable_count >= 3:
                    break
            else:
                stable_count = 0
            
            last_count = current_count
        
        temp_executor.shutdown()
        temp_node.destroy_node()
        return topic_names
    
    def _auto_detect_configuration(self, topic_names: List[str]) -> bool:
        """Auto-detect robot configuration from topics. Returns True if dual-arm detected."""
        is_dual_arm = False
        
        if "/right_target" in topic_names or "/right_current_pose" in topic_names:
            is_dual_arm = True
            if "/right_current_pose" in topic_names:
                self.config.right_end_effector_pose_topic = "/right_current_pose"
            if "/right_target" in topic_names:
                self.config.right_end_effector_target_topic = "/right_target"
        
        # 检测目标位置订阅话题（用于到达判断）
        if "/left_current_target" in topic_names:
            self.config.end_effector_current_target_topic = "/left_current_target"
        if "/right_current_target" in topic_names:
            self.config.right_end_effector_current_target_topic = "/right_current_target"
        
        # 检测位置控制话题（gripper模式）
        if "/left_gripper_joint/position_command" in topic_names:
            self.config.gripper_command_topic = "/left_gripper_joint/position_command"
        if "/right_gripper_joint/position_command" in topic_names:
            self.config.right_gripper_command_topic = "/right_gripper_joint/position_command"
        
        # 检测 target_command 话题，确定控制器类型（hand_controller 或 gripper_controller）
        # 左夹爪控制器检测
        if "/left_hand_controller/target_command" in topic_names:
            self.config.left_gripper_controller_name = "left_hand_controller"
        elif "/left_gripper_controller/target_command" in topic_names:
            self.config.left_gripper_controller_name = "left_gripper_controller"
        elif "/hand_controller/target_command" in topic_names:
            # 单臂模式 - 灵巧手
            self.config.left_gripper_controller_name = "hand_controller"
        elif "/gripper_controller/target_command" in topic_names:
            # 单臂模式 - 夹爪
            self.config.left_gripper_controller_name = "gripper_controller"
        
        # 右夹爪控制器检测（仅双臂模式）
        if is_dual_arm:
            if "/right_hand_controller/target_command" in topic_names:
                self.config.right_gripper_controller_name = "right_hand_controller"
            elif "/right_gripper_controller/target_command" in topic_names:
                self.config.right_gripper_controller_name = "right_gripper_controller"
        
        if "/head_joint_controller/target_joint_position" in topic_names:
            self.config.head_joint_controller_topic = "/head_joint_controller/target_joint_position"
        
        if "/body_joint_controller/target_joint_position" in topic_names:
            self.config.body_joint_controller_topic = "/body_joint_controller/target_joint_position"
        
        # 检测灵巧手关节控制器 topic
        if "/left_hand_controller/target_joint_position" in topic_names:
            self.config.left_hand_joint_controller_topic = "/left_hand_controller/target_joint_position"
        
        if "/right_hand_controller/target_joint_position" in topic_names:
            self.config.right_hand_joint_controller_topic = "/right_hand_controller/target_joint_position"
        
        # 检测分开的左右臂 topic（优先检测，双臂模式）
        if "/ocs2_wbc_controller/target_joint_position/left" in topic_names:
            self.config.left_arm_joint_controller_topic = "/ocs2_wbc_controller/target_joint_position/left"
            if is_dual_arm and "/ocs2_wbc_controller/target_joint_position/right" in topic_names:
                self.config.right_arm_joint_controller_topic = "/ocs2_wbc_controller/target_joint_position/right"
        elif "/ocs2_arm_controller/target_joint_position/left" in topic_names:
            self.config.left_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position/left"
            if is_dual_arm and "/ocs2_arm_controller/target_joint_position/right" in topic_names:
                self.config.right_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position/right"
        
        # 检测统一的双臂关节控制器 topic（仅双臂模式，且可以与分开的 topic 同时存在）
        # 单臂模式下无后缀的 topic 应该设置为 left_arm_joint_controller_topic，而不是统一 topic
        if is_dual_arm:
            # 双臂模式：检测统一 topic（优先检测 /ocs2_wbc_controller/target_joint_position，其次 /ocs2_arm_controller/target_joint_position）
            if "/ocs2_wbc_controller/target_joint_position" in topic_names:
                # 检查是否是统一 topic（没有 /left 或 /right 后缀）
                # 注意：统一 topic 和分开的 topic 可以同时存在
                self.config.unified_arm_joint_controller_topic = "/ocs2_wbc_controller/target_joint_position"
            elif "/ocs2_arm_controller/target_joint_position" in topic_names:
                # 检查是否是统一 topic（没有 /left 或 /right 后缀）
                # 注意：统一 topic 和分开的 topic 可以同时存在
                self.config.unified_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position"
        else:
            # 单臂模式：只检测 /ocs2_arm_controller/target_joint_position（无后缀）
            # 如果没有设置 left_arm_joint_controller_topic（即没有 /left 后缀），则设置为无后缀的 topic
            if "/ocs2_arm_controller/target_joint_position" in topic_names:
                if self.config.left_arm_joint_controller_topic is None:
                    self.config.left_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position"
        
        return is_dual_arm
    
    def connect(self) -> None:
        """Connect to ROS 2 and create subscriptions/publishers."""
        if self.is_connected:
            raise ROS2AlreadyConnectedError("ROS2RobotInterface already connected")
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            is_dual_arm_detected = False
            try:
                topic_names = self._discover_topics()
                is_dual_arm_detected = self._auto_detect_configuration(topic_names)
            except Exception as e:
                logger.warning(f"Failed to auto-detect configuration: {e}")
                is_dual_arm_detected = False
            
            logger.info(f"{'DUAL-ARM' if is_dual_arm_detected else 'SINGLE-ARM'} MODE DETECTED")
            
            if self.config.node_name:
                node_name = self.config.node_name
            elif is_dual_arm_detected:
                node_name = "ros2_robot_interface_dual_arm"
            else:
                node_name = "ros2_robot_interface"
            
            self.robot_node = Node(
                node_name,
                namespace=self.config.namespace if self.config.namespace else ""
            )
            
            self.joint_state_sub = self.robot_node.create_subscription(
                JointState,
                self.config.joint_states_topic,
                self._joint_state_callback,
                10
            )
            
            # Initialize TF buffer first (needed by ArmHandler)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.robot_node)
            
            # Create left arm handler (always created)
            self.left_arm_handler = ArmHandler(
                self.robot_node,
                ArmType.LEFT,
                self.config,
                self.data_lock,
                self.tf_buffer,
                self.config.base_frame,
                self.send_fsm_command
            )
            self.left_arm_handler.initialize()
            
            # Create right arm handler if dual-arm mode detected
            if is_dual_arm_detected and self.config.right_end_effector_pose_topic:
                self.right_arm_handler = ArmHandler(
                    self.robot_node,
                    ArmType.RIGHT,
                    self.config,
                    self.data_lock,
                    self.tf_buffer,
                    self.config.base_frame,
                    self.send_fsm_command
                )
                self.right_arm_handler.initialize()
                self.target_path_pub = self.robot_node.create_publisher(Path, "/target_path", 10)
                self.dual_target_stamped_pub = self.robot_node.create_publisher(Path, "/dual_target/stamped", 10)
            
            # Create left gripper handler (if enabled)
            # 检查是否有位置控制topic或target_command控制器（灵巧手或夹爪）
            if self.config.gripper_enabled and (self.config.gripper_command_topic or self.config.left_gripper_controller_name):
                self.left_gripper_handler = GripperHandler(
                    self.robot_node,
                    GripperType.LEFT,
                    self.config,
                    self.data_lock
                )
                self.left_gripper_handler.initialize()
            
            # Create right gripper handler (if dual-arm mode)
            # 检查是否有位置控制topic或target_command控制器（灵巧手或夹爪）
            if is_dual_arm_detected and (self.config.right_gripper_command_topic or self.config.right_gripper_controller_name):
                self.right_gripper_handler = GripperHandler(
                    self.robot_node,
                    GripperType.RIGHT,
                    self.config,
                    self.data_lock
                )
                self.right_gripper_handler.initialize()
            
            self.fsm_command_pub = self.robot_node.create_publisher(Int32, "/fsm_command", 10)
            
            if self.config.head_joint_controller_topic:
                self.head_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.head_joint_controller_topic, 10
                )
            
            if self.config.body_joint_controller_topic:
                self.body_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.body_joint_controller_topic, 10
                )
            
            if self.config.left_hand_joint_controller_topic:
                self.left_hand_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.left_hand_joint_controller_topic, 10
                )
                logger.info(f"Created left hand joint controller publisher: {self.config.left_hand_joint_controller_topic}")
            
            if self.config.right_hand_joint_controller_topic:
                self.right_hand_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.right_hand_joint_controller_topic, 10
                )
                logger.info(f"Created right hand joint controller publisher: {self.config.right_hand_joint_controller_topic}")
            
            # Create unified arm joint position publisher (for dual-arm joint control)
            if self.config.unified_arm_joint_controller_topic:
                self.unified_arm_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.unified_arm_joint_controller_topic, 10
                )
                logger.info(f"Created unified arm joint position publisher: {self.config.unified_arm_joint_controller_topic}")
            
            # Create trajectory publishers for joint space multi-node trajectory planning
            # Create unified trajectory publisher for joint space multi-node trajectory planning
            # The topic is shared for both arms, controlled by joint_names in the message
            # Try to detect controller name from left_arm_joint_controller_topic or right_arm_joint_controller_topic
            controller_name = None
            if self.config.left_arm_joint_controller_topic:
                topic_parts = self.config.left_arm_joint_controller_topic.strip("/").split("/")
                controller_name = topic_parts[0]
            elif self.config.right_arm_joint_controller_topic:
                topic_parts = self.config.right_arm_joint_controller_topic.strip("/").split("/")
                controller_name = topic_parts[0]
            
            if controller_name:
                trajectory_topic = f"/{controller_name}/target_joint_trajectory"
                self.arm_trajectory_pub = self.robot_node.create_publisher(
                    JointTrajectory, trajectory_topic, 10
                )
                logger.info(f"Created unified arm trajectory publisher: {trajectory_topic} "
                           f"(supports single-arm and dual-arm via joint_names)")
            
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.robot_node)
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()
            
            time.sleep(1.0)
            self._connected = True
            logger.info("Connected to ROS 2 robot interface")
            
        except Exception as e:
            logger.error(f"Failed to connect to ROS 2 robot interface: {e}")
            self.disconnect()
            raise
    
    def _joint_state_callback(self, msg: JointState) -> None:
        """Callback for joint state messages."""
        with self.data_lock:
            current_time = time.time()
            was_recovering = False
            
            if not self._had_joint_state:
                was_recovering = True
            elif self.latest_joint_state is None:
                was_recovering = True
            elif self.config.joint_state_timeout > 0:
                time_since_last = current_time - self.last_joint_state_time
                if time_since_last > self.config.joint_state_timeout:
                    was_recovering = True
            
            self.latest_joint_state = {
                "names": list(msg.name),
                "positions": list(msg.position),
                "velocities": list(msg.velocity),
                "efforts": list(msg.effort),
                "timestamp": current_time
            }
            self.last_joint_state_time = current_time
            self._had_joint_state = True
            
            self._update_gripper_position_history_from_joint_state(msg.name, msg.position)
            
            if was_recovering:
                logger.info("Joint state data recovery: Started receiving joint state messages again")
    
    
    def _update_gripper_position_history_from_joint_state(self, joint_names: List[str], positions: List[float]) -> None:
        """Update gripper position history from joint state."""
        is_dual_arm = self.config.right_end_effector_pose_topic is not None
        
        for i, name in enumerate(joint_names):
            name_lower = name.lower()
            # 检查是否是夹爪/灵巧手关节（包含 'gripper' 或 'hand'）
            is_gripper_or_hand = 'gripper' in name_lower or 'hand' in name_lower
            if is_gripper_or_hand and i < len(positions):
                gripper_position = positions[i]
                
                if is_dual_arm:
                    if name_lower.startswith('left_'):
                        if self.left_gripper_handler:
                            self.left_gripper_handler.update_position_history(gripper_position)
                    elif name_lower.startswith('right_'):
                        if self.right_gripper_handler:
                            self.right_gripper_handler.update_position_history(gripper_position)
                    else:
                        # 默认更新左夹爪
                        if self.left_gripper_handler:
                            self.left_gripper_handler.update_position_history(gripper_position)
                else:
                    # 单臂模式，更新左夹爪
                    if self.left_gripper_handler:
                        self.left_gripper_handler.update_position_history(gripper_position)
    
    def _categorize_joints(self, joint_names: List[str], positions: List[float], 
                          velocities: List[float], efforts: List[float]) -> Dict[str, Dict[str, Any]]:
        """Categorize joints by body part."""
        is_dual_arm = self.config.right_end_effector_pose_topic is not None
        
        base_categories = ['head', 'body', 'other']
        if is_dual_arm:
            categories = {cat: {'names': [], 'positions': [], 'velocities': [], 'efforts': []} 
                         for cat in ['left_arm', 'right_arm', 'left_gripper', 'right_gripper'] + base_categories}
        else:
            categories = {cat: {'names': [], 'positions': [], 'velocities': [], 'efforts': []} 
                         for cat in ['arm', 'gripper'] + base_categories}
        
        for i, name in enumerate(joint_names):
            name_lower = name.lower()
            
            if is_dual_arm:
                # 检查是否是夹爪/灵巧手关节（包含 'gripper' 或 'hand'）
                is_gripper_or_hand = 'gripper' in name_lower or 'hand' in name_lower
                
                if is_gripper_or_hand:
                    category = 'left_gripper' if name_lower.startswith('left_') else 'right_gripper' if name_lower.startswith('right_') else 'left_gripper'
                elif name_lower.startswith('left_'):
                    category = 'left_arm'
                elif name_lower.startswith('right_'):
                    category = 'right_arm'
                else:
                    category = 'head' if 'head' in name_lower else 'body' if 'body' in name_lower else 'other'
            else:
                # 检查是否是夹爪/灵巧手关节（包含 'gripper' 或 'hand'）
                is_gripper_or_hand = 'gripper' in name_lower or 'hand' in name_lower
                
                if is_gripper_or_hand:
                    category = 'gripper'
                elif 'head' in name_lower:
                    category = 'head'
                elif 'body' in name_lower:
                    category = 'body'
                else:
                    category = 'arm' if 'joint' in name_lower else 'other'
            
            cat = categories[category]
            cat['names'].append(name)
            if i < len(positions):
                cat['positions'].append(positions[i])
            if i < len(velocities):
                cat['velocities'].append(velocities[i])
            if i < len(efforts):
                cat['efforts'].append(efforts[i])
        
        return categories
    
    def get_joint_state(self, categorized: bool = False) -> Dict[str, Any] | None:
        """Get the latest joint state."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        with self.data_lock:
            if self.config.joint_state_timeout > 0:
                if (time.time() - self.last_joint_state_time) > self.config.joint_state_timeout:
                    logger.warning("Joint state data is stale")
                    return None
            
            if self.latest_joint_state is None:
                return None
            
            if not categorized:
                return self.latest_joint_state.copy()
            
            categories = self._categorize_joints(
                self.latest_joint_state['names'],
                self.latest_joint_state['positions'],
                self.latest_joint_state['velocities'],
                self.latest_joint_state.get('efforts', [])
            )
            
            categories['timestamp'] = self.latest_joint_state.get('timestamp', 0.0)
            
            return categories
    
    def _copy_pose(self, src: Pose, dst: Pose) -> None:
        """Copy pose data from src to dst."""
        dst.position.x = src.position.x
        dst.position.y = src.position.y
        dst.position.z = src.position.z
        dst.orientation.x = src.orientation.x
        dst.orientation.y = src.orientation.y
        dst.orientation.z = src.orientation.z
        dst.orientation.w = src.orientation.w
    
    def send_target_path(self, left_poses: List[Pose | PoseStamped], right_poses: List[Pose | PoseStamped], frame_id: Optional[str] = None) -> None:
        """Send target path for dual-arm robot."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not self.config.right_end_effector_target_topic:
            raise ROS2NotConnectedError("Target path requires dual-arm mode. Right end-effector target topic not configured.")
        
        if self.target_path_pub is None:
            raise ROS2NotConnectedError("Target path publisher not initialized")
        
        if not left_poses and not right_poses:
            raise ValueError("At least one of left_poses or right_poses must not be empty")
        
        path_msg = Path()
        path_msg.header.stamp = self.robot_node.get_clock().now().to_msg()
        
        default_frame_id = frame_id
        if default_frame_id is None:
            if left_poses and isinstance(left_poses[0], PoseStamped):
                default_frame_id = left_poses[0].header.frame_id
            elif right_poses and isinstance(right_poses[0], PoseStamped):
                default_frame_id = right_poses[0].header.frame_id
            else:
                default_frame_id = "base_link"
        
        path_msg.header.frame_id = default_frame_id
        
        def to_pose_stamped(pose: Pose | PoseStamped, default_frame: str) -> PoseStamped:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id if frame_id is not None else (
                pose.header.frame_id if isinstance(pose, PoseStamped) else default_frame
            )
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose = pose.pose if isinstance(pose, PoseStamped) else pose
            return pose_stamped
        
        for pose in left_poses:
            path_msg.poses.append(to_pose_stamped(pose, default_frame_id))
        
        for pose in right_poses:
            path_msg.poses.append(to_pose_stamped(pose, default_frame_id))
        
        self.target_path_pub.publish(path_msg)
        logger.info(f"Published target path with {len(left_poses)} left arm waypoints and {len(right_poses)} right arm waypoints (total: {len(path_msg.poses)})")
        logger.debug(f"Path frame_id: {path_msg.header.frame_id}")
        
        # Update target poses in handlers internally with the last waypoint of each path (for check_arrival, without publishing to target/stamped topics)
        if left_poses and self.left_arm_handler:
            last_left_pose = left_poses[-1]
            # Extract Pose and frame_id from PoseStamped if needed
            if isinstance(last_left_pose, PoseStamped):
                left_target_pose = last_left_pose.pose
                left_frame_id = frame_id if frame_id is not None else last_left_pose.header.frame_id
            else:
                left_target_pose = last_left_pose
                left_frame_id = frame_id if frame_id is not None else default_frame_id
            # 不再需要 set_target_pose_internal，到达判断完全依赖话题订阅
            logger.debug(f"Updated left arm target pose to last waypoint for arrival checking")
        
        if right_poses and self.right_arm_handler:
            last_right_pose = right_poses[-1]
            # Extract Pose and frame_id from PoseStamped if needed
            if isinstance(last_right_pose, PoseStamped):
                right_target_pose = last_right_pose.pose
                right_frame_id = frame_id if frame_id is not None else last_right_pose.header.frame_id
            else:
                right_target_pose = last_right_pose
                right_frame_id = frame_id if frame_id is not None else default_frame_id
            # 不再需要 set_target_pose_internal，到达判断完全依赖话题订阅
            logger.debug(f"Updated right arm target pose to last waypoint for arrival checking")
    
    def send_dual_arm_target_stamped(self, left_pose: Pose, right_pose: Pose, frame_id: str = "arm_base") -> None:
        """Send dual-arm target poses to /dual_target/stamped topic."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if not self.config.right_end_effector_target_topic:
            raise ROS2NotConnectedError("Dual arm target requires dual-arm mode. Right end-effector target topic not configured.")
        if self.dual_target_stamped_pub is None:
            raise ROS2NotConnectedError("Dual target stamped publisher not initialized")
        
        stamp = self.robot_node.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = frame_id
        
        for pose in [left_pose, right_pose]:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = stamp
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        
        self.dual_target_stamped_pub.publish(path_msg)
        logger.info(f"Published dual arm target to /dual_target/stamped (frame_id: {frame_id})")
        logger.debug(f"Left arm pose: ({left_pose.position.x:.4f}, {left_pose.position.y:.4f}, {left_pose.position.z:.4f})")
        logger.debug(f"Right arm pose: ({right_pose.position.x:.4f}, {right_pose.position.y:.4f}, {right_pose.position.z:.4f})")
        
        # 不再需要 set_target_pose_internal，到达判断完全依赖话题订阅（/left_current_target 或 /right_current_target）
    
    
    def send_fsm_command(self, command: int) -> None:
        """Send FSM command for state switching."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.fsm_command_pub is None:
            logger.warning("FSM command publisher not initialized")
            return
        
        fsm_msg = Int32()
        fsm_msg.data = command
        self.fsm_command_pub.publish(fsm_msg)
        logger.info(f"Published FSM command: {command}")
    
    def send_head_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for head joints."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.head_joint_controller_pub is None:
            logger.warning("Head joint controller publisher not initialized. Set head_joint_controller_topic in config.")
            return
        
        msg = Float64MultiArray()
        msg.data = positions
        self.head_joint_controller_pub.publish(msg)
        logger.debug(f"Published head joint positions: {positions}")
        
        self.head_target_positions = positions.copy() if positions else None
    
    def send_body_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for body joints."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.body_joint_controller_pub is None:
            logger.warning("Body joint controller publisher not initialized. Set body_joint_controller_topic in config.")
            return
        
        msg = Float64MultiArray()
        msg.data = positions
        self.body_joint_controller_pub.publish(msg)
        logger.debug(f"Published body joint positions: {positions}")
        
        self.body_target_positions = positions.copy() if positions else None
    
    def send_left_hand_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for left hand joints."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.left_hand_joint_controller_pub is None:
            logger.warning("Left hand joint controller publisher not initialized. Set left_hand_joint_controller_topic in config.")
            return
        
        msg = Float64MultiArray()
        msg.data = positions
        self.left_hand_joint_controller_pub.publish(msg)
        logger.debug(f"Published left hand joint positions: {positions}")
    
    def send_right_hand_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for right hand joints."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.right_hand_joint_controller_pub is None:
            logger.warning("Right hand joint controller publisher not initialized. Set right_hand_joint_controller_topic in config.")
            return
        
        msg = Float64MultiArray()
        msg.data = positions
        self.right_hand_joint_controller_pub.publish(msg)
        logger.debug(f"Published right hand joint positions: {positions}")
    
    def send_dual_arm_joint_positions(self, left_arm_positions: List[float], right_arm_positions: List[float]) -> None:
        """发送双臂关节位置命令（MoveJ 模式）
        
        同时控制左臂和右臂的所有关节，发布到统一的 topic。
        
        Args:
            left_arm_positions: 左臂关节位置列表（弧度）
            right_arm_positions: 右臂关节位置列表（弧度）
            
        Raises:
            ROS2NotConnectedError: 如果接口未连接或发布器未初始化
            ValueError: 如果双臂模式未启用或参数无效
            
        Example:
            ```python
            # 左臂7个关节，右臂7个关节
            left_positions = [0.0, 0.5, -1.57, 0.0, 1.57, 0.0, 0.0]
            right_positions = [0.0, -0.5, 1.57, 0.0, -1.57, 0.0, 0.0]
            interface.send_dual_arm_joint_positions(left_positions, right_positions)
            ```
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not self.config.right_end_effector_target_topic:
            raise ValueError("Dual-arm joint control requires dual-arm mode. Right end-effector target topic not configured.")
        
        if self.unified_arm_joint_controller_pub is None:
            raise ROS2NotConnectedError(
                "Unified arm joint controller publisher not initialized. "
                "Unified arm joint controller topic not found. "
                "Please ensure /ocs2_wbc_controller/target_joint_position or "
                "/ocs2_arm_controller/target_joint_position topic exists."
            )
        
        if not left_arm_positions or not right_arm_positions:
            raise ValueError("Both left_arm_positions and right_arm_positions must not be empty")
        
        if len(left_arm_positions) != len(right_arm_positions):
            raise ValueError(
                f"Left and right arm must have the same number of joints. "
                f"Got {len(left_arm_positions)} and {len(right_arm_positions)}"
            )
        
        # 发送 FSM 命令切换到 MOVEJ 状态
        try:
            self.send_fsm_command(4)
            logger.debug("Automatically switched to MOVEJ state for dual-arm joint control")
        except Exception as e:
            logger.warning(f"Failed to switch to MOVEJ state: {e}")
        
        # 根据 topic 类型决定是否需要添加身体关节
        # ocs2_wbc_controller 需要：body_joints + left_arm_joints + right_arm_joints
        # ocs2_arm_controller 只需要：left_arm_joints + right_arm_joints
        unified_topic = self.config.unified_arm_joint_controller_topic
        is_wbc_controller = unified_topic and "ocs2_wbc_controller" in unified_topic
        
        if is_wbc_controller:
            # WBC 控制器需要身体关节 + 左臂 + 右臂
            # 获取当前身体关节位置（如果可用）
            body_positions = None
            categorized_state = self.get_joint_state(categorized=True)
            if categorized_state and categorized_state.get('body', {}).get('positions'):
                body_positions = categorized_state['body']['positions']
            
            if body_positions is None:
                # 如果没有身体关节数据，使用零位置或默认值
                logger.warning("Body joint positions not available, using zeros for WBC controller")
                # 尝试从配置中获取身体关节数量（通常是4个）
                body_positions = [0.0] * 4  # 默认4个身体关节
            
            # 合并：身体关节 + 左臂 + 右臂
            combined_positions = list(body_positions) + left_arm_positions + right_arm_positions
            logger.debug(f"WBC controller: body={len(body_positions)}, left={len(left_arm_positions)}, right={len(right_arm_positions)}, total={len(combined_positions)}")
        else:
            # ARM 控制器只需要左臂 + 右臂
            combined_positions = left_arm_positions + right_arm_positions
            logger.debug(f"ARM controller: left={len(left_arm_positions)}, right={len(right_arm_positions)}, total={len(combined_positions)}")
        
        msg = Float64MultiArray()
        msg.data = combined_positions
        self.unified_arm_joint_controller_pub.publish(msg)
        
    
    def send_joint_trajectory(self, 
                             joint_names: List[str],
                             waypoints: List[List[float]]) -> None:
        """Send multi-node joint trajectory for arm joints.
        
        This method uses a unified topic for both single-arm and dual-arm control.
        The controller determines which arm(s) to control based on the joint_names in the message.
        
        Args:
            joint_names: List of joint names (must match controller joints)
                       - For left arm only: use left arm joint names (e.g., ["left_joint1", "left_joint2", ...])
                       - For right arm only: use right arm joint names (e.g., ["right_joint1", "right_joint2", ...])
                       - For dual-arm: use both left and right arm joint names in order
            waypoints: List of waypoints, each waypoint is a list of joint positions
                      Note: Current joint position will be added as first waypoint automatically
                      Each waypoint must have the same length as joint_names
        
        Raises:
            ROS2NotConnectedError: If interface is not connected
            ValueError: If waypoints are invalid
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not waypoints or len(waypoints) < 2:
            raise ValueError("At least 2 waypoints are required (current position will be added as first waypoint)")
        
        if len(joint_names) == 0:
            raise ValueError("joint_names cannot be empty")
        
        if self.arm_trajectory_pub is None:
            raise ROS2NotConnectedError("Arm trajectory publisher not initialized. "
                                       "Arm joint controller topic not configured.")
        
        # Check waypoint dimensions
        for i, waypoint in enumerate(waypoints):
            if len(waypoint) != len(joint_names):
                raise ValueError(f"Waypoint {i} has {len(waypoint)} positions, but expected {len(joint_names)}")
        
        # Create JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.robot_node.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = ""
        trajectory_msg.joint_names = joint_names
        
        # Add waypoints (current position will be added as first point by the controller)
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = waypoint
            # time_from_start is not used currently, but we can set it for future use
            # For now, trajectory_duration parameter in controller will be used
            trajectory_msg.points.append(point)
        
        # Publish trajectory
        self.arm_trajectory_pub.publish(trajectory_msg)
        
        # Determine which arm(s) are being controlled based on joint names
        left_arm_joints = [name for name in joint_names if name.lower().startswith('left_')]
        right_arm_joints = [name for name in joint_names if name.lower().startswith('right_')]
        
        if left_arm_joints and right_arm_joints:
            arm_info = "dual-arm"
        elif left_arm_joints:
            arm_info = "left arm"
        elif right_arm_joints:
            arm_info = "right arm"
        else:
            arm_info = "arms (joint names don't match left_/right_ pattern)"
        
        logger.info(f"Published {arm_info} joint trajectory with {len(waypoints)} waypoints "
                   f"for {len(joint_names)} joints")
        logger.debug(f"Joint names: {joint_names}")
        logger.debug(f"Waypoints: {len(waypoints)} points")
    
    
    def _check_joint_arrival(self, part_name: str, target_positions: Optional[List[float]], 
                            current_positions: Optional[List[float]], threshold: float) -> Dict[str, Any]:
        """Check if joint positions have arrived at target."""
        arrived = False
        distance = float('inf')
        
        if target_positions is not None and current_positions is not None:
            if len(current_positions) == len(target_positions):
                distance = sum((c - t) ** 2 for c, t in zip(current_positions, target_positions)) ** 0.5
                arrived = distance < threshold
                
                print(f"  [位置检查-{part_name}] 当前位置: {[f'{p:.4f}' for p in current_positions]}")
                print(f"  [位置检查-{part_name}] 目标位置: {[f'{p:.4f}' for p in target_positions]}")
                print(f"  [位置检查-{part_name}] 距离: {distance:.4f} 弧度 (阈值: {threshold:.4f})")
                print(f"  [位置检查-{part_name}] {'✓ 已到达目标位置' if arrived else '✗ 未到达目标位置'}")
                print()
        
        return {'arrived': arrived, 'distance': distance}
    
    def check_arrive(self, part: Optional[str] = None, position_threshold: Optional[float] = None) -> Dict[str, Any]:
        """Check if head, body joints, arm poses, or grippers have arrived at target positions/poses.
        
        Args:
            part: 要检查的部分，可选值：None（所有部分）、'head'、'body'、'left_arm'、'right_arm'、
                  'left_gripper'、'right_gripper'、'arm'（单臂模式）、'gripper'（单臂模式）
            position_threshold: 关节位置阈值（仅用于 head 和 body），如果为 None 则使用默认值
        
        Returns:
            包含到达状态和距离信息的字典
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        is_dual_arm = self.config.right_end_effector_pose_topic is not None
        
        if part == 'arm' and not is_dual_arm:
            part = 'left_arm'
        if part == 'gripper' and not is_dual_arm:
            part = 'left_gripper'
        
        threshold = position_threshold if position_threshold is not None else self.config.position_threshold
        result = {}
        
        categorized_state = self.get_joint_state(categorized=True)
        
        if part is None or part == 'head':
            head_data = categorized_state.get('head', {})
            head_result = self._check_joint_arrival('HEAD', self.head_target_positions, 
                                                   head_data.get('positions'), threshold)
            if part == 'head':
                return head_result
            result['head'] = head_result
        
        if part is None or part == 'body':
            body_data = categorized_state.get('body', {})
            body_result = self._check_joint_arrival('BODY', self.body_target_positions,
                                                   body_data.get('positions'), threshold)
            if part == 'body':
                return body_result
            result['body'] = body_result
        
        if part is None or part == 'left_arm':
            # 使用 handler 的默认阈值
            arm_result = self.left_arm_handler.check_arrival()
            if part == 'left_arm':
                return arm_result
            result['left_arm' if is_dual_arm else 'arm'] = arm_result
        
        if is_dual_arm and (part is None or part == 'right_arm'):
            # 使用 handler 的默认阈值
            right_arm_result = self.right_arm_handler.check_arrival()
            if part == 'right_arm':
                return right_arm_result
            result['right_arm'] = right_arm_result
        
        if part is None or part == 'left_gripper':
            if self.left_gripper_handler:
                gripper_category = 'left_gripper' if is_dual_arm else 'gripper'
                gripper_data = categorized_state.get(gripper_category, {})
                gripper_current = gripper_data.get('positions', [None])[0] if gripper_data.get('positions') else None
                
                # 使用 handler 的默认阈值
                left_gripper_result = self.left_gripper_handler.check_arrival(gripper_current)
                
                if part == 'left_gripper':
                    return left_gripper_result
                result['left_gripper' if is_dual_arm else 'gripper'] = left_gripper_result
        
        if is_dual_arm and (part is None or part == 'right_gripper'):
            if self.right_gripper_handler:
                right_gripper_data = categorized_state.get('right_gripper', {})
                right_gripper_current = right_gripper_data.get('positions', [None])[0] if right_gripper_data.get('positions') else None
                
                # 使用 handler 的默认阈值
                right_gripper_result = self.right_gripper_handler.check_arrival(right_gripper_current)
                
                if part == 'right_gripper':
                    return right_gripper_result
                result['right_gripper'] = right_gripper_result
        
        return result
    
    def lookup_transform(self, target_frame: str, source_frame: str, 
                        timeout: Optional[float] = None) -> Optional[TransformStamped]:
        """查询两个坐标系之间的变换关系
        
        **参数语义说明：**
        - target_frame: 参考坐标系（在这个坐标系下观察）
        - source_frame: 被查询的坐标系（要查询它的位置）
        
        **返回的 Transform 方向说明：**
        - 返回的 TransformStamped 表示：**source_frame → target_frame** 的变换
        - 即：source_frame 相对于 target_frame 的位姿（source_frame 在 target_frame 坐标系下的位姿）
        - 语义上：查询 "source_frame 相对于 target_frame 的位置"
        
        **与 tf2_echo 的对应关系：**
        - lookup_transform("head_link2", "left_link6") 等价于：ros2 run tf2_ros tf2_echo head_link2 left_link6
        - 两者都返回 left_link6 → head_link2 的变换（left_link6 在 head_link2 坐标系下的位姿）
        
        **注意：**
        - 参数命名遵循 tf2 底层 API 的约定（与 tf_buffer.lookup_transform 一致）
        - 虽然语义上 source_frame 是"目标"（要查询的），target_frame 是"源"（参考坐标系）
        - 但为了与底层 API 保持一致，保持现有参数顺序
        
        Args:
            target_frame: 参考坐标系（在这个坐标系下观察，对应 tf2_echo 的第一个参数）
            source_frame: 被查询的坐标系（要查询它的位置，对应 tf2_echo 的第二个参数）
            timeout: 可选超时时间（秒），如果为 None 则立即返回（不等待）
            
        Returns:
            TransformStamped 消息，表示 source_frame → target_frame 的变换
            （source_frame 在 target_frame 坐标系下的位姿）
            如果查询失败则返回 None
            
        Example:
            # 查询 left_link6 相对于 head_link2 的位置
            # 返回的是 left_link6 → head_link2 的变换
            transform = interface.lookup_transform("head_link2", "left_link6")
            if transform:
                print(f"Translation: {transform.transform.translation}")
                print(f"Rotation: {transform.transform.rotation}")
        """
        if not self.is_connected:
            logger.warning("ROS2RobotInterface is not connected")
            return None
        
        if self.tf_buffer is None:
            logger.warning("TF buffer is not initialized")
            return None
        
        try:
            time_arg = Time()
            
            # 如果指定了 timeout，使用 Duration；否则不传递 timeout 参数
            # 注意：tf_buffer.lookup_transform(target_frame, source_frame) 返回的是
            # source_frame → target_frame 的变换（source_frame 在 target_frame 坐标系下的位姿）
            if timeout is not None:
                timeout_arg = Duration(seconds=timeout)
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, time_arg, timeout=timeout_arg
                )
            else:
                # 不传递 timeout 参数，使用默认行为（立即返回）
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, time_arg
                )
            # 返回的 transform 表示：source_frame → target_frame 的变换
            return transform
        except TransformException as ex:
            logger.warning(
                f"Failed to lookup transform from '{source_frame}' to '{target_frame}': {ex}"
            )
            return None
    
    def transform_pose(self, pose: Pose, source_frame: str, target_frame: str,
                      timeout: Optional[float] = None) -> Optional[Pose]:
        """将坐标从一个坐标系转换到另一个坐标系
        
        将 pose 从 source_frame 坐标系转换到 target_frame 坐标系。
        
        Args:
            pose: 要转换的 Pose（在 source_frame 坐标系下）
            source_frame: 源坐标系
            target_frame: 目标坐标系
            timeout: 可选超时时间（秒），如果为 None 则立即返回（不等待）
            
        Returns:
            转换后的 Pose（在 target_frame 坐标系下），如果转换失败则返回 None
            
        Example:
            # 将 pose 从 head_link2 坐标系转换到 left_link6 坐标系
            pose_in_head = Pose()  # 某个在 head_link2 坐标系下的 pose
            pose_in_left = interface.transform_pose(pose_in_head, "head_link2", "left_link6")
            if pose_in_left:
                print(f"转换后的位置: {pose_in_left.position}")
        """
        if not self.is_connected:
            logger.warning("ROS2RobotInterface is not connected")
            return None
        
        if self.tf_buffer is None:
            logger.warning("TF buffer is not initialized")
            return None
        
        # 如果源坐标系和目标坐标系相同，直接返回副本
        if source_frame == target_frame:
            result_pose = Pose()
            self._copy_pose(pose, result_pose)
            return result_pose
        
        try:
            time_arg = Time()
            
            # 如果指定了 timeout，使用 Duration；否则不传递 timeout 参数
            if timeout is not None:
                timeout_arg = Duration(seconds=timeout)
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, time_arg, timeout=timeout_arg
                )
            else:
                # 不传递 timeout 参数，使用默认行为（立即返回）
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, time_arg
                )
            
            # 执行坐标转换
            transformed_pose = do_transform_pose(pose, transform)
            
            # 创建结果并复制
            result_pose = Pose()
            self._copy_pose(transformed_pose, result_pose)
            return result_pose
        except TransformException as ex:
            logger.warning(
                f"Failed to transform pose from '{source_frame}' to '{target_frame}': {ex}"
            )
            return None
    
    def send_cartesian_velocity(self, linear: Tuple[float, float, float], angular: Tuple[float, float, float]) -> None:
        """Send cartesian velocity commands."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        logger.warning("Cartesian velocity control not implemented yet")
    
    def disconnect(self) -> None:
        """Disconnect from ROS 2 and cleanup resources."""
        if self.tf_listener is not None:
            self.tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer = None
        self._connected = False
        
        if self.executor:
            self.executor.shutdown()
            self.executor = None
        
        if self.executor_thread:
            self.executor_thread.join(timeout=2.0)
            self.executor_thread = None
        if self.joint_state_sub:
            self.joint_state_sub.destroy()
            self.joint_state_sub = None
        
        # Cleanup arm handlers
        if self.left_arm_handler:
            self.left_arm_handler.cleanup()
            self.left_arm_handler = None
        
        if self.right_arm_handler:
            self.right_arm_handler.cleanup()
            self.right_arm_handler = None
        
        if self.target_path_pub:
            self.target_path_pub.destroy()
            self.target_path_pub = None
        
        if self.dual_target_stamped_pub:
            self.dual_target_stamped_pub.destroy()
            self.dual_target_stamped_pub = None
        
        # Cleanup gripper handlers
        if self.left_gripper_handler:
            self.left_gripper_handler.cleanup()
            self.left_gripper_handler = None
        
        if self.right_gripper_handler:
            self.right_gripper_handler.cleanup()
            self.right_gripper_handler = None
        
        if self.fsm_command_pub:
            self.fsm_command_pub.destroy()
            self.fsm_command_pub = None
        
        if self.head_joint_controller_pub:
            self.head_joint_controller_pub.destroy()
            self.head_joint_controller_pub = None
        
        if self.body_joint_controller_pub:
            self.body_joint_controller_pub.destroy()
            self.body_joint_controller_pub = None
        
        if self.left_hand_joint_controller_pub:
            self.left_hand_joint_controller_pub.destroy()
            self.left_hand_joint_controller_pub = None
        
        if self.right_hand_joint_controller_pub:
            self.right_hand_joint_controller_pub.destroy()
            self.right_hand_joint_controller_pub = None
        
        if self.unified_arm_joint_controller_pub:
            self.unified_arm_joint_controller_pub.destroy()
            self.unified_arm_joint_controller_pub = None
        
        if self.robot_node:
            self.robot_node.destroy_node()
            self.robot_node = None
        
        if self.tf_listener is not None:
            self.tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer = None
        
        with self.data_lock:
            self.latest_joint_state = None
        
        try:
            rclpy.shutdown()
        except Exception as e:
            logger.warning(f"Error during rclpy shutdown: {e}")
        
        logger.info("Disconnected from ROS 2 robot interface")

