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
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from .config import ControlType, ROS2RobotInterfaceConfig
from .exceptions import ROS2AlreadyConnectedError, ROS2NotConnectedError

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
        self.end_effector_pose_sub: Subscription | None = None
        self.right_end_effector_pose_sub: Subscription | None = None
        
        self.end_effector_target_pub: Publisher | None = None
        self.right_end_effector_target_pub: Publisher | None = None
        self.end_effector_target_stamped_pub: Publisher | None = None
        self.right_end_effector_target_stamped_pub: Publisher | None = None
        self.target_path_pub: Publisher | None = None
        self.dual_target_stamped_pub: Publisher | None = None
        self.gripper_command_pub: Publisher | None = None
        self.right_gripper_command_pub: Publisher | None = None
        self.fsm_command_pub: Publisher | None = None
        self.head_joint_controller_pub: Publisher | None = None
        self.body_joint_controller_pub: Publisher | None = None
        self.left_arm_joint_controller_pub: Publisher | None = None
        self.right_arm_joint_controller_pub: Publisher | None = None
        
        self.latest_joint_state: Dict[str, Any] | None = None
        self.latest_end_effector_pose: Pose | None = None
        self.latest_right_end_effector_pose: Pose | None = None
        
        self.data_lock = threading.Lock()
        self._connected = False
        
        self.last_joint_state_time = 0.0
        self.last_end_effector_pose_time = 0.0
        self.last_right_end_effector_pose_time = 0.0
        
        self._had_joint_state = False
        self._had_end_effector_pose = False
        self._had_right_end_effector_pose = False
        
        self.head_target_positions: Optional[List[float]] = None
        self.body_target_positions: Optional[List[float]] = None
        self.position_threshold: float = 0.05
        
        self.left_gripper_target_position: Optional[float] = None
        self.right_gripper_target_position: Optional[float] = None
        self.gripper_position_threshold: float = 0.01
        
        self.left_gripper_position_history: List[float] = []
        self.right_gripper_position_history: List[float] = []
        self.gripper_stability_history_size: int = 15
        self.gripper_stability_threshold: float = 0.0001
        
        self.left_arm_target_pose: Optional[Pose] = None
        self.right_arm_target_pose: Optional[Pose] = None
        self.pose_position_threshold: float = 0.06
        self.pose_orientation_threshold: float = 0.1
        
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self.base_frame: str = "arm_base"
    
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
            
            if "/right_gripper_joint/position_command" in topic_names:
                self.config.right_gripper_command_topic = "/right_gripper_joint/position_command"
                self.config.gripper_command_topic = "/left_gripper_joint/position_command"
        
        if "/head_joint_controller/target_joint_position" in topic_names:
            self.config.head_joint_controller_topic = "/head_joint_controller/target_joint_position"
        
        if "/body_joint_controller/target_joint_position" in topic_names:
            self.config.body_joint_controller_topic = "/body_joint_controller/target_joint_position"
        
        if "/ocs2_wbc_controller/target_joint_position/left" in topic_names:
            self.config.left_arm_joint_controller_topic = "/ocs2_wbc_controller/target_joint_position/left"
            if is_dual_arm and "/ocs2_wbc_controller/target_joint_position/right" in topic_names:
                self.config.right_arm_joint_controller_topic = "/ocs2_wbc_controller/target_joint_position/right"
        elif "/ocs2_arm_controller/target_joint_position/left" in topic_names:
            self.config.left_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position/left"
            if is_dual_arm and "/ocs2_arm_controller/target_joint_position/right" in topic_names:
                self.config.right_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position/right"
        elif "/ocs2_arm_controller/target_joint_position" in topic_names:
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
            
            self.end_effector_pose_sub = self.robot_node.create_subscription(
                PoseStamped,
                self.config.end_effector_pose_topic,
                self._end_effector_pose_callback,
                10
            )
            
            if self.config.right_end_effector_pose_topic:
                self.right_end_effector_pose_sub = self.robot_node.create_subscription(
                    PoseStamped, self.config.right_end_effector_pose_topic,
                    self._right_end_effector_pose_callback, 10
                )
            
            self.end_effector_target_pub = self.robot_node.create_publisher(
                Pose, self.config.end_effector_target_topic, 10
            )
            self.end_effector_target_stamped_pub = self.robot_node.create_publisher(
                PoseStamped, f"{self.config.end_effector_target_topic}/stamped", 10
            )
            
            if self.config.right_end_effector_target_topic:
                self.right_end_effector_target_pub = self.robot_node.create_publisher(
                    Pose, self.config.right_end_effector_target_topic, 10
                )
                self.right_end_effector_target_stamped_pub = self.robot_node.create_publisher(
                    PoseStamped, f"{self.config.right_end_effector_target_topic}/stamped", 10
                )
                self.target_path_pub = self.robot_node.create_publisher(Path, "/target_path", 10)
                self.dual_target_stamped_pub = self.robot_node.create_publisher(Path, "/dual_target/stamped", 10)
            
            if self.config.gripper_enabled and self.config.gripper_command_topic:
                self.gripper_command_pub = self.robot_node.create_publisher(
                    Float64, self.config.gripper_command_topic, 10
                )
            
            if self.config.right_gripper_command_topic:
                self.right_gripper_command_pub = self.robot_node.create_publisher(
                    Float64, self.config.right_gripper_command_topic, 10
                )
            
            self.fsm_command_pub = self.robot_node.create_publisher(Int32, "/fsm_command", 10)
            
            if self.config.head_joint_controller_topic:
                self.head_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.head_joint_controller_topic, 10
                )
            
            if self.config.body_joint_controller_topic:
                self.body_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.body_joint_controller_topic, 10
                )
            
            if self.config.left_arm_joint_controller_topic:
                self.left_arm_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.left_arm_joint_controller_topic, 10
                )
            
            if self.config.right_arm_joint_controller_topic:
                self.right_arm_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.right_arm_joint_controller_topic, 10
                )
            
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.robot_node)
            
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
    
    def _end_effector_pose_callback(self, msg: PoseStamped) -> None:
        """Callback for end-effector pose messages."""
        with self.data_lock:
            current_time = time.time()
            was_recovering = False
            
            if not self._had_end_effector_pose:
                was_recovering = True
            elif self.latest_end_effector_pose is None:
                was_recovering = True
            elif self.config.end_effector_pose_timeout > 0:
                time_since_last = current_time - self.last_end_effector_pose_time
                if time_since_last > self.config.end_effector_pose_timeout:
                    was_recovering = True
            
            self.latest_end_effector_pose = msg.pose
            self.last_end_effector_pose_time = current_time
            self._had_end_effector_pose = True
            
            if was_recovering:
                logger.info("End-effector pose data recovery: Started receiving end-effector pose messages again")
    
    def _right_end_effector_pose_callback(self, msg: PoseStamped) -> None:
        """Callback for right end-effector pose messages (dual-arm mode)."""
        with self.data_lock:
            current_time = time.time()
            was_recovering = False
            
            if not self._had_right_end_effector_pose:
                was_recovering = True
            elif self.latest_right_end_effector_pose is None:
                was_recovering = True
            elif self.config.end_effector_pose_timeout > 0:
                time_since_last = current_time - self.last_right_end_effector_pose_time
                if time_since_last > self.config.end_effector_pose_timeout:
                    was_recovering = True
            
            self.latest_right_end_effector_pose = msg.pose
            self.last_right_end_effector_pose_time = current_time
            self._had_right_end_effector_pose = True
            
            if was_recovering:
                logger.info("Right end-effector pose data recovery: Started receiving right end-effector pose messages again")
    
    def _update_gripper_position_history_from_joint_state(self, joint_names: List[str], positions: List[float]) -> None:
        """Update gripper position history from joint state."""
        is_dual_arm = self.config.right_end_effector_pose_topic is not None
        
        for i, name in enumerate(joint_names):
            name_lower = name.lower()
            if 'gripper' in name_lower and i < len(positions):
                gripper_position = positions[i]
                
                if is_dual_arm:
                    if name_lower.startswith('left_'):
                        self.left_gripper_position_history.append(gripper_position)
                        if len(self.left_gripper_position_history) > self.gripper_stability_history_size:
                            self.left_gripper_position_history.pop(0)
                    elif name_lower.startswith('right_'):
                        self.right_gripper_position_history.append(gripper_position)
                        if len(self.right_gripper_position_history) > self.gripper_stability_history_size:
                            self.right_gripper_position_history.pop(0)
                    else:
                        self.left_gripper_position_history.append(gripper_position)
                        if len(self.left_gripper_position_history) > self.gripper_stability_history_size:
                            self.left_gripper_position_history.pop(0)
                else:
                    self.left_gripper_position_history.append(gripper_position)
                    if len(self.left_gripper_position_history) > self.gripper_stability_history_size:
                        self.left_gripper_position_history.pop(0)
    
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
                if 'gripper' in name_lower:
                    category = 'left_gripper' if name_lower.startswith('left_') else 'right_gripper' if name_lower.startswith('right_') else 'left_gripper'
                elif name_lower.startswith('left_'):
                    category = 'left_arm'
                elif name_lower.startswith('right_'):
                    category = 'right_arm'
                else:
                    category = 'head' if 'head' in name_lower else 'body' if 'body' in name_lower else 'other'
            else:
                if 'gripper' in name_lower:
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
    
    def get_end_effector_pose(self) -> Pose | None:
        """Get the latest end-effector pose (left arm)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        with self.data_lock:
            if self.config.end_effector_pose_timeout > 0:
                if (time.time() - self.last_end_effector_pose_time) > self.config.end_effector_pose_timeout:
                    logger.warning("End-effector pose data is stale")
                    return None
            
            return self.latest_end_effector_pose
    
    def get_right_end_effector_pose(self) -> Pose | None:
        """Get the latest right end-effector pose (dual-arm mode)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not self.config.right_end_effector_pose_topic:
            logger.warning("Right end-effector pose topic not configured. Dual-arm mode not enabled.")
            return None
        
        with self.data_lock:
            if self.config.end_effector_pose_timeout > 0:
                if (time.time() - self.last_right_end_effector_pose_time) > self.config.end_effector_pose_timeout:
                    logger.warning("Right end-effector pose data is stale")
                    return None
            
            return self.latest_right_end_effector_pose
    
    def _copy_pose(self, src: Pose, dst: Pose) -> None:
        """Copy pose data from src to dst."""
        dst.position.x = src.position.x
        dst.position.y = src.position.y
        dst.position.z = src.position.z
        dst.orientation.x = src.orientation.x
        dst.orientation.y = src.orientation.y
        dst.orientation.z = src.orientation.z
        dst.orientation.w = src.orientation.w
    
    def _update_target_pose_with_tf(self, pose: Pose, frame_id: str, target_pose_attr: str) -> None:
        """Update target pose with TF transformation if needed."""
        if self.tf_buffer is None:
            return
        
        try:
            if frame_id == self.base_frame:
                target_pose = Pose()
                self._copy_pose(pose, target_pose)
                setattr(self, target_pose_attr, target_pose)
            else:
                transform = self.tf_buffer.lookup_transform(self.base_frame, frame_id, rclpy.time.Time())
                transformed_pose = do_transform_pose(pose, transform)
                target_pose = Pose()
                self._copy_pose(transformed_pose, target_pose)
                setattr(self, target_pose_attr, target_pose)
        except TransformException as ex:
            logger.warning(f"Failed to transform pose from '{frame_id}' to '{self.base_frame}': {ex}")
            setattr(self, target_pose_attr, None)
    
    def send_end_effector_target(self, pose: Pose) -> None:
        """Send target end-effector pose (left arm)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if self.end_effector_target_pub is None:
            raise ROS2NotConnectedError("End-effector target publisher not initialized")
        
        self.end_effector_target_pub.publish(pose)
        self.left_arm_target_pose = Pose()
        self._copy_pose(pose, self.left_arm_target_pose)
        logger.debug(f"Published end-effector target: {pose}")
    
    def send_right_end_effector_target(self, pose: Pose) -> None:
        """Send target right end-effector pose (dual-arm mode)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if not self.config.right_end_effector_target_topic:
            raise ROS2NotConnectedError("Right end-effector target topic not configured. Dual-arm mode not enabled.")
        if self.right_end_effector_target_pub is None:
            raise ROS2NotConnectedError("Right end-effector target publisher not initialized")
        
        self.right_end_effector_target_pub.publish(pose)
        self.right_arm_target_pose = Pose()
        self._copy_pose(pose, self.right_arm_target_pose)
        logger.debug(f"Published right end-effector target: {pose}")
    
    def send_end_effector_target_stamped(self, frame_id: str, pose: Pose) -> None:
        """Send target end-effector pose with coordinate frame (left arm)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if self.end_effector_target_stamped_pub is None:
            raise ROS2NotConnectedError("End-effector target stamped publisher not initialized")
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.robot_node.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        self.end_effector_target_stamped_pub.publish(pose_stamped)
        logger.debug(f"Published end-effector target (stamped) in frame '{frame_id}': {pose}")
        self._update_target_pose_with_tf(pose, frame_id, 'left_arm_target_pose')
    
    def send_right_end_effector_target_stamped(self, frame_id: str, pose: Pose) -> None:
        """Send target right end-effector pose with coordinate frame (dual-arm mode)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if not self.config.right_end_effector_target_topic:
            raise ROS2NotConnectedError("Right end-effector target topic not configured. Dual-arm mode not enabled.")
        if self.right_end_effector_target_stamped_pub is None:
            raise ROS2NotConnectedError("Right end-effector target stamped publisher not initialized")
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.robot_node.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        self.right_end_effector_target_stamped_pub.publish(pose_stamped)
        logger.debug(f"Published right end-effector target (stamped) in frame '{frame_id}': {pose}")
        self._update_target_pose_with_tf(pose, frame_id, 'right_arm_target_pose')
    
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
        
        self.left_arm_target_pose = Pose()
        self._copy_pose(left_pose, self.left_arm_target_pose)
        
        self.right_arm_target_pose = Pose()
        self._copy_pose(right_pose, self.right_arm_target_pose)
    
    def send_gripper_command(self, position: float) -> None:
        """Send gripper position command."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not self.config.gripper_enabled:
            logger.warning("Gripper is not enabled in configuration")
            return
        
        if self.gripper_command_pub is None:
            logger.warning("Gripper command publisher not initialized")
            return
        
        clamped_position = max(
            self.config.gripper_min_position,
            min(position, self.config.gripper_max_position)
        )
        
        self.left_gripper_target_position = clamped_position
        self.left_gripper_position_history.clear()
        
        gripper_msg = Float64()
        gripper_msg.data = clamped_position
        
        self.gripper_command_pub.publish(gripper_msg)
        logger.debug(f"Published gripper command: {clamped_position}")
    
    def send_right_gripper_command(self, position: float) -> None:
        """Send right gripper position command (dual-arm mode)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not self.config.right_gripper_command_topic:
            logger.warning("Right gripper command topic not configured. Dual-arm mode not enabled.")
            return
        
        if self.right_gripper_command_pub is None:
            logger.warning("Right gripper command publisher not initialized")
            return
        
        clamped_position = max(
            self.config.gripper_min_position,
            min(position, self.config.gripper_max_position)
        )
        
        self.right_gripper_target_position = clamped_position
        self.right_gripper_position_history.clear()
        
        gripper_msg = Float64()
        gripper_msg.data = clamped_position
        
        self.right_gripper_command_pub.publish(gripper_msg)
        logger.debug(f"Published right gripper command: {clamped_position}")
    
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
    
    def send_left_arm_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for left arm (MoveJ mode)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.left_arm_joint_controller_pub is None:
            raise ROS2NotConnectedError("Left arm joint controller publisher not initialized. Arm joint controller topic not found.")
        
        try:
            self.send_fsm_command(4)
            logger.debug("Automatically switched to MOVEJ state for arm joint control")
            time.sleep(0.5)
        except Exception as e:
            logger.warning(f"Failed to switch to MOVEJ state: {e}")
        
        msg = Float64MultiArray()
        msg.data = positions
        self.left_arm_joint_controller_pub.publish(msg)
        logger.info(f"Published left arm joint positions: {positions}")
    
    def send_right_arm_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for right arm (MoveJ mode, dual-arm only)."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if not self.config.right_arm_joint_controller_topic:
            raise ROS2NotConnectedError("Right arm joint controller topic not configured. Dual-arm mode not enabled.")
        
        if self.right_arm_joint_controller_pub is None:
            raise ROS2NotConnectedError("Right arm joint controller publisher not initialized.")
        
        try:
            self.send_fsm_command(4)
            logger.debug("Automatically switched to MOVEJ state for arm joint control")
            time.sleep(0.5)
        except Exception as e:
            logger.warning(f"Failed to switch to MOVEJ state: {e}")
        
        msg = Float64MultiArray()
        msg.data = positions
        self.right_arm_joint_controller_pub.publish(msg)
        logger.info(f"Published right arm joint positions: {positions}")
    
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
    
    def _check_arm_arrival(self, arm_label: str, current_pose: Optional[Pose], target_pose: Optional[Pose],
                          pose_threshold: float, orient_threshold: float) -> Dict[str, Any]:
        """Check if arm pose has arrived at target."""
        arrived = False
        pos_dist = float('inf')
        orient_dist = float('inf')
        total_dist = float('inf')
        status_msg = None
        
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
            
            status_msg = f"{arm_label}已到达目标位置" if arrived else f"{arm_label}未到达目标位置"
            print(f"  [位置检查-{arm_label}] 当前位置: ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
            print(f"  [位置检查-{arm_label}] 目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
            print(f"  [位置检查-{arm_label}] 位置距离: {pos_dist:.4f} 米 (阈值: {pose_threshold:.4f})")
            print(f"  [位置检查-{arm_label}] 姿态距离: {orient_dist:.4f} (阈值: {orient_threshold:.4f})")
            print(f"  [位置检查-{arm_label}] {'✓ 已到达目标位置' if arrived else '✗ 未到达目标位置'}")
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
    
    def _check_gripper_arrival(self, gripper_label: str, current_position: Optional[float],
                              target_position: Optional[float], position_history: List[float],
                              threshold: float, stability_threshold: float, history_size: int) -> Dict[str, Any]:
        """Check if gripper has arrived at target position."""
        arrived = False
        distance = float('inf')
        
        if current_position is not None and target_position is not None:
            distance = abs(current_position - target_position)
            
            is_stable = False
            position_variance = float('inf')
            if len(position_history) == history_size:
                recent_positions = position_history[-history_size:]
                position_variance = max(recent_positions) - min(recent_positions)
                is_stable = position_variance < stability_threshold
            
            is_closing = current_position > target_position
            if is_closing:
                arrived = (distance < threshold) or is_stable
            else:
                arrived = distance < threshold
            
            print(f"  [位置检查-{gripper_label}] 当前位置: {current_position:.4f}")
            print(f"  [位置检查-{gripper_label}] 目标位置: {target_position:.4f}")
            print(f"  [位置检查-{gripper_label}] 距离: {distance:.4f} (阈值: {threshold:.4f})")
            if len(position_history) > 0:
                history_str = ", ".join([f"{p:.4f}" for p in position_history])
                print(f"  [位置检查-{gripper_label}] 位置历史 ({len(position_history)}个值): [{history_str}]")
            if len(position_history) == history_size:
                print(f"  [位置检查-{gripper_label}] 位置稳定性: {is_stable} (变化: {position_variance:.4f}, 阈值: {stability_threshold:.4f})")
            if arrived:
                if is_stable and is_closing and distance >= threshold:
                    print(f"  [位置检查-{gripper_label}] ✓ 已到达关闭状态（位置稳定，可能已夹住物体）")
                else:
                    print(f"  [位置检查-{gripper_label}] ✓ 已到达目标位置")
            else:
                print(f"  [位置检查-{gripper_label}] ✗ 未到达目标位置")
            print()
        
        return {'arrived': arrived, 'distance': distance}
    
    def check_arrive(self, part: Optional[str] = None, position_threshold: Optional[float] = None,
                     pose_position_threshold: Optional[float] = None, 
                     gripper_position_threshold: Optional[float] = None) -> Dict[str, Any]:
        """Check if head, body joints, arm poses, or grippers have arrived at target positions/poses."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        is_dual_arm = self.config.right_end_effector_pose_topic is not None
        
        if part == 'arm' and not is_dual_arm:
            part = 'left_arm'
        if part == 'gripper' and not is_dual_arm:
            part = 'left_gripper'
        
        threshold = position_threshold if position_threshold is not None else self.position_threshold
        pose_threshold = pose_position_threshold if pose_position_threshold is not None else self.pose_position_threshold
        gripper_threshold = gripper_position_threshold if gripper_position_threshold is not None else self.gripper_position_threshold
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
            arm_label = "ARM" if not is_dual_arm else "LEFT_ARM"
            arm_result = self._check_arm_arrival(arm_label, self.get_end_effector_pose(),
                                                self.left_arm_target_pose, pose_threshold,
                                                self.pose_orientation_threshold)
            if part == 'left_arm':
                return arm_result
            result['left_arm' if is_dual_arm else 'arm'] = arm_result
        
        if is_dual_arm and (part is None or part == 'right_arm'):
            right_arm_result = self._check_arm_arrival("RIGHT_ARM", self.get_right_end_effector_pose(),
                                                      self.right_arm_target_pose, pose_threshold,
                                                      self.pose_orientation_threshold)
            if part == 'right_arm':
                return right_arm_result
            result['right_arm'] = right_arm_result
        
        if part is None or part == 'left_gripper':
            gripper_category = 'left_gripper' if is_dual_arm else 'gripper'
            gripper_data = categorized_state.get(gripper_category, {})
            gripper_current = gripper_data.get('positions', [None])[0] if gripper_data.get('positions') else None
            
            gripper_label = "LEFT_GRIPPER" if is_dual_arm else "GRIPPER"
            left_gripper_result = self._check_gripper_arrival(
                gripper_label, gripper_current, self.left_gripper_target_position,
                self.left_gripper_position_history, gripper_threshold,
                self.gripper_stability_threshold, self.gripper_stability_history_size
            )
            
            if part == 'left_gripper':
                return left_gripper_result
            result['left_gripper' if is_dual_arm else 'gripper'] = left_gripper_result
        
        if is_dual_arm and (part is None or part == 'right_gripper'):
            right_gripper_data = categorized_state.get('right_gripper', {})
            right_gripper_current = right_gripper_data.get('positions', [None])[0] if right_gripper_data.get('positions') else None
            
            right_gripper_result = self._check_gripper_arrival(
                "RIGHT_GRIPPER", right_gripper_current, self.right_gripper_target_position,
                self.right_gripper_position_history, gripper_threshold,
                self.gripper_stability_threshold, self.gripper_stability_history_size
            )
            
            if part == 'right_gripper':
                return right_gripper_result
            result['right_gripper'] = right_gripper_result
        
        return result
    
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
        
        if self.end_effector_pose_sub:
            self.end_effector_pose_sub.destroy()
            self.end_effector_pose_sub = None
        
        if self.right_end_effector_pose_sub:
            self.right_end_effector_pose_sub.destroy()
            self.right_end_effector_pose_sub = None
        
        if self.end_effector_target_pub:
            self.end_effector_target_pub.destroy()
            self.end_effector_target_pub = None
        
        if self.end_effector_target_stamped_pub:
            self.end_effector_target_stamped_pub.destroy()
            self.end_effector_target_stamped_pub = None
        
        if self.right_end_effector_target_pub:
            self.right_end_effector_target_pub.destroy()
            self.right_end_effector_target_pub = None
        
        if self.right_end_effector_target_stamped_pub:
            self.right_end_effector_target_stamped_pub.destroy()
            self.right_end_effector_target_stamped_pub = None
        
        if self.target_path_pub:
            self.target_path_pub.destroy()
            self.target_path_pub = None
        
        if self.dual_target_stamped_pub:
            self.dual_target_stamped_pub.destroy()
            self.dual_target_stamped_pub = None
        
        if self.gripper_command_pub:
            self.gripper_command_pub.destroy()
            self.gripper_command_pub = None
        
        if self.right_gripper_command_pub:
            self.right_gripper_command_pub.destroy()
            self.right_gripper_command_pub = None
        
        if self.fsm_command_pub:
            self.fsm_command_pub.destroy()
            self.fsm_command_pub = None
        
        if self.head_joint_controller_pub:
            self.head_joint_controller_pub.destroy()
            self.head_joint_controller_pub = None
        
        if self.body_joint_controller_pub:
            self.body_joint_controller_pub.destroy()
            self.body_joint_controller_pub = None
        
        if self.left_arm_joint_controller_pub:
            self.left_arm_joint_controller_pub.destroy()
            self.left_arm_joint_controller_pub = None
        
        if self.right_arm_joint_controller_pub:
            self.right_arm_joint_controller_pub.destroy()
            self.right_arm_joint_controller_pub = None
        
        if self.robot_node:
            self.robot_node.destroy_node()
            self.robot_node = None
        
        if self.tf_listener is not None:
            self.tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer = None
        
        with self.data_lock:
            self.latest_joint_state = None
            self.latest_end_effector_pose = None
            self.latest_right_end_effector_pose = None
        
        try:
            rclpy.shutdown()
        except Exception as e:
            logger.warning(f"Error during rclpy shutdown: {e}")
        
        logger.info("Disconnected from ROS 2 robot interface")

