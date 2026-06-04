"""
ROS 2 Robot Interface

Interface class for communicating with ROS 2 robots through topics.
This is a standalone implementation independent of LeRobot.
"""

import logging
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

from arms_ros2_control_msgs.action import ExecuteLinear, JointTrajectory as JointTrajectoryAction, MovecUseIK
from arms_ros2_control_msgs.msg import CircleMessage, JointWaypoint, LinearMessage
from arms_ros2_control_msgs.msg import WbcCurrentState
from arms_ros2_control_msgs.srv import ExecutePath

from .config import ControlType, ROS2RobotInterfaceConfig
from .constants import FSM_HOLD, FSM_HOME, FSM_MOVEJ, FSM_OCS2
from .utils.exceptions import ROS2AlreadyConnectedError, ROS2NotConnectedError
from .utils.quat_pose import quat_multiply, rotate_vector_by_quat
from .handler import ArmHandler, ArmType, GripperHandler, GripperType
from .utils.discovery import (
    discover_topics as _discover_topics,
    list_nodes as _list_nodes,
    list_node_parameters as _list_node_parameters,
    set_node_parameters as _set_node_parameters,
)

logger = logging.getLogger(__name__)


class ROS2RobotInterface:
    """Interface for communicating with ROS 2 robots."""

    BODY_MODE_TO_STATE: Dict[str, int] = {
        "BODY_FREE": WbcCurrentState.BODY_FREE,
        "BODY_RELATIVE": WbcCurrentState.BODY_VERTICAL,
        "BODY_VERTICAL": WbcCurrentState.BODY_VERTICAL,  # backward-compatible alias
        "BODY_TRACKING": WbcCurrentState.BODY_TRACKING,
        "BODY_LOCK": WbcCurrentState.BODY_LOCKED,
        "BODY_HEAD_COUPLED": WbcCurrentState.BODY_HEAD_COUPLED,
    }
    BODY_MODE_TO_COMMAND: Dict[str, str] = {
        "BODY_FREE": "BODY_FREE",
        "BODY_RELATIVE": "BODY_RELATIVE",
        "BODY_VERTICAL": "BODY_RELATIVE",  # backward-compatible alias
        "BODY_TRACKING": "BODY_TRACKING",
        "BODY_LOCK": "BODY_LOCK",
        "BODY_HEAD_COUPLED": "BODY_HEAD_COUPLED",
    }
    MODE_SWITCH_SETTLE_TIME_SEC: float = 0.1
    
    def __init__(self, config: ROS2RobotInterfaceConfig):
        """Initialize the ROS 2 robot interface."""
        self.config = config
        self.robot_node: Node | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.executor_thread: threading.Thread | None = None
        
        self.joint_state_sub: Subscription | None = None
        self.fsm_state_sub: Subscription | None = None
        self.robot_description_sub: Subscription | None = None
        self.body_current_target_sub: Subscription | None = None
        self.wbc_state_sub: Subscription | None = None
        self.target_path_pub: Publisher | None = None
        self.execute_path_client: Client | None = None
        self.joint_trajectory_action_client: ActionClient | None = None
        self.movel_action_client: ActionClient | None = None
        self.movec_action_client: ActionClient | None = None
        self.dual_target_stamped_pub: Publisher | None = None
        self.fsm_command_pub: Publisher | None = None
        self.mode_command_pub: Publisher | None = None
        self.head_joint_controller_pub: Publisher | None = None
        self.body_joint_controller_pub: Publisher | None = None
        self.left_hand_joint_controller_pub: Publisher | None = None
        self.right_hand_joint_controller_pub: Publisher | None = None
        self.arm_trajectory_pub: Publisher | None = None  # Unified trajectory publisher for both arms
        self.unified_arm_joint_controller_pub: Publisher | None = None  # Unified joint position publisher for both arms

        self.waist_lifting_command_pub: Publisher | None = None
        self.waist_turning_command_pub: Publisher | None = None

        self.latest_joint_state: Dict[str, Any] | None = None
        self.latest_categorized_joint_state: Dict[str, Any] | None = None  # Cached categorized state
        
        # FSM state tracking
        self._current_fsm_state: int = 2  # Default to HOLD
        self._auto_switch_fsm_before_control: bool = bool(self.config.auto_switch_fsm_before_control)
        self.is_wbc: bool = False
        # WBC unified joint topic capabilities discovered from ROS graph.
        # Some deployments expose body/head as dedicated sub-topics, and we use
        # this to decide whether body/head should go via WBC unified path or
        # fallback split publishers.
        self._wbc_has_body_joint_topic: bool = False
        self._wbc_has_head_joint_topic: bool = False
        
        # Robot description tracking
        self.latest_robot_description: Optional[str] = None
        self._robot_description_received = False
        self._connected = False
        
        self.last_joint_state_time = 0.0
        
        self._had_joint_state = False
        
        self.head_target_positions: Optional[List[float]] = None
        self.body_target_positions: Optional[List[float]] = None
        self.body_current_target: Optional[List[float]] = None
        self.wbc_state: Optional[WbcCurrentState] = None
        
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        
        # Arm handlers
        self.left_arm_handler: Optional[ArmHandler] = None
        self.right_arm_handler: Optional[ArmHandler] = None
        
        # Gripper handlers
        self.left_gripper_handler: Optional[GripperHandler] = None
        self.right_gripper_handler: Optional[GripperHandler] = None

        # Nav2 navigation state（软依赖，connect() 时自动检测）
        self._nav_enabled: bool = False
        self._nav_action_client: Any = None   # rclpy.action.ActionClient 或 None
        self._nav_goal_future: Any = None      # send_goal_async future
        self._nav_goal_handle: Any = None      # GoalHandle
        self._nav_result_future: Any = None    # get_result_async future

    @property
    def is_connected(self) -> bool:
        """Check if the interface is connected."""
        return self._connected and self.robot_node is not None

    @staticmethod
    def _controller_node_from_topic(topic: str | None) -> str:
        if not isinstance(topic, str):
            return ""
        t = topic.strip()
        if not t:
            return ""
        suffix = "/target_joint_position"
        if t.endswith(suffix):
            node = t[: -len(suffix)]
            return node if node else ""
        i = t.rfind("/")
        if i <= 0:
            return ""
        return t[:i]

    @property
    def body_controller(self) -> str:
        """躯干/腰关节控制器节点全名，供 ``set_node_parameters`` 等使用。

        兼容两类栈：
        - split 栈：优先由 ``body_joint_controller_topic`` 推断；
        - WBC unified 栈：若无独立 body topic，但检测到 WBC 统一控制器可携带 body
          通道（或至少识别到 WBC），回退到 unified arm 控制器节点，避免上层
          MoveJ 编排因 ``interface.body_controller`` 为空而直接报错。
        """
        node = self._controller_node_from_topic(self.config.body_joint_controller_topic)
        if node:
            return node

        # WBC unified 栈通常没有独立 body_joint_controller_topic。
        # 此时复用 unified 控制器节点用于参数设置（例如 movej duration）。
        if self.is_wbc and (
            self._wbc_has_body_joint_topic or self.config.unified_arm_joint_controller_topic
        ):
            return self.arm_controller
        return ""

    @property
    def arm_controller(self) -> str:
        """双臂/统一臂控制器节点全名（由 ``unified_arm_joint_controller_topic`` 或 ``left_arm_joint_controller_topic`` 推断）。

        典型 OCS2 / WBC 栈下可用于 ``set_node_parameters``（如 ``movel_duration``）等。
        """
        for topic in (
            self.config.unified_arm_joint_controller_topic,
            self.config.left_arm_joint_controller_topic,
        ):
            n = self._controller_node_from_topic(topic)
            if n:
                return n
        return ""

    def list_nodes(self) -> List[Dict[str, str]]:
        """查询当前运行的 ROS 2 节点列表。详见 utils.discovery.list_nodes() 的文档。"""
        # 如果已连接，使用现有节点；否则传递 None 让函数创建临时节点
        node = self.robot_node if self.is_connected else None
        # 不传 namespace，保持使用空命名空间创建临时节点
        return _list_nodes(node=node)
    
    def list_node_parameters(self, full_node_name: str) -> List[Dict[str, Any]]:
        """查询指定节点的可动态配置参数。详见 utils.discovery.list_node_parameters() 的文档。"""
        # 如果已连接，使用现有节点；否则传递 None 让函数创建临时节点
        node = self.robot_node if self.is_connected else None
        return _list_node_parameters(
            full_node_name=full_node_name,
            node=node,
        )
    
    def set_node_parameters(
        self,
        full_node_name: str,
        parameters: Dict[str, Any]
    ) -> bool:
        """设置指定节点的参数值。详见 utils.discovery.set_node_parameters() 的文档。"""
        # 如果已连接，使用现有节点；否则传递 None 让函数创建临时节点
        node = self.robot_node if self.is_connected else None
        return _set_node_parameters(
            full_node_name=full_node_name,
            parameters=parameters,
            node=node,
            namespace=""
        )
    
    def _auto_detect_configuration(self, topic_names: List[str]) -> bool:
        """Auto-detect robot configuration from topics. Returns True if dual-arm detected."""
        is_dual_arm = False
        self.is_wbc = False
        self._wbc_has_body_joint_topic = False
        self._wbc_has_head_joint_topic = False
        
        if "/right_target" in topic_names or "/right_current_pose" in topic_names:
            is_dual_arm = True
            if "/right_current_pose" in topic_names:
                self.config.right_end_effector_pose_topic = "/right_current_pose"
            if "/right_target" in topic_names:
                self.config.right_end_effector_target_topic = "/right_target"
        # Joint-only dual-arm stacks may not expose right Cartesian targets.
        # Fall back to right arm MoveJ topics so coordinated joint commands can
        # still route through dual-arm/unified paths.
        if not is_dual_arm and (
            "/ocs2_wbc_controller/target_joint_position/right" in topic_names
            or "/ocs2_arm_controller/target_joint_position/right" in topic_names
        ):
            is_dual_arm = True
        
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
            logger.info("Detected left gripper controller: left_hand_controller")
        elif "/left_gripper_controller/target_command" in topic_names:
            self.config.left_gripper_controller_name = "left_gripper_controller"
            logger.info("Detected left gripper controller: left_gripper_controller")
        elif "/hand_controller/target_command" in topic_names:
            # 单臂模式 - 灵巧手
            self.config.left_gripper_controller_name = "hand_controller"
            logger.info("Detected left gripper controller: hand_controller (single-arm mode)")
        elif "/gripper_controller/target_command" in topic_names:
            # 单臂模式 - 夹爪
            self.config.left_gripper_controller_name = "gripper_controller"
            logger.info("Detected left gripper controller: gripper_controller (single-arm mode)")

        if "/right_hand_controller/target_command" in topic_names:
            self.config.right_gripper_controller_name = "right_hand_controller"
            logger.info("Detected right gripper controller: right_hand_controller")
        elif "/right_gripper_controller/target_command" in topic_names:
            self.config.right_gripper_controller_name = "right_gripper_controller"
            logger.info("Detected right gripper controller: right_gripper_controller")

        # 检测 target_percent 话题（百分比控制）
        if "/left_gripper_controller/target_percent" in topic_names:
            self.config.left_gripper_target_percent_topic = "/left_gripper_controller/target_percent"
            logger.info("Detected left gripper target_percent topic")
        if "/right_gripper_controller/target_percent" in topic_names:
            self.config.right_gripper_target_percent_topic = "/right_gripper_controller/target_percent"
            logger.info("Detected right gripper target_percent topic")

        if "/head_joint_controller/target_joint_position" in topic_names:
            self.config.head_joint_controller_topic = "/head_joint_controller/target_joint_position"
        
        if "/body_joint_controller/target_joint_position" in topic_names:
            self.config.body_joint_controller_topic = "/body_joint_controller/target_joint_position"

        if "/body_joint_controller/current_target_joint" in topic_names:
            self.config.body_joint_current_target_topic = "/body_joint_controller/current_target_joint"

        if "/body_joint_controller/waist_lifting" in topic_names:
            self.config.waist_lifting_topic = "/body_joint_controller/waist_lifting"

        if "/body_joint_controller/waist_lifting_command" in topic_names:
            self.config.waist_lifting_command_topic = "/body_joint_controller/waist_lifting_command"

        if "/body_joint_controller/waist_turning_command" in topic_names:
            self.config.waist_turning_command_topic = "/body_joint_controller/waist_turning_command"
        
        # 检测灵巧手关节控制器 topic
        if "/left_hand_controller/target_joint_position" in topic_names:
            self.config.left_hand_joint_controller_topic = "/left_hand_controller/target_joint_position"

        if "/right_hand_controller/target_joint_position" in topic_names:
            self.config.right_hand_joint_controller_topic = "/right_hand_controller/target_joint_position"

        # 检测分开的左右臂 topic（优先检测 WBC / ARM 的 /left）
        # 仅在尚未手动/预置 left_arm_joint_controller_topic 时写入，避免覆盖单臂无后缀 topic 配置
        if "/ocs2_wbc_controller/target_joint_position/left" in topic_names:
            if self.config.left_arm_joint_controller_topic is None:
                self.config.left_arm_joint_controller_topic = (
                    "/ocs2_wbc_controller/target_joint_position/left"
                )
            if "/ocs2_wbc_controller/target_joint_position/right" in topic_names:
                is_dual_arm = True
                self.config.right_arm_joint_controller_topic = "/ocs2_wbc_controller/target_joint_position/right"
        elif "/ocs2_arm_controller/target_joint_position/left" in topic_names:
            if self.config.left_arm_joint_controller_topic is None:
                self.config.left_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position/left"
            if "/ocs2_arm_controller/target_joint_position/right" in topic_names:
                is_dual_arm = True
                self.config.right_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position/right"

        # 检测统一的双臂关节控制器 topic（仅双臂模式，且可以与分开的 topic 同时存在）
        # 单臂模式下无后缀的 topic 应该设置为 left_arm_joint_controller_topic，而不是统一 topic
        if "/ocs2_wbc_controller/target_joint_position" in topic_names:
            self.is_wbc = True
            self._wbc_has_body_joint_topic = "/ocs2_wbc_controller/target_joint_position/body" in topic_names
            self._wbc_has_head_joint_topic = "/ocs2_wbc_controller/target_joint_position/head" in topic_names

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
            # 单臂模式：优先无后缀 `/ocs2_arm_controller/target_joint_position`（常见单链 OCS2）
            if "/ocs2_arm_controller/target_joint_position" in topic_names:
                if self.config.left_arm_joint_controller_topic is None:
                    self.config.left_arm_joint_controller_topic = "/ocs2_arm_controller/target_joint_position"
            # 若无后缀话题（图中尚未出现或仿真仅广告 /left），退回分臂 topic，供 MoveJ 使用
            elif self.config.left_arm_joint_controller_topic is None:
                if "/ocs2_arm_controller/target_joint_position/left" in topic_names:
                    self.config.left_arm_joint_controller_topic = (
                        "/ocs2_arm_controller/target_joint_position/left"
                    )
        
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
                topic_names = _discover_topics()
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
            
            # Subscribe to robot description for URDF tracking
            from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
            fsm_state_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
            self.fsm_state_sub = self.robot_node.create_subscription(
                Int32,
                "/fsm_state",
                self._fsm_state_callback,
                fsm_state_qos
            )
            logger.info("✅ Subscribed to /fsm_state for actual FSM state tracking")

            robot_desc_qos = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Receive latched messages
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
            self.robot_description_sub = self.robot_node.create_subscription(
                String,
                "/robot_description",
                self._robot_description_callback,
                robot_desc_qos
            )
            logger.info("✅ Subscribed to /robot_description for URDF tracking")

            if self.config.body_joint_current_target_topic:
                # Subscribe to current target body joints
                self.body_current_target_sub = self.robot_node.create_subscription(
                    Float64MultiArray,
                    self.config.body_joint_current_target_topic,
                    self._body_current_target_callback,
                    10
                )
                logger.info("✅ Subscribed to {} for body target tracking".format(self.config.body_joint_current_target_topic))

            if self.config.joint_trajectory_action_name:
                self.joint_trajectory_action_client = ActionClient(
                    self.robot_node,
                    JointTrajectoryAction,
                    self.config.joint_trajectory_action_name,
                )
                logger.info(
                    f"Created MoveJ action client: {self.config.joint_trajectory_action_name}"
                )

            if self.config.movel_action_name:
                self.movel_action_client = ActionClient(
                    self.robot_node,
                    ExecuteLinear,
                    self.config.movel_action_name,
                )
                logger.info(f"Created MOVL action client: {self.config.movel_action_name}")

            if self.config.movec_action_name:
                self.movec_action_client = ActionClient(
                    self.robot_node,
                    MovecUseIK,
                    self.config.movec_action_name,
                )
                logger.info(f"Created MOVC action client: {self.config.movec_action_name}")

            # Subscribe WBC current state for mode-aware command timing.
            if self.is_wbc:
                self.wbc_state_sub = self.robot_node.create_subscription(
                    WbcCurrentState,
                    "/ocs2_wbc_controller/current_state",
                    self._wbc_state_callback,
                    10
                )
                logger.info("✅ Subscribed to /ocs2_wbc_controller/current_state for WBC mode tracking")
            
            # Initialize TF buffer first (needed by ArmHandler)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.robot_node)
            
            # Create left arm handler (always created)
            self.left_arm_handler = ArmHandler(
                self.robot_node,
                ArmType.LEFT,
                self.config,
                self.send_fsm_command,
                self.get_fsm_state,
            )
            self.left_arm_handler.initialize()
            
            # Create right arm handler if dual-arm mode detected
            if is_dual_arm_detected and self.config.right_end_effector_pose_topic:
                self.right_arm_handler = ArmHandler(
                    self.robot_node,
                    ArmType.RIGHT,
                    self.config,
                    self.send_fsm_command,
                    self.get_fsm_state,
                )
                self.right_arm_handler.initialize()
                self.target_path_pub = self.robot_node.create_publisher(Path, "/target_path", 10)
                self.execute_path_client = self.robot_node.create_client(
                    ExecutePath, "execute_path"
                )
                self.dual_target_stamped_pub = self.robot_node.create_publisher(Path, "/dual_target/stamped", 10)
            
            # Create left gripper handler (if enabled and controller detected)
            if self.config.gripper_enabled and self.config.left_gripper_controller_name:
                self.left_gripper_handler = GripperHandler(
                    self.robot_node,
                    GripperType.LEFT,
                    self.config
                )
                self.left_gripper_handler.initialize()
                logger.info("Created left gripper handler")
            elif self.config.gripper_enabled:
                logger.debug("Left gripper handler not created: no controller detected")
            else:
                logger.debug("Left gripper handler not created: gripper_enabled=False")
            
            # Create right gripper handler
            if self.config.gripper_enabled and self.config.right_gripper_controller_name:
                self.right_gripper_handler = GripperHandler(
                    self.robot_node,
                    GripperType.RIGHT,
                    self.config
                )
                self.right_gripper_handler.initialize()
                logger.info("Created right gripper handler")
            elif self.config.gripper_enabled:
                logger.debug("Right gripper handler not created: no controller detected")
            else:
                logger.debug("Right gripper handler not created: gripper_enabled=False")
            
            self.fsm_command_pub = self.robot_node.create_publisher(Int32, "/fsm_command", 10)
            self.mode_command_pub = self.robot_node.create_publisher(String, "/mode_command", 10)
            
            if self.config.head_joint_controller_topic:
                self.head_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.head_joint_controller_topic, 10
                )
            
            if self.config.body_joint_controller_topic:
                self.body_joint_controller_pub = self.robot_node.create_publisher(
                    Float64MultiArray, self.config.body_joint_controller_topic, 10
                )

            if self.config.waist_lifting_topic:
                self.waist_lifting_pub = self.robot_node.create_publisher(
                    Float64, self.config.waist_lifting_topic, 10
                )

            if self.config.waist_lifting_command_topic:
                self.waist_lifting_command_pub = self.robot_node.create_publisher(
                    Float64, self.config.waist_lifting_command_topic, 10
                )

            if self.config.waist_turning_command_topic:
                self.waist_turning_command_pub = self.robot_node.create_publisher(
                    Float64, self.config.waist_turning_command_topic, 10
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

            self._try_init_nav2()

            time.sleep(1.0)
            self._connected = True
            logger.info("Connected to ROS 2 robot interface")
            
        except Exception as e:
            logger.error(f"Failed to connect to ROS 2 robot interface: {e}")
            self.disconnect()
            raise
    
    def _fsm_state_callback(self, msg: Int32) -> None:
        """Callback for FSM state topic (/fsm_state)."""
        try:
            state_code = int(msg.data)
            valid_state_codes = {1, 2, 3, 4}
            if state_code not in valid_state_codes:
                logger.debug(f"Ignored unknown FSM state code from /fsm_state: {state_code}")
                return

            self._current_fsm_state = state_code
            logger.debug(f"FSM state received (code={state_code})")
        except Exception as e:
            logger.error(f"Error in FSM state callback: {e}", exc_info=True)
    
    def _robot_description_callback(self, msg: String) -> None:
        """Callback for robot description (URDF) messages."""
        try:
            self.latest_robot_description = msg.data
            self._robot_description_received = True
            logger.debug(f"Robot description received (length: {len(msg.data) if msg.data else 0})")
        except Exception as e:
            logger.error(f"Error in robot description callback: {e}", exc_info=True)
    
    def _body_current_target_callback(self, msg: Float64MultiArray) -> None:
        """Callback for body current target joint messages."""
        try:
            self.body_current_target = list(msg.data) if msg.data else None
            logger.debug(f"Body current target updated: {self.body_current_target}")
        except Exception as e:
            logger.error(f"Error in body current target callback: {e}", exc_info=True)

    def _wbc_state_callback(self, msg: WbcCurrentState) -> None:
        """Callback for /ocs2_wbc_controller/current_state."""
        try:
            self.wbc_state = msg
        except Exception as e:
            logger.error(f"Error in WBC state callback: {e}", exc_info=True)

    def _normalize_body_mode(self, body_mode: Optional[str], body_pose: Optional[Pose]) -> Optional[str]:
        """Normalize/validate body mode and preserve backward compatibility."""
        desired_body_mode = body_mode.strip().upper() if body_mode else None
        if desired_body_mode is None and body_pose is not None:
            return "BODY_TRACKING"
        if desired_body_mode is None:
            return None
        if desired_body_mode not in self.BODY_MODE_TO_STATE:
            raise ValueError(
                "Invalid body_mode='{}', must be one of {}".format(
                    desired_body_mode, sorted(self.BODY_MODE_TO_STATE.keys())
                )
            )
        return desired_body_mode

    def _switch_body_mode_if_needed(self, desired_body_mode: Optional[str]) -> None:
        """Switch body mode only when current WBC state does not match target mode."""
        if desired_body_mode is None:
            return

        desired_state = self.BODY_MODE_TO_STATE[desired_body_mode]
        if self.wbc_state is not None and self.wbc_state.body_state == desired_state:
            return

        self.send_mode_command(self.BODY_MODE_TO_COMMAND[desired_body_mode])
        time.sleep(self.MODE_SWITCH_SETTLE_TIME_SEC)

    def _joint_state_callback(self, msg: JointState) -> None:
        """Callback for joint state messages."""
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
        
        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            msg_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        else:
            msg_timestamp = current_time
            logger.debug("JointState message has invalid timestamp, using system time as fallback")
        
        # Store raw joint state (quick operation)
        self.latest_joint_state = {
            "names": list(msg.name),
            "positions": list(msg.position),
            "velocities": list(msg.velocity),
            "efforts": list(msg.effort),
            "timestamp": msg_timestamp
        }
        self.last_joint_state_time = current_time
        self._had_joint_state = True
        
        # If we were disconnected due to timeout, restore connection when new data arrives
        if not self._connected and self.robot_node is not None:
            self._connected = True
            logger.info("Joint state data recovered - connection restored")
        
        # Update gripper position history OUTSIDE lock to avoid blocking
        self._update_gripper_position_history_from_joint_state(msg.name, msg.position)
        
        # Categorize joints OUTSIDE lock to avoid blocking (this is expensive)
        # Cache the result for get_joint_state(categorized=True)
        try:
            categorized = self._categorize_joints(
                msg.name,
                msg.position,
                msg.velocity,
                msg.effort
            )
            categorized['timestamp'] = msg_timestamp
            
            # Update cached categorized state (quick operation)
            self.latest_categorized_joint_state = categorized
        except Exception as e:
            logger.debug(f"Error categorizing joints: {e}")
        
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
                    # 与 WBC ``joint_names`` 分类一致：Galbot 等用 leg_joint* 表示底盘/腰链，归入 body
                    if "head" in name_lower:
                        category = "head"
                    elif "body" in name_lower or name_lower.startswith("leg_"):
                        category = "body"
                    else:
                        category = "other"
            else:
                # 检查是否是夹爪/灵巧手关节（包含 'gripper' 或 'hand'）
                is_gripper_or_hand = 'gripper' in name_lower or 'hand' in name_lower

                if is_gripper_or_hand:
                    category = 'gripper'
                elif 'head' in name_lower:
                    category = 'head'
                elif 'body' in name_lower or name_lower.startswith("leg_"):
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
    
    def _get_joint_state_ref(self, categorized: bool = False) -> Dict[str, Any] | None:
        """Get latest joint-state cache by reference (no copy, internal use only)."""
        if not self.is_connected or self.latest_joint_state is None:
            return None
        
        # Check timeout if enabled
        if self.config.joint_state_timeout > 0:
            current_time = time.time()
            if (current_time - self.last_joint_state_time) > self.config.joint_state_timeout:
                # Set connected to False when timeout detected
                if self._connected:
                    logger.warning("Joint state data is stale - setting connected to False")
                    self._connected = False
                return None
        
        if not categorized:
            return self.latest_joint_state
        
        # Return cached categorized state (already computed in callback)
        if self.latest_categorized_joint_state is not None:
            return self.latest_categorized_joint_state
        
        # Fallback: categorize on demand if cache is not available
        # (should not happen in normal operation)
        logger.debug("Categorized joint state cache not available, categorizing on demand")
        categories = self._categorize_joints(
            self.latest_joint_state['names'],
            self.latest_joint_state['positions'],
            self.latest_joint_state['velocities'],
            self.latest_joint_state.get('efforts', [])
        )
        categories['timestamp'] = self.latest_joint_state.get('timestamp', 0.0)
        return categories

    def get_joint_state(self, categorized: bool = False) -> Dict[str, Any] | None:
        """Get the latest joint state.
        
        Args:
            categorized: If True, returns joints categorized by body part.
                        Uses cached categorized state for better performance.
        
        Returns:
            Joint state dictionary, or None if not available or stale.
        """
        state_ref = self._get_joint_state_ref(categorized=categorized)
        if state_ref is None:
            return None
        # Keep public API defensive: return a copy to avoid external modifications.
        return state_ref.copy()
    
    def get_last_joint_state_time(self) -> Optional[float]:
        """Get the timestamp of when the last joint state message was received.
        
        This is the system time when the last message was received (not the message timestamp).
        Useful for detecting if joint state data has been updated, even when timeout is disabled.
        
        Returns:
            Timestamp in seconds (system time), or None if not connected or no joint state has been received yet.
        """
        if not self.is_connected:
            return None
        
        return self.last_joint_state_time if self.last_joint_state_time > 0.0 else None

    def get_body_current_target(self) -> Optional[List[float]]:
        """Get latest body current target."""
        if self.body_current_target is None:
            return None
        return list(self.body_current_target)

    def wait_for_movel_action_server(self, timeout: float = 5.0) -> bool:
        """Wait for the configured parameterized MOVL action server."""
        if self.movel_action_client is None:
            return False
        return self.movel_action_client.wait_for_server(timeout_sec=timeout)

    def wait_for_movec_action_server(self, timeout: float = 5.0) -> bool:
        """Wait for the configured parameterized MOVC action server."""
        if self.movec_action_client is None:
            return False
        return self.movec_action_client.wait_for_server(timeout_sec=timeout)

    def wait_for_joint_trajectory_action_server(self, timeout: float = 5.0) -> bool:
        """Wait for the configured parameterized MoveJ action server."""
        if self.joint_trajectory_action_client is None:
            return False
        return self.joint_trajectory_action_client.wait_for_server(timeout_sec=timeout)

    def _wait_for_future_done(self, future: Any, timeout: float) -> bool:
        """Wait for a rclpy future while this interface node is spun by its executor."""
        if future.done():
            return True
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout=max(0.0, timeout))

    def _send_motion_action_goal(
        self,
        action_client: ActionClient | None,
        goal_msg: Any,
        *,
        action_label: str,
        control_type: Optional[str] = "arm_joint",
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: float = 30.0,
        wait_for_server_timeout: float = 5.0,
    ) -> Any:
        """Send a motion action goal and return the action result object."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if action_client is None:
            raise ROS2NotConnectedError(f"{action_label} action client not initialized")
        if not action_client.wait_for_server(timeout_sec=wait_for_server_timeout):
            raise ROS2NotConnectedError(f"{action_label} action server is not available")

        if control_type is not None:
            self.auto_switch_fsm_for_control(control_type)

        def _feedback_adapter(feedback_msg: Any) -> None:
            if feedback_callback is not None:
                feedback_callback(feedback_msg.feedback)

        send_goal_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=_feedback_adapter if feedback_callback is not None else None,
        )
        if not self._wait_for_future_done(send_goal_future, wait_for_server_timeout):
            logger.error("%s action goal send timed out", action_label)
            return None

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            logger.error("%s action goal rejected", action_label)
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future_done(result_future, timeout):
            logger.error("%s action result timed out, canceling goal", action_label)
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future_done(cancel_future, 2.0)
            return None

        result_response = result_future.result()
        result = result_response.result
        message = getattr(result, "message", "")
        estimated_duration = getattr(result, "estimated_duration", None)
        actual_duration = getattr(result, "actual_duration", None)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            logger.error(
                "%s action failed: status=%s, message=%s, estimated_duration=%s, actual_duration=%s",
                action_label,
                result_response.status,
                message,
                estimated_duration,
                actual_duration,
            )
        else:
            logger.info(
                "%s action succeeded: message=%s, estimated_duration=%s, actual_duration=%s",
                action_label,
                message,
                estimated_duration,
                actual_duration,
            )
        return result

    def _convert_eef_pose_to_tcp(
        self,
        eef_pose_in_ref: Pose,
        eef_frame_name: str,
        tcp_frame_name: str,
    ) -> Pose:
        if eef_frame_name == tcp_frame_name:
            return eef_pose_in_ref
        tcp_in_eef = self.lookup_transform(eef_frame_name, tcp_frame_name)
        if tcp_in_eef is None:
            raise ValueError(
                f"无法查询 {tcp_frame_name} 在 {eef_frame_name} 下的变换，"
                "请确认 TF 树中存在该静态变换"
            )
        tcp_in_eef_translation = (
            tcp_in_eef.transform.translation.x,
            tcp_in_eef.transform.translation.y,
            tcp_in_eef.transform.translation.z,
        )
        tcp_in_eef_quaternion = (
            tcp_in_eef.transform.rotation.x,
            tcp_in_eef.transform.rotation.y,
            tcp_in_eef.transform.rotation.z,
            tcp_in_eef.transform.rotation.w,
        )
        eef_quaternion = (
            eef_pose_in_ref.orientation.x,
            eef_pose_in_ref.orientation.y,
            eef_pose_in_ref.orientation.z,
            eef_pose_in_ref.orientation.w,
        )
        offset_in_ref = rotate_vector_by_quat(tcp_in_eef_translation, eef_quaternion)
        tcp_quaternion = quat_multiply(eef_quaternion, tcp_in_eef_quaternion)
        tcp_pose = Pose()
        tcp_pose.position.x = eef_pose_in_ref.position.x + offset_in_ref[0]
        tcp_pose.position.y = eef_pose_in_ref.position.y + offset_in_ref[1]
        tcp_pose.position.z = eef_pose_in_ref.position.z + offset_in_ref[2]
        tcp_pose.orientation.x = tcp_quaternion[0]
        tcp_pose.orientation.y = tcp_quaternion[1]
        tcp_pose.orientation.z = tcp_quaternion[2]
        tcp_pose.orientation.w = tcp_quaternion[3]
        return tcp_pose

    @staticmethod
    def _pose_from_pose_like(pose: Pose | PoseStamped) -> Pose:
        return pose.pose if isinstance(pose, PoseStamped) else pose

    @staticmethod
    def _point_from_value(value: Point | Tuple[float, float, float]) -> Point:
        if isinstance(value, Point):
            return value
        return Point(x=float(value[0]), y=float(value[1]), z=float(value[2]))

    @staticmethod
    def _vector3_from_value(value: Vector3 | Tuple[float, float, float]) -> Vector3:
        if isinstance(value, Vector3):
            return value
        return Vector3(x=float(value[0]), y=float(value[1]), z=float(value[2]))

    @staticmethod
    def _set_optional_float_field(msg: Any, field_name: str, value: Optional[float]) -> None:
        if value is not None:
            setattr(msg, field_name, float(value))

    @staticmethod
    def _set_optional_string_field(msg: Any, field_name: str, value: Optional[str]) -> None:
        if value is not None:
            setattr(msg, field_name, str(value))

    @staticmethod
    def _optional_float_array(value: Any, length: int, field_name: str) -> Optional[List[float]]:
        if value is None:
            return None
        if isinstance(value, (int, float)):
            return [float(value)] * length
        values = [float(v) for v in value]
        if len(values) != length:
            raise ValueError(
                f"{field_name} length mismatch: got {len(values)}, expected {length}"
            )
        return values

    def execute_joint_trajectory_action(
        self,
        joint_names: List[str],
        waypoints: List[List[float]],
        *,
        time_mode: bool = True,
        total_time: Optional[float] = None,
        max_velocity: Any = None,
        max_acceleration: Any = None,
        max_jerk: Any = None,
        auto_switch_fsm: bool = True,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: float = 30.0,
        wait_for_server_timeout: float = 5.0,
    ) -> Any:
        """Execute parameterized MoveJ through ``JointTrajectory`` action.

        Args:
            joint_names: Joint names controlled by the action.
            waypoints: Position arrays. Each waypoint must match ``joint_names`` length.
            max_velocity/max_acceleration/max_jerk: Optional scalar or per-joint list.
            feedback_callback: Receives the action feedback object.

        Returns:
            The action result object, or ``None`` if rejected/timed out.
        """
        if not joint_names:
            raise ValueError("joint_names cannot be empty")
        if not waypoints:
            raise ValueError("waypoints cannot be empty")

        joint_count = len(joint_names)
        goal_msg = JointTrajectoryAction.Goal()
        goal_msg.joint_names = list(joint_names)

        velocity = self._optional_float_array(max_velocity, joint_count, "max_velocity")
        acceleration = self._optional_float_array(
            max_acceleration, joint_count, "max_acceleration"
        )
        jerk = self._optional_float_array(max_jerk, joint_count, "max_jerk")

        for i, positions in enumerate(waypoints):
            if len(positions) != joint_count:
                raise ValueError(
                    f"Waypoint {i} length mismatch: got {len(positions)}, expected {joint_count}"
                )
            waypoint = JointWaypoint()
            waypoint.position = [float(p) for p in positions]
            waypoint.time_mode = bool(time_mode)
            if total_time is not None:
                waypoint.total_time = float(total_time)
            if velocity is not None:
                waypoint.max_velocity = list(velocity)
            if acceleration is not None:
                waypoint.max_acceleration = list(acceleration)
            if jerk is not None:
                waypoint.max_jerk = list(jerk)
            goal_msg.waypoints.append(waypoint)

        return self._send_motion_action_goal(
            self.joint_trajectory_action_client,
            goal_msg,
            action_label="MoveJ",
            control_type="arm_joint" if auto_switch_fsm else None,
            feedback_callback=feedback_callback,
            timeout=timeout,
            wait_for_server_timeout=wait_for_server_timeout,
        )

    def execute_dual_arm_movej_action(
        self,
        left_arm_positions: List[float],
        right_arm_positions: List[float],
        *,
        duration: Optional[float] = None,
        time_mode: bool = True,
        left_joint_names: Optional[List[str]] = None,
        right_joint_names: Optional[List[str]] = None,
        max_velocity: Any = None,
        max_acceleration: Any = None,
        max_jerk: Any = None,
        auto_switch_fsm: bool = True,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: float = 30.0,
        wait_for_server_timeout: float = 5.0,
    ) -> Any:
        """Convenience wrapper for dual-arm parameterized MoveJ action."""
        if left_joint_names is None:
            left_joint_names = [
                "left_joint1", "left_joint2", "left_joint3", "left_joint4",
                "left_joint5", "left_joint6", "left_joint7",
            ]
        if right_joint_names is None:
            right_joint_names = [
                "right_joint1", "right_joint2", "right_joint3", "right_joint4",
                "right_joint5", "right_joint6", "right_joint7",
            ]
        joint_names = list(left_joint_names) + list(right_joint_names)
        positions = list(left_arm_positions) + list(right_arm_positions)
        return self.execute_joint_trajectory_action(
            joint_names,
            [positions],
            time_mode=time_mode,
            total_time=duration,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            max_jerk=max_jerk,
            auto_switch_fsm=auto_switch_fsm,
            feedback_callback=feedback_callback,
            timeout=timeout,
            wait_for_server_timeout=wait_for_server_timeout,
        )

    def execute_movel_action(
        self,
        arm_name: str,
        endpoint_pose: Pose | PoseStamped,
        *,
        # eef_frame_name: str,
        duration: float = 3.0,
        time_mode: bool = True,
        frame_id: Optional[str] = None,
        ik_type: Optional[str] = None,
        right_endpoint_pose: Optional[Pose | PoseStamped] = None,
        max_linear_velocity: Optional[float] = None,
        max_linear_acceleration: Optional[float] = None,
        max_linear_jerk: Optional[float] = None,
        max_angular_velocity: Optional[float] = None,
        max_angular_acceleration: Optional[float] = None,
        max_angular_jerk: Optional[float] = None,
        auto_switch_fsm: bool = True,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: float = 30.0,
        wait_for_server_timeout: float = 5.0,
    ) -> Any:
        """Execute parameterized MOVL through ``ExecuteLinear`` action.

        ``feedback_callback`` receives the action feedback object, not the wrapper
        message. The returned value is the action result object.
        """
        goal_msg = ExecuteLinear.Goal()
        linear = LinearMessage()
        linear.arm_name = arm_name
        linear.duration = float(duration)
        linear.time_mode = bool(time_mode)
        self._set_optional_string_field(linear, "frame_id", frame_id)
        self._set_optional_string_field(linear, "ik_type", ik_type)
        self._set_optional_float_field(linear, "max_linear_velocity", max_linear_velocity)
        self._set_optional_float_field(linear, "max_linear_acceleration", max_linear_acceleration)
        self._set_optional_float_field(linear, "max_linear_jerk", max_linear_jerk)
        self._set_optional_float_field(linear, "max_angular_velocity", max_angular_velocity)
        self._set_optional_float_field(linear, "max_angular_acceleration", max_angular_acceleration)
        self._set_optional_float_field(linear, "max_angular_jerk", max_angular_jerk)
        # linear.endpoint = self._convert_eef_pose_to_tcp(
        #     self._pose_from_pose_like(endpoint_pose),
        #     eef_frame_name,
        #     f"{arm_name}_eef",
        # )
        # if right_endpoint_pose is not None:
        #     linear.right_endpoint = self._pose_from_pose_like(right_endpoint_pose)
        # 直接赋值，不做坐标转换
        linear.endpoint = self._pose_from_pose_like(endpoint_pose)
        if right_endpoint_pose is not None:
            linear.right_endpoint = self._pose_from_pose_like(right_endpoint_pose)
        goal_msg.linear_params = linear
        return self._send_motion_action_goal(
            self.movel_action_client,
            goal_msg,
            action_label="MOVL",
            control_type="arm_joint" if auto_switch_fsm else None,
            feedback_callback=feedback_callback,
            timeout=timeout,
            wait_for_server_timeout=wait_for_server_timeout,
        )

    def execute_movec_action_three_point(
        self,
        arm_name: str,
        midpoint_pose: Pose | PoseStamped,
        endpoint_pose: Pose | PoseStamped,
        rotate_angle: float = 0.0,
        *,
        duration: float = 6.0,
        time_mode: bool = False,
        frame_id: Optional[str] = None,
        ik_type: Optional[str] = None,
        use_slerp_for_orientation: bool = False,
        right_midpoint_pose: Optional[Pose | PoseStamped] = None,
        right_endpoint_pose: Optional[Pose | PoseStamped] = None,
        right_rotate_angle: float = 0.0,
        max_linear_velocity: Optional[float] = None,
        max_linear_acceleration: Optional[float] = None,
        max_linear_jerk: Optional[float] = None,
        max_angular_velocity: Optional[float] = None,
        max_angular_acceleration: Optional[float] = None,
        max_angular_jerk: Optional[float] = None,
        auto_switch_fsm: bool = True,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: float = 60.0,
        wait_for_server_timeout: float = 5.0,
    ) -> Any:
        """Execute parameterized MOVC through ``MovecUseIK`` action using three-point mode."""
        goal_msg = MovecUseIK.Goal()
        circle = CircleMessage()
        circle.arm_name = arm_name
        circle.duration = float(duration)
        circle.time_mode = bool(time_mode)
        self._set_optional_string_field(circle, "frame_id", frame_id)
        self._set_optional_string_field(circle, "ik_type", ik_type)
        circle.use_three_point_method = True
        circle.use_slerp_for_orientation = bool(use_slerp_for_orientation)
        circle.midpoint = self._pose_from_pose_like(midpoint_pose)
        circle.endpoint = self._pose_from_pose_like(endpoint_pose)
        circle.rotate_angle = float(rotate_angle)
        self._set_optional_float_field(circle, "max_linear_velocity", max_linear_velocity)
        self._set_optional_float_field(circle, "max_linear_acceleration", max_linear_acceleration)
        self._set_optional_float_field(circle, "max_linear_jerk", max_linear_jerk)
        self._set_optional_float_field(circle, "max_angular_velocity", max_angular_velocity)
        self._set_optional_float_field(circle, "max_angular_acceleration", max_angular_acceleration)
        self._set_optional_float_field(circle, "max_angular_jerk", max_angular_jerk)
        if right_midpoint_pose is not None:
            circle.right_midpoint = self._pose_from_pose_like(right_midpoint_pose)
        if right_endpoint_pose is not None:
            circle.right_endpoint = self._pose_from_pose_like(right_endpoint_pose)
        circle.right_rotate_angle = float(right_rotate_angle)
        goal_msg.circle_params = circle
        return self._send_motion_action_goal(
            self.movec_action_client,
            goal_msg,
            action_label="MOVC",
            control_type="arm_joint" if auto_switch_fsm else None,
            feedback_callback=feedback_callback,
            timeout=timeout,
            wait_for_server_timeout=wait_for_server_timeout,
        )

    def execute_movec_action_parametric(
        self,
        arm_name: str,
        center: Point | Tuple[float, float, float],
        axis: Vector3 | Tuple[float, float, float],
        rotate_angle: float,
        *,
        endpoint_pose: Optional[Pose | PoseStamped] = None,
        duration: float = 6.0,
        time_mode: bool = False,
        frame_id: Optional[str] = None,
        ik_type: Optional[str] = None,
        use_slerp_for_orientation: bool = False,
        right_center: Optional[Point | Tuple[float, float, float]] = None,
        right_axis: Optional[Vector3 | Tuple[float, float, float]] = None,
        right_rotate_angle: float = 0.0,
        right_endpoint_pose: Optional[Pose | PoseStamped] = None,
        max_linear_velocity: Optional[float] = None,
        max_linear_acceleration: Optional[float] = None,
        max_linear_jerk: Optional[float] = None,
        max_angular_velocity: Optional[float] = None,
        max_angular_acceleration: Optional[float] = None,
        max_angular_jerk: Optional[float] = None,
        auto_switch_fsm: bool = True,
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: float = 60.0,
        wait_for_server_timeout: float = 5.0,
    ) -> Any:
        """Execute parameterized MOVC through ``MovecUseIK`` action using center/axis/angle mode."""
        goal_msg = MovecUseIK.Goal()
        circle = CircleMessage()
        circle.arm_name = arm_name
        circle.duration = float(duration)
        circle.time_mode = bool(time_mode)
        self._set_optional_string_field(circle, "frame_id", frame_id)
        self._set_optional_string_field(circle, "ik_type", ik_type)
        circle.use_three_point_method = False
        circle.use_slerp_for_orientation = bool(use_slerp_for_orientation)
        circle.center = self._point_from_value(center)
        circle.axis = self._vector3_from_value(axis)
        circle.rotate_angle = float(rotate_angle)
        self._set_optional_float_field(circle, "max_linear_velocity", max_linear_velocity)
        self._set_optional_float_field(circle, "max_linear_acceleration", max_linear_acceleration)
        self._set_optional_float_field(circle, "max_linear_jerk", max_linear_jerk)
        self._set_optional_float_field(circle, "max_angular_velocity", max_angular_velocity)
        self._set_optional_float_field(circle, "max_angular_acceleration", max_angular_acceleration)
        self._set_optional_float_field(circle, "max_angular_jerk", max_angular_jerk)
        circle.endpoint.position = Point(x=0.0, y=0.0, z=0.0)
        if endpoint_pose is not None:
            circle.endpoint.orientation = self._pose_from_pose_like(endpoint_pose).orientation
        else:
            circle.endpoint.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        if right_center is not None:
            circle.right_center = self._point_from_value(right_center)
        if right_axis is not None:
            circle.right_axis = self._vector3_from_value(right_axis)
        circle.right_rotate_angle = float(right_rotate_angle)
        circle.right_endpoint.position = Point(x=0.0, y=0.0, z=0.0)
        if right_endpoint_pose is not None:
            circle.right_endpoint.orientation = self._pose_from_pose_like(right_endpoint_pose).orientation
        elif arm_name == "both":
            circle.right_endpoint.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        goal_msg.circle_params = circle
        return self._send_motion_action_goal(
            self.movec_action_client,
            goal_msg,
            action_label="MOVC",
            control_type="arm_joint" if auto_switch_fsm else None,
            feedback_callback=feedback_callback,
            timeout=timeout,
            wait_for_server_timeout=wait_for_server_timeout,
        )

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

        self.auto_switch_fsm_for_control("arm_pose")
        
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
        
        # 清除旧的 current target，避免在收到新目标前误判为已到达
        if left_poses and self.left_arm_handler:
            self.left_arm_handler.latest_target_pose = None
        if right_poses and self.right_arm_handler:
            self.right_arm_handler.latest_target_pose = None
        
        self.target_path_pub.publish(path_msg)
        logger.info(f"Published target path with {len(left_poses)} left arm waypoints and {len(right_poses)} right arm waypoints (total: {len(path_msg.poses)})")
        logger.debug(f"Path frame_id: {path_msg.header.frame_id}")

    def execute_path(
        self,
        left_poses: List[Pose | PoseStamped],
        right_poses: List[Pose | PoseStamped],
        trajectory_duration: float = 0.0,
        frame_id: Optional[str] = None,
    ) -> bool:
        """Send unequal-waypoint dual-arm path via ExecutePath service."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")

        if not self.config.right_end_effector_target_topic:
            raise ROS2NotConnectedError(
                "ExecutePath requires dual-arm mode. Right end-effector target topic not configured."
            )

        if self.execute_path_client is None:
            raise ROS2NotConnectedError("ExecutePath client not initialized")

        if not self.execute_path_client.service_is_ready():
            raise ROS2NotConnectedError("ExecutePath service not available")

        self.auto_switch_fsm_for_control("arm_pose")

        def to_pose_stamped(pose: Pose | PoseStamped) -> PoseStamped:
            ps = PoseStamped()
            if frame_id is not None:
                ps.header.frame_id = frame_id
            elif isinstance(pose, PoseStamped):
                ps.header.frame_id = pose.header.frame_id
            else:
                ps.header.frame_id = ""
            ps.header.stamp = self.robot_node.get_clock().now().to_msg()
            ps.pose = pose.pose if isinstance(pose, PoseStamped) else pose
            return ps

        request = ExecutePath.Request()
        request.left_arm_path.poses = [to_pose_stamped(p) for p in left_poses]
        request.right_arm_path.poses = [to_pose_stamped(p) for p in right_poses]
        request.trajectory_duration = trajectory_duration

        if left_poses and self.left_arm_handler:
            self.left_arm_handler.latest_target_pose = None
        if right_poses and self.right_arm_handler:
            self.right_arm_handler.latest_target_pose = None

        future = self.execute_path_client.call_async(request)

        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=5.0):
            raise ROS2NotConnectedError("ExecutePath service call timed out")

        if future.result() is None:
            raise ROS2NotConnectedError("ExecutePath service call failed")

        response = future.result()
        logger.info(
            f"ExecutePath: success={response.success}, "
            f"duration={response.estimated_duration:.2f}s, msg={response.message}"
        )
        return response.success

    def send_dual_arm_target_stamped(
        self,
        left_pose: Pose,
        right_pose: Pose,
        frame_id: str = "arm_base",
        body_pose: Optional[Pose] = None,
        body_frame_id: str = "base_footprint",
        body_mode: Optional[str] = None,
    ) -> None:
        """Send dual-arm target poses to /dual_target/stamped topic.

        When ``body_pose`` is provided, an additional 3rd pose will be appended:
        ``[left, right, body]``. This matches PoseBasedReferenceManager dual-target
        format that supports both 2-pose and 3-pose messages.

        Args:
            body_mode: Optional body mode command, e.g. ``BODY_TRACKING``,
                ``BODY_FREE``, ``BODY_RELATIVE`` (alias: ``BODY_VERTICAL``),
                ``BODY_LOCK``, ``BODY_HEAD_COUPLED``.
                If set and mode is not BODY_TRACKING,
                body target pose will not be appended.
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        if not self.config.right_end_effector_target_topic:
            raise ROS2NotConnectedError("Dual arm target requires dual-arm mode. Right end-effector target topic not configured.")
        if self.dual_target_stamped_pub is None:
            raise ROS2NotConnectedError("Dual target stamped publisher not initialized")

        self.auto_switch_fsm_for_control("arm_pose")
        
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

        desired_body_mode = self._normalize_body_mode(body_mode, body_pose)
        self._switch_body_mode_if_needed(desired_body_mode)

        # Only BODY_TRACKING needs body target in dual_target/stamped.
        if body_pose is not None and desired_body_mode == "BODY_TRACKING":
            body_pose_stamped = PoseStamped()
            body_pose_stamped.header.stamp = stamp
            body_pose_stamped.header.frame_id = body_frame_id
            body_pose_stamped.pose = body_pose
            path_msg.poses.append(body_pose_stamped)
        
        # 清除旧的 current target，避免在收到新目标前误判为已到达
        if self.left_arm_handler:
            self.left_arm_handler.latest_target_pose = None
        if self.right_arm_handler:
            self.right_arm_handler.latest_target_pose = None
        
        self.dual_target_stamped_pub.publish(path_msg)
        if len(path_msg.poses) == 2:
            logger.info(f"Published dual arm target to /dual_target/stamped (frame_id: {frame_id})")
        else:
            logger.info(
                "Published dual-arm + body target to /dual_target/stamped "
                f"(arm_frame_id: {frame_id}, body_frame_id: {body_frame_id})"
            )
        logger.debug(f"Left arm pose: ({left_pose.position.x:.4f}, {left_pose.position.y:.4f}, {left_pose.position.z:.4f})")
        logger.debug(f"Right arm pose: ({right_pose.position.x:.4f}, {right_pose.position.y:.4f}, {right_pose.position.z:.4f})")
        if len(path_msg.poses) == 3:
            logger.debug(
                f"Body pose: ({body_pose.position.x:.4f}, {body_pose.position.y:.4f}, {body_pose.position.z:.4f})"
            )
    
    
    def send_fsm_command(self, command: int) -> None:
        """Send FSM command for state switching.
        
        Args:
            command: FSM command value
                - 1: HOME
                - 2: HOLD
                - 3: OCS2
                - 4: MOVEJ
                - 0, 100, etc.: Special commands (do not update internal state)
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.fsm_command_pub is None:
            logger.warning("FSM command publisher not initialized")
            return

        def _publish_and_wait(cmd: int) -> None:
            fsm_msg = Int32()
            fsm_msg.data = cmd
            self.fsm_command_pub.publish(fsm_msg)
            # 等待状态机完成切换，避免后续指令在旧状态下执行
            time.sleep(self.config.fsm_state_switch_settle_time)

        # 约束：HOME/OCS2/MOVEJ 只能由 HOLD 切换而来
        hold_required_targets = {FSM_HOME, FSM_OCS2, FSM_MOVEJ}
        current_state = self.get_fsm_state()
        if (
            command in hold_required_targets
            and current_state != FSM_HOLD
            and current_state != command
        ):
            logger.debug(
                f"FSM transition requires HOLD first: current={current_state}, target={command}"
            )
            _publish_and_wait(FSM_HOLD)

        _publish_and_wait(command)

    def send_mode_command(self, command: str) -> None:
        """Send mode command to /mode_command.

        Args:
            command: Mode command string, e.g. BASE_LOCK, BASE_UNLOCK.
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")

        if self.mode_command_pub is None:
            logger.warning("Mode command publisher not initialized")
            return

        mode_msg = String()
        mode_msg.data = str(command)
        self.mode_command_pub.publish(mode_msg)
        time.sleep(self.MODE_SWITCH_SETTLE_TIME_SEC)

    def auto_switch_fsm_state(self, target_state: int) -> bool:
        """Automatically switch FSM state only when needed.

        Args:
            target_state: Target FSM state code (1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ)

        Returns:
            True if a switch command was sent, False if already in target state.

        Raises:
            ValueError: If target_state is not a valid FSM state code.
            ROS2NotConnectedError: If interface is not connected.
        """
        valid_state_codes = {1, 2, 3, 4}
        if target_state not in valid_state_codes:
            raise ValueError(
                f"Invalid target_state={target_state}, must be one of {sorted(valid_state_codes)}"
            )

        current_state = self.get_fsm_state()
        if current_state == target_state:
            logger.debug(f"FSM already in target state {target_state}, skipping switch")
            return False

        self.send_fsm_command(target_state)
        logger.debug(f"Auto-switched FSM state: {current_state} -> {target_state}")
        return True

    def auto_switch_fsm_for_control(self, control_type: str) -> bool:
        """Auto-switch FSM state based on control category rules."""
        if not self._auto_switch_fsm_before_control:
            return False

        normalized_type = control_type.strip().lower()
        valid_types = {"arm_pose", "arm_joint", "body_joint", "head_joint", "other"}
        if normalized_type not in valid_types:
            raise ValueError(
                f"Invalid control_type='{control_type}', must be one of {sorted(valid_types)}"
            )

        if normalized_type == "other":
            return False

        if normalized_type == "arm_pose":
            return self.auto_switch_fsm_state(FSM_OCS2)

        if normalized_type == "arm_joint":
            return self.auto_switch_fsm_state(FSM_MOVEJ)

        if self.is_wbc:
            return self.auto_switch_fsm_state(FSM_MOVEJ)

        # Non-WBC: MOVEJ or OCS2 are both acceptable; OCS2 is preferred when switching is needed.
        current_state = self.get_fsm_state()
        if current_state in (FSM_MOVEJ, FSM_OCS2):
            logger.debug(
                f"FSM state {current_state} is acceptable for {normalized_type} in non-WBC mode"
            )
            return False
        return self.auto_switch_fsm_state(FSM_OCS2)
    
    def get_fsm_state(self) -> int:
        """Get current FSM state code.
        
        Returns:
            Current FSM state code:
            - 1: HOME
            - 2: HOLD
            - 3: OCS2
            - 4: MOVEJ
        """
        return self._current_fsm_state
    
    def get_robot_description(self) -> Optional[str]:
        """Get the latest robot description (URDF).
        
        Returns:
            Robot description string (URDF XML), or None if not received yet.
        """
        return self.latest_robot_description
    
    def has_robot_description(self) -> bool:
        """Check if robot description has been received.
        
        Returns:
            True if robot description has been received, False otherwise.
        """
        return self._robot_description_received
    
    def send_head_joint_positions(self, positions: List[float]) -> None:
        """Send target joint positions for head joints."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.head_joint_controller_pub is None:
            logger.warning("Head joint controller publisher not initialized. Set head_joint_controller_topic in config.")
            return

        self.auto_switch_fsm_for_control("head_joint")
        
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

        self.auto_switch_fsm_for_control("body_joint")
        
        msg = Float64MultiArray()
        msg.data = positions
        self.body_joint_controller_pub.publish(msg)
        logger.debug(f"Published body joint positions: {positions}")
        
        self.body_target_positions = positions.copy() if positions else None

    def send_waist_lifting_relative_position(self, position: float) -> None:
        """Send target relative position for waist lifting."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.waist_lifting_pub is None:
            logger.warning("Waist lifting publisher not initialized. Set waist_lifting_topic in config.")
            return

        self.auto_switch_fsm_for_control("body_joint")
        
        msg = Float64()
        msg.data = position
        self.waist_lifting_pub.publish(msg)
        logger.debug(f"Published waist lifting relative position: {position}")

    def send_waist_lifting_velocity_scale(self, velocity_scale: float) -> None:
        """Send target velocity for waist lifting."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.waist_lifting_pub is None:
            logger.warning("Waist lifting publisher not initialized. Set waist_lifting_command_topic in config.")
            return

        self.auto_switch_fsm_for_control("body_joint")
        
        velocity_scale = max(min(velocity_scale, 1),-1)
        
        msg = Float64()
        msg.data = velocity_scale
        self.waist_lifting_command_pub.publish(msg)
        logger.debug(f"Published waist lifting velocity_scale: {velocity_scale}")

    def send_waist_turning_velocity_scale(self, velocity_scale: float) -> None:
        """Send target velocity for waist turning."""
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")
        
        if self.waist_lifting_pub is None:
            logger.warning("Waist turning publisher not initialized. Set waist_turning_command_topic in config.")
            return

        self.auto_switch_fsm_for_control("body_joint")
        
        velocity_scale = max(min(velocity_scale, 1),-1)
        
        msg = Float64()
        msg.data = velocity_scale
        self.waist_turning_command_pub.publish(msg)
        logger.debug(f"Published waist turning velocity_scale: {velocity_scale}")
    
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

    def send_dual_arm_joint_positions(
        self,
        left_arm_positions: List[float],
        right_arm_positions: List[float],
        body_positions: Optional[List[float]] = None,
        head_positions: Optional[List[float]] = None,
    ) -> None:
        """发送双臂关节位置命令（MoveJ 模式）

        同时控制左臂和右臂的所有关节，发布到统一的 topic。
        对于 WBC 控制器（ocs2_wbc_controller），消息格式为
        ``body_joints + left_arm_joints + right_arm_joints + head_joints``，
        且顺序优先遵循 ``config.joint_names``。

        Args:
            left_arm_positions: 左臂关节位置列表（弧度）
            right_arm_positions: 右臂关节位置列表（弧度）
            body_positions: 躯干关节目标位置列表（弧度），仅 WBC 控制器生效。
                传入时直接使用该值；省略时从当前关节状态读取（保持躯干不动）。
            head_positions: 头部关节目标位置列表（弧度），仅 WBC 控制器生效。
                传入时直接使用该值；省略时从当前关节状态读取（保持头部不动）。

        Raises:
            ROS2NotConnectedError: 如果接口未连接或发布器未初始化
            ValueError: 如果双臂模式未启用或参数无效

        Example:
            ```python
            # 左臂7个关节，右臂7个关节
            left_positions = [0.0, 0.5, -1.57, 0.0, 1.57, 0.0, 0.0]
            right_positions = [0.0, -0.5, 1.57, 0.0, -1.57, 0.0, 0.0]
            # 只动手臂，躯干维持当前姿态
            interface.send_dual_arm_joint_positions(left_positions, right_positions)
            # 同时指定躯干目标（全身 MoveJ）
            body = [-1.353, -2.660, -1.307, 0.0]
            interface.send_dual_arm_joint_positions(left_positions, right_positions, body_positions=body)
            # 可选指定头部目标（未指定时保持当前头部姿态）
            head = [0.1, -0.1]
            interface.send_dual_arm_joint_positions(
                left_positions, right_positions, body_positions=body, head_positions=head
            )
            ```
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")

        if not (self.config.right_end_effector_target_topic or self.config.right_arm_joint_controller_topic):
            raise ValueError(
                "Dual-arm joint control requires right-side configuration. "
                "Neither right_end_effector_target_topic nor right_arm_joint_controller_topic is configured."
            )

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

        self.auto_switch_fsm_for_control("arm_joint")

        # 根据 topic 类型决定是否需要添加身体关节
        # ocs2_wbc_controller 需要：body_joints + left_arm_joints + right_arm_joints
        # ocs2_arm_controller 只需要：left_arm_joints + right_arm_joints
        unified_topic = self.config.unified_arm_joint_controller_topic
        is_wbc_controller = unified_topic and "ocs2_wbc_controller" in unified_topic

        if is_wbc_controller:
            include_body_in_unified = bool(self._wbc_has_body_joint_topic)
            include_head_in_unified = bool(self._wbc_has_head_joint_topic)
            configured_joint_names = list(self.config.joint_names or [])
            if not configured_joint_names:
                logger.warning(
                    "config.joint_names is empty for WBC controller, falling back to "
                    "body + left + right ordering (head joints will be ignored)"
                )
                combined_positions = list(left_arm_positions) + list(right_arm_positions)
                if include_body_in_unified:
                    resolved_body: Optional[List[float]] = body_positions
                    if resolved_body is None:
                        categorized_state = self.get_joint_state(categorized=True)
                        if categorized_state and categorized_state.get('body', {}).get('positions'):
                            resolved_body = list(categorized_state['body']['positions'])
                    if resolved_body is None:
                        logger.warning("Body joint positions not available, using zeros for WBC controller")
                        resolved_body = [0.0] * 4
                    combined_positions = list(resolved_body) + combined_positions
            else:
                body_joint_names: List[str] = []
                left_arm_joint_names: List[str] = []
                right_arm_joint_names: List[str] = []
                head_joint_names: List[str] = []
                ignored_joint_names: List[str] = []

                for joint_name in configured_joint_names:
                    name_lower = joint_name.lower()
                    if "gripper" in name_lower or "hand" in name_lower:
                        continue
                    if name_lower.startswith("left_"):
                        left_arm_joint_names.append(joint_name)
                    elif name_lower.startswith("right_"):
                        right_arm_joint_names.append(joint_name)
                    elif "body" in name_lower or name_lower.startswith("leg_"):
                        # Galbot 等底盘/下肢链常用 leg_joint*，在 WBC 合成里与 body 段同序位
                        body_joint_names.append(joint_name)
                    elif "head" in name_lower:
                        head_joint_names.append(joint_name)
                    else:
                        ignored_joint_names.append(joint_name)

                if ignored_joint_names:
                    logger.warning(
                        "Ignoring unsupported joint names in config.joint_names for WBC: %s",
                        ignored_joint_names,
                    )

                if len(left_arm_positions) != len(left_arm_joint_names):
                    raise ValueError(
                        "Left arm position count does not match config.joint_names: "
                        f"got {len(left_arm_positions)}, expected {len(left_arm_joint_names)}"
                    )
                if len(right_arm_positions) != len(right_arm_joint_names):
                    raise ValueError(
                        "Right arm position count does not match config.joint_names: "
                        f"got {len(right_arm_positions)}, expected {len(right_arm_joint_names)}"
                    )

                if (
                    include_body_in_unified
                    and body_positions is not None
                    and len(body_positions) != len(body_joint_names)
                ):
                    raise ValueError(
                        "Body position count does not match config.joint_names: "
                        f"got {len(body_positions)}, expected {len(body_joint_names)}"
                    )
                if (
                    include_head_in_unified
                    and head_positions is not None
                    and len(head_positions) != len(head_joint_names)
                ):
                    raise ValueError(
                        "Head position count does not match config.joint_names: "
                        f"got {len(head_positions)}, expected {len(head_joint_names)}"
                    )

                latest_state = self.get_joint_state(categorized=False)
                state_name_to_pos: Dict[str, float] = {}
                if latest_state:
                    names = latest_state.get("names", [])
                    positions = latest_state.get("positions", [])
                    for i, name in enumerate(names):
                        if i < len(positions):
                            state_name_to_pos[name] = positions[i]

                left_name_to_cmd = dict(zip(left_arm_joint_names, left_arm_positions))
                right_name_to_cmd = dict(zip(right_arm_joint_names, right_arm_positions))
                body_name_to_cmd = dict(zip(body_joint_names, body_positions or [])) if include_body_in_unified else {}
                head_name_to_cmd = dict(zip(head_joint_names, head_positions or [])) if include_head_in_unified else {}

                target_joint_names: List[str] = []
                if include_body_in_unified:
                    target_joint_names.extend(body_joint_names)
                target_joint_names.extend(left_arm_joint_names)
                target_joint_names.extend(right_arm_joint_names)
                if include_head_in_unified:
                    target_joint_names.extend(head_joint_names)
                ordered_joint_names: List[str] = list(target_joint_names)

                missing_in_state = [n for n in ordered_joint_names if n not in state_name_to_pos]
                if missing_in_state:
                    logger.warning(
                        "Some WBC joints are missing in latest joint_states, using config order fallback "
                        "and/or zeros for: %s",
                        missing_in_state,
                    )

                combined_positions = []
                for joint_name in ordered_joint_names:
                    if joint_name in left_name_to_cmd:
                        combined_positions.append(left_name_to_cmd[joint_name])
                    elif joint_name in right_name_to_cmd:
                        combined_positions.append(right_name_to_cmd[joint_name])
                    elif joint_name in body_name_to_cmd:
                        combined_positions.append(body_name_to_cmd[joint_name])
                    elif joint_name in head_name_to_cmd:
                        combined_positions.append(head_name_to_cmd[joint_name])
                    elif joint_name in state_name_to_pos:
                        combined_positions.append(state_name_to_pos[joint_name])
                    else:
                        logger.warning("Joint '%s' unavailable, using 0.0 for WBC command", joint_name)
                        combined_positions.append(0.0)

                expected_total = len(target_joint_names)
                if len(combined_positions) != expected_total:
                    raise ValueError(
                        "WBC command dimension mismatch: "
                        f"got {len(combined_positions)}, expected {expected_total}"
                    )

            logger.debug(
                "WBC controller: left=%s, right=%s, body=%s, head=%s, total=%s",
                len(left_arm_positions),
                len(right_arm_positions),
                len(body_positions) if body_positions is not None else "auto",
                len(head_positions) if head_positions is not None else "auto",
                len(combined_positions),
            )
        else:
            # ARM 控制器只需要左臂 + 右臂
            combined_positions = left_arm_positions + right_arm_positions
            logger.debug(f"ARM controller: left={len(left_arm_positions)}, right={len(right_arm_positions)}, total={len(combined_positions)}")

        msg = Float64MultiArray()
        msg.data = combined_positions
        self.unified_arm_joint_controller_pub.publish(msg)

    def _is_wbc_unified_joint_topic(self) -> bool:
        ut = self.config.unified_arm_joint_controller_topic or ""
        return "ocs2_wbc_controller" in ut

    def send_coordinated_joint_positions(
        self,
        body_positions: Optional[List[float]] = None,
        left_arm_positions: Optional[List[float]] = None,
        right_arm_positions: Optional[List[float]] = None,
        head_positions: Optional[List[float]] = None,
    ) -> None:
        """一次性下发关节空间目标（MoveJ 语义），在 WBC 合成与 split 栈之间自动选路。

        低层 API（``send_dual_arm_joint_positions``、``send_body_joint_positions``、
        ``ArmHandler.send_joint_positions`` 等）仍适合**并行/分时**组合（例如手臂
        MoVEL 与腰部 MoveJ 分开发）；本方法面向**单步**「能一次发就一次发」的编排。

        路由概要：

        - **WBC**（``unified_arm_joint_controller_topic`` 含 ``ocs2_wbc_controller``）且
          已连接双臂统一发布器、且配置为双臂模式：一律经
          ``send_dual_arm_joint_positions``；缺某一臂目标时从 ``/joint_states`` 读当前角
          作为 hold；仅躯干/头时双臂均 hold。
        - **非 WBC** 但存在统一臂 topic：双臂走 ``send_dual_arm_joint_positions``；若提供
          ``body_positions`` 且已初始化 ``body_joint_controller_pub``，再发躯干 topic
          （与「臂 unified + 腰 split」的旧栈兼容）。
        - **其它**：按可用的左右臂 handler 与 ``send_body_joint_positions`` 回退。

        ``head_positions`` 仅在 WBC 合成路径中交给 ``send_dual_arm_joint_positions``；
        非 WBC 时若配置了头部发布器，会在臂与躯干之后调用 ``send_head_joint_positions``。

        Raises:
            ROS2NotConnectedError: 未连接。
            ValueError: 四个列表均为空/缺省，或 WBC 下无法从状态补全缺失臂。
        """
        if not self.is_connected:
            raise ROS2NotConnectedError("ROS2RobotInterface is not connected")

        def _as_vec(xs: Optional[List[float]]) -> Optional[List[float]]:
            if xs is None or len(xs) == 0:
                return None
            return [float(x) for x in xs]

        body = _as_vec(body_positions)
        left = _as_vec(left_arm_positions)
        right = _as_vec(right_arm_positions)
        head = _as_vec(head_positions)

        if body is None and left is None and right is None and head is None:
            raise ValueError(
                "send_coordinated_joint_positions: at least one of body_positions, "
                "left_arm_positions, right_arm_positions, head_positions must be non-empty"
            )

        unified_topic = self.config.unified_arm_joint_controller_topic or ""
        using_wbc_controller = "ocs2_wbc_controller" in unified_topic
        using_arm_controller = "ocs2_arm_controller" in unified_topic
        has_unified = self.unified_arm_joint_controller_pub is not None
        dual_mode = bool(self.config.right_end_effector_target_topic or self.config.right_arm_joint_controller_topic)

        def _hold_arm(side: str) -> Optional[List[float]]:
            cat = self.get_joint_state(categorized=True) or {}
            p = (cat.get(f"{side}_arm") or {}).get("positions")
            if not p or len(p) < 7:
                return None
            return [float(x) for x in p[:7]]

        # ----- WBC：按 body/head 子 topic 能力决定 unified or split -----
        if using_wbc_controller and has_unified and dual_mode:
            wbc_body = body if self._wbc_has_body_joint_topic else None
            wbc_head = head if self._wbc_has_head_joint_topic else None
            if left and right:
                self.send_dual_arm_joint_positions(
                    left, right, body_positions=wbc_body, head_positions=wbc_head
                )
                if body is not None and not self._wbc_has_body_joint_topic:
                    if self.body_joint_controller_pub is not None:
                        self.send_body_joint_positions(body)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /body channel and "
                            "body_joint_controller_pub is not initialized; body command skipped"
                        )
                if head is not None and not self._wbc_has_head_joint_topic:
                    if self.head_joint_controller_pub is not None:
                        self.send_head_joint_positions(head)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /head channel and "
                            "head_joint_controller_pub is not initialized; head command skipped"
                        )
                return
            if left and not right:
                rh = _hold_arm("right")
                if not rh:
                    raise ValueError(
                        "send_coordinated_joint_positions (WBC): right arm omitted and "
                        "right arm positions unavailable from joint_states"
                    )
                self.send_dual_arm_joint_positions(
                    left, rh, body_positions=wbc_body, head_positions=wbc_head
                )
                if body is not None and not self._wbc_has_body_joint_topic:
                    if self.body_joint_controller_pub is not None:
                        self.send_body_joint_positions(body)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /body channel and "
                            "body_joint_controller_pub is not initialized; body command skipped"
                        )
                if head is not None and not self._wbc_has_head_joint_topic:
                    if self.head_joint_controller_pub is not None:
                        self.send_head_joint_positions(head)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /head channel and "
                            "head_joint_controller_pub is not initialized; head command skipped"
                        )
                return
            if right and not left:
                lh = _hold_arm("left")
                if not lh:
                    raise ValueError(
                        "send_coordinated_joint_positions (WBC): left arm omitted and "
                        "left arm positions unavailable from joint_states"
                    )
                self.send_dual_arm_joint_positions(
                    lh, right, body_positions=wbc_body, head_positions=wbc_head
                )
                if body is not None and not self._wbc_has_body_joint_topic:
                    if self.body_joint_controller_pub is not None:
                        self.send_body_joint_positions(body)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /body channel and "
                            "body_joint_controller_pub is not initialized; body command skipped"
                        )
                if head is not None and not self._wbc_has_head_joint_topic:
                    if self.head_joint_controller_pub is not None:
                        self.send_head_joint_positions(head)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /head channel and "
                            "head_joint_controller_pub is not initialized; head command skipped"
                        )
                return
            if body is not None or head is not None:
                lh = _hold_arm("left")
                rh = _hold_arm("right")
                if not lh or not rh:
                    raise ValueError(
                        "send_coordinated_joint_positions (WBC): only body/head given but "
                        "could not read both arms from joint_states"
                    )
                self.send_dual_arm_joint_positions(
                    lh, rh, body_positions=wbc_body, head_positions=wbc_head
                )
                if body is not None and not self._wbc_has_body_joint_topic:
                    if self.body_joint_controller_pub is not None:
                        self.send_body_joint_positions(body)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /body channel and "
                            "body_joint_controller_pub is not initialized; body command skipped"
                        )
                if head is not None and not self._wbc_has_head_joint_topic:
                    if self.head_joint_controller_pub is not None:
                        self.send_head_joint_positions(head)
                    else:
                        logger.warning(
                            "send_coordinated_joint_positions: WBC topic has no /head channel and "
                            "head_joint_controller_pub is not initialized; head command skipped"
                        )
                return

        # ----- ARM unified：分体控制（臂 unified，body/head split）-----
        if using_arm_controller and has_unified and dual_mode and left and right:
            self.send_dual_arm_joint_positions(left, right)
            if body is not None:
                if self.body_joint_controller_pub is not None:
                    self.send_body_joint_positions(body)
                else:
                    logger.warning(
                        "send_coordinated_joint_positions: body_positions set but "
                        "body_joint_controller_pub is not initialized; body command skipped"
                    )
            if head is not None and self.head_joint_controller_pub is not None:
                self.send_head_joint_positions(head)
            elif head is not None:
                logger.warning(
                    "send_coordinated_joint_positions: head_positions set but "
                    "head_joint_controller_pub is not initialized; head command skipped"
                )
            return

        # ----- 回退：分臂 handler + 躯干 -----
        sent_any = False
        if left and self.left_arm_handler is not None:
            self.left_arm_handler.send_joint_positions(left)
            sent_any = True
        if right and self.right_arm_handler is not None:
            self.right_arm_handler.send_joint_positions(right)
            sent_any = True
        if body is not None:
            if self.body_joint_controller_pub is not None:
                self.send_body_joint_positions(body)
                sent_any = True
            else:
                logger.warning(
                    "send_coordinated_joint_positions: body_positions set but "
                    "body_joint_controller_pub is not initialized; body command skipped"
                )
        if head is not None and self.head_joint_controller_pub is not None:
            self.send_head_joint_positions(head)
            sent_any = True
        elif head is not None:
            logger.warning(
                "send_coordinated_joint_positions: head_positions set but "
                "head_joint_controller_pub is not initialized; head command skipped"
            )

        if not sent_any:
            raise ValueError(
                "send_coordinated_joint_positions: could not send any targets "
                "(check dual-arm mode, unified topic, and arm handlers)"
            )

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

        self.auto_switch_fsm_for_control("arm_joint")
        
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
    
    def check_arrive(
        self,
        part: Optional[str] = None,
        position_threshold: Optional[float] = None,
        *,
        arm_pose_threshold: Optional[float] = None,
        arm_orient_threshold: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        """Check if head, body joints, arm poses, or grippers have arrived at target positions/poses.
        
        Args:
            part: 要检查的部分，可选值：None（所有部分）、'head'、'body'、'left_arm'、'right_arm'、
                  'left_gripper'、'right_gripper'、'arm'（单臂模式）、'gripper'（单臂模式）
            position_threshold: 关节位置阈值（仅用于 head 和 body），如果为 None 则使用默认值
            arm_pose_threshold: 笛卡尔末端位置容差（米），传给左右臂 ``check_arrival``；None 用 handler 默认
            arm_orient_threshold: 笛卡尔末端姿态角度容差（度）；None 用 handler 默认
        
        Returns:
            包含到达状态和距离信息的字典，如果未连接则返回 None
        """
        if not self.is_connected:
            return None
        
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
            arm_result = self.left_arm_handler.check_arrival(arm_pose_threshold, arm_orient_threshold)
            if part == 'left_arm':
                return arm_result
            result['left_arm' if is_dual_arm else 'arm'] = arm_result
        
        if is_dual_arm and (part is None or part == 'right_arm'):
            right_arm_result = self.right_arm_handler.check_arrival(arm_pose_threshold, arm_orient_threshold)
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

    def wait_until_arrive(
        self,
        part: str = "arm",
        timeout: float = 3.0,
        poll_period: float = 0.05,
        position_threshold: Optional[float] = None,
        *,
        arm_pose_threshold: Optional[float] = None,
        arm_orient_threshold: Optional[float] = None,
        time_now_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        wall_timeout_guard: Optional[float] = None,
        on_poll: Optional[Callable[[Optional[Dict[str, Any]], float], None]] = None,
    ) -> Dict[str, Any]:
        """Wait until the specified part arrives at target.

        This method wraps ``check_arrive`` with a polling loop and timeout control.
        It prefers arrival checks over fixed sleeps, while still providing a timeout
        safeguard for unstable communication or unreachable targets.

        Args:
            part: Part name accepted by ``check_arrive`` (e.g. ``arm``, ``gripper``,
                ``left_arm``, ``left_gripper``).
            timeout: Maximum wait time in seconds.
            poll_period: Polling interval in seconds.
            position_threshold: Optional threshold forwarded to ``check_arrive`` (head/body 关节)。
            arm_pose_threshold: 左右臂笛卡尔到位：位置阈值（米），见 ``check_arrive``。
            arm_orient_threshold: 左右臂笛卡尔到位：姿态角度阈值（度），见 ``check_arrive``。
            time_now_fn: Optional custom clock function. Use this when timeout should
                follow simulation time instead of wall time.
            sleep_fn: Optional sleep function paired with ``time_now_fn``.
            wall_timeout_guard: Optional wall-time guard (seconds) to prevent hard
                hangs when simulation clock is paused forever.

        Returns:
            Dict containing:
            - ``arrived`` (bool): Whether target was reached.
            - ``elapsed`` (float): Elapsed wait time in seconds.
            - ``result`` (dict | None): Last result from ``check_arrive``.
        """
        now_fn = time_now_fn or time.monotonic
        wait_fn = sleep_fn or time.sleep

        if timeout <= 0.0:
            result = self.check_arrive(
                part=part,
                position_threshold=position_threshold,
                arm_pose_threshold=arm_pose_threshold,
                arm_orient_threshold=arm_orient_threshold,
            )
            return {"arrived": bool(result and result.get("arrived", False)), "elapsed": 0.0, "result": result}

        start = now_fn()
        wall_start = time.monotonic()
        last_result: Optional[Dict[str, Any]] = None
        while (now_fn() - start) <= timeout:
            result = self.check_arrive(
                part=part,
                position_threshold=position_threshold,
                arm_pose_threshold=arm_pose_threshold,
                arm_orient_threshold=arm_orient_threshold,
            )
            if isinstance(result, dict):
                last_result = result
                if on_poll is not None:
                    try:
                        on_poll(result, now_fn() - start)
                    except Exception:
                        pass
                if result.get("arrived", False):
                    return {
                        "arrived": True,
                        "elapsed": now_fn() - start,
                        "result": result,
                    }
            elif on_poll is not None:
                try:
                    on_poll(None, now_fn() - start)
                except Exception:
                    pass
            if wall_timeout_guard is not None and (time.monotonic() - wall_start) > wall_timeout_guard:
                break
            wait_fn(max(0.0, poll_period))

        return {
            "arrived": False,
            "elapsed": now_fn() - start,
            "result": last_result,
        }

    def _extract_arm_positions_from_joint_state(
        self,
        joint_state: Dict[str, Any],
        *,
        categorized: bool,
    ) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """Extract left/right arm joint arrays from a joint-state snapshot."""
        if categorized:
            left_positions = joint_state.get("left_arm", {}).get("positions")
            right_positions = joint_state.get("right_arm", {}).get("positions")
            if left_positions is None:
                left_positions = joint_state.get("arm", {}).get("positions")
            left = list(left_positions) if left_positions else None
            right = list(right_positions) if right_positions else None
            return left, right

        names = joint_state.get("names") or []
        positions = joint_state.get("positions") or []
        if not names or not positions or len(names) != len(positions):
            return None, None
        left: List[float] = []
        right: List[float] = []
        for name, pos in zip(names, positions):
            n = str(name).lower()
            if "gripper" in n or "hand" in n or "head" in n or "body" in n:
                continue
            if "joint" not in n:
                continue
            if n.startswith("left_"):
                left.append(float(pos))
            elif n.startswith("right_"):
                right.append(float(pos))
        return (left or None), (right or None)

    def _get_current_arm_joint_positions(self) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """Get latest left/right arm joints from categorized or raw state."""
        try:
            categorized_state = self._get_joint_state_ref(categorized=True) or {}
            left, right = self._extract_arm_positions_from_joint_state(categorized_state, categorized=True)
            if left is not None or right is not None:
                return left, right
        except Exception:
            pass
        try:
            raw_state = self._get_joint_state_ref(categorized=False) or {}
            return self._extract_arm_positions_from_joint_state(raw_state, categorized=False)
        except Exception:
            return None, None

    def wait_until_joint_arrive(
        self,
        *,
        left_target_positions: Optional[List[float]] = None,
        right_target_positions: Optional[List[float]] = None,
        body_target_positions: Optional[List[float]] = None,
        timeout: float = 3.0,
        poll_period: float = 0.05,
        joint_tolerance: float = 0.03,
        time_now_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        on_poll: Optional[Callable[[Dict[str, Any], float], None]] = None,
    ) -> Dict[str, Any]:
        """Wait until joints reach target positions (MoveJ-friendly).

        Checks left arm, right arm, and optionally body joints.
        All specified groups must be within ``joint_tolerance`` to be considered arrived.
        """
        now_fn = time_now_fn or time.monotonic
        wait_fn = sleep_fn or time.sleep

        if left_target_positions is None and right_target_positions is None and body_target_positions is None:
            return {
                "arrived": False,
                "elapsed": 0.0,
                "left_error_max_abs": None,
                "right_error_max_abs": None,
                "body_error_max_abs": None,
                "reason": "no_target",
            }

        def _max_abs_error(current: Optional[List[float]], target: Optional[List[float]]) -> Optional[float]:
            if target is None:
                return 0.0
            if current is None or len(current) != len(target) or len(target) == 0:
                return None
            return max(abs(float(c) - float(t)) for c, t in zip(current, target))

        def _get_body_positions() -> Optional[List[float]]:
            try:
                state = self._get_joint_state_ref(categorized=True) or {}
                positions = state.get("body", {}).get("positions")
                return list(positions) if positions else None
            except Exception:
                return None

        start = now_fn()
        last_result: Dict[str, Any] = {
            "arrived": False,
            "elapsed": 0.0,
            "left_error_max_abs": None,
            "right_error_max_abs": None,
            "body_error_max_abs": None,
            "left_current_len": 0,
            "right_current_len": 0,
            "left_target_len": len(left_target_positions) if left_target_positions is not None else 0,
            "right_target_len": len(right_target_positions) if right_target_positions is not None else 0,
            "reason": "timeout",
        }

        while (now_fn() - start) <= timeout:
            left_current, right_current = self._get_current_arm_joint_positions()
            body_current = _get_body_positions() if body_target_positions is not None else None

            left_err  = _max_abs_error(left_current,  left_target_positions)
            right_err = _max_abs_error(right_current, right_target_positions)
            body_err  = _max_abs_error(body_current,  body_target_positions)

            left_ok  = (left_err  is not None and left_err  <= joint_tolerance) if left_target_positions  is not None else True
            right_ok = (right_err is not None and right_err <= joint_tolerance) if right_target_positions is not None else True
            body_ok  = (body_err  is not None and body_err  <= joint_tolerance) if body_target_positions  is not None else True

            elapsed = now_fn() - start
            all_ok = bool(left_ok and right_ok and body_ok)
            last_result = {
                "arrived": all_ok,
                "elapsed": elapsed,
                "left_error_max_abs": left_err,
                "right_error_max_abs": right_err,
                "body_error_max_abs": body_err,
                "left_current_len": len(left_current) if left_current is not None else 0,
                "right_current_len": len(right_current) if right_current is not None else 0,
                "left_target_len": len(left_target_positions) if left_target_positions is not None else 0,
                "right_target_len": len(right_target_positions) if right_target_positions is not None else 0,
                "reason": "ok" if all_ok else "waiting",
            }
            if on_poll is not None:
                try:
                    on_poll(last_result, elapsed)
                except Exception:
                    pass
            if all_ok:
                return last_result
            wait_fn(max(0.0, poll_period))

        return last_result
    
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

    # ============================================================================
    # Nav2 导航支持
    # ============================================================================

    def _try_init_nav2(self) -> bool:
        """尝试检测并初始化 Nav2 NavigateToPose action client。

        检测条件（全部满足才启用）：
          1. ``config.nav_enabled`` 不为 ``False``
          2. ``nav2_msgs`` 已安装（否则 ImportError 静默跳过）
          3. ROS 图中存在名称含 ``controller_server`` 的节点，或 ``config.nav_enabled=True`` 强制启用

        返回 ``True`` 表示导航已启用。
        """
        if self.config.nav_enabled is False:
            logger.info("Nav2 navigation disabled by config (nav_enabled=False)")
            return False

        try:
            from nav2_msgs.action import NavigateToPose  # type: ignore[import]
            from rclpy.action import ActionClient as _ActionClient  # type: ignore[import]
        except ImportError:
            logger.info("nav2_msgs not installed — navigation support disabled")
            return False

        if self.config.nav_enabled is not True:
            # 自动检测：检查 controller_server 是否在运行
            try:
                nodes = self.list_nodes()
                nav2_running = any(
                    "controller_server" in (n.get("name") or "") for n in nodes
                )
            except Exception as e:
                logger.warning(f"Nav2 node detection failed: {e}")
                nav2_running = False

            if not nav2_running:
                logger.info(
                    "Nav2 not detected (no controller_server node running) — navigation disabled"
                )
                return False

        server = self.config.nav_action_server
        self._nav_action_client = _ActionClient(
            self.robot_node, NavigateToPose, server
        )
        self._nav_enabled = True
        logger.info(f"✅ Nav2 detected — NavigateToPose action client ready (server={server!r})")
        return True

    @property
    def nav_enabled(self) -> bool:
        """True 表示 Nav2 已检测到并可用。"""
        return self._nav_enabled

    def send_nav_goal(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        frame_id: str = "map",
    ) -> None:
        """向 Nav2 发送导航目标（非阻塞，立即返回）。

        与 ``send_end_effector_target`` 对称：发出后即返回，
        用 ``check_nav_arrived`` 查询状态，或 ``wait_nav_arrived`` 阻塞等待。

        Args:
            x: 目标位置 X（米，map 坐标系）。
            y: 目标位置 Y（米，map 坐标系）。
            yaw: 目标朝向（弧度，绕 Z 轴）。
            frame_id: 目标坐标所在的 TF 帧，默认 ``"map"``。

        Raises:
            RuntimeError: Nav2 未启用时抛出。
        """
        if not self._nav_enabled or self._nav_action_client is None:
            raise RuntimeError(
                "Navigation not available — nav2_msgs not installed or "
                "controller_server not detected. Set nav_enabled=True in config to force-enable."
            )

        from geometry_msgs.msg import PoseStamped
        import math

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.robot_node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        half = yaw / 2.0
        goal_pose.pose.orientation.z = math.sin(half)
        goal_pose.pose.orientation.w = math.cos(half)

        try:
            from nav2_msgs.action import NavigateToPose  # type: ignore[import]
        except ImportError:
            raise RuntimeError("nav2_msgs not available")

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # 重置上一次的状态
        self._nav_goal_handle = None
        self._nav_result_future = None

        self._nav_goal_future = self._nav_action_client.send_goal_async(nav_goal)
        logger.info(f"Nav goal sent → x={x:.3f} y={y:.3f} yaw={yaw:.3f} frame={frame_id!r}")

    def check_nav_arrived(self) -> Optional[bool]:
        """非阻塞查询当前导航状态。

        与 ``check_arrive`` 对称：可在任意时刻轮询，不阻塞调用线程。

        Returns:
            ``None``  — 尚未发送目标，或 goal 仍在被 action server 接受中，或仍在导航。
            ``True``  — 导航成功到达（``STATUS_SUCCEEDED``）。
            ``False`` — 导航失败或被中止（ABORTED / CANCELED）。
        """
        if self._nav_goal_future is None:
            return None

        # 等待 goal 被接受
        if not self._nav_goal_future.done():
            return None

        if self._nav_goal_handle is None:
            goal_handle = self._nav_goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                logger.warning("Nav2 goal was rejected by action server")
                return False
            self._nav_goal_handle = goal_handle
            self._nav_result_future = goal_handle.get_result_async()

        if self._nav_result_future is None or not self._nav_result_future.done():
            return None

        # 解析结果
        try:
            from action_msgs.msg import GoalStatus  # type: ignore[import]
            result_response = self._nav_result_future.result()
            status = result_response.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                logger.info("Nav2 navigation SUCCEEDED")
                return True
            else:
                logger.warning(f"Nav2 navigation ended with status={status}")
                return False
        except Exception as e:
            logger.warning(f"Nav2 result parse error: {e}")
            return False

    def wait_nav_arrived(
        self,
        timeout: float = 60.0,
        poll_period: float = 0.1,
        time_now_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
    ) -> bool:
        """阻塞等待导航完成（与 ``wait_until_arrive`` 对称）。

        Args:
            timeout: 最大等待时间（秒）。
            poll_period: 轮询间隔（秒）。
            time_now_fn: 自定义时钟函数（仿真时间场景使用）。
            sleep_fn: 自定义 sleep 函数，与 ``time_now_fn`` 配套使用。

        Returns:
            ``True`` 表示成功到达，``False`` 表示超时或失败。
        """
        now_fn = time_now_fn or time.monotonic
        wait_fn = sleep_fn or time.sleep

        start = now_fn()
        while (now_fn() - start) <= timeout:
            status = self.check_nav_arrived()
            if status is True:
                return True
            if status is False:
                return False
            wait_fn(max(0.0, poll_period))

        logger.warning(f"Navigation timed out after {timeout:.1f}s")
        return False

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        frame_id: str = "map",
        timeout: float = 60.0,
        poll_period: float = 0.1,
        time_now_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
    ) -> bool:
        """发送导航目标并阻塞等待到达（send_nav_goal + wait_nav_arrived 的便捷组合）。

        顺序任务场景使用此方法；并行任务场景请分别调用
        ``send_nav_goal`` 和 ``wait_nav_arrived``。

        Returns:
            ``True`` 表示成功到达，``False`` 表示超时或失败。
        """
        self.send_nav_goal(x, y, yaw, frame_id=frame_id)
        return self.wait_nav_arrived(
            timeout=timeout,
            poll_period=poll_period,
            time_now_fn=time_now_fn,
            sleep_fn=sleep_fn,
        )

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
        
        if self.fsm_state_sub:
            self.fsm_state_sub.destroy()
            self.fsm_state_sub = None
        
        if self.robot_description_sub:
            self.robot_description_sub.destroy()
            self.robot_description_sub = None

        if self.body_current_target_sub:
            self.body_current_target_sub.destroy()
            self.body_current_target_sub = None

        if self.wbc_state_sub:
            self.wbc_state_sub.destroy()
            self.wbc_state_sub = None
        
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

        if self.execute_path_client:
            self.execute_path_client.destroy()
            self.execute_path_client = None

        if self.joint_trajectory_action_client:
            self.joint_trajectory_action_client.destroy()
            self.joint_trajectory_action_client = None

        if self.movel_action_client:
            self.movel_action_client.destroy()
            self.movel_action_client = None

        if self.movec_action_client:
            self.movec_action_client.destroy()
            self.movec_action_client = None

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

        if self.mode_command_pub:
            self.mode_command_pub.destroy()
            self.mode_command_pub = None
        
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

        if self.waist_lifting_command_pub:
            self.waist_lifting_command_pub.destroy()
            self.waist_lifting_command_pub = None

        if self.waist_turning_command_pub:
            self.waist_turning_command_pub.destroy()
            self.waist_turning_command_pub = None

        if self._nav_action_client is not None:
            self._nav_action_client.destroy()
            self._nav_action_client = None
        self._nav_enabled = False
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None

        if self.robot_node:
            self.robot_node.destroy_node()
            self.robot_node = None
        
        if self.tf_listener is not None:
            self.tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer = None
        
        self.latest_joint_state = None
        self.latest_categorized_joint_state = None
        self.wbc_state = None

        
        logger.info("Disconnected from ROS 2 robot interface")
