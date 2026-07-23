"""
ROS 2 机器人接口配置

ROS 2 机器人接口的配置类。
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Sequence


class ControlType(Enum):
    """机器人控制类型"""
    CARTESIAN_POSE = "cartesian_pose"


@dataclass
class ROS2RobotInterfaceConfig:
    """Configuration for ROS 2 robot interface.
    
    此配置类独立于 LeRobot，可在任何 ROS 2 环境中使用。
    
    Example:
        ```python
        from ros2_robot_interface import ROS2RobotInterfaceConfig, ControlType
        
        config = ROS2RobotInterfaceConfig(
            joint_states_topic="/joint_states",
            end_effector_pose_topic="/left_current_pose",
            end_effector_target_topic="/left_target",
            control_type=ControlType.CARTESIAN_POSE,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )
        ```
    """
    
    # ============================================================================
    # ROS 2 话题配置（必需，手动配置）
    # ============================================================================
    joint_states_topic: str = "/joint_states"  # 【必需】关节状态订阅话题
    end_effector_pose_topic: str = "/left_current_pose"  # 【必需】左臂末端执行器当前位姿话题
    end_effector_target_topic: str = "/left_target"  # 【必需】左臂末端执行器目标位姿话题
    
    # ============================================================================
    # 双臂配置（可选，可自动检测）
    # ============================================================================
    # 【可自动检测】系统会在 connect() 时自动检测这些话题，如果检测到会自动设置
    # 也可以手动配置这些话题来显式启用双臂模式
    right_end_effector_pose_topic: str | None = None  # 【可自动检测】右臂末端执行器位姿话题，例如 "/right_current_pose"
    right_end_effector_target_topic: str | None = None  # 【可自动检测】右臂末端执行器目标话题，例如 "/right_target"
    right_gripper_command_topic: str | None = None  # 【可自动检测】右臂夹爪控制话题，例如 "/right_gripper_joint/position_command"
    
    # ============================================================================
    # 目标位置订阅话题（可选，可自动检测）
    # ============================================================================
    # 【可自动检测】用于从话题订阅获取目标位置，而不是从内部存储获取
    # 如果配置了这些话题，check_arrival 将从订阅的话题获取目标位置
    end_effector_current_target_topic: str | None = None  # 【可自动检测】左臂当前目标位置话题，例如 "/left_current_target"
    right_end_effector_current_target_topic: str | None = None  # 【可自动检测】右臂当前目标位置话题，例如 "/right_current_target"
    
    # ============================================================================
    # 头部和身体关节控制器话题（可选，可自动检测）
    # ============================================================================
    # 【可自动检测】系统会在 connect() 时自动检测这些话题，如果检测到会自动设置
    # 也可以手动配置
    head_joint_controller_topic: str | None = None  # 【可自动检测】头部关节；WBC: /ocs2_wbc_controller/target_joint_position/head，split: /head_joint_controller/target_joint_position
    body_joint_controller_topic: str | None = None  # 【可自动检测】躯干关节；WBC: /ocs2_wbc_controller/target_joint_position/body，split: /body_joint_controller/target_joint_position
    left_hand_joint_controller_topic: str | None = None  # 【可自动检测】左灵巧手关节控制器话题，例如 "/left_hand_controller/target_joint_position"
    right_hand_joint_controller_topic: str | None = None  # 【可自动检测】右灵巧手关节控制器话题，例如 "/right_hand_controller/target_joint_position"
    waist_lifting_topic: str | None = None  # 【可自动检测】腰部升降相对位置控制话题，例如 "/body_joint_controller/waist_lifting"
    waist_lifting_pose_relative_topic: str | None = None  # 【可自动检测】腰部局部 x/z/phi 相对移动话题，例如 "/body_joint_controller/waist_lifting_pose_relative" 或 "/ocs2_wbc_controller/waist_lifting_pose_relative"
    waist_lifting_pose_absolute_topic: str | None = None  # 【可自动检测】腰部 x/z/phi 绝对目标话题，例如 "/body_joint_controller/waist_lifting_pose_absolute" 或 "/ocs2_wbc_controller/waist_lifting_pose_absolute"
    waist_lifting_command_topic: str | None = None  # 【可自动检测】腰部升降速度控制话题，例如 "/body_joint_controller/waist_lifting_command"
    waist_turning_command_topic: str | None = None  # 【可自动检测】腰部旋转速度控制话题，例如 "/body_joint_controller/waist_turning_command"
    body_joint_current_target_topic: str | None = None  # 【可自动检测】腰部关节控制器话题，例如 "/body_joint_controller/target_joint_position"
    body_current_pose_topic: str | None = None  # 【可自动检测】body 当前笛卡尔位姿话题（PoseStamped），例如 "/body_current_pose"，用于 body pose 到位判定
    body_current_target_pose_topic: str | None = None  # 【可自动检测】body 目标笛卡尔位姿话题（PoseStamped），例如 "/body_current_target"，用于 body pose 到位判定
    
    # ============================================================================
    # 手臂关节控制器话题（自动检测，无需手动配置）
    # ============================================================================
    # 【自动检测】系统会在 connect() 时自动检测这些话题，不应手动设置
    left_arm_joint_controller_topic: str | None = None  # 【自动检测或可手动】左臂/单臂链关节 MoveJ 话题；单臂 OCS2 常为 /ocs2_arm_controller/target_joint_position
    right_arm_joint_controller_topic: str | None = None  # 【自动检测】右臂关节控制器话题（双臂模式时自动检测）
    unified_arm_joint_controller_topic: str | None = None  # 【自动检测】统一双臂关节控制器话题（如 /ocs2_wbc_controller/target_joint_position 或 /ocs2_arm_controller/target_joint_position）

    # ============================================================================
    # 参数化笛卡尔轨迹 Action（可选，手动配置）
    # ============================================================================
    joint_trajectory_action_name: str | None = "/ocs2_arm_controller/joint_trajectory_with_para"  # 带参数/反馈的 MoveJ action
    movel_action_name: str | None = "/ocs2_arm_controller/execute_linear"  # 带参数/反馈的 MOVEL action
    movec_action_name: str | None = "/ocs2_arm_controller/execute_circle_use_ik"  # 带参数/反馈的 MOVEC action
    waist_lifting_pose_action_name: str | None = None  # 【可自动检测】腰部位姿 action，例如 "/ocs2_wbc_controller/waist_lifting_pose" 或 "/body_joint_controller/waist_lifting_pose"
    
    # ============================================================================
    # 关节配置（未使用，预留）
    # ============================================================================
    # 【未使用】此参数目前未被代码使用，代码实际使用的是从 ROS 2 topic 接收到的关节名称
    # 保留此参数用于未来可能的验证或过滤功能
    joint_names: list[str] = field(default_factory=lambda: [
                    "body_joint1",
                    "body_joint2",
                    "body_joint3",
                    "body_joint4",
                    "left_joint1",
                    "left_joint2",
                    "left_joint3",
                    "left_joint4",
                    "left_joint5",
                    "left_joint6",
                    "left_joint7",
                    "right_joint1",
                    "right_joint2",
                    "right_joint3",
                    "right_joint4",
                    "right_joint5",
                    "right_joint6",
                    "right_joint7",
                    "head_joint1",
                    "head_joint2",      
    ])  # 【未使用】关节名称列表（预留，目前代码使用从 /joint_states topic 接收的实际关节名称）
    
    # ============================================================================
    # 夹爪配置（手动配置）
    # ============================================================================
    gripper_enabled: bool = True  # 【可选】是否启用夹爪控制
    gripper_joint_name: str = "gripper_joint"  # 【未使用】夹爪关节名称（预留，目前代码使用从 /joint_states topic 接收的实际关节名称）
    gripper_state_topic: str = "/gripper_state"  # 【未使用】夹爪状态订阅话题（预留，目前未实现单独的状态话题订阅）
    gripper_command_topic: str = "/gripper_joint/position_command"  # 【自动检测到会覆盖当前值】夹爪控制发布话题（必需，用于夹爪控制）
    # ⚠️ 重要：使用夹爪前必须根据实际硬件正确配置以下两个参数。
    # 这两个值直接影响：
    #   1. send_joint_positions() 的位置限幅（clamping）
    #   2. send_position_percent() 的百分比 → 实际行程换算
    #   3. check_arrival() / wait_until_arrive() 的到位检测精度
    # 默认值仅为示例，不同夹爪型号的行程范围差异较大，请务必替换为实测值。
    gripper_min_position: float = 0.0     # 夹爪完全闭合时的关节位置（最小行程，单位：rad 或 m，取决于硬件）
    gripper_max_position: float = 0.0384  # 夹爪完全打开时的关节位置（最大行程，单位：rad 或 m，取决于硬件）
    # 【自动检测】夹爪控制器名称（系统会在 connect() 时自动检测，不应手动设置）
    left_gripper_controller_name: str | None = None  # 【自动检测】左臂夹爪控制器名称（自动检测：hand_controller、left_hand_controller 或 left_gripper_controller）
    right_gripper_controller_name: str | None = None  # 【自动检测】右臂夹爪控制器名称（自动检测：right_hand_controller 或 right_gripper_controller）
    left_gripper_target_percent_topic: str | None = None  # 【自动检测】左臂夹爪百分比控制话题，例如 "/left_gripper_controller/target_percent"
    right_gripper_target_percent_topic: str | None = None  # 【自动检测】右臂夹爪百分比控制话题，例如 "/right_gripper_controller/target_percent"
    
    # ============================================================================
    # 控制参数（未使用，预留）
    # ============================================================================
    control_type: ControlType = ControlType.CARTESIAN_POSE  # 【未使用】控制类型（预留，目前代码中未使用）
    
    # ============================================================================
    # 安全限制参数（未使用，预留）
    # ============================================================================
    # 【未使用】以下参数用于速度限制，但 send_cartesian_velocity 方法尚未实现
    max_linear_velocity: float = 0.1  # 【未使用】最大线性速度（单位：m/s），预留用于速度控制功能
    max_angular_velocity: float = 0.5  # 【未使用】最大角速度（单位：rad/s），预留用于速度控制功能
    
    # ============================================================================
    # 关节限位（未使用，预留）
    # ============================================================================
    # 【未使用】以下参数预留用于关节限位检查功能，目前代码中未实现
    min_joint_positions: list[float] | None = None  # 【未使用】关节最小位置限制（列表），预留用于关节限位检查
    max_joint_positions: list[float] | None = None  # 【未使用】关节最大位置限制（列表），预留用于关节限位检查
    
    # ============================================================================
    # 超时设置（可选，手动配置）
    # ============================================================================
    # 设置为 0 表示禁用超时检查（适用于 ROS2 节点可能重启的场景）
    joint_state_timeout: float = 0.0  # 【可选】关节状态超时时间（单位：秒）
    end_effector_pose_timeout: float = 0.0  # 【可选】末端执行器位姿超时时间（单位：秒）
    
    # ============================================================================
    # 状态机切换（可选，手动配置）
    # ============================================================================
    auto_switch_fsm_before_control: bool = True  # 【可选】是否在发送部分控制指令前自动发送 FSM 切换命令，默认关闭
    fsm_state_switch_settle_time: float = 0.3  # 【可选】发送 FSM 状态切换命令后的等待时间（单位：秒），用于确保对端完成状态切换
    
    # ============================================================================
    # ROS 2 节点配置（可选，手动配置）
    # ============================================================================
    namespace: str = ""  # 【可选】ROS 2 节点命名空间，默认为空字符串
    node_name: str | None = None  # 【可选】ROS 2 节点名称，未配置时根据单臂/双臂模式自动生成
    
    # ============================================================================
    # Nav2 导航配置（可选，自动检测）
    # ============================================================================
    # connect() 时自动检测：nav2_msgs 已安装 且 controller_server 节点正在运行时启用导航支持
    nav_action_server: str = "navigate_to_pose"  # 【可选】Nav2 NavigateToPose action server 名称
    nav_enabled: bool | None = None  # 【可选】None=自动检测，True=强制启用，False=强制禁用

    # ============================================================================
    # 阈值参数（可选，手动配置）
    # ============================================================================
    # 用于到达检测和稳定性检查的阈值参数
    position_threshold: float = 0.05  # 【可选】关节位置阈值（单位：弧度），用于 check_arrive 检查头部、身体关节是否到达目标位置
    gripper_position_threshold: float = 0.01  # 【可选】夹爪位置阈值（单位：0.0-1.0），用于 check_arrive 检查夹爪是否到达目标位置
    pose_position_threshold: float = 0.05  # 【可选】位姿位置阈值（单位：米），用于 arm_handler.check_arrival 检查末端执行器位置是否到达目标
    pose_orientation_threshold: float = 5.0  # 【可选】位姿姿态阈值（单位：度），用于 arm_handler.check_arrival 检查末端执行器姿态是否到达目标
    gripper_stability_history_size: int = 20  # 【可选】夹爪位置历史记录数量，用于夹爪稳定性检查
    gripper_stability_threshold: float = 0.0001  # 【可选】夹爪位置稳定性阈值，用于判断夹爪位置是否稳定

    @classmethod
    def default_single_arm(
        cls,
        *,
        joint_states_topic: str = "/joint_states",
        current_pose_topic: str = "/left_current_pose",
        target_pose_topic: str = "/left_target",
        current_target_topic: str | None = "/left_current_target",
        joint_names: Sequence[str] | None = None,
        gripper_joint_name: str = "left_gripper_joint",
        gripper_command_topic: str = "/left_gripper_joint/position_command",
        **kwargs: object,
    ) -> "ROS2RobotInterfaceConfig":
        """Create a practical default preset for single-arm robots.

        Extra keyword arguments are forwarded to the dataclass constructor,
        allowing overrides such as ``pose_position_threshold``.
        """
        return cls(
            joint_states_topic=joint_states_topic,
            end_effector_pose_topic=current_pose_topic,
            end_effector_target_topic=target_pose_topic,
            end_effector_current_target_topic=current_target_topic,
            joint_names=list(joint_names) if joint_names is not None else [],
            gripper_enabled=True,
            gripper_joint_name=gripper_joint_name,
            gripper_command_topic=gripper_command_topic,
            control_type=ControlType.CARTESIAN_POSE,
            **kwargs,
        )

    @classmethod
    def default_single_arm_ocs2_arm_controller(
        cls,
        *,
        arm_joint_topic: str = "/ocs2_arm_controller/target_joint_position",
        **kwargs: object,
    ) -> "ROS2RobotInterfaceConfig":
        """单臂 + ``ocs2_arm_controller`` 典型栈（如 Isaac Sim / 本仓库 OCS2 关节命令）。

        预先设置 ``left_arm_joint_controller_topic``，避免仅依赖 ``connect()`` 时 ROS 图探测
        尚未列出该话题而导致 MoveJ 发布器未创建。若控制器带命名空间，传入
        ``arm_joint_topic="/namespace/ocs2_arm_controller/target_joint_position"`` 覆盖即可。

        其余参数与 :meth:`default_single_arm` 相同，经 ``**kwargs`` 传入.
        """
        merged: dict[str, object] = {
            "left_arm_joint_controller_topic": arm_joint_topic,
        }
        merged.update(kwargs)
        return cls.default_single_arm(**merged)

    @classmethod
    def default_bimanual(
        cls,
        *,
        joint_states_topic: str = "/joint_states",
        left_current_pose_topic: str = "/left_current_pose",
        right_current_pose_topic: str = "/right_current_pose",
        left_target_topic: str = "/left_target",
        right_target_topic: str = "/right_target",
        left_current_target_topic: str | None = "/left_current_target",
        right_current_target_topic: str | None = "/right_current_target",
        joint_names: Sequence[str] | None = None,
        left_gripper_joint_name: str = "left_gripper_joint",
        left_gripper_command_topic: str = "/left_gripper_joint/position_command",
        right_gripper_command_topic: str = "/right_gripper_joint/position_command",
        **kwargs: object,
    ) -> "ROS2RobotInterfaceConfig":
        """Create a practical default preset for bimanual robots.

        Extra keyword arguments are forwarded to the dataclass constructor,
        allowing overrides such as ``pose_position_threshold``.
        """
        return cls(
            joint_states_topic=joint_states_topic,
            end_effector_pose_topic=left_current_pose_topic,
            end_effector_target_topic=left_target_topic,
            right_end_effector_pose_topic=right_current_pose_topic,
            right_end_effector_target_topic=right_target_topic,
            end_effector_current_target_topic=left_current_target_topic,
            right_end_effector_current_target_topic=right_current_target_topic,
            joint_names=list(joint_names) if joint_names is not None else [],
            gripper_enabled=True,
            gripper_joint_name=left_gripper_joint_name,
            gripper_command_topic=left_gripper_command_topic,
            right_gripper_command_topic=right_gripper_command_topic,
            control_type=ControlType.CARTESIAN_POSE,
            **kwargs,
        )
    
