"""
ROS 2 机器人接口配置

ROS 2 机器人接口的配置类。
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


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
    # 头部和身体关节控制器话题（可选，可自动检测）
    # ============================================================================
    # 【可自动检测】系统会在 connect() 时自动检测这些话题，如果检测到会自动设置
    # 也可以手动配置
    head_joint_controller_topic: str | None = None  # 【可自动检测】头部关节控制器话题，例如 "/head_joint_controller/target_joint_position"
    body_joint_controller_topic: str | None = None  # 【可自动检测】身体关节控制器话题，例如 "/body_joint_controller/target_joint_position"
    
    # ============================================================================
    # 手臂关节控制器话题（自动检测，无需手动配置）
    # ============================================================================
    # 【自动检测】系统会在 connect() 时自动检测这些话题，不应手动设置
    left_arm_joint_controller_topic: str | None = None  # 【自动检测】左臂关节控制器话题（根据可用话题自动检测）
    right_arm_joint_controller_topic: str | None = None  # 【自动检测】右臂关节控制器话题（双臂模式时自动检测）
    
    # ============================================================================
    # 关节配置（未使用，预留）
    # ============================================================================
    # 【未使用】此参数目前未被代码使用，代码实际使用的是从 ROS 2 topic 接收到的关节名称
    # 保留此参数用于未来可能的验证或过滤功能
    joint_names: list[str] = field(default_factory=lambda: [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ])  # 【未使用】关节名称列表（预留，目前代码使用从 /joint_states topic 接收的实际关节名称）
    
    # ============================================================================
    # 夹爪配置（手动配置）
    # ============================================================================
    gripper_enabled: bool = True  # 【可选】是否启用夹爪控制
    gripper_joint_name: str = "gripper_joint"  # 【未使用】夹爪关节名称（预留，目前代码使用从 /joint_states topic 接收的实际关节名称）
    gripper_state_topic: str = "/gripper_state"  # 【未使用】夹爪状态订阅话题（预留，目前未实现单独的状态话题订阅）
    gripper_command_topic: str = "/gripper_joint/position_command"  # 【自动检测到会覆盖当前值】夹爪控制发布话题（必需，用于夹爪控制）
    gripper_min_position: float = 0.0  # 夹爪闭合位置（最小开度）
    gripper_max_position: float = 0.0384  # 夹爪打开位置（最大开度）
    # 【自动检测】夹爪控制器名称（系统会在 connect() 时自动检测，不应手动设置）
    left_gripper_controller_name: str | None = None  # 【自动检测】左臂夹爪控制器名称（自动检测：hand_controller、left_hand_controller 或 left_gripper_controller）
    right_gripper_controller_name: str | None = None  # 【自动检测】右臂夹爪控制器名称（自动检测：right_hand_controller 或 right_gripper_controller）
    
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
    # ROS 2 节点配置（可选，手动配置）
    # ============================================================================
    namespace: str = ""  # 【可选】ROS 2 节点命名空间，默认为空字符串
    node_name: str | None = None  # 【可选】ROS 2 节点名称，未配置时根据单臂/双臂模式自动生成
    
    # ============================================================================
    # 阈值参数（可选，手动配置）
    # ============================================================================
    # 用于到达检测和稳定性检查的阈值参数
    position_threshold: float = 0.05  # 【可选】关节位置阈值（单位：弧度），用于 check_arrive 检查头部、身体关节是否到达目标位置
    gripper_position_threshold: float = 0.01  # 【可选】夹爪位置阈值（单位：0.0-1.0），用于 check_arrive 检查夹爪是否到达目标位置
    pose_position_threshold: float = 0.05  # 【可选】位姿位置阈值（单位：米），用于 arm_handler.check_arrival 检查末端执行器位置是否到达目标
    pose_orientation_threshold: float = 0.1  # 【可选】位姿姿态阈值（单位：四元数距离），用于 arm_handler.check_arrival 检查末端执行器姿态是否到达目标
    gripper_stability_history_size: int = 20  # 【可选】夹爪位置历史记录数量，用于夹爪稳定性检查
    gripper_stability_threshold: float = 0.0001  # 【可选】夹爪位置稳定性阈值，用于判断夹爪位置是否稳定
    
    # ============================================================================
    # TF 坐标系配置（可选，手动配置）
    # ============================================================================
    base_frame: str = "arm_base"  # 【可选】基础坐标系名称（用于 TF 变换）

