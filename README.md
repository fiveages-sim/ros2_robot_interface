# ROS2 Robot Interface

A standalone Python package for communicating with ROS 2 robots through topics. This package is independent of LeRobot and can be used in any ROS 2 environment.

## Features

- Subscribe to joint states from ROS 2 topics
- Subscribe to end-effector pose information
- Publish target end-effector poses
- Control gripper position
- Thread-safe data access
- Configurable timeouts and recovery mechanisms

## Installation

### From Source (Development)

```bash
conda activate lerobot_ros2
cd ~/libraries/ros2_robot_interface/
pip install -e .
```

## Usage

### Basic Example

```python
from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig
from geometry_msgs.msg import Pose

# Create configuration
config = ROS2RobotInterfaceConfig(
    joint_states_topic="/joint_states",
    end_effector_pose_topic="/left_current_pose",
    end_effector_target_topic="/left_target",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
)

# Create and connect interface
interface = ROS2RobotInterface(config)
interface.connect()

# Get joint state
joint_state = interface.get_joint_state()
if joint_state:
    print(f"Joint positions: {joint_state['positions']}")

# Get end-effector pose (returns None if not connected)
pose = interface.left_arm_handler.get_pose()
if pose:
    print(f"End-effector position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
else:
    print("Interface not connected or pose not available")

# Send target pose
target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0
interface.left_arm_handler.send_target(target_pose)

# Control gripper
interface.left_gripper_handler.send_joint_positions(0.5)  # 行程/开度目标值为 0.5（非“50%百分比”语义）

# Disconnect
interface.disconnect()
```

### Configuration Options

```python
from ros2_robot_interface import ROS2RobotInterfaceConfig, ControlType

config = ROS2RobotInterfaceConfig(
    # ROS 2 topics
    joint_states_topic="/joint_states",
    end_effector_pose_topic="/left_current_pose",
    end_effector_target_topic="/left_target",
    
    # Joint names
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    
    # Gripper configuration
    gripper_enabled=True,
    gripper_joint_name="gripper_joint",
    gripper_command_topic="gripper_joint/position_command",
    gripper_min_position=0.0,
    gripper_max_position=1.0,
    
    # Control parameters
    control_type=ControlType.CARTESIAN_POSE,
    
    # Safety limits
    max_linear_velocity=0.1,  # m/s
    max_angular_velocity=0.5,  # rad/s
    
    # Timeout settings (0 = disabled)
    joint_state_timeout=0.0,  # seconds
    end_effector_pose_timeout=0.0,  # seconds
    
    # ROS 2 namespace
    namespace=""
)
```

## API Reference

详细的 API 参考文档请查看 [API_REFERENCE.md](API_REFERENCE.md)，其中包含：

- 每个部分（手臂、夹爪、头部、身体）的详细函数说明
- 每个函数的功能、参数、返回值
- 完整的使用示例
- 各部分功能总结表

### 快速参考

#### ROS2RobotInterface 主要方法

**连接与发现**
- `connect()` / `disconnect()` - 连接 / 断开 ROS 2
- `list_nodes()` - 查询当前运行的 ROS 2 节点列表
- `get_joint_state(categorized=False)` - 获取关节状态

**FSM / Mode**
- `send_fsm_command(command)` - 发送 FSM 状态切换命令
- `send_mode_command(command)` - 向 `/mode_command` 发布 WBC / 底盘模式
- `wait_until_mode_commands_applied(commands, ...)` - 对照 `/ocs2_wbc_controller/current_state` 确认一组 mode

**六维力 / COMPLIANCE**
- `get_original_wrench(side)` - 读取左右 FT 原始 wrench 缓存
- `get_filtered_wrench(side)` - 读取左右 FT 滤波 wrench 缓存（通常 COMPLIANCE 下有数据）
- `call_compliance_zero_wrench()` - 请求零力校准（不等待 `zero_cal_done`）
- `enter_compliance()` - 进入 COMPLIANCE FSM（自动 HOLD 中转）
- `set_compliance_force(task_selection, force_setpoint)` - 写入 6 维任务选择与目标力

**运动下发**
- `send_coordinated_joint_positions(...)` - 一次性协调下发臂/躯干/头关节（默认隐式 MOVEJ）
- `send_head_joint_positions(positions)` / `send_body_joint_positions(positions)` - 头部 / 身体关节
- `send_dual_arm_target_stamped(left_pose, right_pose, frame_id)` - 双臂笛卡尔目标 pose

**到达检查**
- `check_arrive(part, ...)` / `wait_until_arrive(...)` - 按 part 检查/等待（臂为笛卡尔位姿；头身为缓存关节目标）
- `wait_until_joint_arrive(...)` - 按显式关节角目标等待（支持部分索引 / 角距离，适合 MoveJ）

#### 属性

- `is_connected` - 检查接口是否已连接
- `left_arm_handler` - 左臂处理器（ArmHandler 实例）
- `right_arm_handler` - 右臂处理器（ArmHandler 实例，双臂模式）
- `left_gripper_handler` - 左夹爪处理器（GripperHandler 实例）
- `right_gripper_handler` - 右夹爪处理器（GripperHandler 实例，双臂模式）

#### 各部分 Handler 方法

**ArmHandler（手臂）：**
- `get_pose()` - 获取当前 pose
- `get_target_pose()` - 获取目标 pose
- `send_target(pose)` - 发送目标 pose
- `send_target_stamped(frame_id, pose)` / `send_target_stamped(pose)` - 发送带坐标系的目标 pose
- `send_joint_positions(positions)` - 发送关节位置（MoveJ 模式）
- `check_arrival(pose_threshold, orient_threshold)` - 检查到达状态

**GripperHandler（夹爪）：**
- `send_joint_positions(position)` - 发送夹爪关节位置命令（位置控制方式）
- `send_target_command(target_value)` - 发送夹爪开关控制命令（开关控制方式，0=关闭，1=打开）
- `check_arrival(current_position, threshold)` - 检查到达状态
- `get_target_position()` - 获取目标位置

**注意：** 控制器名称（`hand_controller` 或 `gripper_controller`）会在 `connect()` 时自动检测，无需手动配置。

### ROS2RobotInterfaceConfig

配置数据类，用于设置接口参数。

### Exceptions

- `ROS2NotConnectedError` - 当接口未连接时尝试使用接口
- `ROS2AlreadyConnectedError` - 当接口已连接时尝试再次连接

## Requirements

- Python >= 3.10
- ROS 2 (tested with Humble and later)
- rclpy
- sensor-msgs
- geometry-msgs
- std-msgs
- numpy

## Development

For local development, install in editable mode:

```bash
cd ros2_robot_interface
pip install -e .
```

## License

Apache-2.0
