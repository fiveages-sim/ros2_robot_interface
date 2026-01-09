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

# Get end-effector pose
pose = interface.get_end_effector_pose()
if pose:
    print(f"End-effector position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")

# Send target pose
target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0
interface.send_end_effector_target(target_pose)

# Control gripper
interface.send_gripper_command(0.5)  # 50% open

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

- `connect()` - 连接到 ROS 2 并创建订阅器和发布器
- `disconnect()` - 断开连接并清理资源
- `get_joint_state(categorized=False)` - 获取关节状态
- `send_fsm_command(command)` - 发送 FSM 状态切换命令
- `send_head_joint_positions(positions)` - 发送头部关节位置
- `send_body_joint_positions(positions)` - 发送身体关节位置
- `send_dual_arm_target_stamped(left_pose, right_pose, frame_id)` - 发送双臂目标 pose
- `check_arrive(part, ...)` - 统一检查到达状态

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
- `send_target_stamped(frame_id, pose)` - 发送带坐标系的目标 pose
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
