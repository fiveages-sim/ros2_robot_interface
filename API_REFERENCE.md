# ROS2 Robot Interface API 参考文档

本文档详细说明每个机器人部分（part）可用的函数及其功能。文档按部分分类，每个部分内按功能（发送命令、获取状态、检查到达）组织。

## 目录

- [手臂控制 (Arm Handler)](#手臂控制-arm-handler)
  - [发送命令](#发送命令-1)
  - [获取状态](#获取状态-1)
  - [检查到达](#检查到达-1)
  - [双臂协调控制](#双臂协调控制-1)
- [夹爪控制 (Gripper Handler)](#夹爪控制-gripper-handler)
  - [发送命令](#发送命令-2)
  - [获取状态](#获取状态-2)
  - [检查到达](#检查到达-2)
- [头部控制](#头部控制)
  - [发送命令](#发送命令-3)
  - [获取状态](#获取状态-3)
  - [检查到达](#检查到达-3)
- [身体控制](#身体控制)
  - [发送命令](#发送命令-4)
  - [获取状态](#获取状态-4)
  - [检查到达](#检查到达-4)
- [统一接口方法](#统一接口方法)
  - [FSM状态切换](#fsm状态切换)
  - [关节状态获取](#关节状态获取)
  - [末端执行器位姿获取](#末端执行器位姿获取)
  - [统一到达检查](#统一到达检查)
  - [坐标转换](#坐标转换)
- [快速参考表](#快速参考表)
- [注意事项](#注意事项)

---

## 手臂控制 (Arm Handler)

手臂控制通过 `left_arm_handler` 和 `right_arm_handler` 访问。

### 访问方式

```python
# 左臂（始终可用）
interface.left_arm_handler

# 右臂（仅双臂模式）
interface.right_arm_handler
```

---

### 发送命令

#### `send_target_stamped(frame_id: str, pose: Pose) -> None` ⭐ **推荐使用**

**功能：** 发送带坐标系的目标 pose（推荐使用）

**参数：**
- `frame_id` (str): 坐标系 ID（如 `"arm_base"`, `"base_link"`, `"head_link2"`, `"left_eef"`, `"left_link6"` 等）
- `pose` (Pose): 目标 pose 对象（在 `frame_id` 指定的坐标系下）

**说明：**
- 发布到 `/left_target/stamped` 或 `/right_target/stamped` topic
- **会自动进行 TF 坐标转换**到 `base_frame`
- 如果 `frame_id` 与 `base_frame` 不同，会自动转换
- 目标位姿会通过话题订阅获取（`/left_current_target` 或 `/right_current_target`），用于到达判断


**示例：**
```python
from geometry_msgs.msg import Pose

# 场景1：使用其他坐标系（如 head_link2）
target_pose = Pose()
target_pose.position.x = 0.5  # 在 head_link2 坐标系下
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0
interface.left_arm_handler.send_target_stamped("head_link2", target_pose)

# 场景2：相对运动（使用末端执行器坐标系 left_eef）
relative_pose = Pose()
relative_pose.position.x = 0.15   # 相对于当前末端向前 0.15m
relative_pose.position.y = 0.0
relative_pose.position.z = 0.0
relative_pose.orientation.w = 1.0
interface.left_arm_handler.send_target_stamped("left_eef", relative_pose)
```

---

#### `send_target(pose: Pose) -> None`

**功能：** 发送目标 pose（不带坐标系信息）

**参数：**
- `pose` (Pose): 目标 pose 对象（应在 `base_frame` 坐标系下）

**说明：**
- 直接发布到 `/left_target` 或 `/right_target` topic
- **不进行 TF 坐标转换**，假设传入的 pose 已经在 `base_frame` 坐标系下
- 目标位姿会通过话题订阅获取（`/left_current_target` 或 `/right_current_target`），用于到达判断

**注意事项：**
- ⚠️ 需要确保传入的 pose 在正确的坐标系下（`base_frame`）
- ⚠️ 不支持其他坐标系，如果 pose 在其他坐标系下，应使用 `send_target_stamped()`

**示例：**
```python
from geometry_msgs.msg import Pose

# 场景：目标位姿已经在 arm_base 坐标系下
target_pose = Pose()
target_pose.position.x = 0.5  # 在 arm_base 坐标系下
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0

interface.left_arm_handler.send_target(target_pose)
```

---

#### `send_joint_positions(positions: List[float]) -> None`

**功能：** 发送关节位置命令（MoveJ 模式）

**参数：**
- `positions` (List[float]): 目标关节位置列表（弧度）

**说明：**
- 自动切换到 MOVEJ 状态（FSM 命令 4）
- 使用初始化时传入的 FSM 命令回调函数（在 `ROS2RobotInterface` 中自动配置）
- 发布到关节控制器 topic（如 `/ocs2_wbc_controller/target_joint_position/left`）
- 关节数量必须与配置一致

**Raises:**
- `ROS2NotConnectedError`: 如果发布器未初始化
- `ValueError`: 如果初始化时未提供 FSM 命令回调函数

**示例：**
```python
# 发送6个关节的位置（弧度）
joint_positions = [0.0, 0.5, -1.57, 0.0, 1.57, 0.0]
interface.left_arm_handler.send_joint_positions(joint_positions)
```

---

### 获取状态

#### `get_pose() -> Optional[Pose]` ⭐ **最常用**

**功能：** 获取当前 end-effector 的 pose（位置和姿态）

**返回值：**
- `Pose` 对象：包含当前位置和姿态（在 `base_frame` 坐标系下）
- `None`：如果数据过期或不存在

**使用场景：**
- **状态监控**：获取机器人末端执行器的当前实际位姿，用于显示或记录
- **初始位姿保存**：保存初始位姿，用于后续计算相对运动
- **内部调用**：`check_arrival()` 内部会调用此函数获取当前位姿进行比较

**示例：**
```python
# 场景1：获取当前位姿用于显示
current_pose = interface.left_arm_handler.get_pose()
if current_pose:
    print(f"当前位置: ({current_pose.position.x}, {current_pose.position.y}, {current_pose.position.z})")
    print(f"姿态: ({current_pose.orientation.x}, {current_pose.orientation.y}, {current_pose.orientation.z}, {current_pose.orientation.w})")

# 场景2：保存初始位姿，用于后续计算相对运动
left_initial_pose = interface.left_arm_handler.get_pose()
# 然后基于初始位姿计算目标位姿
```

---

#### `get_target_pose() -> Optional[Pose]`

**功能：** 获取当前设置的目标 pose

**返回值：**
- `Pose` 对象：目标位置和姿态（在 `base_frame` 坐标系下）
- `None`：如果未设置目标或数据不可用

**说明：**
- 从话题订阅获取目标位置（`/left_current_target` 或 `/right_current_target`）
- 如果未配置目标位置话题，返回 `None`

**使用场景：**
- **查询目标**：查询当前的目标位姿（从话题订阅获取）
- **调试验证**：验证目标位姿是否正确设置
- **内部调用**：`check_arrival()` 内部会调用此函数获取目标位姿进行比较

**示例：**
```python
# 场景1：查询当前目标位姿
target_pose = interface.left_arm_handler.get_target_pose()
if target_pose:
    print(f"目标位置: ({target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z})")

# 场景2：验证目标是否已设置
target_pose = interface.left_arm_handler.get_target_pose()
if target_pose is None:
    print("警告：未设置目标位姿或目标位置话题未配置")
```

---

### 检查到达

#### `check_arrival(pose_threshold: float | None = None, orient_threshold: float | None = None) -> Dict[str, Any]` ⭐ **最常用**

**功能：** 检查手臂是否到达目标位置

**参数：**
- `pose_threshold` (float | None): 位置距离阈值（米），如果为 None 则使用 `config.pose_position_threshold`（默认 0.05）
- `orient_threshold` (float | None): 姿态距离阈值，如果为 None 则使用 `config.pose_orientation_threshold`（默认 0.1）

**返回值：**
```python
{
    'arrived': bool,              # 是否到达目标位置
    'distance': float,            # 总距离（位置 + 姿态）
    'position_distance': float,   # 位置距离（米）
    'orientation_distance': float,# 姿态距离
    'status_message': str         # 状态消息
}
```

**说明：**
- **内部调用**：自动调用 `get_pose()` 获取当前位姿，调用 `get_target_pose()` 获取目标位姿（从话题订阅）
- **坐标系一致性**：比较的两个位姿都在 `base_frame` 坐标系下
  - `get_pose()` 返回的是 `base_frame` 下的当前位姿
  - `get_target_pose()` 返回的是 `base_frame` 下的目标位姿（从 `/left_current_target` 或 `/right_current_target` 话题订阅获取）
- **位置距离**：欧氏距离（米）
- **姿态距离**：基于四元数的点积计算
- **默认阈值**：如果未指定参数，使用配置中的默认值（`config.pose_position_threshold` 和 `config.pose_orientation_threshold`）
- **会打印详细的检查信息**

**示例：**
```python
# 场景1：使用默认阈值
result = interface.left_arm_handler.check_arrival()
if result['arrived']:
    print("手臂已到达目标位置")

# 场景2：使用自定义阈值
result = interface.left_arm_handler.check_arrival(
    pose_threshold=0.05,      # 5厘米
    orient_threshold=0.08     # 更严格的姿态要求
)
print(f"位置距离: {result['position_distance']:.4f} 米")
print(f"姿态距离: {result['orientation_distance']:.4f}")

# 场景3：完整的工作流程
# 1. 发送目标（使用 send_target_stamped，自动转换到 base_frame）
target_pose = Pose()
target_pose.position.x = 0.15
target_pose.position.y = 0.0
target_pose.position.z = 0.0
target_pose.orientation.w = 1.0
interface.left_arm_handler.send_target_stamped("left_eef", target_pose)

# 2. 等待到达（内部会调用 get_pose() 和 get_target_pose()）
while True:
    result = interface.left_arm_handler.check_arrival()
    if result['arrived']:
        print("到达目标！")
        break
    print(f"距离目标: {result['position_distance']:.4f} 米")
    time.sleep(0.1)
```

---

### 双臂协调控制

这些方法通过 `ROS2RobotInterface` 直接访问，用于同时控制左臂和右臂。

#### `send_dual_arm_target_stamped(left_pose: Pose, right_pose: Pose, frame_id: str = "arm_base") -> None`

**功能：** 发送双臂目标 pose（仅双臂模式）

**参数：**
- `left_pose` (Pose): 左臂目标 pose（在 `frame_id` 指定的坐标系下）
- `right_pose` (Pose): 右臂目标 pose（在 `frame_id` 指定的坐标系下）
- `frame_id` (str): 坐标系 ID，默认 "arm_base"

**说明：**
- 发布到 `/dual_target/stamped` topic（使用 `nav_msgs/Path` 消息类型，包含2个 `PoseStamped`：第一个是左臂，第二个是右臂）
- **内部实现**：直接发布到 `/dual_target/stamped` topic，不调用 handler 的方法
- 目标位姿会通过话题订阅获取（`/left_current_target` 和 `/right_current_target`），用于到达判断
- 可以使用 `left_arm_handler.check_arrival()` 和 `right_arm_handler.check_arrival()` 分别检查到达状态

**示例：**
```python
from geometry_msgs.msg import Pose

left_pose = Pose()
left_pose.position.x = 0.5
left_pose.position.y = 0.2
left_pose.position.z = 0.3
left_pose.orientation.w = 1.0

right_pose = Pose()
right_pose.position.x = 0.5
right_pose.position.y = -0.2
right_pose.position.z = 0.3
right_pose.orientation.w = 1.0

# 发送双臂目标（pose 在 frame_id 指定的坐标系下，接收端会进行坐标转换）
interface.send_dual_arm_target_stamped(left_pose, right_pose, frame_id="arm_base")

# 检查到达状态
left_result = interface.left_arm_handler.check_arrival()
right_result = interface.right_arm_handler.check_arrival()
if left_result['arrived'] and right_result['arrived']:
    print("双臂都已到达目标位置")
```

---

#### `send_dual_arm_joint_positions(left_arm_positions: List[float], right_arm_positions: List[float]) -> None` ⭐ **新增**

**功能：** 发送双臂关节位置命令（MoveJ 模式，统一 topic 控制）

**参数：**
- `left_arm_positions` (List[float]): 左臂关节位置列表（弧度）
- `right_arm_positions` (List[float]): 右臂关节位置列表（弧度）

**说明：**
- 同时控制左臂和右臂的所有关节，发布到统一的 topic
- **自动检测控制器类型**：
  - **WBC 控制器** (`/ocs2_wbc_controller/target_joint_position`)：需要 `body_joints + left_arm_joints + right_arm_joints`（18个关节）
    - 自动从当前关节状态获取身体关节位置
    - 如果没有身体关节数据，使用默认值（4个零值）
  - **ARM 控制器** (`/ocs2_arm_controller/target_joint_position`)：只需要 `left_arm_joints + right_arm_joints`（14个关节）
- 自动切换到 MOVEJ 状态（FSM 命令 4）
- 左臂和右臂的关节数量必须相同
- 关节顺序：WBC 控制器为 `[body_joint1-4] + [left_joint1-7] + [right_joint1-7]`，ARM 控制器为 `[left_joint1-7] + [right_joint1-7]`

**Raises:**
- `ROS2NotConnectedError`: 如果接口未连接或发布器未初始化
- `ValueError`: 如果双臂模式未启用、参数无效或左右臂关节数量不一致

**示例：**
```python
# 左臂7个关节，右臂7个关节
left_positions = [0.0, 0.5, -1.57, 0.0, 1.57, 0.0, 0.0]
right_positions = [0.0, -0.5, 1.57, 0.0, -1.57, 0.0, 0.0]

# 发送双臂关节位置（自动检测控制器类型并添加身体关节）
interface.send_dual_arm_joint_positions(left_positions, right_positions)

# WBC 控制器会自动添加身体关节：
# 最终发送：[body_joint1-4] + [left_joint1-7] + [right_joint1-7] = 18个关节
# ARM 控制器只发送双臂关节：
# 最终发送：[left_joint1-7] + [right_joint1-7] = 14个关节
```

**注意事项：**
- ⚠️ WBC 控制器会自动添加身体关节，确保身体关节数据可用（从 `/joint_states` 获取）
- ⚠️ 如果使用 WBC 控制器且没有身体关节数据，会使用默认值（4个零值），可能导致意外运动

---

## 夹爪控制 (Gripper Handler)

夹爪控制通过 `left_gripper_handler` 和 `right_gripper_handler` 访问。

### 访问方式

```python
# 左夹爪（如果启用）
interface.left_gripper_handler

# 右夹爪（双臂模式，如果启用）
interface.right_gripper_handler
```

---

### 发送命令

#### `send_target_command(target_value: int) -> None` ⭐ **推荐使用**

**功能：** 发送夹爪开关控制命令（开关控制方式，使用 `target_command` 话题）

**参数：**
- `target_value` (int): 目标值，`0` = 关闭，`1` = 打开

**说明：**
- 使用 `target_command` 话题进行开关控制，与 VR、RViz、Joystick 保持一致
- **控制器名称自动检测**：系统会根据实际存在的 topic 自动检测控制器类型
  - 灵巧手：`hand_controller` 或 `left_hand_controller` / `right_hand_controller`
  - 夹爪：`gripper_controller` 或 `left_gripper_controller` / `right_gripper_controller`
- 发布到对应的 `target_command` 话题（如 `/left_hand_controller/target_command` 或 `/left_gripper_controller/target_command`）
- 使用 `std_msgs/Int32` 消息类型
- 会自动订阅相同话题以同步状态（通过回调更新 `is_open` 属性）
- 如果传入的值不是 0 或 1，会发出警告并返回

**示例：**
```python
# 打开夹爪
interface.left_gripper_handler.send_target_command(1)

# 关闭夹爪
interface.left_gripper_handler.send_target_command(0)

# 根据当前状态切换
current_state = interface.left_gripper_handler.is_open
target_value = 0 if current_state else 1
interface.left_gripper_handler.send_target_command(target_value)
```

**话题映射（自动检测）：**
- **双臂模式 - 灵巧手：**
  - 左夹爪：`/left_hand_controller/target_command`
  - 右夹爪：`/right_hand_controller/target_command`
- **双臂模式 - 夹爪：**
  - 左夹爪：`/left_gripper_controller/target_command`
  - 右夹爪：`/right_gripper_controller/target_command`
- **单臂模式 - 灵巧手：**
  - 夹爪：`/hand_controller/target_command`
- **单臂模式 - 夹爪：**
  - 夹爪：`/gripper_controller/target_command`

**注意：** 控制器名称会在 `connect()` 时自动检测，无需手动配置。

---

#### `send_joint_positions(position: float) -> None`

**功能：** 发送夹爪关节位置命令（位置控制方式）

**参数：**
- `position` (float): 目标关节位置（夹爪通常只有一个关节）

**说明：**
- 为了与手臂的 API 保持一致，使用相同的命名 `send_joint_positions()`
- 夹爪通常只有一个关节，所以直接传入位置值即可
- 位置会自动限制在 `gripper_min_position` 和 `gripper_max_position` 之间
- 发布到夹爪命令 topic（如 `/gripper_joint/position_command`）
- 会自动更新目标位置并清空历史记录

**示例：**
```python
# 完全闭合
interface.left_gripper_handler.send_joint_positions(0.0)

# 50% 张开
interface.left_gripper_handler.send_joint_positions(0.5)

# 完全张开
interface.left_gripper_handler.send_joint_positions(1.0)
```

---

### 获取状态

#### `get_target_position() -> Optional[float]`

**功能：** 获取当前设置的目标位置

**返回值：**
- `float`：目标位置值
- `None`：如果未设置目标

**说明：**
- 目标位置是**内部存储**的变量 `self.target_position`（定义在 `GripperHandler.__init__()` 中，初始值为 `None`）
- 通过以下方式设置：
  1. **位置控制方式**：调用 `send_joint_positions(position)` 时（`gripper_handler.py:157`），会将位置限制在 `gripper_min_position` 和 `gripper_max_position` 之间，然后更新 `self.target_position = clamped_position`
  2. **开关控制方式**：当订阅的 `target_command` 话题收到消息时（`gripper_handler.py:214-216`），会自动更新目标位置
     - `msg.data == 1`（打开）→ `self.target_position = self.config.gripper_max_position`
     - `msg.data == 0`（关闭）→ `self.target_position = self.config.gripper_min_position`
- 与手臂的 `get_target_pose()` 不同，夹爪的目标位置不是从外部话题订阅获取的，而是内部维护的状态变量
- 该变量在 `check_arrival()` 方法中用于与当前位置比较，判断是否到达目标

**示例：**
```python
target_position = interface.left_gripper_handler.get_target_position()
if target_position is not None:
    print(f"目标位置: {target_position}")
```

---

### 检查到达

#### `check_arrival(current_position: Optional[float], threshold: float | None = None) -> Dict[str, Any]`

**功能：** 检查夹爪是否到达目标位置

**参数：**
- `current_position` (Optional[float]): 当前位置（需要从 `get_joint_state()` 获取）
- `threshold` (float | None): 位置距离阈值，如果为 None 则使用 `config.gripper_position_threshold`（默认 0.01）

**返回值：**
```python
{
    'arrived': bool,      # 是否到达目标位置
    'distance': float     # 位置距离
}
```

**说明：**
- **需要手动传入当前位置**（从 joint_state 中提取）
- **默认阈值**：如果未指定 `threshold` 参数，使用配置中的默认值（`config.gripper_position_threshold`）
- 关闭时会考虑位置稳定性（可能已夹住物体）
- 打开时只考虑距离阈值
- 会打印详细的检查信息，包括位置历史

**示例：**
```python
# 1. 获取当前位置
categorized_state = interface.get_joint_state(categorized=True)
gripper_data = categorized_state.get('gripper', {})  # 单臂模式
# 或
gripper_data = categorized_state.get('left_gripper', {})  # 双臂模式
current_position = gripper_data.get('positions', [None])[0]

# 2. 检查到达状态
if current_position is not None:
    result = interface.left_gripper_handler.check_arrival(current_position)
    if result['arrived']:
        print("夹爪已到达目标位置")

# 3. 使用自定义阈值
result = interface.left_gripper_handler.check_arrival(
    current_position=current_position,
    threshold=0.005  # 更严格的阈值
)
```

---

## 头部控制

头部控制通过 `ROS2RobotInterface` 的直接方法访问。

---

### 发送命令

#### `send_head_joint_positions(positions: List[float]) -> None`

**功能：** 发送头部关节位置命令

**参数：**
- `positions` (List[float]): 目标关节位置列表（弧度）

**说明：**
- 发布到头部关节控制器 topic（如 `/head_joint_controller/target_joint_position`）
- 会自动保存目标位置（用于 `check_arrive()`）
- 关节数量必须与配置一致

**示例：**
```python
# 发送头部关节位置（例如：2个关节）
head_positions = [0.5, -0.3]  # 弧度
interface.send_head_joint_positions(head_positions)
```

---

### 获取状态

#### `get_joint_state(categorized: bool = False) -> Dict[str, Any] | None`

**功能：** 获取关节状态（包括头部关节）

**参数：**
- `categorized` (bool): 是否返回分类后的状态，默认 False

**返回值：**

**原始模式（categorized=False）：**
```python
{
    'names': List[str],           # 关节名称列表
    'positions': List[float],     # 关节位置列表（弧度）
    'velocities': List[float],    # 关节速度列表（弧度/秒）
    'efforts': List[float],       # 关节力矩列表
    'timestamp': float            # 时间戳
}
```

**分类模式（categorized=True）：**
```python
{
    'head': {...},                # 头部关节
    'body': {...},                # 身体关节
    'left_arm': {...},            # 左臂关节（双臂模式）
    'right_arm': {...},           # 右臂关节（双臂模式）
    'arm': {...},                 # 手臂关节（单臂模式）
    'left_gripper': {...},        # 左夹爪（双臂模式）
    'right_gripper': {...},       # 右夹爪（双臂模式）
    'gripper': {...},             # 夹爪（单臂模式）
    'other': {...},               # 其他关节
    'timestamp': float
}
```

**示例：**
```python
# 获取分类后的关节状态
categorized_state = interface.get_joint_state(categorized=True)
if categorized_state:
    head_data = categorized_state.get('head', {})
    head_positions = head_data.get('positions', [])
```

---

### 检查到达

#### `check_arrive(part='head', position_threshold: Optional[float] = None) -> Dict[str, Any]`

**功能：** 检查头部是否到达目标位置

**参数：**
- `part` (str): 设置为 `'head'`
- `position_threshold` (Optional[float]): 关节位置阈值，如果为 None 则使用默认值

**返回值：**
```python
{
    'arrived': bool,      # 是否到达
    'distance': float     # 距离
}
```

**示例：**
```python
# 检查头部到达状态
result = interface.check_arrive('head')
if result['arrived']:
    print("头部已到达目标位置")

# 使用自定义阈值
result = interface.check_arrive('head', position_threshold=0.01)
```

---

## 身体控制

身体控制通过 `ROS2RobotInterface` 的直接方法访问。

---

### 发送命令

#### `send_body_joint_positions(positions: List[float]) -> None`

**功能：** 发送身体关节位置命令

**参数：**
- `positions` (List[float]): 目标关节位置列表（弧度）

**说明：**
- 发布到身体关节控制器 topic（如 `/body_joint_controller/target_joint_position`）
- 会自动保存目标位置（用于 `check_arrive()`）
- 关节数量必须与配置一致

**示例：**
```python
# 发送身体关节位置（例如：4个关节）
body_positions = [0.0, 0.0, 0.0, 0.0]  # 弧度
interface.send_body_joint_positions(body_positions)
```

---

### 获取状态

#### `get_joint_state(categorized: bool = False) -> Dict[str, Any] | None`

**功能：** 获取关节状态（包括身体关节）

详见 [头部控制 - 获取状态](#获取状态-3) 部分，使用 `categorized_state.get('body', {})` 获取身体关节数据。

---

### 检查到达

#### `check_arrive(part='body', position_threshold: Optional[float] = None) -> Dict[str, Any]`

**功能：** 检查身体是否到达目标位置

**参数：**
- `part` (str): 设置为 `'body'`
- `position_threshold` (Optional[float]): 关节位置阈值，如果为 None 则使用默认值

**返回值：**
```python
{
    'arrived': bool,      # 是否到达
    'distance': float     # 距离
}
```

**示例：**
```python
# 检查身体到达状态
result = interface.check_arrive('body')
if result['arrived']:
    print("身体已到达目标位置")
```

---

## 统一接口方法

这些方法通过 `ROS2RobotInterface` 直接访问，可以操作多个部分。

---

### FSM状态切换

#### `send_fsm_command(command: int) -> None`

**功能：** 发送 FSM（有限状态机）状态切换命令

**参数：**
- `command` (int): FSM 命令值
  - `1`: HOME 状态
  - `2`: HOLD 状态
  - `3`: OCS2 状态（笛卡尔空间控制）
  - `4`: MOVEJ 状态（关节空间控制）

**说明：**
- 发布到 `/fsm_command` topic
- 用于切换机器人的控制模式

**示例：**
```python
# 切换到 HOME 状态
interface.send_fsm_command(1)

# 切换到 OCS2 状态（用于 pose 控制）
interface.send_fsm_command(3)

# 切换到 MOVEJ 状态（用于关节控制）
interface.send_fsm_command(4)
```

---

### 关节状态获取

#### `get_joint_state(categorized: bool = False) -> Dict[str, Any] | None`

**功能：** 获取所有关节状态

**参数：**
- `categorized` (bool): 是否返回分类后的状态，默认 False

**返回值：**

**原始模式（categorized=False）：**
```python
{
    'names': List[str],           # 关节名称列表
    'positions': List[float],     # 关节位置列表（弧度）
    'velocities': List[float],    # 关节速度列表（弧度/秒）
    'efforts': List[float],       # 关节力矩列表
    'timestamp': float            # 时间戳
}
```

**分类模式（categorized=True）：**
```python
{
    'left_arm': {                 # 左臂关节（双臂模式）
        'names': List[str],
        'positions': List[float],
        'velocities': List[float],
        'efforts': List[float]
    },
    'right_arm': {...},           # 右臂关节（双臂模式）
    'left_gripper': {...},        # 左夹爪（双臂模式）
    'right_gripper': {...},       # 右夹爪（双臂模式）
    'arm': {...},                 # 手臂关节（单臂模式）
    'gripper': {...},             # 夹爪（单臂模式）
    'head': {...},                # 头部关节
    'body': {...},                # 身体关节
    'other': {...},               # 其他关节
    'timestamp': float
}
```

**示例：**
```python
# 获取原始关节状态
joint_state = interface.get_joint_state()
if joint_state:
    print(f"关节数: {len(joint_state['names'])}")
    print(f"位置: {joint_state['positions']}")

# 获取分类后的关节状态
categorized_state = interface.get_joint_state(categorized=True)
if categorized_state:
    left_arm_data = categorized_state.get('left_arm', {})
    gripper_data = categorized_state.get('gripper', {})
    head_data = categorized_state.get('head', {})
```

---

### 末端执行器位姿获取

#### `get_end_effector_pose() -> Optional[Pose]`

**功能：** 获取左臂末端执行器的当前位姿（便捷方法）

**返回值：**
- `Pose` 对象：左臂末端执行器的当前位置和姿态（在 `base_frame` 坐标系下）
- `None`：如果未连接或位姿不可用

**说明：**
- 这是 `left_arm_handler.get_pose()` 的便捷方法
- 如果接口未连接，返回 `None` 而不是抛出异常
- 返回的位姿在 `base_frame` 坐标系下

**示例：**
```python
# 获取左臂末端执行器位姿
pose = interface.get_end_effector_pose()
if pose:
    print(f"位置: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
    print(f"姿态: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
else:
    print("未连接或位姿不可用")
```

---

#### `get_right_end_effector_pose() -> Optional[Pose]`

**功能：** 获取右臂末端执行器的当前位姿（双臂模式）

**返回值：**
- `Pose` 对象：右臂末端执行器的当前位置和姿态（在 `base_frame` 坐标系下）
- `None`：如果未连接、非双臂模式或位姿不可用

**说明：**
- 这是 `right_arm_handler.get_pose()` 的便捷方法
- 仅在双臂模式下可用
- 如果接口未连接，返回 `None` 而不是抛出异常
- 返回的位姿在 `base_frame` 坐标系下

**示例：**
```python
# 获取右臂末端执行器位姿（双臂模式）
pose = interface.get_right_end_effector_pose()
if pose:
    print(f"位置: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
else:
    print("未连接、非双臂模式或位姿不可用")
```

---

#### `get_last_joint_state_time() -> Optional[float]`

**功能：** 获取最后一次接收到关节状态消息的时间戳

**返回值：**
- `float`：时间戳（秒，系统时间）
- `None`：如果未连接或尚未接收到关节状态消息

**说明：**
- 返回的是系统时间（不是消息中的时间戳）
- 用于检测关节状态数据是否已更新（即使超时检查被禁用）
- 如果接口未连接，返回 `None` 而不是抛出异常

**示例：**
```python
# 获取最后接收关节状态的时间
last_time = interface.get_last_joint_state_time()
if last_time:
    print(f"最后接收时间: {last_time}")
    current_time = time.time()
    time_since_last = current_time - last_time
    print(f"距离上次接收: {time_since_last:.2f} 秒")
else:
    print("未连接或尚未接收到关节状态")
```

---

### 统一到达检查

#### `check_arrive(part: Optional[str] = None, position_threshold: Optional[float] = None) -> Optional[Dict[str, Any]]`

**功能：** 统一检查多个部分的到达状态

**参数：**
- `part` (Optional[str]): 要检查的部分，可选值：
  - `None`: 检查所有部分
  - `'left_arm'`: 左臂
  - `'right_arm'`: 右臂（双臂模式）
  - `'left_gripper'`: 左夹爪
  - `'right_gripper'`: 右夹爪（双臂模式）
  - `'head'`: 头部
  - `'body'`: 身体
  - `'arm'`: 手臂（单臂模式，会自动转换为 'left_arm'）
  - `'gripper'`: 夹爪（单臂模式，会自动转换为 'left_gripper'）
- `position_threshold` (Optional[float]): 关节位置阈值（仅用于 head/body），如果为 None 则使用默认值

**说明：**
- 手臂和夹爪使用各自 handler 的默认阈值（可通过直接调用 handler 的方法来自定义）
- 头部和身体使用 `position_threshold` 参数或默认的 `self.position_threshold`
- 如果接口未连接，返回 `None` 而不是抛出异常

**返回值：**
- `None`：如果接口未连接

**检查单个部分：**
```python
{
    'arrived': bool,      # 是否到达
    'distance': float     # 距离
}
```

**检查所有部分：**
```python
{
    'left_arm': {
        'arrived': bool,
        'distance': float,
        'position_distance': float,
        'orientation_distance': float,
        'status_message': str
    },
    'right_arm': {...},      # 双臂模式
    'left_gripper': {
        'arrived': bool,
        'distance': float
    },
    'right_gripper': {...},  # 双臂模式
    'head': {
        'arrived': bool,
        'distance': float
    },
    'body': {
        'arrived': bool,
        'distance': float
    }
}
```

**示例：**
```python
# 检查单个部分
result = interface.check_arrive('left_arm')
if result and result['arrived']:
    print("左臂已到达")

# 检查所有部分
results = interface.check_arrive()
if results:
    print(f"左臂到达: {results.get('left_arm', {}).get('arrived', False)}")
    print(f"夹爪到达: {results.get('gripper', {}).get('arrived', False)}")
else:
    print("接口未连接")

# 使用自定义阈值（仅用于 head/body）
results = interface.check_arrive(
    part='head',
    position_threshold=0.01  # 头部关节位置阈值
)

# 如果需要自定义手臂或夹爪的阈值，直接调用 handler 的方法
arm_result = interface.left_arm_handler.check_arrival(pose_threshold=0.05)
gripper_result = interface.left_gripper_handler.check_arrival(current_position, threshold=0.005)
```

---

### 坐标转换

#### `lookup_transform(target_frame: str, source_frame: str, timeout: Optional[float] = None) -> Optional[TransformStamped]`

**功能：** 查询两个坐标系之间的变换关系

**参数：**
- `target_frame` (str): 参考坐标系（在这个坐标系下观察）
- `source_frame` (str): 被查询的坐标系（要查询它的位置）
- `timeout` (Optional[float]): 可选超时时间（秒），如果为 `None` 则立即返回（不等待）

**返回值：**
- `TransformStamped` 对象：表示 **`source_frame` 相对于 `target_frame`** 的位姿（`source_frame` 在 `target_frame` 坐标系下的位姿）
- `None`：如果查询失败

**示例：**
```python
# 查询 left_link6 相对于 head_link2 的位置
# 返回结果表示：left_link6 在 head_link2 坐标系下的位姿
transform = interface.lookup_transform("head_link2", "left_link6")
if transform:
    trans = transform.transform.translation
    rot = transform.transform.rotation
    print(f"平移: ({trans.x:.4f}, {trans.y:.4f}, {trans.z:.4f}) 米")
    print(f"旋转: ({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})")
```

---

#### `transform_pose(pose: Pose, source_frame: str, target_frame: str, timeout: Optional[float] = None) -> Optional[Pose]`

**功能：** 将坐标从一个坐标系转换到另一个坐标系

**参数：**
- `pose` (Pose): 要转换的 Pose（在 `source_frame` 坐标系下）
- `source_frame` (str): 源坐标系（`pose` 当前所在的坐标系）
- `target_frame` (str): 目标坐标系（转换后的 `pose` 所在的坐标系）
- `timeout` (Optional[float]): 可选超时时间（秒），如果为 `None` 则立即返回（不等待）

**返回值：**
- `Pose` 对象：转换后的 Pose（在 `target_frame` 坐标系下）
- `None`：如果转换失败

**示例：**
```python
from geometry_msgs.msg import Pose

# 将 pose 从 head_link2 坐标系转换到 arm_base 坐标系
pose_in_head = Pose()
pose_in_head.position.x = 0.5  # 在 head_link2 坐标系下
pose_in_head.position.y = 0.0
pose_in_head.position.z = 0.3
pose_in_head.orientation.w = 1.0

# 参数说明：
# - pose_in_head: 要转换的 Pose
# - "head_link2": 源坐标系（pose_in_head 当前所在的坐标系）
# - "arm_base": 目标坐标系（转换后的 pose 所在的坐标系）
pose_in_base = interface.transform_pose(pose_in_head, "head_link2", "arm_base")
if pose_in_base:
    print(f"转换后的位置: ({pose_in_base.position.x:.4f}, {pose_in_base.position.y:.4f}, {pose_in_base.position.z:.4f})")
```

---

### 系统信息查询

#### `list_nodes() -> List[Dict[str, str]]`

**功能：** 查询当前运行的 ROS 2 节点列表

**说明：**
- 此方法可以在连接或未连接状态下使用
- 如果接口已连接，会使用现有节点进行查询
- 如果未连接，会创建一个临时节点来查询

**返回值：**
- `List[Dict[str, str]]`: 节点信息列表，每个字典包含：
  - `'name'`: 节点名称（不含命名空间）
  - `'namespace'`: 节点命名空间
  - `'full_name'`: 完整节点名称（命名空间 + 名称）

**示例：**
```python
# 查询节点列表
nodes = interface.list_nodes()
print(f"当前运行的节点数量: {len(nodes)}")
for node in nodes:
    print(f"节点: {node['full_name']}")
    print(f"  名称: {node['name']}")
    print(f"  命名空间: {node['namespace']}")

# 查找特定节点
target_node = "ros2_robot_interface"
matching_nodes = [n for n in nodes if target_node in n['name']]
if matching_nodes:
    print(f"找到节点: {matching_nodes[0]['full_name']}")
```

---

## 快速参考表

| 部分 | Handler 访问 | 发送命令 | 获取状态 | 检查到达 |
|------|-------------|---------|---------|---------|
| **左臂** | `left_arm_handler` | `send_target_stamped()` ⭐<br>`send_target()`<br>`send_joint_positions()` | `get_pose()` ⭐<br>`get_target_pose()` | `check_arrival()` ⭐ |
| **右臂** | `right_arm_handler` | `send_target_stamped()` ⭐<br>`send_target()`<br>`send_joint_positions()` | `get_pose()` ⭐<br>`get_target_pose()` | `check_arrival()` ⭐ |
| **左夹爪** | `left_gripper_handler` | `send_target_command()` ⭐<br>`send_joint_positions()` | `get_target_position()` | `check_arrival()` |
| **右夹爪** | `right_gripper_handler` | `send_target_command()` ⭐<br>`send_joint_positions()` | `get_target_position()` | `check_arrival()` |
| **头部** | 直接方法 | `send_head_joint_positions()` | `get_joint_state()` | `check_arrive('head')` |
| **身体** | 直接方法 | `send_body_joint_positions()` | `get_joint_state()` | `check_arrive('body')` |

---

## 注意事项

1. **线程安全**：所有方法都是线程安全的，可以在多线程环境中使用。

2. **连接检查**：
   - **状态获取方法**（如 `get_end_effector_pose()`, `get_joint_state()`, `check_arrive()` 等）：如果接口未连接，会返回 `None` 而不是抛出异常，调用者需要检查返回值
   - **命令发送方法**（如 `send_end_effector_target()`, `send_fsm_command()` 等）：如果接口未连接，会抛出 `ROS2NotConnectedError` 异常

3. **数据可用性**：
   - `get_pose()`, `get_end_effector_pose()`, `get_joint_state()` 等状态获取方法可能返回 `None`（未连接、数据未到达或过期）
   - `check_arrive()` 在未连接时返回 `None`，需要检查返回值
   - 夹爪的 `check_arrival()` 需要手动传入当前位置
   - 手臂的 `get_target_pose()` 需要配置目标位置话题（`/left_current_target` 或 `/right_current_target`）

4. **坐标系转换**：
   - `send_target_stamped()` 会自动进行 TF 转换到 `base_frame`
   - `send_target()` 不进行转换，需要确保传入的 pose 已在 `base_frame` 下
   - `get_pose()` 和 `get_target_pose()` 返回的位姿都在 `base_frame` 坐标系下
   - `check_arrival()` 比较的两个位姿都在 `base_frame` 下，因此可以安全比较

5. **函数选择建议**：
   - **推荐使用** `send_target_stamped()` 而不是 `send_target()`，因为它支持坐标系转换，更灵活
   - 使用 `get_pose()` 获取当前实际位姿，用于状态监控或计算相对运动
   - 使用 `get_target_pose()` 查询已设置的目标位姿，主要用于调试
   - `check_arrival()` 会自动调用 `get_pose()` 和 `get_target_pose()`，通常不需要手动调用这两个函数

6. **默认阈值**（可在配置中修改）：
   - 手臂位置阈值：`config.pose_position_threshold`（默认 0.05 米）
   - 手臂姿态阈值：`config.pose_orientation_threshold`（默认 0.1）
   - 夹爪位置阈值：`config.gripper_position_threshold`（默认 0.01）
   - 头部/身体关节阈值：`config.position_threshold`（默认 0.05 弧度）

7. **双臂模式检测**：
   - 如果配置了 `right_end_effector_pose_topic`，会自动检测为双臂模式
   - 单臂模式下，`'arm'` 和 `'gripper'` 会自动转换为 `'left_arm'` 和 `'left_gripper'`

8. **目标位置获取**：
   - 手臂的目标位置通过话题订阅获取（`/left_current_target` 或 `/right_current_target`）
   - 如果未配置这些话题，`get_target_pose()` 和 `check_arrival()` 将无法正常工作
