# ROS2 Robot Interface API 参考文档

本文档详细说明每个机器人部分（part）可用的函数及其功能。

## 目录

- [手臂控制 (Arm Handler)](#手臂控制-arm-handler)
- [夹爪控制 (Gripper Handler)](#夹爪控制-gripper-handler)
- [头部控制](#头部控制)
- [身体控制](#身体控制)
- [统一接口方法](#统一接口方法)

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

### 可用函数

#### 1. `get_pose() -> Optional[Pose]`

**功能：** 获取当前 end-effector 的 pose（位置和姿态）

**返回值：**
- `Pose` 对象：包含当前位置和姿态
- `None`：如果数据过期或不存在

**示例：**
```python
current_pose = interface.left_arm_handler.get_pose()
if current_pose:
    print(f"位置: ({current_pose.position.x}, {current_pose.position.y}, {current_pose.position.z})")
    print(f"姿态: ({current_pose.orientation.x}, {current_pose.orientation.y}, {current_pose.orientation.z}, {current_pose.orientation.w})")
```

---

#### 2. `get_target_pose() -> Optional[Pose]`

**功能：** 获取当前设置的目标 pose

**返回值：**
- `Pose` 对象：目标位置和姿态
- `None`：如果未设置目标

**示例：**
```python
target_pose = interface.left_arm_handler.get_target_pose()
if target_pose:
    print(f"目标位置: ({target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z})")
```

---

#### 3. `send_target(pose: Pose) -> None`

**功能：** 发送目标 pose（不带坐标系信息）

**参数：**
- `pose` (Pose): 目标 pose 对象

**说明：**
- 直接发布到 `/left_target` 或 `/right_target` topic
- 不进行 TF 坐标转换
- 会自动更新内部目标 pose 状态

**示例：**
```python
from geometry_msgs.msg import Pose

target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0

interface.left_arm_handler.send_target(target_pose)
```

---

#### 4. `send_target_stamped(frame_id: str, pose: Pose) -> None`

**功能：** 发送带坐标系的目标 pose（推荐使用）

**参数：**
- `frame_id` (str): 坐标系 ID（如 "arm_base", "base_link"）
- `pose` (Pose): 目标 pose 对象

**说明：**
- 发布到 `/left_target/stamped` 或 `/right_target/stamped` topic
- **会自动进行 TF 坐标转换**到 base_frame
- 如果 frame_id 与 base_frame 不同，会自动转换
- 会自动更新内部目标 pose 状态（转换后的）

**示例：**
```python
from geometry_msgs.msg import Pose

target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0

# 发送带坐标系的目标（会自动转换到 arm_base）
interface.left_arm_handler.send_target_stamped("base_link", target_pose)
```

---

#### 5. `send_joint_positions(positions: List[float], fsm_command_callback: Optional[Callable] = None) -> None`

**功能：** 发送关节位置命令（MoveJ 模式）

**参数：**
- `positions` (List[float]): 目标关节位置列表（弧度）
- `fsm_command_callback` (Optional[Callable]): FSM 命令回调函数（可选，默认使用初始化时传入的回调）

**说明：**
- 自动切换到 MOVEJ 状态（FSM 命令 4）
- 发布到关节控制器 topic（如 `/ocs2_wbc_controller/target_joint_position/left`）
- 关节数量必须与配置一致

**示例：**
```python
# 发送6个关节的位置（弧度）
joint_positions = [0.0, 0.5, -1.57, 0.0, 1.57, 0.0]
interface.left_arm_handler.send_joint_positions(joint_positions)

# 使用自定义回调（通常不需要）
interface.left_arm_handler.send_joint_positions(joint_positions, custom_callback)
```

---

#### 6. `check_arrival(pose_threshold: float = 0.06, orient_threshold: float = 0.1) -> Dict[str, Any]`

**功能：** 检查手臂是否到达目标位置

**参数：**
- `pose_threshold` (float): 位置距离阈值（米），默认 0.06
- `orient_threshold` (float): 姿态距离阈值，默认 0.1

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
- 比较当前 pose 和目标 pose
- 位置距离：欧氏距离
- 姿态距离：基于四元数的点积计算
- 会打印详细的检查信息

**示例：**
```python
# 使用默认阈值
result = interface.left_arm_handler.check_arrival()
if result['arrived']:
    print("手臂已到达目标位置")

# 使用自定义阈值
result = interface.left_arm_handler.check_arrival(
    pose_threshold=0.05,      # 5厘米
    orient_threshold=0.08     # 更严格的姿态要求
)
print(f"位置距离: {result['position_distance']:.4f} 米")
print(f"姿态距离: {result['orientation_distance']:.4f}")
```

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

### 可用函数

#### 1. `send_joint_positions(position: float) -> None`

**功能：** 发送夹爪关节位置命令

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

#### 2. `check_arrival(current_position: Optional[float], threshold: float = 0.01) -> Dict[str, Any]`

**功能：** 检查夹爪是否到达目标位置

**参数：**
- `current_position` (Optional[float]): 当前位置（需要从 `get_joint_state()` 获取）
- `threshold` (float): 位置距离阈值，默认 0.01

**返回值：**
```python
{
    'arrived': bool,      # 是否到达目标位置
    'distance': float     # 位置距离
}
```

**说明：**
- **需要手动传入当前位置**（从 joint_state 中提取）
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

#### 3. `get_target_position() -> Optional[float]`

**功能：** 获取当前设置的目标位置

**返回值：**
- `float`：目标位置值
- `None`：如果未设置目标

**示例：**
```python
target_position = interface.left_gripper_handler.get_target_position()
if target_position is not None:
    print(f"目标位置: {target_position}")
```

---

## 头部控制

头部控制通过 `ROS2RobotInterface` 的直接方法访问。

### 可用函数

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

## 身体控制

身体控制通过 `ROS2RobotInterface` 的直接方法访问。

### 可用函数

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

## 统一接口方法

这些方法通过 `ROS2RobotInterface` 直接访问，可以操作多个部分。

### 1. `get_joint_state(categorized: bool = False) -> Dict[str, Any] | None`

**功能：** 获取关节状态

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

### 2. `check_arrive(part: Optional[str] = None, position_threshold: Optional[float] = None) -> Dict[str, Any]`

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

**返回值：**

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
if result['arrived']:
    print("左臂已到达")

# 检查所有部分
results = interface.check_arrive()
print(f"左臂到达: {results.get('left_arm', {}).get('arrived', False)}")
print(f"夹爪到达: {results.get('gripper', {}).get('arrived', False)}")

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

### 3. `send_fsm_command(command: int) -> None`

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

### 4. `send_dual_arm_target_stamped(left_pose: Pose, right_pose: Pose, frame_id: str = "arm_base") -> None`

**功能：** 发送双臂目标 pose（仅双臂模式）

**参数：**
- `left_pose` (Pose): 左臂目标 pose
- `right_pose` (Pose): 右臂目标 pose
- `frame_id` (str): 坐标系 ID，默认 "arm_base"

**说明：**
- 发布到 `/dual_target/stamped` topic
- 会自动更新左右臂 handler 的目标 pose（带 TF 转换）

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

interface.send_dual_arm_target_stamped("arm_base", left_pose, right_pose)
```

---

## 各部分功能总结表

| 部分 | Handler 访问 | 发送命令 | 获取状态 | 检查到达 |
|------|-------------|---------|---------|---------|
| **左臂** | `left_arm_handler` | `send_target()`<br>`send_target_stamped()`<br>`send_joint_positions()` | `get_pose()`<br>`get_target_pose()` | `check_arrival()` |
| **右臂** | `right_arm_handler` | `send_target()`<br>`send_target_stamped()`<br>`send_joint_positions()` | `get_pose()`<br>`get_target_pose()` | `check_arrival()` |
| **左夹爪** | `left_gripper_handler` | `send_joint_positions()` | `get_target_position()` | `check_arrival()` |
| **右夹爪** | `right_gripper_handler` | `send_joint_positions()` | `get_target_position()` | `check_arrival()` |
| **头部** | 直接方法 | `send_head_joint_positions()` | `get_joint_state()` | `check_arrive('head')` |
| **身体** | 直接方法 | `send_body_joint_positions()` | `get_joint_state()` | `check_arrive('body')` |

---

## 注意事项

1. **线程安全**：所有方法都是线程安全的，可以在多线程环境中使用。

2. **连接检查**：使用前确保接口已连接（`interface.is_connected == True`）。

3. **数据可用性**：
   - `get_pose()` 和 `get_joint_state()` 可能返回 `None`（数据未到达或过期）
   - 夹爪的 `check_arrival()` 需要手动传入当前位置

4. **坐标系转换**：
   - `send_target_stamped()` 会自动进行 TF 转换
   - `send_target()` 不进行转换

5. **默认阈值**：
   - 手臂位置阈值：0.06 米
   - 手臂姿态阈值：0.1
   - 夹爪位置阈值：0.01

6. **双臂模式检测**：
   - 如果配置了 `right_end_effector_pose_topic`，会自动检测为双臂模式
   - 单臂模式下，`'arm'` 和 `'gripper'` 会自动转换为 `'left_arm'` 和 `'left_gripper'`

