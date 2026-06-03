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
  - [FSM状态查询](#fsm状态查询)
  - [关节状态获取](#关节状态获取)
  - [手部关节控制](#手部关节控制)
  - [末端执行器位姿获取](#末端执行器位姿获取)
  - [双臂路径与轨迹执行](#双臂路径与轨迹执行)
  - [统一到达检查](#统一到达检查)
  - [等待到达](#等待到达)
  - [坐标转换](#坐标转换)
  - [机器人描述与参数查询](#机器人描述与参数查询)
- [常量 (Constants)](#常量-constants)
- [几何与四元数 (utils.quat_pose)](#几何与四元数-utilsquat_pose)
- [快速参考表](#快速参考表)
- [注意事项](#注意事项)

---

## 手臂控制 (Arm Handler)

手臂控制统一通过 `left_arm_handler` 和 `right_arm_handler` 访问。

### 访问方式

```python
# 左臂（始终可用）
interface.left_arm_handler

# 右臂（仅双臂模式）
interface.right_arm_handler
```

---

### 发送命令


https://github.com/user-attachments/assets/f7fef847-4e6d-4905-93cb-176b4fc81c34


#### `send_target_stamped(frame_id: Optional[str] = None, pose: Optional[Pose] = None) -> None` ⭐ 推荐使用

**功能：** 发送带坐标系信息的目标位姿。

**参数：**
- `frame_id` (`Optional[str]`): 目标位姿所在坐标系，如 `arm_base`、`base_link`、`head_link2`、`left_eef`；可省略，省略时回退到最近订阅到的 `self.frame_id`
- `pose` (`Optional[Pose]`): 目标位姿

**特点：**
- 发布到 `/left_target/stamped` 或 `/right_target/stamped`
- 会自动做 TF 变换，统一转换到 `base_frame`
- 适合跨坐标系目标和相对位姿控制
- 目标位姿后续会通过 `/left_current_target` 或 `/right_current_target` 订阅回来，用于到位判断
- 兼容两种调用方式：`send_target_stamped("frame_id", pose)` 和 `send_target_stamped(pose)`
- 如果省略 `frame_id` 且当前还没有可用的默认 `self.frame_id`，会抛出 `ValueError`

**示例：**
```python
from geometry_msgs.msg import Pose

# 使用其他坐标系
target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0
interface.left_arm_handler.send_target_stamped("head_link2", target_pose)

# 使用末端坐标系做相对运动
relative_pose = Pose()
relative_pose.position.x = 0.15
relative_pose.position.y = 0.0
relative_pose.position.z = 0.0
relative_pose.orientation.w = 1.0
interface.left_arm_handler.send_target_stamped("left_eef", relative_pose)

# 如果已经收到过 pose 订阅并缓存了 self.frame_id，也可以省略 frame_id
interface.left_arm_handler.send_target_stamped(relative_pose)
```

---


https://github.com/user-attachments/assets/b80e994e-e374-4580-b947-769c8c169ba2


#### `send_target(pose: Pose) -> None`

**功能：** 发送不带坐标系信息的目标位姿。

**参数：**
- `pose` (`Pose`): 已经位于 `base_frame` 坐标系下的目标位姿

**特点：**
- 直接发布到 `/left_target` 或 `/right_target`
- 不做 TF 变换
- 只适合你已经明确知道目标位姿就在 `base_frame` 下的场景

**注意：**
- 如果目标位姿不在 `base_frame` 下，应使用 `send_target_stamped()`

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


https://github.com/user-attachments/assets/eed9c1d2-6d9d-4c84-9152-1a9a472440a6


#### `send_joint_positions(positions: List[float]) -> None`

**功能：** 发送关节位置命令（MoveJ）。

**参数：**
- `positions` (`List[float]`): 目标关节角列表，单位为弧度

**特点：**
- 会自动切换到 MOVEJ 状态（FSM 命令 4）
- 发布到对应关节控制器话题，如 `/ocs2_wbc_controller/target_joint_position/left`
- 关节数量必须与当前配置一致

**异常：**
- `ROS2NotConnectedError`: 发布器未初始化
- `ValueError`: 初始化时未提供 FSM 命令回调函数

**示例：**
```python
joint_positions = [0.0, 0.5, -1.57, 0.0, 1.57, 0.0]
interface.left_arm_handler.send_joint_positions(joint_positions)
```

---

### 获取状态

#### `get_pose() -> Optional[Pose]` ⭐ 最常用

**功能：** 获取当前末端执行器实际位姿。

**返回值：**
- `Pose`：当前位置和姿态，位于 `base_frame` 坐标系下
- `None`：数据不存在、过期或暂不可用

**适用场景：**
- 实时状态监控
- 保存初始位姿，用于后续相对运动
- 配合 `check_arrival()` 观察当前状态

**示例：**
```python
current_pose = interface.left_arm_handler.get_pose()
if current_pose:
    print(f"当前位置: ({current_pose.position.x}, {current_pose.position.y}, {current_pose.position.z})")
    print(
        f"姿态: ({current_pose.orientation.x}, {current_pose.orientation.y}, "
        f"{current_pose.orientation.z}, {current_pose.orientation.w})"
    )
```

---

#### `get_target_pose() -> Optional[Pose]`

**功能：** 获取当前目标位姿。

**返回值：**
- `Pose`：当前目标位姿，位于 `base_frame` 坐标系下
- `None`：尚未设置目标，或未配置目标位姿订阅话题

**说明：**
- 目标位姿来自 `/left_current_target` 或 `/right_current_target`
- 常用于调试目标是否已正确下发
- `check_arrival()` 内部也会使用这个目标位姿进行比较

**示例：**
```python
target_pose = interface.left_arm_handler.get_target_pose()
if target_pose:
    print(f"目标位置: ({target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z})")
else:
    print("未设置目标位姿或目标位姿话题未配置")
```

---

#### `get_frame_id() -> Optional[str]`

**功能：** 获取当前目标位姿对应的 `frame_id`。

**返回值：**
- `str`：当前目标位姿的坐标系 ID
- `None`：未配置目标位姿订阅话题

**说明：**
- 该值与 `get_target_pose()` 配套使用
- 常用于确认目标位姿最终使用的是哪个参考坐标系
- 记录数据时也可以一起保存

**示例：**
```python
frame_id = interface.left_arm_handler.get_frame_id()
if frame_id is not None:
    print(f"当前目标 frame_id: {frame_id}")
else:
    print("未配置目标位姿订阅话题，无法获取 frame_id")
```

---

### 检查到达

#### `check_arrival(pose_threshold: float | None = None, orient_threshold: float | None = None) -> Dict[str, Any]` ⭐ 最常用

**功能：** 检查手臂是否到达目标位姿。

**参数：**
- `pose_threshold` (float | None): 位置距离阈值，单位米；为 `None` 时使用 `config.pose_position_threshold`
- `orient_threshold` (float | None): 姿态距离阈值；为 `None` 时使用 `config.pose_orientation_threshold`

**返回值：**
```python
{
    "arrived": bool,
    "distance": float,
    "position_distance": float,
    "orientation_distance": float,
    "status_message": str,
}
```

**特点：**
- 内部会自动读取当前位姿 `get_pose()` 和目标位姿 `get_target_pose()`
- 比较的两个位姿都在 `base_frame` 下
- 位置距离为欧氏距离，姿态距离基于四元数计算
- 若未传阈值，则使用配置中的默认阈值

**示例：**
```python
# 使用默认阈值
result = interface.left_arm_handler.check_arrival()
if result["arrived"]:
    print("手臂已到达目标位置")

# 使用自定义阈值
result = interface.left_arm_handler.check_arrival(
    pose_threshold=0.05,
    orient_threshold=0.08,
)
print(f"位置距离: {result['position_distance']:.4f} 米")
print(f"姿态距离: {result['orientation_distance']:.4f}")

# 一个完整流程
target_pose = Pose()
target_pose.position.x = 0.15
target_pose.position.y = 0.0
target_pose.position.z = 0.0
target_pose.orientation.w = 1.0
interface.left_arm_handler.send_target_stamped("left_eef", target_pose)

while True:
    result = interface.left_arm_handler.check_arrival()
    if result["arrived"]:
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
- `frame_id` (str, 可选): 坐标系 ID；省略时默认 `"arm_base"`（见下方说明）

**说明：**
- **坐标系默认值**：不传 `frame_id` 时默认使用 `"arm_base"`。若你的机器人 TF 或末端位姿话题中**没有**名为 `arm_base` 的坐标系，必须显式传入与实际一致的 `frame_id`（例如与 `left_arm_handler.get_frame_id()` 或当前 pose 消息的 `header.frame_id` 一致）。
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

> ⚠️ **重要：使用夹爪前必须正确配置行程范围**
>
> `ROS2RobotInterfaceConfig` 中的以下两个参数必须根据实际硬件设置，默认值仅为示例：
>
> ```python
> config.gripper_min_position = 0.0
> config.gripper_max_position = 0.0384
> ```
>
> 这两个值会直接影响：
> 1. `send_joint_positions()` 的位置限幅
> 2. `send_position_percent()` 的百分比到实际行程换算
> 3. `get_target_position()` 的内部目标值
> 4. `check_arrival()`         的到位检测结果
>
> 不同夹爪型号的行程范围差异较大，请在使用前替换为实测值。

### 访问方式

```python
# 左夹爪（单臂模式下也使用这个）
interface.left_gripper_handler

# 右夹爪（仅双臂模式）
interface.right_gripper_handler
```

---

### 发送命令


https://github.com/user-attachments/assets/16c98f20-b795-4ef3-9872-06cc6ece01f1


#### `interface.left(right)_gripper_handler.send_target_command(target_value: int) -> None`

**功能：** 发送夹爪开关控制命令，使用 `target_command` 话题。

**参数：**
- `target_value` (int): `0` 表示关闭，`1` 表示打开

**特点：**
- 单臂模式统一使用 `interface.left_gripper_handler.send_target_command(...)`
- 使用 `std_msgs/Int32` 消息类型
- 会自动订阅相同话题并同步 `is_open` 状态
- 如果参数不是 `0` 或 `1`，会抛出 `ValueError`

**自动检测到的话题：**
- 双臂灵巧手：`/left_hand_controller/target_command`、`/right_hand_controller/target_command`
- 双臂夹爪：`/left_gripper_controller/target_command`、`/right_gripper_controller/target_command`
- 单臂灵巧手：`/hand_controller/target_command`
- 单臂夹爪：`/gripper_controller/target_command`

**示例：**
```python
# 打开
interface.left_gripper_handler.send_target_command(1)

# 关闭
interface.left_gripper_handler.send_target_command(0)

# 根据当前状态切换
current_state = interface.left_gripper_handler.is_open
target_value = 0 if current_state else 1
interface.left_gripper_handler.send_target_command(target_value)
```

---


https://github.com/user-attachments/assets/17ff1a2b-1496-4f19-aa37-cbce55c11c04


#### `interface.left(right)_gripper_handler.send_joint_positions(position: float) -> None`

**功能：** 发送夹爪实际行程位置命令，使用位置控制话题。

**参数：**
- `position` (float): 目标行程位置值

**特点：**
- 单臂模式统一使用 `interface.left_gripper_handler.send_joint_positions(...)`
- 位置会自动限制在 `gripper_min_position` 和 `gripper_max_position` 之间
- 会更新内部 `target_position`，并清空位置历史
- 这里的 `position` 是实际行程值，不是百分比

**示例：**
```python
interface.left_gripper_handler.send_joint_positions(0.01)
```

---


https://github.com/user-attachments/assets/5e9842b9-376f-4167-8072-0ac6dab29ca0


#### `interface.left(right)_gripper_handler.send_position_percent(percent: float) -> None`

**功能：** 发送夹爪百分比位置控制命令，使用 `target_percent` 话题。

**参数：**
- `percent` (float): 百分比目标值，范围 `0.0` 到 `1.0`

**特点：**
- 单臂模式统一使用 `interface.left_gripper_handler.send_position_percent(...)`
- 使用 `std_msgs/Float64` 消息类型
- 传入 `int` 会自动转换为 `float`
- 超出 `[0.0, 1.0]` 会抛出 `ValueError`
- 如果 `target_percent` 话题未检测到，会抛出 `ROS2NotConnectedError`
- 会按 `gripper_min_position ~ gripper_max_position` 线性换算并更新内部 `target_position`

**自动检测到的话题：**
- 双臂模式：`/left_gripper_controller/target_percent`、`/right_gripper_controller/target_percent`
- 单臂模式：`/left_gripper_controller/target_percent`

**可用性判断：**
```python
if interface.left_gripper_handler.target_percent_pub is not None:
    interface.left_gripper_handler.send_position_percent(0.5)
```

**示例：**
```python
# 完全关闭
interface.left_gripper_handler.send_position_percent(0.0)

# 中间位置
interface.left_gripper_handler.send_position_percent(0.5)

# 完全打开
interface.left_gripper_handler.send_position_percent(1.0)

# 双臂模式右夹爪
interface.right_gripper_handler.send_position_percent(0.75)
```

---

### 获取状态

#### 获取夹爪当前位置

夹爪当前位置从 `get_joint_state(categorized=True)` 返回的分类字典中读取。

**单个夹爪分类数据结构：**
```python
{
    "names": ["left_gripper_joint"],
    "positions": [0.012],
    "velocities": [0.0],
    "efforts": [0.0],
}
```

**单臂模式：**
```python
joint_state = interface.get_joint_state(categorized=True)
if joint_state:
    gripper_data = joint_state.get("gripper", {})
    current_pos = gripper_data.get("positions", [None])[0]
    print(f"夹爪当前位置: {current_pos}")
```

**双臂模式：**
```python
joint_state = interface.get_joint_state(categorized=True)
if joint_state:
    left_pos = joint_state.get("left_gripper", {}).get("positions", [None])[0]
    right_pos = joint_state.get("right_gripper", {}).get("positions", [None])[0]
    print(f"左夹爪: {left_pos}, 右夹爪: {right_pos}")
```

**通用写法：**
```python
is_dual_arm = interface.config.right_end_effector_pose_topic is not None
joint_state = interface.get_joint_state(categorized=True)
if joint_state:
    key = "left_gripper" if is_dual_arm else "gripper"
    current_pos = joint_state.get(key, {}).get("positions", [None])[0]
```

> **注意：** 如果 `current_pos` 为 `None`，说明 `/joint_states` 里还没有夹爪关节数据，通常在连接后等待 1~2 秒即可。

---

#### `get_target_position() -> Optional[float]`

**功能：** 获取当前内部维护的夹爪目标位置。

**返回值：**
- `float`: 当前目标位置
- `None`: 尚未设置目标

**说明：**
- 这是 `GripperHandler` 内部维护的状态，不是单独从外部话题订阅得到的
- 以下操作会更新它：
  1. `send_joint_positions(position)`：更新为限幅后的实际位置
  2. `send_position_percent(percent)`：先换算为实际位置，再更新
  3. `send_target_command(0/1)` 对应话题回调：关闭时更新为 `gripper_min_position`，打开时更新为 `gripper_max_position`
- `check_arrival()` 会使用该值与当前位置比较，判断是否到达目标

**示例：**
```python
target_position = interface.left_gripper_handler.get_target_position()
if target_position is not None:
    print(f"目标位置: {target_position}")
```

---

### 检查到达

#### `check_arrival(current_position: Optional[float], threshold: float | None = None) -> Dict[str, Any]`

**功能：** 检查夹爪是否到达目标位置。

**参数：**
- `current_position` (Optional[float]): 当前夹爪位置，需要从 `get_joint_state(categorized=True)` 中提取
- `threshold` (float | None): 可选的距离阈值；为 `None` 时使用 `config.gripper_position_threshold`

**返回值：**
```python
{
    "arrived": bool,
    "distance": float,
}
```

**特点：**
- 需要手动传入当前位置
- 打开方向主要按距离阈值判断
- 关闭方向除了距离阈值，还会结合位置历史的稳定性判断，适合夹住物体时使用

**示例：**
```python
categorized_state = interface.get_joint_state(categorized=True)
gripper_data = categorized_state.get("gripper", {})  # 单臂模式
# gripper_data = categorized_state.get("left_gripper", {})  # 双臂模式
current_position = gripper_data.get("positions", [None])[0]

if current_position is not None:
    result = interface.left_gripper_handler.check_arrival(current_position)
    if result["arrived"]:
        print("夹爪已到达目标位置")

result = interface.left_gripper_handler.check_arrival(
    current_position=current_position,
    threshold=0.005,
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

#### `send_waist_lifting_pose_relative(dx: float, dz: float, dphi: float) -> None`

**功能：** 发送腰部局部 x/z/phi 相对移动命令

**参数：**
- `dx` (`float`): 局部 x 方向相对位移
- `dz` (`float`): 局部 z 方向相对位移
- `dphi` (`float`): 局部平面角相对变化，单位 rad

**说明：**
- 发布到 `/body_joint_controller/waist_lifting_pose_relative`
- 消息类型为 `std_msgs/msg/Float64MultiArray`
- 实际数据为 `[dx, dz, dphi]`
- 控制器侧要求当前状态为 MOVEJ

**示例：**
```python
interface.send_waist_lifting_pose_relative(0.02, 0.05, 0.10)
```

#### `send_waist_lifting_pose_absolute(x: float, z: float, phi: float) -> None`

**功能：** 发送腰部 x/z/phi 绝对目标命令

**参数：**
- `x` (`float`): `base_footprint` 坐标系下目标 x
- `z` (`float`): `base_footprint` 坐标系下目标 z
- `phi` (`float`): `body_base` 平面角目标，单位 rad

**说明：**
- 发布到 `/body_joint_controller/waist_lifting_pose_absolute`
- 消息类型为 `std_msgs/msg/Float64MultiArray`
- 实际数据为 `[x, z, phi]`
- 控制器侧会将 `(x, z)` 从 `base_footprint` 转到 `body_base` 后执行
- 控制器侧要求当前状态为 MOVEJ

**示例：**
```python
interface.send_waist_lifting_pose_absolute(0.12, 0.45, 0.20)
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

### FSM状态查询

#### `get_fsm_command() -> int`

**功能：** 获取当前 FSM 命令值。

**返回值：**
- `1`: HOME
- `2`: HOLD
- `3`: OCS2
- `4`: MOVEJ

#### `get_fsm_state() -> str`

**功能：** 获取当前 FSM 状态名。

**返回值：**
- `"HOME"` / `"HOLD"` / `"OCS2"` / `"MOVEJ"`

**示例：**
```python
cmd = interface.get_fsm_command()
state = interface.get_fsm_state()
print(f"FSM command={cmd}, state={state}")
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

### 手部关节控制

#### `send_left_hand_joint_positions(positions: List[float]) -> None`

**功能：** 发送左手（灵巧手）关节位置命令。

**说明：**
- 发布到 `left_hand_joint_controller_topic`（自动检测或手动配置）
- 适用于需要独立控制灵巧手每个关节的位置场景

#### `send_right_hand_joint_positions(positions: List[float]) -> None`

**功能：** 发送右手（灵巧手）关节位置命令（双臂模式）。

**示例：**
```python
# 例如每只手 6 个关节
interface.send_left_hand_joint_positions([0.0, 0.3, 0.5, 0.2, 0.1, 0.0])
interface.send_right_hand_joint_positions([0.0, 0.3, 0.5, 0.2, 0.1, 0.0])
```

---

### 末端执行器位姿获取

#### `interface.left_arm_handler.get_pose() -> Optional[Pose]`

**功能：** 获取左臂末端执行器的当前位姿。

**返回值：**
- `Pose` 对象：左臂末端执行器的当前位置和姿态（在 `base_frame` 坐标系下）
- `None`：如果位姿不可用

**说明：**
- 返回的位姿在 `base_frame` 坐标系下

**示例：**
```python
# 获取左臂末端执行器位姿
pose = interface.left_arm_handler.get_pose()
if pose:
    print(f"位置: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
    print(f"姿态: ({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})")
else:
    print("位姿不可用")
```

---

#### `interface.right_arm_handler.get_pose() -> Optional[Pose]`

**功能：** 获取右臂末端执行器的当前位姿（双臂模式）。

**返回值：**
- `Pose` 对象：右臂末端执行器的当前位置和姿态（在 `base_frame` 坐标系下）
- `None`：如果位姿不可用

**说明：**
- 仅在双臂模式下可用
- 返回的位姿在 `base_frame` 坐标系下

**示例：**
```python
# 获取右臂末端执行器位姿（双臂模式）
pose = interface.right_arm_handler.get_pose()
if pose:
    print(f"位置: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
else:
    print("位姿不可用")
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

### 双臂路径与轨迹执行

#### `send_target_path(left_poses, right_poses, frame_id: Optional[str] = None) -> None`

**功能：** 通过 topic 发送双臂路径（`nav_msgs/Path`）。

**说明：**
- 要求双臂模式
- 左右轨迹点数量需要一致（按阶段一一对应）
- 可传入 `Pose` 或 `PoseStamped`
- 会清空旧 target 缓存，避免到达判断误判

#### `execute_path(left_poses, right_poses, trajectory_duration: float = 0.0, frame_id: Optional[str] = None) -> bool`

**功能：** 通过 `ExecutePath` service 执行双臂路径并等待服务响应。

**说明：**
- 不要求左右轨迹点数量一致（支持不等长路径）

**返回值：**
- `bool`: 服务返回的执行成功标志

**示例：**
```python
from geometry_msgs.msg import Pose

def make_pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p

# 左右路径点数量可以不一致
left_poses = [
    make_pose(0.40, 0.20, 0.30),
    make_pose(0.45, 0.20, 0.32),
    make_pose(0.50, 0.20, 0.34),
]
right_poses = [
    make_pose(0.40, -0.20, 0.30),
    make_pose(0.48, -0.20, 0.33),
]

ok = interface.execute_path(
    left_poses=left_poses,
    right_poses=right_poses,
    trajectory_duration=2.0,
    frame_id="arm_base",
)
print(f"execute_path success: {ok}")
```

#### `send_joint_trajectory(joint_names: List[str], waypoints: List[List[float]], trajectory_duration: float | None = None) -> None`

**功能：** 发送多路点关节轨迹（`JointTrajectory`），支持单臂/双臂统一接口。

**参数：**
- `trajectory_duration`：可选，整条轨迹总时长（秒）。若指定，会在发布前将臂控制器节点参数 `movej_trajectory_duration` 设为该值（与 ros2-viser 控制器配置一致）；`None` 则沿用控制器当前参数。

**示例：**
```python
# 左臂轨迹示例
joint_names = ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"]
waypoints = [
    [0.0, 0.3, -0.2, 0.0, 1.2, 0.0, 0.0],
    [0.1, 0.4, -0.3, 0.1, 1.1, 0.1, 0.0],
]
interface.send_joint_trajectory(joint_names, waypoints, trajectory_duration=5.0)
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

### 等待到达

#### `wait_until_arrive(part: str = "arm", timeout: float = 3.0, poll_period: float = 0.05, position_threshold: Optional[float] = None, ...) -> Dict[str, Any]`

**功能：** 轮询 `check_arrive()` 并在超时前等待指定部分到达目标，替代固定 `sleep`。

**返回值：**
```python
{
    "arrived": bool,
    "elapsed": float,
    "result": dict | None
}
```

**示例：**
```python
wait_result = interface.wait_until_arrive(
    part="left_arm",
    timeout=5.0,
    poll_period=0.05,
)
if not wait_result["arrived"]:
    print(f"超时未到达，耗时: {wait_result['elapsed']:.2f}s")
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

### 机器人描述与参数查询

#### `get_robot_description() -> Optional[str]`

**功能：** 获取最近一次接收到的 `robot_description`（URDF XML 字符串）。

#### `has_robot_description() -> bool`

**功能：** 判断是否已接收到 `robot_description`。

#### `list_node_parameters(full_node_name: str) -> List[Dict[str, Any]]`

**功能：** 查询指定节点的可动态参数及其信息。

#### `set_node_parameters(full_node_name: str, parameters: Dict[str, Any]) -> bool`

**功能：** 批量设置指定节点参数。

**示例：**
```python
node_name = "/my_controller"
params = interface.list_node_parameters(node_name)
print(f"可配置参数数量: {len(params)}")

ok = interface.set_node_parameters(node_name, {
    "trajectory_duration": 2.0,
    "enable_safety_check": True,
})
print(f"参数设置结果: {ok}")
```

---

## 常量 (Constants)

可直接从包顶层导入 FSM 状态常量：

- `FSM_HOME = 1`
- `FSM_HOLD = 2`
- `FSM_OCS2 = 3`
- `FSM_MOVEJ = 4`

```python
from ros2_robot_interface import FSM_HOME, FSM_OCS2
interface.send_fsm_command(FSM_OCS2)
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
   - **状态获取方法**（如 `left_arm_handler.get_pose()`, `get_joint_state()`, `check_arrive()` 等）：如果接口未连接或数据不可用，可能返回 `None`，调用者需要检查返回值
   - **命令发送方法**（如 `left_arm_handler.send_target()`, `send_fsm_command()` 等）：如果接口未连接，会抛出 `ROS2NotConnectedError` 异常

3. **数据可用性**：
   - `get_pose()`, `get_joint_state()` 等状态获取方法可能返回 `None`（未连接、数据未到达或过期）
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

---

## 几何与四元数 (utils.quat_pose)

模块路径：`ros2_robot_interface.utils.quat_pose`（也可从 `ros2_robot_interface.utils` 导入已导出符号）。

四元数为 **scalar-last (x, y, z, w)**，与 `geometry_msgs/Pose.orientation` 一致。

| 函数 | 说明 |
|------|------|
| `quat_multiply(q1, q2)` | 四元数乘积 |
| `quat_conjugate(q)` | 共轭 |
| `quat_normalize(q)` | 单位化（近零时回退为 `(0,0,0,1)`） |
| `rotate_vector_by_quat_inverse(vec, quat_xyzw)` | 用 `quat` 的逆旋转三维向量 |
| `pose_from_tuple(position, orientation_xyzw)` | 元组 → `geometry_msgs.msg.Pose` |

与 LeRobot 扁平 action/观测键相关的 `action_from_pose` / `obs_to_pose` 仍见 **`lerobot_robot_ros2.utils.pose_utils`**。
