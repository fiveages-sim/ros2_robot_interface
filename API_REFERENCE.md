# ROS2 Robot Interface API 参考文档

本文档详细说明每个机器人部分（part）可用的函数及其功能。文档按部分分类，每个部分内按功能（发送命令、获取状态、检查到达）组织。

每个公开方法都有 **原理** 字段，说明底层用的是 Topic、Action、Service、参数服务、TF，还是本地计算（以及它依赖哪条订阅缓存）。名称带 WBC / split 两种写法时，以 `connect()` 自动检测或 `ROS2RobotInterfaceConfig` 里显式配置的为准。

## 目录

- [手臂控制 (Arm Handler)](#手臂控制-arm-handler)
  - [发送命令](#发送命令-1)
  - [获取状态](#获取状态-1)
  - [检查到达](#检查到达-1)
  - [双臂协调控制](#双臂协调控制-1)
  - [笛卡尔 Action（MoveL / MoveC）](#笛卡尔-actionmovel--movec)
  - [关节轨迹 Action（MoveJ）](#关节轨迹-actionmovej)
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
  - [腰部位姿 Action](#腰部位姿-action)
  - [获取状态](#获取状态-4)
  - [检查到达](#检查到达-4)
- [统一接口方法](#统一接口方法)
  - [FSM状态切换](#fsm状态切换)
  - [六维力与 COMPLIANCE 力控](#六维力与-compliance-力控)
  - [FSM状态查询](#fsm状态查询)
  - [模式命令与 WBC 状态确认](#模式命令与-wbc-状态确认)
  - [协调关节下发](#协调关节下发)
  - [关节状态获取](#关节状态获取)
  - [手部关节控制](#手部关节控制)
  - [灵巧手触觉读取](#灵巧手触觉读取)
  - [末端执行器位姿获取](#末端执行器位姿获取)
  - [双臂路径与轨迹执行](#双臂路径与轨迹执行)
  - [统一到达检查](#统一到达检查)
  - [等待到达](#等待到达)
  - [坐标转换](#坐标转换)
  - [系统信息查询](#系统信息查询)
  - [控制器节点名](#控制器节点名)
  - [控制器执行参数](#控制器执行参数)
  - [笛卡尔速度](#笛卡尔速度)
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

**原理：**
- 类型：Topic 发布
- 名称：`/left_target/stamped` 或 `/right_target/stamped`（即 `{end_effector_target_topic}/stamped`）
- 消息：`geometry_msgs/msg/PoseStamped`
- 配置：`config.end_effector_target_topic` / `config.right_end_effector_target_topic`
- 到位依赖：控制器回写 `/left_current_target` 或 `/right_current_target`（Topic 订阅，`PoseStamped`）
- 隐式 FSM：切到 OCS2（经 `/fsm_command`）
- 执行参数：消息无时长；`arm_controller` 上的 `movel_*`（见 [控制器执行参数](#控制器执行参数)）

**参数：**
- `frame_id` (`Optional[str]`): 目标位姿所在坐标系，如 `arm_base`、`base_link`、`head_link2`、`left_eef`；可省略，省略时回退到最近订阅到的 `self.frame_id`
- `pose` (`Optional[Pose]`): 目标位姿

**特点：**
- 会自动做 TF 变换，统一转换到 `base_frame`
- 适合跨坐标系目标和相对位姿控制
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


#### `send_relative(dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0, frame_id="") -> None`

**功能：** 发送一次笛卡尔相对位移，叠到当前指令目标上并走 MoveL。

**原理：**
- 类型：Topic 发布
- 名称：`{end_effector_target_topic}/relative`，即 `/left_target/relative` 或 `/right_target/relative`
- 消息：`geometry_msgs/msg/TwistStamped`
- 到位依赖：控制器回写 `/left_current_target` 或 `/right_current_target`
- 隐式 FSM：切到 OCS2（经 `/fsm_command`）
- 执行参数：同上，`arm_controller` 上的 `movel_*`

**参数：**
- `dx` / `dy` / `dz` (`float`): 平移增量，单位米，表达在 `frame_id` 下
- `droll` / `dpitch` / `dyaw` (`float`): 姿态增量，单位弧度，对应 roll / pitch / yaw
- `frame_id` (`str`): 增量坐标系；默认空字符串，控制器按内部 `base_frame` 处理

**说明：**
- **不是**该坐标系下的绝对位姿。`dx=0.05` 表示沿该轴再偏 5 cm，不会把末端送到 `(0.05, 0, 0)`
- `header.stamp` 由接口填写，控制器不用于 TF 查询
- 发布前会清空 `latest_target_pose`，避免到位误判
- 未创建 publisher 时抛 `ROS2NotConnectedError`

**示例：**
```python
# 沿控制器 base_frame 的 X 正向偏移 3 cm
interface.left_arm_handler.send_relative(0.03, 0.0, 0.0)

# 沿末端 X 伸出 3 cm
interface.left_arm_handler.send_relative(0.03, 0.0, 0.0, frame_id="left_eef")
```

---


https://github.com/user-attachments/assets/b80e994e-e374-4580-b947-769c8c169ba2


#### `send_target(pose: Pose) -> None`

**功能：** 发送不带坐标系信息的目标位姿。

**原理：**
- 类型：Topic 发布
- 名称：`/left_target` 或 `/right_target`
- 消息：`geometry_msgs/msg/Pose`
- 配置：`config.end_effector_target_topic` / `config.right_end_effector_target_topic`
- 隐式 FSM：切到 OCS2（经 `/fsm_command`）
- 执行参数：同上，`arm_controller` 上的 `movel_*`

**参数：**
- `pose` (`Pose`): 已经位于 `base_frame` 坐标系下的目标位姿

**特点：**
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

**原理：**
- 类型：Topic 发布
- 名称（自动检测，WBC 优先）：
  - 左臂：`/ocs2_wbc_controller/target_joint_position/left` 或 `/ocs2_arm_controller/target_joint_position/left`；单臂也可为 `/ocs2_arm_controller/target_joint_position`
  - 右臂：`/ocs2_wbc_controller/target_joint_position/right` 或 `/ocs2_arm_controller/target_joint_position/right`
- 消息：`std_msgs/msg/Float64MultiArray`
- 配置：`config.left_arm_joint_controller_topic` / `config.right_arm_joint_controller_topic`
- 隐式 FSM：切到 MOVEJ（经 `/fsm_command`）；已是 MOVEJ 则跳过
- 执行参数：对应臂控节点上的 `movej_duration`、`movej_interpolation_type`、`movej_tanh_scale`（见 [控制器执行参数](#控制器执行参数)）

**参数：**
- `positions` (`List[float]`): 目标关节角列表，单位为弧度

**特点：**
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

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`/left_current_pose` 或 `/right_current_pose`
- 消息：`geometry_msgs/msg/PoseStamped`（返回其中的 `.pose`）
- 配置：`config.end_effector_pose_topic` / `config.right_end_effector_pose_topic`

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

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`/left_current_target` 或 `/right_current_target`
- 消息：`geometry_msgs/msg/PoseStamped`（返回其中的 `.pose`）
- 配置：`config.end_effector_current_target_topic` / `config.right_end_effector_current_target_topic`（`connect()` 可自动检测）

**返回值：**
- `Pose`：当前目标位姿，位于 `base_frame` 坐标系下
- `None`：尚未设置目标，或未配置目标位姿订阅话题

**说明：**
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

**功能：** 获取当前末端位姿订阅消息里缓存的 `frame_id`。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：与 `get_pose()` 相同（`/left_current_pose` 或 `/right_current_pose`）
- 来源：第一次收到的 `PoseStamped.header.frame_id`
- 注意：若未配置 `current_target` 话题，此方法直接返回 `None`（实现上与 `get_target_pose()` 共用可用性守卫）

**返回值：**
- `str`：当前位姿的坐标系 ID
- `None`：未配置目标位姿订阅话题，或尚未收到 pose

**说明：**
- 常用于 `send_target_stamped()` 省略 `frame_id` 时的默认坐标系
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

**原理：**
- 类型：本地计算（比较缓存位姿）
- 当前位姿：Topic 订阅 `/left_current_pose` 或 `/right_current_pose`
- 目标位姿：Topic 订阅 `/left_current_target` 或 `/right_current_target`
- 未配置目标话题时无法判定到达

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

#### `send_dual_arm_target_stamped(left_pose: Pose, right_pose: Pose, frame_id: str = "arm_base", body_pose: Optional[Pose] = None, body_frame_id: str = "base_footprint", body_mode: Optional[str] = None, movel_duration: Optional[float] = None) -> None`

**功能：** 发送双臂目标 pose（仅双臂模式）；WBC 模式下可附带腰部/身体目标。

**原理：**
- 类型：Topic 发布（主通道）+ 可选参数服务
- 名称：`/dual_target/stamped`
- 消息：`nav_msgs/msg/Path`（双臂 2 个 `PoseStamped`；WBC + `body_pose` 时为 `[left, right, body]`）
- 可选 `movel_duration`：对 `arm_controller` 节点调用 **参数服务** `SetParameters`，写 `movel_duration`
- 其余 `movel_*`（速度/加速度/jerk、`movel_auto_extend_duration`）同样生效，见 [控制器执行参数](#控制器执行参数)
- 可选 `body_mode`：Topic 发布 `/mode_command`（`std_msgs/String`）
- 到位依赖：订阅 `/left_current_target`、`/right_current_target`
- 隐式 FSM：切到 OCS2

**参数：**
- `left_pose` (Pose): 左臂目标 pose（在 `frame_id` 指定的坐标系下）
- `right_pose` (Pose): 右臂目标 pose（在 `frame_id` 指定的坐标系下）
- `frame_id` (str, 可选): 坐标系 ID；省略时默认 `"arm_base"`（见下方说明）
- `body_pose` (Pose, 可选): 腰部/身体目标 pose。仅 WBC 模式支持；非 WBC 模式传入会抛出 `ValueError`。
- `body_frame_id` (str, 可选): `body_pose` 所在坐标系，默认 `"base_footprint"`。
- `body_mode` (str, 可选): body 模式。传入 `body_pose` 时只能为 `"BODY_TRACKING"` 或省略；省略时自动使用 `"BODY_TRACKING"`。
- `movel_duration` (float, 可选): 发布目标前设置控制器参数 `movel_duration`，单位秒。该参数是控制器全局参数，会影响后续调用，直到再次修改。

**说明：**
- **坐标系默认值**：不传 `frame_id` 时默认使用 `"arm_base"`。若你的机器人 TF 或末端位姿话题中**没有**名为 `arm_base` 的坐标系，必须显式传入与实际一致的 `frame_id`（例如与 `left_arm_handler.get_frame_id()` 或当前 pose 消息的 `header.frame_id` 一致）。
- **内部实现**：直接发布到 `/dual_target/stamped`，不调用 handler 的 `send_target_stamped()`
- 可以使用 `left_arm_handler.check_arrival()` 和 `right_arm_handler.check_arrival()` 分别检查到达状态
- 只有 WBC 模式支持通过该接口发送 `body_pose`。分体模式如果传入 `body_pose` 会直接报错，避免“看起来发送了腰部目标但实际不生效”的误用。
- 腰部目标要参与控制时必须使用 `BODY_TRACKING`。因此传入 `body_pose` 时，`body_mode` 只能省略或显式传 `"BODY_TRACKING"`。
- `movel_duration` 不是消息字段，而是控制器节点参数。接口会在发布目标前修改参数，并输出日志说明修改后的值。

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

# WBC：双臂 + 身体目标，并设置本次 movel_duration
body_pose = Pose()
body_pose.position.x = 0.1
body_pose.position.z = 0.75
body_pose.orientation.w = 1.0
interface.send_dual_arm_target_stamped(
    left_pose,
    right_pose,
    body_pose=body_pose,
    body_mode="BODY_TRACKING",
    movel_duration=3.0,
)

# 检查到达状态
left_result = interface.left_arm_handler.check_arrival()
right_result = interface.right_arm_handler.check_arrival()
if left_result['arrived'] and right_result['arrived']:
    print("双臂都已到达目标位置")
```

---

#### `send_dual_arm_joint_positions(left_arm_positions: List[float], right_arm_positions: List[float], body_positions: Optional[List[float]] = None, head_positions: Optional[List[float]] = None) -> None`

**功能：** 发送双臂关节位置命令（MoveJ 模式，统一 topic 控制）

**原理：**
- 类型：Topic 发布
- 名称：`config.unified_arm_joint_controller_topic`
  - WBC：`/ocs2_wbc_controller/target_joint_position`
  - ARM：`/ocs2_arm_controller/target_joint_position`
- 消息：`std_msgs/msg/Float64MultiArray`
- 缺某一臂 / 躯干 / 头部时从 `/joint_states` hold 当前角
- 隐式 FSM：切到 MOVEJ
- 执行参数：统一控制器节点上的 `movej_*`

**参数：**
- `left_arm_positions` (List[float]): 左臂关节位置列表（弧度）
- `right_arm_positions` (List[float]): 右臂关节位置列表（弧度）
- `body_positions` (Optional[List[float]]): 躯干目标；仅 WBC 生效。省略时从 `/joint_states` 读当前角
- `head_positions` (Optional[List[float]]): 头部目标；仅 WBC 生效。省略时从 `/joint_states` 读当前角

**说明：**
- 同时控制左臂和右臂的所有关节，发布到统一的 topic
- **自动检测控制器类型**：
  - **WBC 控制器** (`/ocs2_wbc_controller/target_joint_position`)：`body_joints + left_arm_joints + right_arm_joints + head_joints`，顺序优先遵循 `config.joint_names`
    - 未传入 `body_positions` / `head_positions` 时从当前关节状态补齐
    - 如果没有身体关节数据，使用默认值（4 个零值）
  - **ARM 控制器** (`/ocs2_arm_controller/target_joint_position`)：只需要 `left_arm_joints + right_arm_joints`（14 个关节）；传入 body/head 会被忽略并告警
- 左臂和右臂的关节数量必须相同

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

### 笛卡尔 Action（MoveL / MoveC）

这组接口走 **Action**，会阻塞等待结果（或超时）。默认 action 名指向 `ocs2_arm_controller`；若控制器带命名空间，在 config 里覆盖。

#### `wait_for_movel_action_server(timeout: float = 5.0) -> bool`

**功能：** 等待 MoveL action server 就绪。

**原理：**
- 类型：Action（探测 server）
- 名称：`config.movel_action_name`（默认 `/ocs2_arm_controller/execute_linear`）
- 类型：`arms_ros2_control_msgs/action/ExecuteLinear`
- client 未创建时返回 `False`

#### `wait_for_movec_action_server(timeout: float = 5.0) -> bool`

**功能：** 等待 MoveC action server 就绪。

**原理：**
- 类型：Action（探测 server）
- 名称：`config.movec_action_name`（默认 `/ocs2_arm_controller/execute_circle_use_ik`）
- 类型：`arms_ros2_control_msgs/action/MovecUseIK`

#### `execute_movel_action(arm_name, endpoint_pose, *, duration=3.0, time_mode=True, frame_id=None, ik_type=None, right_endpoint_pose=None, max_linear_velocity=None, max_linear_acceleration=None, max_linear_jerk=None, max_angular_velocity=None, max_angular_acceleration=None, max_angular_jerk=None, auto_switch_fsm=True, feedback_callback=None, timeout=30.0, wait_for_server_timeout=5.0) -> Any`

**功能：** 参数化直线笛卡尔运动（MoveL），发送 goal 并等待 result。

**原理：**
- 类型：Action
- 名称：`config.movel_action_name`（默认 `/ocs2_arm_controller/execute_linear`）
- 类型：`arms_ros2_control_msgs/action/ExecuteLinear`（goal 内嵌 `LinearMessage`）
- 隐式 FSM：`auto_switch_fsm=True` 时切到 MOVEJ（经 `/fsm_command`）

**参数（节选）：**
- `arm_name` (`str`): `"left"` / `"right"` / `"both"`
- `endpoint_pose`: 终点位姿（`Pose` 或 `PoseStamped`）
- `right_endpoint_pose`: 双臂时右臂终点
- `feedback_callback`: 收到的是 action feedback 对象
- 返回：action result；被拒绝或超时返回 `None`

**示例：**
```python
from geometry_msgs.msg import Pose

goal = Pose()
goal.position.x = 0.45
goal.position.y = 0.20
goal.position.z = 0.30
goal.orientation.w = 1.0
result = interface.execute_movel_action("left", goal, duration=3.0, frame_id="arm_base")
```

#### `execute_movec_action_three_point(arm_name, midpoint_pose, endpoint_pose, rotate_angle=0.0, *, duration=6.0, ...) -> Any`

**功能：** 三点法圆弧笛卡尔运动（MoveC）。

**原理：**
- 类型：Action
- 名称：`config.movec_action_name`（默认 `/ocs2_arm_controller/execute_circle_use_ik`）
- 类型：`arms_ros2_control_msgs/action/MovecUseIK`（`use_three_point_method=True`）
- 隐式 FSM：默认切到 MOVEJ

#### `execute_movec_action_parametric(arm_name, center, axis, rotate_angle, *, endpoint_pose=None, duration=6.0, ...) -> Any`

**功能：** 圆心 / 轴 / 转角参数化圆弧（MoveC）。

**原理：**
- 类型：Action
- 名称：同上 `config.movec_action_name`
- 类型：`arms_ros2_control_msgs/action/MovecUseIK`（`use_three_point_method=False`）
- 隐式 FSM：默认切到 MOVEJ

---

### 关节轨迹 Action（MoveJ）

#### `wait_for_joint_trajectory_action_server(timeout: float = 5.0) -> bool`

**功能：** 等待参数化 MoveJ action server 就绪。

**原理：**
- 类型：Action（探测 server）
- 名称：`config.joint_trajectory_action_name`（默认 `/ocs2_arm_controller/joint_trajectory_with_para`）
- 类型：`arms_ros2_control_msgs/action/JointTrajectory`

#### `execute_joint_trajectory_action(joint_names, waypoints, *, time_mode=True, total_time=None, max_velocity=None, max_acceleration=None, max_jerk=None, auto_switch_fsm=True, feedback_callback=None, timeout=30.0, wait_for_server_timeout=5.0) -> Any`

**功能：** 带速度/加速度/jerk 约束的参数化关节轨迹（MoveJ action）。

**原理：**
- 类型：Action
- 名称：`config.joint_trajectory_action_name`（默认 `/ocs2_arm_controller/joint_trajectory_with_para`）
- 类型：`arms_ros2_control_msgs/action/JointTrajectory`（waypoint 为 `JointWaypoint`）
- 隐式 FSM：默认切到 MOVEJ
- 与 `send_joint_trajectory()` 不同：后者是 **Topic 发布** `/{controller}/target_joint_trajectory`，本方法走 Action 并等待结果

**示例：**
```python
result = interface.execute_joint_trajectory_action(
    ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"],
    [[0.0, 0.3, -0.2, 0.0, 1.2, 0.0, 0.0]],
    total_time=4.0,
)
```

#### `execute_dual_arm_movej_action(left_arm_positions, right_arm_positions, *, duration=None, ...) -> Any`

**功能：** 双臂参数化 MoveJ 的便捷封装（把左右臂关节拼成一条 `execute_joint_trajectory_action`）。

**原理：** 同 `execute_joint_trajectory_action`（同一个 Action）。

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

**原理：**
- 类型：Topic 发布 + 同名 Topic 订阅（同步 `is_open`）
- 名称：`/{controller_name}/target_command`
  - 双臂灵巧手：`/left_hand_controller/target_command`、`/right_hand_controller/target_command`
  - 双臂夹爪：`/left_gripper_controller/target_command`、`/right_gripper_controller/target_command`
  - 单臂灵巧手：`/hand_controller/target_command`
  - 单臂夹爪：`/gripper_controller/target_command`
- 消息：`std_msgs/msg/Int32`（`0` 关闭，`1` 打开）
- 配置：由 `connect()` 检测 `left_gripper_controller_name` / `right_gripper_controller_name`

**参数：**
- `target_value` (int): `0` 表示关闭，`1` 表示打开

**特点：**
- 单臂模式统一使用 `interface.left_gripper_handler.send_target_command(...)`
- 会自动订阅相同话题并同步 `is_open` 状态
- 如果参数不是 `0` 或 `1`，会抛出 `ValueError`

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

**原理：**
- 类型：Topic 发布
- 名称：`config.gripper_command_topic` / `config.right_gripper_command_topic`
  - 常见：`/left_gripper_joint/position_command`、`/right_gripper_joint/position_command`
- 消息：`std_msgs/msg/Float64`
- 无对应 ROS 订阅：目标值缓存在 handler 内部，供 `check_arrival()` 使用

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

**原理：**
- 类型：Topic 发布
- 名称：`config.left_gripper_target_percent_topic` / `config.right_gripper_target_percent_topic`
  - 双臂：`/left_gripper_controller/target_percent`、`/right_gripper_controller/target_percent`
  - 单臂：`/gripper_controller/target_percent` 或 `/left_gripper_controller/target_percent`
- 消息：`std_msgs/msg/Float64`（0.0~1.0）
- 话题未检测到时抛 `ROS2NotConnectedError`

**参数：**
- `percent` (float): 百分比目标值，范围 `0.0` 到 `1.0`

**特点：**
- 单臂模式统一使用 `interface.left_gripper_handler.send_position_percent(...)`
- 传入 `int` 会自动转换为 `float`
- 超出 `[0.0, 1.0]` 会抛出 `ValueError`
- 会按 `gripper_min_position ~ gripper_max_position` 线性换算并更新内部 `target_position`

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

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.joint_states_topic`（默认 `/joint_states`）
- 消息：`sensor_msgs/msg/JointState`
- handler 本身不单独订阅夹爪状态话题（`config.gripper_state_topic` 目前未使用）

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

**原理：**
- 类型：本地缓存（不是独立 ROS 订阅）
- 由 `send_joint_positions` / `send_position_percent` 写入；`send_target_command` 经同名话题回调更新为 min/max 行程

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

**原理：**
- 类型：本地计算
- 目标：handler 内部 `target_position`
- 当前值：调用方从 `/joint_states`（经 `get_joint_state(categorized=True)`）传入

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

**原理：**
- 类型：Topic 发布
- 名称：`config.head_joint_controller_topic`
  - WBC：`/ocs2_wbc_controller/target_joint_position/head`
  - split：`/head_joint_controller/target_joint_position`
- 消息：`std_msgs/msg/Float64MultiArray`
- 隐式 FSM：切到 MOVEJ
- 目标缓存在接口内部，供 `check_arrive(part='head')` 使用
- 执行参数：头部控制器（或 WBC）节点上的 `movej_*`

**参数：**
- `positions` (List[float]): 目标关节位置列表（弧度）

**说明：**
- 会自动保存目标位置（用于 `check_arrive()`）
- 关节数量必须与配置一致
- 未检测到 topic 时 `logger.warning` 后返回，不抛异常

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

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.joint_states_topic`（默认 `/joint_states`）
- 消息：`sensor_msgs/msg/JointState`
- `categorized=True` 时按名称规则拆成 arm / gripper / head / body 等（`is_head_joint_name` / `is_body_joint_name`）

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

**原理：**
- 类型：本地计算
- 当前值：Topic 订阅 `/joint_states` 中的 head 关节
- 目标值：最近一次 `send_head_joint_positions()` 缓存（或 WBC 统一下发时写入的 head 缓存）

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

**原理：**
- 类型：Topic 发布
- 名称：`config.body_joint_controller_topic`
  - WBC：`/ocs2_wbc_controller/target_joint_position/body`
  - split：`/body_joint_controller/target_joint_position`
- 消息：`std_msgs/msg/Float64MultiArray`
- 隐式 FSM：切到 MOVEJ
- 目标缓存在接口内部，供 `check_arrive(part='body')` 使用
- 执行参数：`body_controller` 上的 `movej_*`

**参数：**
- `positions` (List[float]): 目标关节位置列表（弧度）

**说明：**
- 会自动保存目标位置（用于 `check_arrive()`）
- 关节数量必须与配置一致
- 未检测到 topic 时 `logger.warning` 后返回，不抛异常

**示例：**
```python
# 发送身体关节位置（例如：4个关节）
body_positions = [0.0, 0.0, 0.0, 0.0]  # 弧度
interface.send_body_joint_positions(body_positions)
```

#### `send_body_relative(dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0, frame_id="") -> None`

**功能：** 发送身体一次笛卡尔相对位移，叠到当前身体指令目标上并走 MoveL。

**原理：**
- 类型：Topic 发布
- 名称：`config.body_target_relative_topic`（自动检测 `/body_target/relative`）
- 消息：`geometry_msgs/msg/TwistStamped`
- 隐式 FSM：切到 OCS2；再经 `/mode_command` 切到 `BODY_TRACKING`（已是该模式则不重复发）
- 到位依赖：`/body_current_target`
- 执行参数：与手臂笛卡尔相同，`arm_controller` 上的 `movel_*`

**参数：** 与 `ArmHandler.send_relative` 相同（米 / 弧度 RPY；`frame_id` 默认空）

**说明：**
- 同样是一次增量，不是绝对位姿
- 仅 WBC 图上通常才有该 topic；未检测到时 `logger.warning` 后返回，不抛异常
- 发布前清空 `body_current_target_pose`

**示例：**
```python
interface.send_body_relative(0.03, 0.0, 0.0)
```

#### `send_waist_lifting_relative_position(position: float) -> None`

**功能：** 发送腰部升降相对位置命令（单标量）。

**原理：**
- 类型：Topic 发布
- 名称：`config.waist_lifting_topic`
  - split：`/body_joint_controller/waist_lifting`
  - WBC：`/ocs2_wbc_controller/waist_lifting`
- 消息：`std_msgs/msg/Float64`
- 隐式 FSM：切到 MOVEJ
- 执行参数：`body_controller` 上的 `waist_lifting_duration`（默认 3 s）

**示例：**
```python
interface.send_waist_lifting_relative_position(0.05)
```

#### `send_waist_lifting_pose_relative(dx: float, dz: float, dphi: float) -> None`

**功能：** 发送腰部局部 x/z/phi 相对移动命令

**原理：**
- 类型：Topic 发布
- 名称：`config.waist_lifting_pose_relative_topic`
  - split：`/body_joint_controller/waist_lifting_pose_relative`
  - WBC：`/ocs2_wbc_controller/waist_lifting_pose_relative`
- 消息：`std_msgs/msg/Float64MultiArray`，数据 `[dx, dz, dphi]`
- 隐式 FSM：切到 MOVEJ
- 执行参数：`body_controller` 上的 `waist_lifting_duration`（默认 3 s）

**参数：**
- `dx` (`float`): 局部 x 方向相对位移
- `dz` (`float`): 局部 z 方向相对位移
- `dphi` (`float`): 局部平面角相对变化，单位 rad

**说明：**
- 控制器侧要求当前状态为 MOVEJ

**示例：**
```python
interface.send_waist_lifting_pose_relative(0.02, 0.05, 0.10)
```

#### `send_waist_lifting_pose_absolute(x: float, z: float, phi: float) -> None`

**功能：** 发送腰部 x/z/phi 绝对目标命令

**原理：**
- 类型：Topic 发布
- 名称：`config.waist_lifting_pose_absolute_topic`
  - split：`/body_joint_controller/waist_lifting_pose_absolute`
  - WBC：`/ocs2_wbc_controller/waist_lifting_pose_absolute`
- 消息：`std_msgs/msg/Float64MultiArray`，数据 `[x, z, phi]`
- 隐式 FSM：切到 MOVEJ
- 执行参数：`body_controller` 上的 `waist_lifting_duration`（默认 3 s）

**参数：**
- `x` (`float`): `base_footprint` 坐标系下目标 x
- `z` (`float`): `base_footprint` 坐标系下目标 z
- `phi` (`float`): `body_base` 平面角目标，单位 rad

**说明：**
- 控制器侧会将 `(x, z)` 从 `base_footprint` 转到 `body_base` 后执行
- 控制器侧要求当前状态为 MOVEJ

**示例：**
```python
interface.send_waist_lifting_pose_absolute(0.12, 0.45, 0.20)
```

#### `send_waist_lifting_velocity_scale(velocity_scale: float) -> None`

**功能：** 发送腰部升降速度比例（[-1, 1]）。

**原理：**
- 类型：Topic 发布
- 名称：`config.waist_lifting_command_topic`
  - split：`/body_joint_controller/waist_lifting_command`
  - WBC：`/ocs2_wbc_controller/waist_lifting_command`
- 消息：`std_msgs/msg/Float64`
- 隐式 FSM：切到 MOVEJ
- 执行参数：实际速度 = `scale × waist_lifting_default_parameter[0]`，三项为 `[目标速度, 最大加速度, 最大加加速度]`

**示例：**
```python
interface.send_waist_lifting_velocity_scale(0.3)
```

#### `send_waist_turning_velocity_scale(velocity_scale: float) -> None`

**功能：** 发送腰部旋转速度比例（[-1, 1]）。

**原理：**
- 类型：Topic 发布
- 名称：`config.waist_turning_command_topic`
  - split：`/body_joint_controller/waist_turning_command`
  - WBC：`/ocs2_wbc_controller/waist_turning_command`
- 消息：`std_msgs/msg/Float64`
- 隐式 FSM：切到 MOVEJ
- 执行参数：实际速度 = `scale × waist_turning_default_parameter[0]`，三项含义同上

**示例：**
```python
interface.send_waist_turning_velocity_scale(-0.2)
```

---

### 腰部位姿 Action

相对/绝对 topic 是“发了就走”；这组 Action 会等待规划/到位结果。

#### `wait_for_waist_lifting_pose_action_server(timeout: float = 5.0) -> bool`

**功能：** 等待腰部位姿 action server 就绪。

**原理：**
- 类型：Action（探测 server）
- 名称：`config.waist_lifting_pose_action_name`（可自动检测 `/ocs2_wbc_controller/waist_lifting_pose` 或 `/body_joint_controller/waist_lifting_pose`）
- 类型：`arms_ros2_control_msgs/action/WaistLiftingPose`

#### `execute_waist_lifting_pose_action(mode, x, z, phi, *, auto_switch_fsm=True, feedback_callback=None, timeout=30.0, wait_for_server_timeout=5.0) -> Any`

**功能：** 发送腰部位姿 Action goal 并等待 result。

**原理：**
- 类型：Action
- 名称：`config.waist_lifting_pose_action_name`
- 类型：`arms_ros2_control_msgs/action/WaistLiftingPose`
- `mode`：`WaistLiftingPose.Goal.MODE_ABSOLUTE` 或 `MODE_RELATIVE`
- 隐式 FSM：默认切到 MOVEJ
- 返回 result（含 `reachable` / `success` / `planned_x/z/phi` 等）；拒绝或超时返回 `None`
- 执行参数：`body_controller` 上的 `waist_lifting_duration`（默认 3 s）

#### `execute_waist_lifting_pose_absolute_action(x, z, phi, ...) -> Any`

**功能：** 腰部绝对位姿运动（`MODE_ABSOLUTE` 包装）。x/z/phi 为 `body_base` 下绝对目标。

**原理：** 同 `execute_waist_lifting_pose_action`。

#### `execute_waist_lifting_pose_relative_action(dx, dz, dphi, ...) -> Any`

**功能：** 腰部相对位姿运动（`MODE_RELATIVE` 包装）。

**原理：** 同 `execute_waist_lifting_pose_action`。

---

### 获取状态

#### `get_joint_state(categorized: bool = False) -> Dict[str, Any] | None`

**功能：** 获取关节状态（包括身体关节）

**原理：** 同头部一节，Topic 订阅 `/joint_states`。使用 `categorized_state.get('body', {})` 取身体关节。

详见 [头部控制 - 获取状态](#获取状态-3)。

---

### 检查到达

#### `check_arrive(part='body', position_threshold: Optional[float] = None) -> Dict[str, Any]`

**功能：** 检查身体是否到达目标位置

**原理：**
- 类型：本地计算
- 当前值：Topic 订阅 `/joint_states` 中的 body 关节
- 目标值：最近一次 `send_body_joint_positions()` / 协调下发缓存

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

#### `get_body_current_target() -> Optional[List[float]]`

**功能：** 获取躯干关节控制器当前目标（关节空间）。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.body_joint_current_target_topic`（自动检测 `/body_joint_controller/current_target_joint`）
- 消息：`std_msgs/msg/Float64MultiArray`

**返回值：** `List[float]` 或 `None`（尚未收到消息时）

#### `get_body_current_pose() -> Optional[Pose]`

**功能：** 获取最新的 body 当前笛卡尔位姿。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.body_current_pose_topic`（自动检测 `/body_current_pose`）
- 消息：`geometry_msgs/msg/PoseStamped`（返回 `.pose`）

**返回值：** `geometry_msgs.msg.Pose` 或 `None`（尚未收到消息时）

#### `get_body_current_target_pose() -> Optional[Pose]`

**功能：** 获取最新的 body 目标笛卡尔位姿。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.body_current_target_pose_topic`（自动检测 `/body_current_target`）
- 消息：`geometry_msgs/msg/PoseStamped`（返回 `.pose`）

**返回值：** `geometry_msgs.msg.Pose` 或 `None`（尚未收到消息时）

#### `check_arrive(part='body_pose', arm_pose_threshold=None, arm_orient_threshold=None) -> Dict[str, Any]`

**功能：** 检查 body 笛卡尔位姿是否到达目标（位置欧氏距离 + 姿态角度双阈值，与手臂一致）

**原理：**
- 类型：本地计算
- 当前：`/body_current_pose`
- 目标：`/body_current_target`

**参数：**
- `part` (str): 设置为 `'body_pose'`
- `arm_pose_threshold` (Optional[float]): 位置阈值（米）；`None` 时用 `config.pose_position_threshold`
- `arm_orient_threshold` (Optional[float]): 姿态角度阈值（度）；`None` 时用 `config.pose_orientation_threshold`

**返回值：**
```python
{
    'arrived': bool,
    'distance': float,
    'position_distance': float,
    'orientation_distance': float,
    'orientation_angle_deg': float,
    'status_message': str | None,
}
```

**示例：**
```python
result = interface.check_arrive('body_pose')
# 或轮询等待
wait_result = interface.wait_until_arrive(
    part='body_pose',
    timeout=10.0,
    arm_pose_threshold=0.05,
    arm_orient_threshold=5.0,
)
```

**配置：** `connect()` 会自动检测 `/body_current_pose` 与 `/body_current_target`；也可手动设置 `body_current_pose_topic` / `body_current_target_pose_topic`。

**示例脚本：** `examples/test/10_body_waist/check_body_pose_arrive.py`

---

## 统一接口方法

这些方法通过 `ROS2RobotInterface` 直接访问，可以操作多个部分。

---

### FSM状态切换

#### `send_fsm_command(command: int) -> None`

**功能：** 发送 FSM（有限状态机）状态切换命令

**原理：**
- 类型：Topic 发布
- 名称：`/fsm_command`
- 消息：`std_msgs/msg/Int32`
- 实际状态回读：Topic 订阅 `/fsm_state`（latched `Int32`，见 `get_fsm_state()`）
- HOME / OCS2 / MOVEJ / COMPLIANCE 若当前不是 HOLD，会先发 HOLD 再发目标（各等 `fsm_state_switch_settle_time`）
- 执行参数：仅 HOME（`command=1`）受控制器 `home_duration`（默认 5 s）、`home_interpolation_type`、`home_1`…`home_N` 影响；其它态切换没有时长参数

**参数：**
- `command` (int): FSM 命令值
  - `1`: HOME 状态
  - `2`: HOLD 状态
  - `3`: OCS2 状态（笛卡尔空间控制）
  - `4`: MOVEJ 状态（关节空间控制）
  - `5`: COMPLIANCE 状态（柔顺/力控；须从 HOLD 进入，接口会自动 HOLD 中转）

**说明：**
- 用于切换机器人的控制模式
- 多数控制 API（笛卡尔 / 关节下发）在 `config.auto_switch_fsm_before_control=True`（默认）时会**隐式**切 FSM：位姿类 → OCS2，关节类（臂/躯干/头）→ MOVEJ；一般无需手写 `send_fsm_command`

**示例：**
```python
# 切换到 HOME 状态
interface.send_fsm_command(1)

# 切换到 OCS2 状态（用于 pose 控制）
interface.send_fsm_command(3)

# 切换到 MOVEJ 状态（用于关节控制）
interface.send_fsm_command(4)

# 切换到 COMPLIANCE（力控）；也可直接用 enter_compliance()
interface.send_fsm_command(5)
```

#### `auto_switch_fsm_state(target_state: int) -> bool`

**功能：** 仅在当前 FSM 不是目标态时发送切换命令。

**原理：**
- 类型：本地判断 + Topic 发布 `/fsm_command`
- 当前态来自 `/fsm_state` 缓存
- 合法目标：`1=HOME, 2=HOLD, 3=OCS2, 4=MOVEJ`（不含 COMPLIANCE）
- 返回：确实发了切换命令为 `True`，已在目标态为 `False`

#### `auto_switch_fsm_for_control(control_type: str) -> bool`

**功能：** 按控制类别自动切 FSM。多数发送 API 内部会调用。

**原理：**
- 类型：本地路由 + `/fsm_command`
- `arm_pose` → OCS2；`arm_joint` / `body_joint` / `head_joint` → MOVEJ；`other` 不切
- `config.auto_switch_fsm_before_control=False` 时直接返回 `False`

---

### 六维力与 COMPLIANCE 力控

依赖真机 `robot.local.yaml` 中启用的 FT（如 `left_ft=kwr75_485`）以及 `ocs2_arm_controller`。`connect()` 会自动探测并订阅 `/left_ft_broadcaster/wrench`、`/right_ft_broadcaster/wrench`（也可在 config 中手动指定）；探测到 original 时默认同时订阅对应的 `/…/wrench_filtered`（connect 时 publisher 可能尚未出现，先挂订阅），并创建 `/compliance_zero_wrench` 服务客户端。

轴序约定（与 COMPLIANCE.md 一致）：`[Fx, Fy, Fz, Mx, My, Mz]`（力 N，力矩 Nm）。

**暂不封装** `wait_zero_force_calibration`：清零结束状态可通过 `/compliance_force_status.zero_cal_done` 自行确认。

#### `get_original_wrench(side: str) -> dict`

**功能：** 返回指定侧原始 `WrenchStamped` 缓存的浅拷贝。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.left_ft_wrench_topic` / `config.right_ft_wrench_topic`
  - 默认探测 `/left_ft_broadcaster/wrench`、`/right_ft_broadcaster/wrench`
- 消息：`geometry_msgs/msg/WrenchStamped`（QoS：SensorData / BEST_EFFORT）

**参数：**
- `side`: `"left"` / `"right"`

**返回值：**
```python
{
    "force": [fx, fy, fz],
    "torque": [mx, my, mz],
    "frame_id": str,
    "stamp": float,  # 秒
}
```

**异常：** 未连接 → `ROS2NotConnectedError`；侧非法 → `ValueError`；话题未配置 / 尚无消息 → `ROS2InterfaceError`。

#### `get_filtered_wrench(side: str) -> dict`

**功能：** 返回指定侧滤波后 wrench 缓存的浅拷贝。返回值结构与 `get_original_wrench` 相同。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.left_ft_wrench_filtered_topic` / `config.right_ft_wrench_filtered_topic`
  - 通常为 `/left_ft_broadcaster/wrench_filtered`、`/right_ft_broadcaster/wrench_filtered`
- 消息：`geometry_msgs/msg/WrenchStamped`；通常仅 COMPLIANCE 期间有数据

**异常：** 同 `get_original_wrench`。

#### `call_compliance_zero_wrench(timeout_sec: float = 5.0) -> None`

**功能：** 发起零力校准。**只请求，不等待校准完成**；结束请自行观察 `/compliance_force_status.zero_cal_done`。

**原理：**
- 类型：Service
- 名称：`/compliance_zero_wrench`
- 类型：`std_srvs/srv/Trigger`
- 服务通常仅在进入 COMPLIANCE 后可用

**异常：** 未连接 → `ROS2NotConnectedError`；client 未初始化 / 服务不可用 / 超时 / `success=False` → `ROS2InterfaceError`。

#### `enter_compliance() -> None`

**功能：** 进入 COMPLIANCE FSM。不等待零力校准。

**原理：**
- 类型：Topic 发布（内部 `send_fsm_command(FSM_COMPLIANCE)`）
- 名称：`/fsm_command`（会先 HOLD 中转）

#### `set_compliance_force(task_selection, force_setpoint, force_xmax_lin=None, force_xmax_ang=None) -> None`

**功能：** 向臂控制器写入完整 6 维力控选择与力目标。

**原理：**
- 类型：参数服务（不是 topic）
- 节点：`interface.arm_controller`（通常 `/ocs2_arm_controller`）
- 服务：`{node}/set_parameters`（`rcl_interfaces/srv/SetParameters`）
- 参数名：`compliance_task_selection`、`compliance_force_setpoint`；可选 `compliance_hybrid_force_xmax_lin` / `compliance_hybrid_force_xmax_ang`

**参数：**
- `task_selection`: 长度 6；`1.0` = 该轴力控，`0.0` = 位置控
- `force_setpoint`: 长度 6；目标 wrench（仅 `task_selection==1` 的轴作为力目标生效）
- `force_xmax_lin`: 可选，平移软限 [m]；`None` 表示不修改控制器现有值（控制器默认通常为 `0.2`）
- `force_xmax_ang`: 可选，旋转软限 [rad]；`None` 表示不修改（控制器默认通常为 `0.3`）

**示例：**
```python
from ros2_robot_interface import (
    FSM_HOLD,
    ROS2InterfaceError,
    ROS2RobotInterface,
    ROS2RobotInterfaceConfig,
)

interface = ROS2RobotInterface(ROS2RobotInterfaceConfig())
interface.connect()
original = interface.get_original_wrench("left")
# COMPLIANCE 前 filtered 可能尚无数据
try:
    filtered = interface.get_filtered_wrench("left")
except ROS2InterfaceError:
    filtered = None
interface.enter_compliance()
interface.call_compliance_zero_wrench()  # 不等待 zero_cal_done
# 必要时自行确认 /compliance_force_status.zero_cal_done 后再设力
interface.set_compliance_force(
    [1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    force_xmax_lin=0.05,
    force_xmax_ang=0.3,
)
interface.send_fsm_command(FSM_HOLD)
interface.disconnect()
```

**示例脚本：** `examples/test/13_ft_and_compliance/check_ft_and_compliance.py`

---

### FSM状态查询

没有名为 `get_fsm_command()` 的公开方法。当前命令/状态都以 `/fsm_state` 为准。

#### `get_fsm_state() -> int`

**功能：** 获取当前 FSM 状态码。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`/fsm_state`（latched / TRANSIENT_LOCAL）
- 消息：`std_msgs/msg/Int32`

**返回值：**
- `1`: HOME
- `2`: HOLD
- `3`: OCS2
- `4`: MOVEJ
- `5`: COMPLIANCE
- 尚未收到消息时为内部默认值（实现里初始化为 `FSM_HOLD`）

**示例：**
```python
state = interface.get_fsm_state()
print(f"FSM state={state}")
```

---

### 模式命令与 WBC 状态确认

订阅 `/ocs2_wbc_controller/current_state`（`WbcCurrentState`），用于确认 `/mode_command` 是否生效。

`MODE_COMMAND_TO_WBC_EXPECT` 将命令映射到消息字段：

| `/mode_command` | 检查字段 | 说明 |
|-----------------|----------|------|
| `BODY_*` | `body_state` | 由 `BODY_MODE_TO_STATE` 派生（含 `BODY_VERTICAL` 别名） |
| `ARMS_COUPLED` / `ARMS_INDEPENDENT` | `bimanual_state` | |
| `BASE_LOCK` / `BASE_UNLOCK` | `base_state` | |

#### `send_mode_command(command: str) -> None`

**功能：** 向 `/mode_command` 发布模式字符串。

**原理：**
- 类型：Topic 发布
- 名称：`/mode_command`
- 消息：`std_msgs/msg/String`
- publish 后盲等 `MODE_SWITCH_SETTLE_TIME_SEC`（默认 `0.1s`），**不**轮询 `current_state`

**说明：**
- 需要确认生效时，请在发完命令后调用下方 `wait_until_mode_commands_applied`。
- 常见前提：FSM 已在 OCS2，否则控制器可能忽略 mode。

#### `mode_command_matches_wbc_state(command: str) -> bool | None`

**原理：** 包装 `mode_commands_match_wbc_state([command])`。见下条。

#### `mode_commands_match_wbc_state(commands: list[str] \| tuple[str, ...]) -> bool | None`

**功能：** 对照最新 `wbc_state` 判断一条 / 一组 mode 是否已生效。

**原理：**
- 类型：本地比较（读 Topic 订阅缓存）
- 名称：`/ocs2_wbc_controller/current_state`
- 消息：`arms_ros2_control_msgs/msg/WbcCurrentState`

**返回值：**
- `True` / `False`：可判定且全部匹配 / 未匹配
- `None`：尚无 `wbc_state`，或命令均无映射

**说明：** 多条命令合并期望字段；**同一字段以后者为准**（例如 `BODY_FREE` 再 `BODY_TRACKING` 只检查 TRACKING）。

#### `wait_until_mode_command_applied(command, *, timeout=5.0, ...) -> bool`

**原理：** 包装 `wait_until_mode_commands_applied([command], ...)`。见下条。

#### `wait_until_mode_commands_applied(commands, *, timeout=5.0, poll_period=0.05, time_now_fn=None, sleep_fn=None) -> bool`

**功能：** 轮询 `current_state`，直到一组 mode 的期望字段**同时**满足（或超时）。

**原理：**
- 类型：本地轮询
- 依赖订阅：`/ocs2_wbc_controller/current_state`

**返回值：** 成功 `True`；超时 `False`；全部无映射则跳过等待返回 `True`。

**推荐用法（多条 mode 往往只反映在一次 state 更新里）：**
```python
interface.send_mode_command("BODY_TRACKING")
interface.send_mode_command("ARMS_COUPLED")
ok = interface.wait_until_mode_commands_applied(
    ["BODY_TRACKING", "ARMS_COUPLED"],
    timeout=5.0,
)
```

---

### 协调关节下发

#### `send_coordinated_joint_positions(body_positions=None, left_arm_positions=None, right_arm_positions=None, head_positions=None, *, auto_switch_fsm=True) -> None`

**功能：** 一次性下发关节空间目标（MoveJ 语义），在 WBC 合成与分体栈之间自动选路。

**原理：**
- 类型：本地路由到已有 Topic 发布（本身不是单独一条 ROS 接口）
- WBC：发布 `unified_arm_joint_controller_topic`（经 `send_dual_arm_joint_positions`）；缺某一臂时从 `/joint_states` hold
- 非 WBC 但有统一臂 topic：双臂走统一 topic；躯干可再走 split body topic
- 其它：回退到左右臂 handler 的分臂 topic / `send_body_joint_positions` / `send_head_joint_positions`
- 隐式 FSM：默认切到 MOVEJ
- 执行参数：各部位对应控制器上的 `movej_*`

**参数：**
- `body_positions` / `left_arm_positions` / `right_arm_positions` / `head_positions`：至少一组非空
- `auto_switch_fsm`（默认 `True`）：下发前隐式切到 MOVEJ（关闭则保持当前 FSM）

**路由概要：**
- **WBC**：经 `send_dual_arm_joint_positions`；缺某一臂时从 `/joint_states` hold 当前角
- **非 WBC 但有统一臂 topic**：双臂走统一 topic；躯干可再走 split body topic
- **其它**：回退到左右臂 handler / `send_body_joint_positions`

**示例：**
```python
interface.send_coordinated_joint_positions(
    body_positions=[0.0, 0.0, 0.0, 0.5],
    left_arm_positions=left_q,
    right_arm_positions=right_q,
)
```

---

### 关节状态获取

#### `get_joint_state(categorized: bool = False) -> Dict[str, Any] | None`

**功能：** 获取所有关节状态

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`config.joint_states_topic`（默认 `/joint_states`）
- 消息：`sensor_msgs/msg/JointState`

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

**原理：**
- 类型：Topic 发布
- 名称：`config.left_hand_joint_controller_topic`（自动检测 `/left_hand_controller/target_joint_position`）
- 消息：`std_msgs/msg/Float64MultiArray`
- 执行参数：该手控制器节点上的 `movej_*`

**说明：**
- 适用于需要独立控制灵巧手每个关节的位置场景

#### `send_right_hand_joint_positions(positions: List[float]) -> None`

**功能：** 发送右手（灵巧手）关节位置命令（双臂模式）。

**原理：**
- 类型：Topic 发布
- 名称：`config.right_hand_joint_controller_topic`（自动检测 `/right_hand_controller/target_joint_position`）
- 消息：`std_msgs/msg/Float64MultiArray`
- 执行参数：该手控制器节点上的 `movej_*`

**示例：**
```python
# 例如每只手 6 个关节
interface.send_left_hand_joint_positions([0.0, 0.3, 0.5, 0.2, 0.1, 0.0])
interface.send_right_hand_joint_positions([0.0, 0.3, 0.5, 0.2, 0.1, 0.0])
```

---

### 灵巧手触觉读取

依赖 `can-ros2-control` 的 LinkerHand 硬件插件（O6 / L6 / O7）**以 `read_tactile:=true` 启动**，否则驱动不会创建触觉发布器，话题根本不存在。`connect()` 会用正则扫描 ROS 图匹配 `/<o6|l6|o7>_hand/<left|right>/tactile/<finger>`，反推出型号前缀写入 `config.left_hand_tactile_topic_prefix` / `config.right_hand_tactile_topic_prefix`，随后为该侧五根手指各挂一个订阅。也可在 `connect()` 前手动指定前缀跳过探测。

手指名固定为 `thumb` / `index` / `middle` / `ring` / `pinky`，与驱动侧 `finger_name()` 一一对应；另有 `all` 表示一次取回五根手指。**读取只有 `get_hand_tactile()` 一个方法**，取单指还是取全部由第二个参数决定。

> 排查订阅链路时，`interface.left_hand_tactile_handler` / `right_hand_tactile_handler` 上另有一个 `get_rate(finger)`，返回缓存更新频率（Hz，口径与 `ros2 topic hz` 一致：10000 个帧间隔的计数窗、`1 / 间隔均值`；停发超 1 秒返回 `0.0`）。它是诊断用的 handler 内部方法，不属于稳定公开 API。

#### `get_hand_tactile(side: str, finger: str) -> UInt8MultiArray | Dict[str, UInt8MultiArray]`

**功能：** 返回指定灵巧手的触觉阵列消息，**就是话题上的原始消息对象**，不 reshape、不拷贝、不包装。`finger` 传具体手指名时返回一条消息；传 `"all"` 时返回五根手指的字典。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`<config.{side}_hand_tactile_topic_prefix>/<finger>`，例如 `/o7_hand/left/tactile/index`
- 消息：`std_msgs/msg/UInt8MultiArray`（QoS：depth 10）
- `finger="all"` 不额外订阅任何话题，只是把五根手指各自的缓存一次取出

**参数：**
- `side`: `"left"` / `"right"`（大小写不敏感）
- `finger`: `"thumb"` / `"index"` / `"middle"` / `"ring"` / `"pinky"`，或 `"all"`（大小写不敏感）

**返回值（单指）：**
```python
msg = interface.get_hand_tactile("left", "index")

msg.layout.dim[0].label   # "row"
msg.layout.dim[0].size    # 行数：O6 = 10，L6 / O7 = 12
msg.layout.dim[1].label   # "column"
msg.layout.dim[1].size    # 列数：O6 = 4，L6 / O7 = 6
msg.data                  # 行优先的 uint8 序列：O6 长度 40，L6 / O7 长度 72
```

行列数由消息自带的 `layout` 决定，**调用方无需事先知道是哪个型号**。需要二维形态时自行 reshape：

```python
rows, cols = msg.layout.dim[0].size, msg.layout.dim[1].size
matrix = [list(msg.data[r * cols:(r + 1) * cols]) for r in range(rows)]
```

**返回值（`finger="all"`）：**
```python
{"thumb": msg, "index": msg, "middle": msg, "ring": msg, "pinky": msg}
```

**说明：**
- `UInt8MultiArray` 不带 header，因此**没有硬件时间戳**；需要时间信息请在调用侧自行记录接收时刻。
- 回调每帧整体替换消息对象而非原地改写，已返回的消息不会被后续帧改动。
- ⚠️ `finger="all"` 时，驱动侧是逐指轮询 CAN（`0xB1`～`0xB5`）、五个话题独立发布，因此这五条消息**不保证属于同一个采样批次**，彼此可能有数十毫秒级错位。需要严格同批次时请自行做时间窗口聚合。

**异常：** 未连接 → `ROS2NotConnectedError`；`side` / `finger` 非法 → `ValueError`；该侧未检测到触觉话题 / 对应手指尚无消息 → `ROS2InterfaceError`（`finger="all"` 时任一手指无数据即抛出，不做部分返回）。

**示例：**
```python
# 单指
msg = interface.get_hand_tactile("left", "index")
print(f"{msg.layout.dim[0].size}x{msg.layout.dim[1].size}, max={max(msg.data)}")

# 五指
for finger, msg in interface.get_hand_tactile("right", "all").items():
    print(f"{finger}: max={max(msg.data)}")
```

---

### 末端执行器位姿获取

#### `interface.left_arm_handler.get_pose() -> Optional[Pose]`

**功能：** 获取左臂末端执行器的当前位姿。

**原理：** 见手臂一节 `get_pose()`：Topic 订阅 `/left_current_pose`，`geometry_msgs/PoseStamped`。

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

**原理：** Topic 订阅 `/right_current_pose`，`geometry_msgs/PoseStamped`。

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

**原理：**
- 类型：本地时间戳（收到 `/joint_states` 回调时记录 `time.time()`）
- 不是消息 `header.stamp`

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

**功能：** 通过 topic 发送双臂路径。**已弃用**，新代码请用 `execute_path` / `execute_left_path` / `execute_right_path`。

**原理：**
- 类型：Topic 发布
- 名称：`/target_path`
- 消息：`nav_msgs/msg/Path`
- 隐式 FSM：切到 OCS2

**说明：**
- 要求双臂模式
- 左右轨迹点数量需要一致（按阶段一一对应）
- 可传入 `Pose` 或 `PoseStamped`
- 会清空旧 target 缓存，避免到达判断误判

#### `execute_path(left_poses, right_poses, trajectory_duration: float = 0.0, frame_id: Optional[str] = "arm_base") -> bool`

**功能：** 通过 ExecutePath service 执行双臂路径并等待服务响应。

**原理：**
- 类型：Service
- 名称：`execute_path`
- 类型：`arms_ros2_control_msgs/srv/ExecutePath`
- 空列表表示该臂不更新（控制器保持原参考轨迹）
- 隐式 FSM：切到 OCS2
- 执行参数：`trajectory_duration=0` 时用 `arm_controller` 节点参数 `movel_trajectory_duration`（臂控 YAML 默认 6 s）

**说明：**
- 不要求左右轨迹点数量一致（支持不等长路径）
- 仅双臂模式可用（需要右臂 target topic 与 client）

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

#### `execute_left_path(left_poses, trajectory_duration: float = 0.0, frame_id: Optional[str] = "arm_base") -> bool`

**功能：** 只更新左臂笛卡尔路径，右臂保持当前参考。

**原理：** 包装 `execute_path(..., right_poses=[])`，同一 Service `execute_path`。

#### `execute_right_path(right_poses, trajectory_duration: float = 0.0, frame_id: Optional[str] = "arm_base") -> bool`

**功能：** 只更新右臂笛卡尔路径，左臂保持当前参考。

**原理：** 包装 `execute_path(..., left_poses=[])`，同一 Service `execute_path`。

#### `send_joint_trajectory(joint_names: List[str], waypoints: List[List[float]], trajectory_duration: float | None = None) -> None`

**功能：** 发送多路点关节轨迹，支持单臂/双臂统一接口。

**原理：**
- 类型：Topic 发布；可选参数服务
- 名称：`/{controller_name}/target_joint_trajectory`（`controller_name` 从左/右臂关节 topic 第一段解析，如 `/ocs2_wbc_controller/target_joint_trajectory`）
- 消息：`trajectory_msgs/msg/JointTrajectory`
- `trajectory_duration` 若给定：对 `arm_controller` 调用 `SetParameters` 写 `movej_trajectory_duration`
- 路点融合另可读节点参数 `movej_trajectory_blend_ratio`
- 隐式 FSM：切到 MOVEJ
- 需要等待结果时请用 `execute_joint_trajectory_action`

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

**原理：**
- 类型：本地计算（按 `part` 读不同缓存）
- `left_arm` / `right_arm`：`/left_current_pose` vs `/left_current_target`（及右臂对应话题）
- `left_gripper` / `right_gripper`：`/joint_states` + handler 内部目标
- `head` / `body`：`/joint_states` + 最近一次关节下发缓存
- `body_pose`：`/body_current_pose` vs `/body_current_target`

**参数：**
- `part` (Optional[str]): 要检查的部分，可选值：
  - `None`: 检查所有部分
  - `'left_arm'`: 左臂
  - `'right_arm'`: 右臂（双臂模式）
  - `'left_gripper'`: 左夹爪
  - `'right_gripper'`: 右夹爪（双臂模式）
  - `'head'`: 头部
  - `'body'`: 身体（关节空间）
  - `'body_pose'`: 身体笛卡尔位姿（位置 + 姿态，阈值见 `arm_pose_threshold` / `arm_orient_threshold`）
  - `'arm'`: 手臂（单臂模式，会自动转换为 'left_arm'）
  - `'gripper'`: 夹爪（单臂模式，会自动转换为 'left_gripper'）
- `position_threshold` (Optional[float]): 关节位置阈值（仅用于 head/body），如果为 None 则使用默认值
- `arm_pose_threshold` (Optional[float]): 笛卡尔位置阈值（米），用于手臂与 `body_pose`
- `arm_orient_threshold` (Optional[float]): 笛卡尔姿态角度阈值（度），用于手臂与 `body_pose`

**说明：**
- 手臂和夹爪使用各自 handler 的默认阈值（可通过直接调用 handler 的方法来自定义）
- 头部和身体关节使用 `position_threshold` 参数或默认的 `self.position_threshold`
- `body_pose` 使用 `arm_pose_threshold` / `arm_orient_threshold`（或 `config.pose_position_threshold` / `config.pose_orientation_threshold`）
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

**原理：**
- 类型：本地轮询（不额外发 ROS 命令）
- 底层数据源与 `check_arrive(part=...)` 相同

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

#### `wait_until_joint_arrive(*, left_target_positions=None, right_target_positions=None, body_target_positions=None, left_check_indices=None, right_check_indices=None, body_check_indices=None, timeout=3.0, poll_period=0.05, joint_tolerance=0.03, angular_wrap=True, ...) -> Dict[str, Any]`

**功能：** 按关节角目标等待到达（适合 MoveJ / `send_coordinated_joint_positions` 后）。

**原理：**
- 类型：本地轮询
- 当前值：Topic 订阅 `/joint_states`
- 目标值：调用方传入的列表，不读控制器 current_target 话题

**参数：**
- `*_target_positions`：各组目标角；至少一组非空
- `*_check_indices`：仅比较这些 0-based 索引（偏绝对目标时跳过 hold 关节）
- `joint_tolerance`：最大绝对误差阈值（弧度）
- `angular_wrap`（默认 `True`）：用最短角距离比较旋转关节

**返回值（节选）：**
```python
{
    "arrived": bool,
    "elapsed": float,
    "left_error_max_abs": float | None,
    "right_error_max_abs": float | None,
    "body_error_max_abs": float | None,
    "left_joint_errors": dict[int, float],   # 被检查关节的逐关节误差
    "right_joint_errors": dict[int, float],
    "body_joint_errors": dict[int, float],
}
```

**示例：**
```python
# 只校验躯干第 4 个关节（相对腰转）
result = interface.wait_until_joint_arrive(
    body_target_positions=body_q,
    body_check_indices=[3],
    timeout=5.0,
    joint_tolerance=0.03,
)
```

---

### 坐标转换

#### `lookup_transform(target_frame: str, source_frame: str, timeout: Optional[float] = None) -> Optional[TransformStamped]`

**功能：** 查询两个坐标系之间的变换关系

**原理：**
- 类型：TF2（`tf2_ros.Buffer` / `TransformListener`）
- 底层话题：`/tf`、`/tf_static`（不是自定义控制 topic）

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

**原理：**
- 类型：TF2（`lookup_transform` + `tf2_geometry_msgs.do_transform_pose`）

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

**原理：**
- 类型：ROS 图查询（`rclpy` `get_node_names_and_namespaces`），不是 topic / action / service

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

### 控制器节点名

#### `body_controller` (`property`) -> `str`

**功能：** 躯干/腰关节控制器节点全名，供 `set_node_parameters` 等使用。

**原理：**
- 类型：从 topic 名解析节点名（不发 ROS 消息）
- split：由 `body_joint_controller_topic` 推断，如 `/body_joint_controller`
- WBC 无独立 body topic 时回退到 `arm_controller`

#### `arm_controller` (`property`) -> `str`

**功能：** 双臂/统一臂控制器节点全名。

**原理：**
- 类型：从 `unified_arm_joint_controller_topic` 或 `left_arm_joint_controller_topic` 解析
- 典型值：`/ocs2_wbc_controller` 或 `/ocs2_arm_controller`

---

### 控制器执行参数

Topic 类命令的消息里通常没有时长/速度字段。控制器在收到目标时读**本节点参数**做规划。用 `set_node_parameters` 写入；参数是节点全局的，改完一直生效到下次再改。节点名用上面的 `arm_controller` / `body_controller`。W2 默认值来自 `fa_w2_ws` 的 `fa-w2-description/config/ros2_control/common.yaml`。

```python
interface.set_node_parameters(interface.arm_controller, {"movel_duration": 3.0})
interface.set_node_parameters(interface.body_controller, {"waist_lifting_duration": 10.0})
```

| 接口 | 节点 | 可改参数 |
|------|------|----------|
| `send_fsm_command(1)` HOME | 对应控制器 | `home_duration`（默认 5 s）、`home_interpolation_type`、`home_1`…`home_N` |
| `send_target` / `send_target_stamped` / `send_relative` | `arm_controller` | `movel_duration`（默认 2 s）、`movel_max_linear_*` / `movel_max_angular_*`、`movel_auto_extend_duration`、`movel_sample_interval` |
| `send_dual_arm_target_stamped` | 同上 | 同一套 `movel_*`（方法参数 `movel_duration` 会先写入节点） |
| `send_body_relative` | 同上 | 同一套 `movel_*` |
| `execute_path` / `execute_left_path` / `execute_right_path` | `arm_controller` | Service 字段 `trajectory_duration`；`0` 时用 `movel_trajectory_duration`（臂控 YAML 默认 6 s） |
| `send_joint_positions`、`send_head_joint_positions`、`send_body_joint_positions`、`send_*_hand_joint_positions`、`send_dual_arm_joint_positions`、`send_coordinated_joint_positions` | 各控制器 | `movej_duration`、`movej_interpolation_type`（`linear` / `tanh` / `doubles`）、`movej_tanh_scale` |
| `send_joint_trajectory` | `arm_controller` | `movej_trajectory_duration`（方法参数会先写入）、`movej_trajectory_blend_ratio` |
| `send_waist_lifting_relative_position`、`send_waist_lifting_pose_*`、`execute_waist_lifting_pose_*_action` | `body_controller` | `waist_lifting_duration`（默认 3 s） |
| `send_waist_lifting_velocity_scale` / `send_waist_turning_velocity_scale` | `body_controller` | `waist_lifting_default_parameter` / `waist_turning_default_parameter` = `[目标速度, 最大加速度, 最大加加速度]`；实际速度 = `scale × 第一项` |

MoveL / MoveC Action 和 `joint_trajectory_with_para` 的时长写在 goal / request 里，不是这张表。力控见 `set_compliance_force`。

---

### 笛卡尔速度

#### `send_cartesian_velocity(linear, angular) -> None`

**功能：** 预留的笛卡尔速度接口。

**原理：**
- **未实现**。调用只打 `logger.warning("Cartesian velocity control not implemented yet")`，不发布任何 topic。
- `config.max_linear_velocity` / `max_angular_velocity` 目前也未使用。

---

### 机器人描述与参数查询

#### `get_robot_description() -> Optional[str]`

**功能：** 获取最近一次接收到的 `robot_description`（URDF XML 字符串）。

**原理：**
- 类型：Topic 订阅（读缓存）
- 名称：`/robot_description`（latched / TRANSIENT_LOCAL）
- 消息：`std_msgs/msg/String`

#### `has_robot_description() -> bool`

**功能：** 判断是否已接收到 `robot_description`。

**原理：** 本地标志，依赖上述 `/robot_description` 订阅。

#### `list_node_parameters(full_node_name: str) -> List[Dict[str, Any]]`

**功能：** 查询指定节点的可动态参数及其信息。

**原理：**
- 类型：参数服务
- 名称：`{full_node_name}/list_parameters`、`describe_parameters`、`get_parameters`
- 类型：`rcl_interfaces/srv/ListParameters` 等

#### `set_node_parameters(full_node_name: str, parameters: Dict[str, Any]) -> bool`

**功能：** 批量设置指定节点参数。

**原理：**
- 类型：参数服务
- 名称：`{full_node_name}/set_parameters`
- 类型：`rcl_interfaces/srv/SetParameters`

**示例：**
```python
ctrl = interface.arm_controller
params = interface.list_node_parameters(ctrl)
print(f"可配置参数数量: {len(params)}")

ok = interface.set_node_parameters(ctrl, {
    "movel_duration": 3.0,
    "movej_duration": 4.0,
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
- `FSM_COMPLIANCE = 5`

关节分组判断（用于 `get_joint_state(categorized=True)`）：

- `is_body_joint_name(name)`：名称含 `body`、`leg_` 前缀、`lift_joint` / `*_lift_joint`
- `is_head_joint_name(name)`：名称含 `head`

```python
from ros2_robot_interface import FSM_HOME, FSM_OCS2, FSM_COMPLIANCE
interface.send_fsm_command(FSM_OCS2)
```

---

## 快速参考表

| 部分 | Handler 访问 | 发送命令 | 获取状态 | 检查到达 |
|------|-------------|---------|---------|---------|
| **左臂** | `left_arm_handler` | `send_target_stamped()` ⭐<br>`send_relative()`<br>`send_target()`<br>`send_joint_positions()`<br>`execute_movel_action()` | `get_pose()` ⭐<br>`get_target_pose()` | `check_arrival()` ⭐ |
| **右臂** | `right_arm_handler` | 同上 | 同上 | 同上 |
| **双臂** | 直接方法 | `send_dual_arm_target_stamped()`<br>`send_dual_arm_joint_positions()`<br>`execute_path()` | — | `check_arrive()` |
| **左夹爪** | `left_gripper_handler` | `send_target_command()` ⭐<br>`send_joint_positions()` | `get_target_position()` | `check_arrival()` |
| **右夹爪** | `right_gripper_handler` | `send_target_command()` ⭐<br>`send_joint_positions()` | `get_target_position()` | `check_arrival()` |
| **灵巧手触觉** | 直接方法 | — | `get_hand_tactile(side, finger)` ⭐<br>`finger="all"` 取五指 | — |
| **头部** | 直接方法 | `send_head_joint_positions()` | `get_joint_state()` | `check_arrive('head')` |
| **身体** | 直接方法 | `send_body_joint_positions()`<br>`send_body_relative()`<br>`send_waist_lifting_pose_*()`<br>`execute_waist_lifting_pose_*_action()` | `get_joint_state()`<br>`get_body_current_pose()` | `check_arrive('body')`<br>`check_arrive('body_pose')` |

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

9. **Mode 与 FSM**：
   - `/mode_command` 控制 WBC 身体/双臂/底盘模式；`/fsm_command` 控制 HOME/HOLD/OCS2/MOVEJ
   - 多条 mode 建议先全部 `send_mode_command`，再一次 `wait_until_mode_commands_applied`，不要按条等待（对端常只更新一次合并后的 `current_state`）
   - 关节 / 位姿控制在开启 `auto_switch_fsm_before_control` 时会隐式切 FSM（关节→MOVEJ，位姿→OCS2）；一般不必显式调用自动切换 API

10. **控制器执行参数**：Topic 类命令的快慢不在消息里，而在控制器节点参数（`movel_duration`、`movej_duration`、`waist_lifting_duration` 等）。改完全局生效，详见 [控制器执行参数](#控制器执行参数)。

---

## 几何与四元数 (utils.quat_pose)

模块路径：`ros2_robot_interface.utils.quat_pose`（也可从 `ros2_robot_interface.utils` 导入已导出符号）。

四元数为 **scalar-last (x, y, z, w)**，与 `geometry_msgs/Pose.orientation` 一致。

| 函数 | 说明 |
|------|------|
| `euler_rpy_to_quat_wxyz(roll, pitch, yaw)` | ZYX 欧拉角 → `(w, x, y, z)` |
| `euler_rpy_to_quat_xyzw(roll, pitch, yaw)` | 同上，输出 `(x, y, z, w)` |
| `quat_multiply(q1, q2)` | 四元数乘积 |
| `quat_conjugate(q)` | 共轭 |
| `quat_normalize(q)` | 单位化（近零时回退为 `(0,0,0,1)`） |
| `rotate_vector_by_quat(vec, quat_xyzw)` | 用 `quat` 旋转三维向量 |
| `rotate_vector_by_quat_inverse(vec, quat_xyzw)` | 用 `quat` 的逆旋转三维向量 |
| `pose_from_tuple(position, orientation_xyzw)` | 元组 → `geometry_msgs.msg.Pose` |
| `check_pose_arrival(...)` | 笛卡尔到位判定（手臂 / body_pose 共用） |

与 LeRobot 扁平 action/观测键相关的 `action_from_pose` / `obs_to_pose` 仍见 **`lerobot_robot_ros2.utils.pose_utils`**。
