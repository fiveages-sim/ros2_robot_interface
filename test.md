# ROS2 Robot Interface 测试指南.

## 📑 目录

快速跳转：
- [1. test_interface.py - 通用接口测试](#1-test_interfacepy---通用接口测试)
- [2. test_interface_isaac.py - Isaac Sim 联合仿真测试](#2-test_interface_isaacpy---isaac-sim-联合仿真测试)
- [3. test_dual_arm_target_stamped.py - 双臂目标位姿测试](#3-test_dual_arm_target_stampedpy---双臂目标位姿测试)
- [4. test_arm_joint_movej.py - 手臂关节 MoveJ 模式测试](#4-test_arm_joint_movejpy---手臂关节-movej-模式测试)
- [5. test_w2_path.py - W2 机器人路径测试](#5-test_w2_pathpy---w2-机器人路径测试)
- [6. test_tf_transform.py - TF 变换查询和坐标转换测试](#6-test_tf_transformpy---tf-变换查询和坐标转换测试)
- [7. test_gripper.py - 夹爪开关控制测试](#7-test_gripperpy---夹爪开关控制测试)
- [8. test_gripper_position.py - 夹爪位置到达测试](#8-test_gripper_positionpy---夹爪位置到达测试)
- [9. test_check_arrival.py - 手臂到达判断测试](#9-test_check_arrivalpy---手臂到达判断测试)
- [10. test_dual_arm_joint_positions.py - 双臂关节位置统一控制测试](#10-test_dual_arm_joint_positionspy---双臂关节位置统一控制测试)
- [11. demo_wavearm.py - W2 机器人挥手演示（手臂）](#11-examplesfancydemo_wavearmpy---w2-机器人挥手演示手臂)
- [12. demo_wavehand.py - W2 机器人挥手演示（灵巧手）](#12-examplesfancydemo_wavehandpy---w2-机器人挥手演示灵巧手)
- [13. demo_pourbeer_o6.py - W2 机器人倒酒演示（O6灵巧手）](#13-examplesfancydemo_pourbeer_o6py---w2-机器人倒酒演示o6灵巧手)

其他部分：
- [🔗 相关文档](#-相关文档)

---

## 📁 测试文件说明

所有测试文件位于 `examples/` 目录下，每个文件用于测试不同的功能模块：

### 1. `test_interface.py` - 通用接口测试

**功能：** 最全面的测试脚本，测试 ROS2RobotInterface 的所有主要功能

**测试内容：**
- ✅ 接口初始化和连接
- ✅ FSM 状态切换（HOME、MOVEJ、MOVEL、MOVEC）
- ✅ 单臂/双臂自动检测
- ✅ 手臂位姿获取（`get_pose()`）
- ✅ 手臂目标位姿发送（`send_target()`、`send_target_stamped()`）
- ✅ 夹爪控制（`send_joint_positions()`、`send_target_command()`）
- ✅ 到达检查（`check_arrive()`）
- ✅ 头部和身体关节控制
- ✅ 关节状态获取（`get_joint_state()`）

**适用场景：**
- 首次使用接口时的完整功能验证
- 开发新功能后的回归测试
- 学习 API 使用方法的参考示例

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_interface.py
```

---

### 2. `test_interface_isaac.py` - Isaac Sim 联合仿真测试

**功能：** 专门用于 Isaac Sim 联合仿真环境的测试脚本

**测试内容：**
- ✅ 接口连接和初始化
- ✅ FSM 状态切换
- ✅ 手臂位姿控制
- ✅ **夹爪关闭判断逻辑验证**（重点）
- ✅ 到达检查

**适用场景：**
- Isaac Sim 仿真环境下的功能验证
- 夹爪控制逻辑的专门测试
- 仿真环境与真实机器人的对比测试

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_interface_isaac.py
```

---

### 3. `test_dual_arm_target_stamped.py` - 双臂目标位姿测试

**功能：** 专门测试双臂机器人的同步目标位姿发送功能

**测试内容：**
- ✅ `send_dual_arm_target_stamped()` 方法
- ✅ 双臂同步位姿控制
- ✅ `/dual_target/stamped` 话题发布
- ✅ 双臂位姿协调运动

**适用场景：**
- 双臂机器人协调控制
- 双臂同步运动测试
- 双臂抓取任务开发

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_dual_arm_target_stamped.py
```

---

### 4. `test_arm_joint_movej.py` - 手臂关节 MoveJ 模式测试

**功能：** 测试手臂关节位置控制（MoveJ 模式）

**测试内容：**
- ✅ `send_joint_positions()` 方法
- ✅ 关节位置直接控制
- ✅ FSM 自动切换到 MOVEJ 状态
- ✅ 关节位置增量控制（每2秒将最后一个关节增加0.1弧度）

**适用场景：**
- 关节空间路径规划
- 关节位置精确控制
- MoveJ 模式功能验证

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_arm_joint_movej.py
```

---

### 5. `test_w2_path.py` - W2 机器人路径测试

**功能：** 测试 W2 机器人的路径轨迹执行功能

**测试内容：**
- ✅ `target_path` 接口
- ✅ 轨迹文件读取（`w2轨迹测试.txt`）
- ✅ 多段路径连续执行
- ✅ 路径执行完成等待

**适用场景：**
- W2 机器人路径规划
- 轨迹文件执行
- 复杂路径任务测试

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_w2_path.py
```

---

### 6. `test_tf_transform.py` - TF 变换查询和坐标转换测试

**功能：** 测试 TF 变换查询和坐标转换功能

**测试内容：**
- ✅ `lookup_transform()` 方法 - 查询两个坐标系之间的变换关系
- ✅ `transform_pose()` 方法 - 将位姿从一个坐标系转换到另一个坐标系
- ✅ 持续查询变换（每2秒查询一次）
- ✅ 变换信息格式化输出（平移、旋转四元数、RPY角度）
- ✅ 坐标转换结果验证

**函数说明：**

- **`lookup_transform(target_frame, source_frame)`**
  - 查询两个坐标系之间的变换关系
  - 例如：`lookup_transform("left_link1", "left_link7")` 返回 **left_link7 → left_link1** 的变换
  - 返回结果表示：left_link7 在 left_link1 坐标系下的位姿（包含平移和旋转）

- **`transform_pose(pose, source_frame, target_frame)`**
  - 将位姿从一个坐标系转换到另一个坐标系
  - 例如：`transform_pose(pose, "left_link7", "left_link1")` 将位姿从 left_link7 坐标系转换到 left_link1 坐标系
  - 返回结果表示：转换后的位姿（在 left_link1 坐标系下）

**适用场景：**
- TF 坐标系变换查询
- 不同坐标系之间的位姿转换
- TF 系统功能验证
- 多坐标系协作任务开发

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_tf_transform.py
```

**注意事项：**
- 测试脚本会持续运行，按 `Ctrl+C` 停止
- 需要确保机器人控制器已启动，TF 系统正在发布变换数据
- 默认测试 `left_link7` → `left_link1` 的变换，可根据需要修改

---

### 7. `test_gripper.py` - 夹爪开关控制测试

**功能：** 测试夹爪开关控制功能（使用 `target_command` 话题）

**测试内容：**
- ✅ `send_target_command()` 方法 - 发送开关控制命令（0=关闭，1=打开）
- ✅ 状态同步 - 通过订阅器回调更新本地状态
- ✅ 自动切换 - 每3秒根据当前实际状态切换到相反状态
- ✅ 到达检测 - 使用 `check_arrival()` 检查是否到达目标位置
- ✅ 支持单臂和双臂模式

**适用场景：**
- 夹爪开关控制功能验证
- 状态同步机制测试
- 与 VR、RViz、Joystick 控制的一致性验证
- 夹爪到达检测功能测试

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_gripper.py
```

**注意事项：**
- 测试脚本会持续运行，每3秒切换一次夹爪状态，按 `Ctrl+C` 停止
- 控制器名称自动检测：系统会根据实际存在的 topic 自动选择正确的控制器
  - 灵巧手：`/left_hand_controller/target_command` 或 `/right_hand_controller/target_command`
  - 夹爪：`/left_gripper_controller/target_command` 或 `/right_gripper_controller/target_command`
  - 单臂模式：`/hand_controller/target_command`（灵巧手）或 `/gripper_controller/target_command`（夹爪）

---

### 8. `test_gripper_position.py` - 夹爪位置到达测试

**功能：** 测试夹爪位置控制功能（发送具体位置值并检测到达）

**测试内容：**
- ✅ `send_joint_positions()` 方法 - 发送位置命令（如 0.2）
- ✅ `check_arrival()` 方法 - 检查是否到达目标位置
- ✅ 位置稳定性检测 - 关闭时考虑位置稳定性（可能已夹住物体）
- ✅ 每3秒发送一次位置命令并检测到达
- ✅ 支持单臂和双臂模式

**适用场景：**
- 夹爪位置精确控制
- 位置到达检测功能验证
- 稳定性检测机制测试
- 夹爪抓取物体检测

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_gripper_position.py
```

**注意事项：**
- 测试脚本会持续运行，每3秒发送一次位置命令（0.2），按 `Ctrl+C` 停止
- 默认目标位置为 0.2，可在代码中修改 `test_position` 变量
- `check_arrival()` 方法会打印详细的检查信息，包括位置历史记录

---

### 9. `test_check_arrival.py` - 手臂到达判断测试

**功能：** 专门用于测试手臂的到达判断功能

**测试内容：**
- ✅ `check_arrival()` 方法 - 检查手臂是否到达目标位置
- ✅ 循环检查到达状态（每1秒检查一次）
- ✅ 位置距离和姿态距离计算
- ✅ 默认阈值、自定义阈值测试
- ✅ 支持单臂和双臂模式
- ✅ 从话题订阅获取目标位置（`/left_current_target` 或 `/right_current_target`）

**适用场景：**
- 手臂到达判断功能验证
- 到达检测逻辑测试
- 目标位置话题订阅功能验证
- 实时监控手臂到达状态

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_check_arrival.py
```

**注意事项：**
- 测试脚本会持续循环检查到达状态，按 `Ctrl+C` 停止
- 需要确保 `/left_current_target` 或 `/right_current_target` 话题正在发布目标位置
- 如果没有配置目标位置话题，到达判断将返回 `None`
- 每1秒检查一次，显示当前位置、目标位置、距离等信息

---

### 10. `test_dual_arm_joint_positions.py` - 双臂关节位置统一控制测试

**功能：** 测试双臂关节位置统一控制功能（使用统一 topic 同时控制双臂）

**测试内容：**
- ✅ `send_dual_arm_joint_positions()` 方法 - 同时控制双臂的所有关节
- ✅ 统一 topic 自动检测（`/ocs2_wbc_controller/target_joint_position` 或 `/ocs2_arm_controller/target_joint_position`）
- ✅ WBC 控制器支持（自动添加身体关节）
- ✅ ARM 控制器支持（仅双臂关节）
- ✅ FSM 自动切换到 MOVEJ 状态
- ✅ 关节位置增量控制（每2秒将最后一个关节增加指定弧度）

**适用场景：**
- 双臂关节空间协调控制
- 统一 topic 功能验证
- WBC 和 ARM 控制器兼容性测试
- 双臂同步关节运动

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_dual_arm_joint_positions.py
```

**注意事项：**
- 测试脚本会持续运行，每2秒发送一次关节位置命令，按 `Ctrl+C` 停止
- 需要双臂模式（检测到 `/right_target` 或 `/right_current_pose`）
- 需要统一 topic（`/ocs2_wbc_controller/target_joint_position` 或 `/ocs2_arm_controller/target_joint_position`）
- WBC 控制器会自动添加身体关节位置（从当前关节状态获取）
- ARM 控制器只需要双臂关节位置

---

### 11. `examples/fancy/demo_wavearm.py` - W2 机器人挥手演示（手臂）

**功能：** W2 机器人挥手演示脚本，测试双臂协调挥手动作（手臂运动）

**测试内容：**
- ✅ `send_target_path()` 方法 - 发送路径轨迹
- ✅ 双臂协调运动 - 左臂或右臂单独挥手，另一臂保持位置
- ✅ 重复执行挥手动作（重复2次）
- ✅ 路径轨迹执行和到达检测

**适用场景：**
- W2 机器人挥手演示
- 双臂协调运动测试
- 路径轨迹执行功能验证
- 演示场景应用

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/fancy/demo_wavearm.py
```

**注意事项：**
- 适用于 W2 机器人分体控制模式（Split Body Control）
- 演示脚本会自动切换到 OCS2 状态并执行挥手轨迹
- 执行完成后会自动回到 HOME 位置并切换到 HOLD 状态

---

### 12. `examples/fancy/demo_wavehand.py` - W2 机器人挥手演示（灵巧手）

**功能：** W2 机器人挥手演示脚本，测试双臂协调挥手动作（灵巧手版本）

**测试内容：**
- ✅ `send_target_path()` 方法 - 发送路径轨迹
- ✅ 双臂协调运动 - 左臂或右臂单独挥手，另一臂保持位置
- ✅ 重复执行挥手动作（重复2次）
- ✅ 路径轨迹执行和到达检测
- ✅ 与 `demo_wavearm.py` 类似，但针对灵巧手配置优化

**适用场景：**
- W2 机器人挥手演示（灵巧手版本）
- 双臂协调运动测试
- 路径轨迹执行功能验证
- 演示场景应用

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/fancy/demo_wavehand.py
```

**注意事项：**
- 适用于 W2 机器人分体控制模式（Split Body Control）
- 演示脚本会自动切换到 OCS2 状态并执行挥手轨迹
- 执行完成后会自动回到 HOME 位置并切换到 HOLD 状态
- 与 `demo_wavearm.py` 的区别：针对灵巧手（Jodell Hand）的配置和轨迹优化

---

### 13. `examples/fancy/demo_pourbeer_o6.py` - W2 机器人倒酒演示（O6灵巧手）

**功能：** W2 机器人倒酒动作演示脚本，基于 `pick_by_registration.py` 中的 `pour_beer` 方法实现，适用于 O6 灵巧手

**测试内容：**
- ✅ `send_target_path()` 方法 - 发送路径轨迹
- ✅ 双臂协调运动 - 左臂持杯子，右臂持酒枪
- ✅ 倒酒动作轨迹执行（前进和返回两段轨迹）
- ✅ 四元数格式轨迹点支持
- ✅ 轨迹执行完成等待

**适用场景：**
- W2 机器人倒酒演示（O6灵巧手版本）
- 双臂协调倒酒任务
- 路径轨迹执行功能验证
- 演示场景应用

**运行方式：**
```bash
conda activate lerobot_ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/fancy/demo_pourbeer_o6.py
```

**注意事项：**
- 适用于 W2 机器人分体控制模式（Split Body Control）
- 演示脚本会自动切换到 OCS2 状态并执行倒酒轨迹
- 执行完成后会自动回到 HOME 位置并切换到 HOLD 状态
- 使用四元数格式的轨迹点，支持精确的位姿控制

---

## 🔗 相关文档

- [API 参考文档](API_REFERENCE.md) - 详细的 API 说明
- [README](README.md) - 项目概述和快速开始