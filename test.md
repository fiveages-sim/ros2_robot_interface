# ROS2 Robot Interface 测试指南.

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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
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
cd /home/fiveages/PythonProject/ros2_robot_interface
python examples/test_gripper_position.py
```

**注意事项：**
- 测试脚本会持续运行，每3秒发送一次位置命令（0.2），按 `Ctrl+C` 停止
- 默认目标位置为 0.2，可在代码中修改 `test_position` 变量
- `check_arrival()` 方法会打印详细的检查信息，包括位置历史记录

---

## 🤖 启动机器人

### 单臂机器人（CR5）

```bash
# 终端 1: 启动 OCS2 控制器（Mock）
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5 type:=AG2F90-C-Soft
```

### 双臂机器人（FiveAges W1 with Jodell Hand）

```bash
# 终端 1: 启动双臂机器人控制器
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller full_body.launch.py robot:=fiveages_w1 type:=srs_rg75
```

### 双臂机器人（FiveAges W2）

**Split Body Control:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller split_body.launch.py robot:=fiveages_w2
```

**Full Body Control:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller full_body.launch.py robot:=fiveages_w2
```

---

## 📝 测试建议

1. **首次使用：** 建议先运行 `test_interface.py` 进行完整功能测试
2. **功能验证：** 根据要测试的具体功能选择对应的测试文件
3. **开发调试：** 可以基于这些测试文件修改，创建自己的测试脚本
4. **环境准备：** 确保机器人控制器已启动，ROS 2 环境已配置

---

## 🔗 相关文档

- [API 参考文档](API_REFERENCE.md) - 详细的 API 说明
- [README](README.md) - 项目概述和快速开始
