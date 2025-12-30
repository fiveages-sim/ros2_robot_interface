# ROS2 Robot Interface 测试指南

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
- ✅ 夹爪控制（`send_joint_positions()`）
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
