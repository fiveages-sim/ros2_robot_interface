# ROS2 Robot Interface 测试指南.

## 📑 目录

快速跳转：
- [1. test_interface.py - 通用接口测试](#1-test_interfacepy---通用接口测试)
- [2. test_interface_isaac.py - Isaac Sim 联合仿真测试](#2-test_interface_isaacpy---isaac-sim-联合仿真测试)
- [3. examples/test/dual_target_stamped.py - 双臂目标位姿测试](#3-examplestestdual_target_stampedpy---双臂目标位姿测试)
- [4. test_arm_joint_movej.py - 手臂关节 MoveJ 模式测试](#4-test_arm_joint_movejpy---手臂关节-movej-模式测试)
- [5. examples/test/dual_cartesian_path.py - 双臂 Path 轨迹测试](#5-examplestestdual_cartesian_pathpy---双臂-path-轨迹测试)
- [6. test_tf_transform.py - TF 变换查询和坐标转换测试](#6-test_tf_transformpy---tf-变换查询和坐标转换测试)
- [7. test_gripper.py - 夹爪开关控制测试](#7-test_gripperpy---夹爪开关控制测试)
- [8. test_gripper_position.py - 夹爪位置到达测试](#8-test_gripper_positionpy---夹爪位置到达测试)
- [9. examples/test/check_arrival.py - 手臂到达判断测试](#9-examplestestcheck_arrivalpy---手臂到达判断测试)
- [10. test_dual_arm_joint_positions.py - 双臂关节位置统一控制测试](#10-test_dual_arm_joint_positionspy---双臂关节位置统一控制测试)
- [11. demo_wavearm.py - W2 机器人挥手演示（手臂）](#11-examplesfancydemo_wavearmpy---w2-机器人挥手演示手臂)
- [12. demo_wavehand.py - W2 机器人挥手演示（灵巧手）](#12-examplesfancydemo_wavehandpy---w2-机器人挥手演示灵巧手)
- [13. demo_pourbeer_o6.py - W2 机器人倒酒演示（O6灵巧手）](#13-examplesfancydemo_pourbeer_o6py---w2-机器人倒酒演示o6灵巧手)
- [14. test_movej_with_para.py - 机器人可以调节参数的movej演示](#14-examplestest_movej_with_parapy---机器人可以调节参数的movej演示)
- [15. dual_cartesian_execute_path.py - ExecutePath 不对称轨迹测试](#15-examplestestdual_cartesian_execute_pathpy---executepath-不对称轨迹测试)
- [16. dual_cartesian_path.py - 双臂 Path 轨迹测试](#16-examplestestdual_cartesian_pathpy---双臂-path-轨迹测试)
- [17. list_nodes.py - 节点列表查询测试](#17-examplestestlist_nodespy---节点列表查询测试)
- [18. list_parameters.py - 节点参数查询/设置测试](#18-examplestestlist_parameterspy---节点参数查询设置测试)
- [19. demo_wavearm_jointspace.py - W2 挥手演示（关节空间）](#19-examplesfancydemo_wavearm_jointspacepy---w2-挥手演示关节空间)
- [20. waist_lifting.py - 腰部升降旋转测试](#20-examplestestwaist_liftingpy---腰部升降旋转功能测试)

其他部分：
- [🔗 相关文档](#-相关文档)

---

## 📁 测试文件说明

所有测试文件位于 `examples/` 目录下，每个文件用于测试不同的功能模块：

> 运行说明（已更新）：当前脚本运行**不需要**执行 `source ~/ros2_ws/install/setup.bash`，激活 Python 环境后直接运行即可。

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
conda activate fa-ros2
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
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/test_interface_isaac.py
```

---

### 3. `examples/test/dual_target_stamped.py` - 双臂目标位姿测试

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
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/test/dual_target_stamped.py
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
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/test_arm_joint_movej.py
```

---

### 5. `examples/test/dual_cartesian_path.py` - 双臂 Path 轨迹测试

**功能：** 测试双臂笛卡尔路径轨迹发送功能（`send_target_path()`）。

**测试内容：**
- ✅ `send_target_path()` 接口
- ✅ 双臂路径点成对发送
- ✅ 多段路径连续执行
- ✅ 路径执行完成等待

**适用场景：**
- 双臂路径规划功能验证
- Path 话题接口验证
- 复杂路径任务测试

**运行方式：**
```bash
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/test/dual_cartesian_path.py
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
conda activate fa-ros2
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
conda activate fa-ros2
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
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/test_gripper_position.py
```

**注意事项：**
- 测试脚本会持续运行，每3秒发送一次位置命令（0.2），按 `Ctrl+C` 停止
- 默认目标位置为 0.2，可在代码中修改 `test_position` 变量
- `check_arrival()` 方法会打印详细的检查信息，包括位置历史记录

---

### 9. `examples/test/check_arrival.py` - 手臂到达判断测试

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
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/test/check_arrival.py
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
conda activate fa-ros2
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
conda activate fa-ros2
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
conda activate fa-ros2
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
conda activate fa-ros2
cd ~/libraries/ros2_robot_interface
python examples/fancy/demo_pourbeer_o6.py
```

**注意事项：**
- 适用于 W2 机器人分体控制模式（Split Body Control）
- 演示脚本会自动切换到 OCS2 状态并执行倒酒轨迹
- 执行完成后会自动回到 HOME 位置并切换到 HOLD 状态
- 使用四元数格式的轨迹点，支持精确的位姿控制

---
### 14. `examples/test_movej_with_para.py` - 机器人可以调节参数的movej演示
**功能：**  机器人关节单点和多点调用movej的service，可以通过msg调整参数

**测试内容：**
- ✅ 测试单点movej，发送目标点，在非时间模式下，按照最大速度、最大加速度、最大加加速度运动
- ✅ 测试多点movej，第一个点选择的是非时间模式的参数，第二个点选择的是时间模式，运动时间为5s，没有设置最大速度、最大加速度、最大加加速度参数（会使用程序内部默认的参数比例），第三个点同时设置了时间模式和最大速度、最大加速度、最大加加速度，会使用设置的参数规划，然后进行时间缩放到目标时间，第一个点和第二个点设置了转接比例（0-0.5）会在这两个点进行转接不会停下来，最后一个点的转接比例要设置为0,因为需要停止在这个点

**适用场景：**
- 需要设置不同参数的movej单点或者多点的情况，多点需要平滑过渡时候

**运行方式：**
```bash
conda activate fa-ros2
source ~/ros2_ws/install/setup.bash
cd ~/libraries/ros2_robot_interface
python examples/test_movej_with_para.py
```
运行效果如下所示：




https://github.com/user-attachments/assets/1d24e339-ebff-4b6a-aefe-e2ea927f66b7




---

### 15. `examples/test/dual_cartesian_execute_path.py` - ExecutePath 不对称轨迹测试

**功能：** 使用 `execute_path()` Service 测试双臂不对称路径点数量与不同轨迹持续时间。

**测试内容：**
- ✅ `execute_path()` 服务调用
- ✅ 仅左臂/仅右臂轨迹发送（另一臂路径为空）
- ✅ 左右臂不对称路径点数量（如左2右1、左2右3、左3右2等）
- ✅ 不同 `trajectory_duration` 对执行效果影响（1s 到 10s）
- ✅ 到达检测（`check_arrival()`）

**运行方式：**
```bash
conda activate fa-ros2
source ~/ros2_ws/install/setup.bash
python examples/test/dual_cartesian_execute_path.py
```

---

### 16. `examples/test/dual_cartesian_path.py` - 双臂 Path 轨迹测试

**功能：** 使用 `send_target_path()` 进行双臂笛卡尔路径发送测试。

**测试内容：**
- ✅ `send_target_path()` 话题发布路径
- ✅ 双臂路径点按阶段配对执行
- ✅ 路径发送后的到达检测

**运行方式：**
```bash
conda activate fa-ros2
python examples/test/dual_cartesian_path.py
```

---

### 17. `examples/test/list_nodes.py` - 节点列表查询测试

**功能：** 测试 `list_nodes()` 系统信息查询能力。

**测试内容：**
- ✅ 查询当前 ROS2 节点列表
- ✅ 打印节点名称、命名空间、完整路径

**运行方式：**
```bash
conda activate fa-ros2
python examples/test/list_nodes.py
```

---

### 18. `examples/test/list_parameters.py` - 节点参数查询/设置测试

**功能：** 测试 `list_node_parameters()` 与 `set_node_parameters()`。

**测试内容：**
- ✅ 查询目标节点动态参数
- ✅ 设置参数并校验设置结果

**运行方式：**
```bash
conda activate fa-ros2
python examples/test/list_parameters.py
```

---

### 19. `examples/fancy/demo_wavearm_jointspace.py` - W2 挥手演示（关节空间）

**功能：** 使用关节空间方式执行挥手演示（相对笛卡尔轨迹版）。

**测试内容：**
- ✅ 关节空间轨迹/目标发送
- ✅ 双臂协调动作编排
- ✅ 演示流程状态切换与收尾

**运行方式：**
```bash
conda activate fa-ros2
python examples/fancy/demo_wavearm_jointspace.py
```

---

### 20. `examples/test/waist_lifting.py` - 腰部升降旋转功能测试

**功能：** 测试腰部的升降和旋转功能

**测试内容：**
- ✅ 调节单次升降运动的期望时间，并发送期望移动的相对距离
- ✅ 以一定速度持续上升与下降测试
- ✅ 以一定速度持续左转与右转测试

**运行方式：**
```bash
conda activate fa-ros2
python examples/test/waist_lifting.py
```
运行效果如下所示：

https://github.com/user-attachments/assets/a0bbc49a-6443-405d-99b3-d6ae84a00de3

---





## 🔗 相关文档

- [API 参考文档](API_REFERENCE.md) - 详细的 API 说明
- [README](README.md) - 项目概述和快速开始
