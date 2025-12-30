# ROS2 Robot Interface 测试指南

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
* Split Body Control
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller split_body.launch.py robot:=fiveages_w2
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller full_body.launch.py robot:=fiveages_w2
```

### 测试

```bash
# 激活 conda 环境
conda activate lerobot_ros2

# 配置 ROS 2 环境
source ~/ros2_ws/install/setup.bash

# 进入测试目录
cd /home/fiveages/PythonProject/ros2_robot_interface

# 运行通用测试脚本（自动检测单臂/双臂模式）
python test_interface.py
```

```bash
# 激活 conda 环境
conda activate lerobot_ros2

# 配置 ROS 2 环境
source ~/ros2_ws/install/setup.bash

# 进入测试目录
cd /home/fiveages/PythonProject/ros2_robot_interface

# 运行通用测试脚本（自动检测单臂/双臂模式）
python test_interface_isaac.py
```

### 双臂目标位姿测试（dual_target/stamped）

```bash
# 激活 conda 环境
conda activate lerobot_ros2

# 配置 ROS 2 环境
source ~/ros2_ws/install/setup.bash

# 进入测试目录
cd /home/fiveages/PythonProject/ros2_robot_interface

# 运行双臂目标位姿测试（测试 send_dual_arm_target_stamped 功能）
python test_dual_arm_target_stamped.py
```

### 手臂关节 MoveJ 模式测试

```bash
# 激活 conda 环境
conda activate lerobot_ros2

# 配置 ROS 2 环境
source ~/ros2_ws/install/setup.bash

# 进入测试目录
cd /home/fiveages/PythonProject/ros2_robot_interface

# 运行手臂关节 MoveJ 模式测试（每2秒将双臂最后一个关节弧度增加0.1）
python test_arm_joint_movej.py
```

