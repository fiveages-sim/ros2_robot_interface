# W2 双臂倾倒任务：OCS2 与 IK 示例流程

两份示例使用相同的关节目标和末端 Pose 点位，均来自项目根目录的
《倾倒任务.txt》。它们只控制双臂和腰部，不控制夹爪，也不会在结束时自动回
HOME。

## 文件说明

- `demo_pour_box_ocs2.py`：MoveJ 与 OCS2 笛卡尔参考规划组合版本。
- `demo_pour_box_ik.py`：MoveJ 与 MoveL/MoveC 逐点 IK Motion Planning 组合版本。

## OCS2 示例流程

OCS2 版本通过 `arm_pose` 控制类型进入 `FSM_OCS2`。单点目标由
`send_dual_arm_target_stamped()` 发布，多点倾倒路径由 `execute_path()` 发送。
MoveJ 步骤仍进入 `FSM_MOVEJ`，因此该文件并非所有步骤都使用 OCS2。

| 步骤 | 动作 | 接口 | FSM/规划方式 | 时长 | 完成判断 |
|---|---|---|---|---:|---|
| 启动 | 等待状态数据 | `time.sleep()` | 无规划 | 2s | 固定等待 |
| 1 | 双臂和腰部到初始关节位置 | `send_coordinated_joint_positions()` | `FSM_MOVEJ` 关节插值 | 5s | sleep 后检查关节到位 |
| 2 | 移动到抓取点两侧 | `send_dual_arm_target_stamped()` | `FSM_OCS2` 双臂笛卡尔参考 | 5s | sleep 后检查双臂末端到位 |
| 3 | 移动到抓取点 | `send_dual_arm_target_stamped()` | `FSM_OCS2` 双臂笛卡尔参考 | 5s | sleep 后检查双臂末端到位 |
| 4 | 抬高箱体 | `send_dual_arm_target_stamped()` | `FSM_OCS2` 双臂笛卡尔参考 | 5s | sleep 后检查双臂末端到位 |
| 5 | 移动到准备倾倒点 | `send_dual_arm_target_stamped()` | `FSM_OCS2` 双臂笛卡尔参考 | 5s | sleep 后检查双臂末端到位 |
| 6 | 腰部移动到倾倒起始角度 | `send_coordinated_joint_positions()` | `FSM_MOVEJ` 腰部关节插值 | 5s | sleep 后检查腰部到位 |
| 7 | 准备点→point1→point2→准备点 | `execute_path()` | `FSM_OCS2` 双臂多点笛卡尔路径 | 10s | sleep 后检查双臂末端到位 |

### OCS2 第 7 步

```text
POUR_READY
  → point1
  → point2
  → POUR_READY
```

三个目标点作为同一条双臂路径提交，控制器可能根据速度、加速度和 jerk 限制延长
请求时长。固定 sleep 结束后，到位检测继续覆盖自动延长部分。

## IK Motion Planning 示例流程

IK 版本使用 `execute_movel_action()` 和 `execute_movec_action_three_point()`。两个 Action
通过 `arm_joint` 控制类型进入 `FSM_MOVEJ`，先规划笛卡尔轨迹，再对采样点逐点求
DLS IK，最后执行生成的关节轨迹。Action 是阻塞式调用，返回时已经得到最终执行
结果，因此 MoveL/MoveC 后不再额外 sleep 或轮询末端到位。

| 步骤 | 动作 | 接口 | FSM/规划方式 | 时长 | 完成判断 |
|---|---|---|---|---:|---|
| 启动 | 等待状态并检查Action | `wait_for_movel_action_server()`、`wait_for_movec_action_server()` | 能力检查 | 2s + 最多各5s | 两个Action均可用 |
| 1 | 双臂和腰部到初始关节位置 | `send_coordinated_joint_positions()` | `FSM_MOVEJ` 关节插值 | 5s | sleep 后检查关节到位 |
| 2 | 移动到抓取点两侧 | `execute_movel_action("both", ...)` | 双臂 MoveL + DLS IK | 5s | 阻塞 Action result |
| 3 | 移动到抓取点 | `execute_movel_action("both", ...)` | 双臂 MoveL + DLS IK | 5s | 阻塞 Action result |
| 4 | 抬高箱体 | `execute_movel_action("both", ...)` | 双臂 MoveL + DLS IK | 5s | 阻塞 Action result |
| 5 | 移动到准备倾倒点 | `execute_movel_action("both", ...)` | 双臂 MoveL + DLS IK | 5s | 阻塞 Action result |
| 6 | 腰部移动到倾倒起始角度 | `send_coordinated_joint_positions()` | `FSM_MOVEJ` 腰部关节插值 | 5s | sleep 后检查腰部到位 |
| 7A | 准备点经point1到point2 | `execute_movec_action_three_point("both", ...)` | 双臂 MoveC + DLS IK | 5s | 阻塞 Action result |
| 7B | point2回到准备点 | `execute_movel_action("both", ...)` | 双臂 MoveL + DLS IK | 5s | 阻塞 Action result |

### IK 第 7 步

```text
MoveC（5s）
POUR_READY（隐式起点）
  → point1（圆弧中间点）
  → point2（圆弧终点）

MoveL（5s）
point2
  → POUR_READY
```

MoveC 使用三点位置确定左右臂各自的圆弧，并使用 `use_slerp_for_orientation=True`
在起点与终点四元数之间插值。中间点四元数不会被严格经过，因此必须在实机上确认
箱体中间姿态、双臂间距和 IK 连续性。

## 主要差异

| 对比项 | OCS2版本 | IK版本 |
|---|---|---|
| 单点手臂运动 | 向 OCS2 发布末端参考 | MoveL笛卡尔轨迹逐点求IK |
| 倾倒运动 | 一条三目标点 OCS2路径 | MoveC圆弧倾倒 + MoveL回正 |
| 完成判断 | sleep 后检查到位 | 阻塞 Action result |
| 第7步总请求时长 | 10s | 5s + 5s |
| IK求解器 | 不适用 | DLS |
| 主要控制器 | `ocs2_wbc_controller` 或具有 OCS2位姿链路的控制器 | `ocs2_arm_controller` |

## IK版本运行前提

IK Action 当前由 `ocs2_arm_controller` 提供，运行前应确认：

```bash
ros2 action list | grep -E 'execute_linear|execute_circle_use_ik'
ros2 control list_controllers
```

必须存在：

```text
/ocs2_arm_controller/execute_linear
/ocs2_arm_controller/execute_circle_use_ik
```

`ocs2_wbc_controller` 当前没有注册这两个 Action。仅切换接口内部 FSM 并不会替代
`controller_manager` 的控制器切换；若当前双臂关节由 WBC 独占，需要先切换为
`ocs2_arm_controller` 控制双臂，并确保腰部仍有可用的控制器。

## 安全验证建议

首次运行 IK 版本时应空载或低速验证：

1. 第 2～5 步左右臂所有采样点均有连续 IK 解。
2. 第 7A 步圆弧不会造成双臂拉扯或箱体碰撞。
3. MoveC 的中间姿态变化符合倾倒要求。
4. 第 7B 步回正路径不会穿过箱体、料斗或机器人本体。
