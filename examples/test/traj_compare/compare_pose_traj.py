#!/usr/bin/env python3
"""
机械臂末端位姿轨迹对比可视化脚本
===================================

功能
----
读取两个 CSV 文件——参考轨迹与实际轨迹（real）——按绝对时间对齐后输出统计与三张对比图。
支持两种参考通道：
  - pred：MPC 一步预测末端位姿（future_state FK，时间戳为被预测绝对时刻）↔ real
  - cal ：IK 笛卡尔规划逐点（MoveL/MoveC）↔ real

主要对比内容
------------
脚本重点比较三类结果：
1. 轨迹形状：观察参考轨迹与 actual 是否经过相同的空间路径，使用 DTW 指标衡量。
2. Cross-track 路径偏差：Cross-track 使用 actual 第一帧为基准，以 0.01 mm、连续 3 帧检测运动起点；
   从运动起点开始，不做时间对齐，计算 actual 点到完整参考 3D 折线的最近线段距离；
   Mean/Max 为单向 actual → reference 路径偏差，静止段被排除。
3. 共同时间区间末尾误差：比较两条轨迹在最后一个共同绝对时刻的位置与姿态差异。
   位置误差是 3D 欧氏距离，单位为 mm；姿态误差是四元数间的最短旋转角，单位为度。
终端还会输出绝对时间对齐后整个重叠区间内的位置/姿态 RMSE 和最大误差。

DTW 如何处理 pred 与 actual 时间不同
-------------------------------------
轨迹形状使用 DTW（Dynamic Time Warping，动态时间规整）比较原始 3D 位置序列。
DTW 不使用 CSV 中的绝对时间戳，只保持轨迹点的先后顺序，以两个 3D 点之间的欧氏距离
作为局部代价，寻找累计代价最小的匹配路径。匹配路径允许一对一、一对多和多对一，
相当于对两条序列进行局部拉伸或压缩。

因此，pred 与 actual 即使采样点数、采样频率或运动速度不同，DTW 仍可以把空间位置相近
的轨迹段对应起来，比较路径的几何形状。DTW 输出累计距离、按匹配路径长度归一化的
mm/step，以及匹配路径上的最大点距。DTW 越小，表示两条空间路径的形状越接近。

需要注意：DTW 忽略时间，所以不能反映跟踪延迟、速度差异或同一绝对时刻的位置偏差；
这些时间相关误差由绝对时间对齐后的 RMSE、最大误差和共同时间区间末尾误差反映。
DTW 算法本身不要求两条轨迹的绝对时间范围重叠，但当前完整对比流程还会计算时间相关
指标，因此输入的 pred/cal 与 real 仍需具有有效的绝对时间重叠区间。

CSV 格式（pred / cal / real 统一同一套列，均由控制器录制产出）
------------------------------------------------------------
必需列（顺序不限，多余列忽略，缺列会报 KeyError）：
    timestamp_sec, timestamp_nanosec,
    position_x, position_y, position_z,
    orientation_x, orientation_y, orientation_z, orientation_w
- 时间由 t = timestamp_sec + timestamp_nanosec * 1e-9 合成，单位秒。
- 四元数顺序为 [qx, qy, qz, qw]（w 在最后），与 ROS geometry_msgs/Quaternion 一致。
- 参考与 real 必须来自同一次运动，且位姿在同一坐标系（base_frame）下。

数据来源
--------
文件通常由控制器内 TrajectoryRecorder 录制（同一控制器时钟打绝对时间戳）：
    pred -> pred_left.csv / pred_right.csv   （MPC）
    cal  -> cal_left.csv / cal_right.csv     （IK）
    real -> real_left.csv / real_right.csv
录制开关为 ROS 参数 traj_record_enabled，输出目录为 traj_record_dir。

时间对齐逻辑（重点）
--------------------
默认走“绝对时间对齐”，因为参考与 real 由控制器同一时钟录制，时间戳在同一绝对轴上：
1. load_data_from_csv(..., align_motion_start=False)（默认）：直接使用绝对时间戳，
   不做任何平移/归零。pred 的 stamp 已是「被预测时刻」，可与同刻 real 直接重叠对齐。
2. interpolate_to_common_time / interpolate_quaternions_to_common_time 取两条轨迹
   时间戳的重叠区间 [max(起点), min(终点)]，在该区间上重采样后逐点比较；位置用线性
   插值，四元数用 Slerp。
3. 因此参考轨迹开始前 real 的静止段会自然落在重叠区间之外，被排除，无需手工裁剪。

回退模式（非同源数据）
----------------------
若两个文件不是同源时钟（例如外部 rosbag 录制、历史数据），可对相应文件传
align_motion_start=True 启用“运动起点检测归零”：detect_motion_start_index 以首帧为
参考，找到第一段连续 consecutive_frames 帧位移都超过 position_threshold(默认 2mm) 的
窗口作为运动起点，并把该时刻平移为 t=0，从而用“运动起点”这一物理事件对齐。

使用方式
--------
直接运行，优先走两级目录选择：先选 record_data/ 下的会话，再选该会话下的 step 子目录
（可选「全部 step」依次对比）。若会话目录本身已含 pred_/cal_/real_ CSV（旧扁平结构），
则跳过 step 选择，直接对该目录对比：
    python3 compare_pose_traj.py
选手臂后，若同目录同时有 pred 与 cal，再选对比模式。倾倒 demo 分段录制产物通常只有
pred_* + real_*，会自动走 pred↔real。
也可回退为手动输入两个 CSV 路径（先参考轨迹后 real）。
运行后在所选目录生成三张图片（文件名带手臂后缀）：
    - trajectory_combined_3d_{arm}.png
    - position_comparison_aligned_{arm}.png
    - quaternion_comparison_aligned_{arm}.png
图标题与图例会标注 Left/Right。并弹出窗口显示（关闭窗口结束）。
终端另会打印点数、时间范围、位置范围与总路径长度。
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R, Slerp
from scipy import interpolate

# 设置matplotlib使用英文
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

CROSS_TRACK_MOTION_THRESHOLD_M = 0.01e-3
CROSS_TRACK_CONSECUTIVE_FRAMES = 3

def quaternion_to_euler(quat):
    """四元数转欧拉角（度）。输入格式 [qx, qy, qz, qw]，与 ROS geometry_msgs/Quaternion 一致。"""
    return R.from_quat(quat).as_euler('xyz', degrees=True)

def dtw_distance(seq_a, seq_b):
    """标准 DP 版 DTW 距离，衡量两条 3D 位置序列的形状相似度（忽略时间/速度错位）。

    seq_a、seq_b 形如 (N,3) / (M,3)。局部代价用逐点欧氏距离，返回累计 DTW 距离、
    归一化距离（累计距离 / 匹配路径长度）以及匹配路径上配对点距离的最大值，单位米。
    纯 numpy 实现，无外部依赖。
    """
    n, m = len(seq_a), len(seq_b)
    cost = np.full((n + 1, m + 1), np.inf)
    cost[0, 0] = 0.0
    for i in range(1, n + 1):
        diff = seq_b - seq_a[i - 1]          # (m,3)
        d = np.sqrt(np.sum(diff * diff, axis=1))  # (m,) 逐点欧氏距离
        for j in range(1, m + 1):
            cost[i, j] = d[j - 1] + min(
                cost[i - 1, j],       # 插入
                cost[i, j - 1],       # 删除
                cost[i - 1, j - 1],   # 匹配
            )
    # 回溯匹配路径：记录路径长度（归一化用）与配对点距离最大值
    i, j, path_len = n, m, 0
    max_pair = 0.0
    while i > 0 and j > 0:
        path_len += 1
        pair_dist = float(np.linalg.norm(seq_a[i - 1] - seq_b[j - 1]))  # 该配对点距离
        if pair_dist > max_pair:
            max_pair = pair_dist
        step = min(cost[i - 1, j], cost[i, j - 1], cost[i - 1, j - 1])
        if step == cost[i - 1, j - 1]:
            i, j = i - 1, j - 1
        elif step == cost[i - 1, j]:
            i -= 1
        else:
            j -= 1
    total = cost[n, m]
    normalized = total / path_len if path_len > 0 else float('nan')
    return total, normalized, max_pair

def prompt_file_path(prompt):
    """提示用户输入文件路径，支持去掉引号并展开 ~。"""
    while True:
        file_path = input(prompt).strip().strip('"').strip("'")
        if file_path:
            return str(Path(file_path).expanduser())
        print("Path cannot be empty, please enter a CSV file path.")

def detect_motion_start_index(positions, position_threshold=0.002, consecutive_frames=3):
    """根据连续帧位置位移阈值检测实际轨迹运动起点。

    以第一帧位置为参考，找到第一段连续 consecutive_frames 帧位移都超过
    position_threshold 的窗口，返回该窗口第一帧索引。
    检测失败时返回 0，表示回退到第一帧作为时间零点。
    """
    if len(positions) == 0:
        return 0

    if consecutive_frames <= 1:
        consecutive_frames = 1

    displacement = np.linalg.norm(positions - positions[0], axis=1)
    above_threshold = displacement > position_threshold

    for start_index in range(0, len(above_threshold) - consecutive_frames + 1):
        window = above_threshold[start_index:start_index + consecutive_frames]
        if np.all(window):
            return start_index

    print(
        "Warning: Motion start was not detected with "
        f"position_threshold={position_threshold} m and "
        f"consecutive_frames={consecutive_frames}; using first sample as actual t=0."
    )
    return 0

def point_to_polyline_distances(points, polyline):
    """返回每个 3D point 到 3D polyline 最近线段的欧氏距离，单位米。"""
    points = np.asarray(points, dtype=float)
    polyline = np.asarray(polyline, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("points must be a non-empty array with shape (N, 3)")
    if polyline.ndim != 2 or polyline.shape[1] != 3 or len(polyline) == 0:
        raise ValueError("polyline must be a non-empty array with shape (M, 3)")
    if len(polyline) == 1:
        return np.linalg.norm(points - polyline[0], axis=1)

    starts = polyline[:-1]
    segments = polyline[1:] - starts
    length_sq = np.sum(segments * segments, axis=1)
    distances = np.empty(len(points), dtype=float)
    for index, point in enumerate(points):
        projection = np.zeros(len(segments), dtype=float)
        np.divide(
            np.sum((point - starts) * segments, axis=1),
            length_sq,
            out=projection,
            where=length_sq > 0.0,
        )
        projection = np.clip(projection, 0.0, 1.0)
        closest = starts + projection[:, None] * segments
        distances[index] = np.min(np.linalg.norm(point - closest, axis=1))
    return distances

def compute_cross_track_metrics(reference_positions, actual_positions,
                                position_threshold=CROSS_TRACK_MOTION_THRESHOLD_M,
                                consecutive_frames=CROSS_TRACK_CONSECUTIVE_FRAMES):
    """对运动起点后的 actual 计算到完整 reference 折线的单向 Cross-track。"""
    reference_positions = np.asarray(reference_positions, dtype=float)
    actual_positions = np.asarray(actual_positions, dtype=float)
    if len(reference_positions) == 0 or len(actual_positions) == 0:
        raise ValueError("reference_positions and actual_positions must not be empty")
    start_index = detect_motion_start_index(
        actual_positions,
        position_threshold=position_threshold,
        consecutive_frames=consecutive_frames,
    )
    distances = point_to_polyline_distances(
        actual_positions[start_index:], reference_positions)
    return {
        "motion_start_index": start_index,
        "position_threshold_m": position_threshold,
        "consecutive_frames": consecutive_frames,
        "distances_m": distances,
        "mean_mm": float(np.mean(distances) * 1000.0),
        "max_mm": float(np.max(distances) * 1000.0),
    }

def load_data_from_csv(file_path, file_type='cal', align_motion_start=False):
    df = pd.read_csv(file_path)
    positions = df[['position_x', 'position_y', 'position_z']].values
    quaternions = df[['orientation_x', 'orientation_y',
                      'orientation_z', 'orientation_w']].values
    raw_timestamps = (df['timestamp_sec'] + df['timestamp_nanosec'] * 1e-9).values
    if align_motion_start:
        start_index = detect_motion_start_index(positions)
        timestamps = raw_timestamps - raw_timestamps[start_index]
        print(
            "Trajectory time aligned to motion start: "
            f"index={start_index}, raw_time={raw_timestamps[start_index]:.9f}s"
            f" (file_type={file_type})"
        )
    else:
        timestamps = raw_timestamps
    return positions, quaternions, timestamps, df

def interpolate_to_common_time(cal_timestamps, cal_data, real_timestamps, real_data):
    """将两组数据插值到共同的时间轴上"""
    # 找到共同的时间范围
    common_start = max(cal_timestamps[0], real_timestamps[0])
    common_end = min(cal_timestamps[-1], real_timestamps[-1])
    
    if common_start >= common_end:
        print("Warning: No overlapping time range, using min time range")
        common_start = min(cal_timestamps[0], real_timestamps[0])
        common_end = max(cal_timestamps[-1], real_timestamps[-1])
    
    # 创建共同的时间轴（使用更密集的采样）
    common_time = np.linspace(common_start, common_end, max(len(cal_timestamps), len(real_timestamps)))
    
    # 插值计算数据
    cal_interp = np.zeros((len(common_time), cal_data.shape[1]))
    for i in range(cal_data.shape[1]):
        f = interpolate.interp1d(cal_timestamps, cal_data[:, i], 
                                 kind='linear', bounds_error=False, fill_value='extrapolate')
        cal_interp[:, i] = f(common_time)
    
    # 插值实际数据
    real_interp = np.zeros((len(common_time), real_data.shape[1]))
    for i in range(real_data.shape[1]):
        f = interpolate.interp1d(real_timestamps, real_data[:, i], 
                                 kind='linear', bounds_error=False, fill_value='extrapolate')
        real_interp[:, i] = f(common_time)
    
    return common_time, cal_interp, real_interp

def interpolate_quaternions_to_common_time(cal_timestamps, cal_quaternions,
                                           real_timestamps, real_quaternions):
    """将两组四元数轨迹用 Slerp 插值到共同时间轴上。

    输入与返回格式均为 [qx, qy, qz, qw]，与 ROS geometry_msgs/Quaternion 及 SciPy 一致。
    """
    common_start = max(cal_timestamps[0], real_timestamps[0])
    common_end = min(cal_timestamps[-1], real_timestamps[-1])

    if common_start >= common_end:
        print("Warning: No overlapping time range, using min time range")
        common_start = min(cal_timestamps[0], real_timestamps[0])
        common_end = max(cal_timestamps[-1], real_timestamps[-1])

    common_time = np.linspace(
        common_start,
        common_end,
        max(len(cal_timestamps), len(real_timestamps))
    )

    cal_slerp = Slerp(cal_timestamps, R.from_quat(cal_quaternions))
    real_slerp = Slerp(real_timestamps, R.from_quat(real_quaternions))

    cal_interp = cal_slerp(common_time).as_quat()
    real_interp = real_slerp(common_time).as_quat()

    return common_time, cal_interp, real_interp

def plot_combined_3d_trajectory(cal_positions, cal_quaternions, cal_timestamps,
                               real_positions, real_quaternions, real_timestamps,
                               ref_label='Calculated', arm_label=''):
    """将两个轨迹绘制在同一个3D坐标系中对比。ref_label 为参考轨迹图例名（Calculated/Predicted）。"""
    arm_tag = f' [{arm_label}]' if arm_label else ''

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制计算/预测轨迹（蓝色实线）
    ax.plot(cal_positions[:, 0], cal_positions[:, 1], cal_positions[:, 2], 
            'b-', linewidth=2, label=f'{ref_label}{arm_tag}', alpha=0.8)
    
    # 绘制实际轨迹（红色虚线）
    ax.plot(real_positions[:, 0], real_positions[:, 1], real_positions[:, 2], 
            'r--', linewidth=2, label=f'Actual{arm_tag}', alpha=0.8)
    
    # 标记起点和终点
    ax.scatter(*cal_positions[0], color='blue', s=150, marker='o', 
               label=f'Start ({ref_label}){arm_tag}', edgecolors='black', linewidth=2)
    ax.scatter(*cal_positions[-1], color='blue', s=150, marker='s', 
               label=f'End ({ref_label}){arm_tag}', edgecolors='black', linewidth=2)
    
    ax.scatter(*real_positions[0], color='red', s=150, marker='o', 
               label=f'Start (Actual){arm_tag}', edgecolors='black', linewidth=2)
    ax.scatter(*real_positions[-1], color='red', s=150, marker='s', 
               label=f'End (Actual){arm_tag}', edgecolors='black', linewidth=2)
    
    # 每隔N个点绘制姿态坐标系；轴长按轨迹包围盒最大边长自适应
    step_cal = max(1, len(cal_positions) // 20)  # 大约显示20个姿态
    step_real = max(1, len(real_positions) // 20)
    all_positions = np.vstack([cal_positions, real_positions])
    extent = float(np.ptp(all_positions, axis=0).max())
    # 约为轨迹跨度的 8%；过短路径不低于 2mm，过长不超过 5cm
    arrow_length = float(np.clip(0.08 * max(extent, 1e-6), 0.002, 0.05))
    
    # 计算轨迹的姿态箭头（蓝色实线）
    for i in range(0, len(cal_positions), step_cal):
        pos = cal_positions[i]
        r = R.from_quat(cal_quaternions[i])
        
        # 计算三个轴的方向
        x_axis = r.apply([arrow_length, 0, 0])
        y_axis = r.apply([0, arrow_length, 0])
        z_axis = r.apply([0, 0, arrow_length])
        
        # 绘制坐标轴 - 使用实线样式，蓝色系
        ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], 
                  color='blue', arrow_length_ratio=0.1, alpha=0.6, linewidth=1)
        ax.quiver(pos[0], pos[1], pos[2], y_axis[0], y_axis[1], y_axis[2], 
                  color='blue', arrow_length_ratio=0.1, alpha=0.6, linewidth=1)
        ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], 
                  color='blue', arrow_length_ratio=0.1, alpha=0.6, linewidth=1)
    
    # 实际轨迹的姿态箭头（红色虚线）
    for i in range(0, len(real_positions), step_real):
        pos = real_positions[i]
        r = R.from_quat(real_quaternions[i])
        
        x_axis = r.apply([arrow_length, 0, 0])
        y_axis = r.apply([0, arrow_length, 0])
        z_axis = r.apply([0, 0, arrow_length])
        
        # 绘制坐标轴 - 使用虚线样式，红色系
        ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], 
                  color='red', arrow_length_ratio=0.1, alpha=0.6, linewidth=1)
        ax.quiver(pos[0], pos[1], pos[2], y_axis[0], y_axis[1], y_axis[2], 
                  color='red', arrow_length_ratio=0.1, alpha=0.6, linewidth=1)
        ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], 
                  color='red', arrow_length_ratio=0.1, alpha=0.6, linewidth=1)
    
    # 添加图例说明
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='blue', lw=2, label=f'{ref_label} Trajectory{arm_tag}'),
        Line2D([0], [0], color='red', lw=2, linestyle='--', label=f'Actual Trajectory{arm_tag}'),
        Line2D([0], [0], color='blue', marker='o', lw=0, markersize=8,
               label=f'Start ({ref_label}){arm_tag}'),
        Line2D([0], [0], color='blue', marker='s', lw=0, markersize=8,
               label=f'End ({ref_label}){arm_tag}'),
        Line2D([0], [0], color='red', marker='o', lw=0, markersize=8,
               label=f'Start (Actual){arm_tag}'),
        Line2D([0], [0], color='red', marker='s', lw=0, markersize=8,
               label=f'End (Actual){arm_tag}'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=9)
    
    # 设置标签
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    if arm_label:
        ax.set_title(f'[{arm_label}] 3D Trajectory Comparison: {ref_label} vs Actual',
                     fontsize=14)
    else:
        ax.set_title(f'3D Trajectory Comparison: {ref_label} vs Actual', fontsize=14)
    
    # 设置相同的视角
    ax.view_init(elev=20, azim=45)
    
    # 使坐标轴等比例
    max_range = extent / 2
    mid = np.mean(all_positions, axis=0)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
    
    # 添加网格
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_comparison_position(cal_timestamps, cal_positions, real_timestamps, real_positions,
                             ref_label='Calculated', arm_label=''):
    """绘制位置对比图（时间轴对齐）"""
    arm_tag = f' [{arm_label}]' if arm_label else ''

    # 插值到共同时间轴
    common_time, cal_interp, real_interp = interpolate_to_common_time(
        cal_timestamps, cal_positions, real_timestamps, real_positions)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    labels = ['X', 'Y', 'Z']
    colors = ['red', 'green', 'blue']
    
    for i, (ax, label, color) in enumerate(zip(axes, labels, colors)):
        ax.plot(common_time, cal_interp[:, i], 
               color=color, linestyle='-', linewidth=1.5,
               label=f'{ref_label} {label}{arm_tag}')
        ax.plot(common_time, real_interp[:, i], 
               color=color, linestyle='--', linewidth=1.5,
               label=f'Actual {label}{arm_tag}')
        ax.set_ylabel(f'{label} Position (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        if i == 0:
            title = (f'[{arm_label}] Position vs Time (Time-aligned)'
                     if arm_label else 'Position vs Time (Time-aligned)')
            ax.set_title(title, fontsize=14)
        if i == 2:
            ax.set_xlabel('Time (s)')
    
    # 添加时间范围说明
    fig.text(0.02, 0.02, f'Common time range: [{common_time[0]:.2f}, {common_time[-1]:.2f}] s', 
             fontsize=10, style='italic')
    
    plt.tight_layout()
    return fig

def plot_comparison_orientation(cal_timestamps, cal_quaternions, real_timestamps, real_quaternions,
                                ref_label='Calculated'):
    """绘制姿态对比图（时间轴对齐）"""
    
    # 计算欧拉角
    cal_euler = np.array([quaternion_to_euler(q) for q in cal_quaternions])
    real_euler = np.array([quaternion_to_euler(q) for q in real_quaternions])
    
    # 插值到共同时间轴
    common_time, cal_interp, real_interp = interpolate_to_common_time(
        cal_timestamps, cal_euler, real_timestamps, real_euler)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    
    euler_names = ['Roll', 'Pitch', 'Yaw']
    colors = ['red', 'green', 'blue']
    
    for i, (ax, name, color) in enumerate(zip(axes, euler_names, colors)):
        ax.plot(common_time, cal_interp[:, i], 
               color=color, linestyle='-', linewidth=1.5, label=f'{ref_label} {name}')
        ax.plot(common_time, real_interp[:, i], 
               color=color, linestyle='--', linewidth=1.5, label=f'Actual {name}')
        ax.set_ylabel(f'{name} (deg)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        if i == 0:
            ax.set_title('Orientation (Euler Angles) vs Time (Time-aligned)', fontsize=14)
        if i == 2:
            ax.set_xlabel('Time (s)')
    
    # 添加时间范围说明
    fig.text(0.02, 0.02, f'Common time range: [{common_time[0]:.2f}, {common_time[-1]:.2f}] s', 
             fontsize=10, style='italic')
    
    plt.tight_layout()
    return fig

def plot_comparison_quaternion(cal_timestamps, cal_quaternions, real_timestamps, real_quaternions,
                               ref_label='Calculated', arm_label=''):
    """绘制四元数对比图（时间轴对齐）"""
    arm_tag = f' [{arm_label}]' if arm_label else ''

    # 使用 Slerp 将四元数插值到共同时间轴
    common_time, cal_interp, real_interp = interpolate_quaternions_to_common_time(
        cal_timestamps, cal_quaternions, real_timestamps, real_quaternions)
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 14))
    
    quat_names = ['qx', 'qy', 'qz', 'qw']
    colors = ['red', 'green', 'blue', 'black']
    
    for i, (ax, name, color) in enumerate(zip(axes, quat_names, colors)):
        ax.plot(common_time, cal_interp[:, i], 
               color=color, linestyle='-', linewidth=1.5,
               label=f'{ref_label} {name}{arm_tag}')
        ax.plot(common_time, real_interp[:, i], 
               color=color, linestyle='--', linewidth=1.5,
               label=f'Actual {name}{arm_tag}')
        ax.set_ylabel(f'{name}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        if i == 0:
            title = (f'[{arm_label}] Quaternion vs Time (Time-aligned)'
                     if arm_label else 'Quaternion vs Time (Time-aligned)')
            ax.set_title(title, fontsize=14)
        if i == 3:
            ax.set_xlabel('Time (s)')
    
    # 添加时间范围说明
    fig.text(0.02, 0.02, f'Common time range: [{common_time[0]:.2f}, {common_time[-1]:.2f}] s', 
             fontsize=10, style='italic')
    
    plt.tight_layout()
    return fig

def compute_and_print_error_metrics(cal_positions, cal_quaternions, cal_timestamps,
                                    real_positions, real_quaternions, real_timestamps,
                                    ref_label='Calculated'):
    """计算并打印贴合指标：位置(合成3D) RMSE/Max、姿态(角度) RMSE/Max、Cross-track、形状 DTW。

    - 位置与姿态：在时间对齐（重采样）后的共同时间轴上逐点比较。
    - Cross-track：actual 运动起点起，到完整参考折线的单向路径偏差（不做时间对齐）。
    - DTW：用原始位置序列（忽略时间/速度错位），衡量路径形状相似度。
    """
    # 时间对齐后的位置逐点欧氏误差
    _, cal_p, real_p = interpolate_to_common_time(
        cal_timestamps, cal_positions, real_timestamps, real_positions)
    pos_err = np.sqrt(np.sum((cal_p - real_p) ** 2, axis=1))  # (N,) 合成 3D
    pos_rmse = np.sqrt(np.mean(pos_err ** 2))
    pos_max = np.max(pos_err)

    # 时间对齐后的姿态测地角度误差（度）
    _, cal_q, real_q = interpolate_quaternions_to_common_time(
        cal_timestamps, cal_quaternions, real_timestamps, real_quaternions)
    dots = np.abs(np.sum(cal_q * real_q, axis=1))       # |<q_ref, q_real>|，消 ±q 二义
    dots = np.clip(dots, -1.0, 1.0)
    ang_err = np.degrees(2.0 * np.arccos(dots))          # (N,) 最短旋转角，度
    ang_rmse = np.sqrt(np.mean(ang_err ** 2))
    ang_max = np.max(ang_err)

    # 终点误差（口径 B：重叠区间末尾同一时刻，即对齐序列最后一点）
    endpoint_pos_err = pos_err[-1]      # 3D 欧氏，米
    endpoint_ang_err = ang_err[-1]      # 测地角度，度

    # Cross-track（actual 运动起点起，到完整参考折线的单向路径偏差）
    cross_track = compute_cross_track_metrics(cal_positions, real_positions)
    motion_start_index = cross_track["motion_start_index"]
    motion_start_time = real_timestamps[motion_start_index]

    # 形状 DTW（原始位置序列，忽略时间）
    dtw_total, dtw_norm, dtw_max = dtw_distance(cal_positions, real_positions)

    print(f"\nError metrics ({ref_label} vs Actual):")
    print(f"  Position (3D)  RMSE: {pos_rmse * 1000:.2f} mm, Max: {pos_max * 1000:.2f} mm")
    print(f"  Orientation    RMSE: {ang_rmse:.3f} deg, Max: {ang_max:.3f} deg")
    print(
        "  Actual motion  "
        f"index: {motion_start_index}, time: {motion_start_time:.9f} s, "
        f"threshold: {cross_track['position_threshold_m'] * 1000:.2f} mm, "
        f"consecutive: {cross_track['consecutive_frames']} frames"
    )
    print(
        f"  Cross-track    Mean: {cross_track['mean_mm']:.2f} mm, "
        f"Max: {cross_track['max_mm']:.2f} mm"
    )
    print(f"  Shape DTW      total: {dtw_total:.4f} m, normalized: {dtw_norm * 1000:.2f} mm/step, "
          f"Max(matched): {dtw_max * 1000:.2f} mm")
    print(f"  Endpoint       Pos: {endpoint_pos_err * 1000:.2f} mm, "
          f"Ori: {endpoint_ang_err:.3f} deg")

    return {
        'pos_rmse_mm': pos_rmse * 1000,
        'pos_max_mm': pos_max * 1000,
        'ang_rmse_deg': ang_rmse,
        'ang_max_deg': ang_max,
        'cross_track_mean_mm': cross_track["mean_mm"],
        'cross_track_max_mm': cross_track["max_mm"],
        'cross_track_motion_start_index': motion_start_index,
        'cross_track_motion_start_time_sec': motion_start_time,
        'dtw_total_m': dtw_total,
        'dtw_norm_mm': dtw_norm * 1000,
        'dtw_max_mm': dtw_max * 1000,
        'endpoint_pos_mm': endpoint_pos_err * 1000,
        'endpoint_ang_deg': endpoint_ang_err,
    }


def _annotate_metrics(fig, lines):
    """在图右上角叠加一个指标文本框，lines 为字符串列表。"""
    fig.text(
        0.98, 0.98, "\n".join(lines),
        fontsize=10, family='monospace',
        va='top', ha='right',
        bbox=dict(boxstyle='round', facecolor='lightyellow',
                  edgecolor='gray', alpha=0.9),
    )


def print_comparison_statistics(cal_positions, cal_quaternions, cal_timestamps,
                               real_positions, real_quaternions, real_timestamps,
                               ref_label='Calculated'):
    """打印对比统计信息。ref_label 为参考轨迹名（Calculated/Predicted）。"""
    print("=" * 60)
    print("Trajectory Comparison Statistics")
    print("=" * 60)
    
    print(f"\nNumber of data points:")
    print(f"  {ref_label}: {len(cal_positions)} points")
    print(f"  Actual: {len(real_positions)} points")
    
    print(f"\nTime range:")
    print(f"  {ref_label}: [{cal_timestamps[0]:.3f}, {cal_timestamps[-1]:.3f}] s")
    print(f"  Actual: [{real_timestamps[0]:.3f}, {real_timestamps[-1]:.3f}] s")
    
    print(f"\nPosition range ({ref_label}):")
    print(f"  X: [{cal_positions[:, 0].min():.3f}, {cal_positions[:, 0].max():.3f}] m")
    print(f"  Y: [{cal_positions[:, 1].min():.3f}, {cal_positions[:, 1].max():.3f}] m")
    print(f"  Z: [{cal_positions[:, 2].min():.3f}, {cal_positions[:, 2].max():.3f}] m")
    
    print(f"\nPosition range (Actual):")
    print(f"  X: [{real_positions[:, 0].min():.3f}, {real_positions[:, 0].max():.3f}] m")
    print(f"  Y: [{real_positions[:, 1].min():.3f}, {real_positions[:, 1].max():.3f}] m")
    print(f"  Z: [{real_positions[:, 2].min():.3f}, {real_positions[:, 2].max():.3f}] m")
    
    # 计算总路程
    cal_distances = np.sqrt(np.sum(np.diff(cal_positions, axis=0)**2, axis=1))
    cal_total = np.sum(cal_distances)
    
    real_distances = np.sqrt(np.sum(np.diff(real_positions, axis=0)**2, axis=1))
    real_total = np.sum(real_distances)
    
    print(f"\nTotal path length:")
    print(f"  {ref_label}: {cal_total:.3f} m")
    print(f"  Actual: {real_total:.3f} m")
    print(f"  Difference: {cal_total - real_total:.3f} m")

    # 贴合指标：位置(3D) RMSE/Max、姿态(角度) RMSE/Max、Cross-track、形状 DTW
    return compute_and_print_error_metrics(
        cal_positions, cal_quaternions, cal_timestamps,
        real_positions, real_quaternions, real_timestamps,
        ref_label=ref_label,
    )

def _dir_has_traj_csv(path: str) -> bool:
    """目录内是否直接含 pred_/cal_/real_ CSV（旧扁平录制结构）。"""
    names = os.listdir(path) if os.path.isdir(path) else []
    return any(
        n.endswith(".csv") and (
            n.startswith("pred_") or n.startswith("cal_") or n.startswith("real_")
        )
        for n in names
    )


def select_session_and_steps():
    """返回待对比的 run 目录列表（每个元素是含 CSV 的目录绝对路径）。

    流程：先选 record_data 下会话；若会话根已有 CSV 则直接返回该目录（兼容旧扁平）；
    否则再选 step 子目录，或选「全部 step」依次对比。找不到数据时返回 None。
    """
    record_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "record_data")
    if not os.path.isdir(record_root):
        print(f"未找到 record_data 目录: {record_root}")
        return None
    sessions = sorted(
        (d for d in os.listdir(record_root)
         if os.path.isdir(os.path.join(record_root, d))),
        reverse=True,
    )
    if not sessions:
        print(f"record_data 下没有子目录: {record_root}")
        return None

    print("\n可用的录制会话（record_data 下）:")
    for idx, name in enumerate(sessions, start=1):
        print(f"  {idx}. {name}")
    while True:
        choice = input(f"请选择会话 (1-{len(sessions)}): ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(sessions):
            session_dir = os.path.join(record_root, sessions[int(choice) - 1])
            break
        print("无效选择，请重新输入。")

    if _dir_has_traj_csv(session_dir):
        return [session_dir]

    steps = sorted(
        d for d in os.listdir(session_dir)
        if os.path.isdir(os.path.join(session_dir, d))
    )
    if not steps:
        print(f"会话下无 step 子目录且无 CSV: {session_dir}")
        return None

    print("\n该会话下的 step 目录:")
    print("  0. 全部 step（依次对比）")
    for idx, name in enumerate(steps, start=1):
        print(f"  {idx}. {name}")
    while True:
        choice = input(f"请选择 step (0-{len(steps)}): ").strip()
        if choice == "0":
            return [os.path.join(session_dir, s) for s in steps]
        if choice.isdigit() and 1 <= int(choice) <= len(steps):
            return [os.path.join(session_dir, steps[int(choice) - 1])]
        print("无效选择，请重新输入。")


def _arm_label_from_suffix_or_files(out_suffix: str, cal_file: str, real_file: str) -> str:
    """返回 'Left' / 'Right' / ''。优先 out_suffix，其次 CSV 文件名。"""
    token = (out_suffix or "").lstrip("_").lower()
    if token == "left":
        return "Left"
    if token == "right":
        return "Right"
    for path in (cal_file, real_file):
        base = os.path.basename(path).lower()
        if "_left" in base:
            return "Left"
        if "_right" in base:
            return "Right"
    return ""


def select_arm():
    """选择对比左臂、右臂还是两者，返回 'left'、'right' 或 'both'。"""
    print("\n选择要对比的手臂:")
    print("  1. 左臂 (left)")
    print("  2. 右臂 (right)")
    print("  3. 双臂 (both，左右各出一套图与统计)")
    while True:
        choice = input("请选择 (1-3): ").strip()
        if choice == "1":
            return "left"
        if choice == "2":
            return "right"
        if choice == "3":
            return "both"
        print("无效选择，请重新输入。")


def select_ref_channel(run_dir, arm):
    """在目录中选择参考通道：pred（MPC 预测）或 cal（IK 规划）。

    返回 (ref_file, channel)，channel 为 'pred' 或 'cal'。
    若两者都不存在，返回 (None, None)。
    """
    pred_file = os.path.join(run_dir, f"pred_{arm}.csv")
    cal_file = os.path.join(run_dir, f"cal_{arm}.csv")
    has_pred = os.path.isfile(pred_file)
    has_cal = os.path.isfile(cal_file)
    if has_pred and has_cal:
        print("\n选择对比模式:")
        print("  1. pred↔real  （MPC 预测偏差）")
        print("  2. cal↔real   （IK 规划 vs 实际）")
        while True:
            choice = input("请选择 (1-2): ").strip()
            if choice == "1":
                return pred_file, "pred"
            if choice == "2":
                return cal_file, "cal"
            print("无效选择，请重新输入。")
    if has_pred:
        return pred_file, "pred"
    if has_cal:
        return cal_file, "cal"
    return None, None


def run_comparison(cal_file, real_file, ref_channel, out_dir, out_suffix):
    """对单条参考轨迹与 real 做一次对比：加载→统计→生成并保存三张图。

    返回 True 表示成功（图已保存），False 表示失败（缺文件或异常）。
    图窗不在此显示，由调用方在全部对比完成后统一 plt.show()。
    """
    # 参考通道对应的图例/统计标签：pred -> Predicted，其余（cal）-> Calculated
    ref_label = 'Predicted' if ref_channel == 'pred' else 'Calculated'
    try:
        # 加载数据（pred/cal/real 同为 timestamped 列，复用同一读取逻辑）
        print(f"\nLoading {ref_channel} data: {cal_file}")
        cal_positions, cal_quaternions, cal_timestamps, cal_df = load_data_from_csv(
            cal_file, ref_channel, align_motion_start=False)

        print(f"Loading actual data: {real_file}")
        real_positions, real_quaternions, real_timestamps, real_df = load_data_from_csv(
            real_file, 'real', align_motion_start=False)

        # 打印对比统计信息，并拿到贴合指标用于图上标注
        metrics = print_comparison_statistics(
            cal_positions, cal_quaternions, cal_timestamps,
            real_positions, real_quaternions, real_timestamps,
            ref_label=ref_label)

        # 图上标注 Cross-track、DTW 与终点误差；完整 RMSE 等仅在终端打印
        cross_track_line = (
            f"Cross-track Mean {metrics['cross_track_mean_mm']:.2f} mm | "
            f"Max {metrics['cross_track_max_mm']:.2f} mm"
        )
        dtw_line = (f"Shape DTW {metrics['dtw_norm_mm']:.2f} mm/step | "
                    f"Max {metrics['dtw_max_mm']:.2f} mm")
        endpoint_line = (f"Endpoint err  Pos {metrics['endpoint_pos_mm']:.2f} mm | "
                         f"Ori {metrics['endpoint_ang_deg']:.3f} deg")

        # 创建图表
        print("\nGenerating comparison plots...")

        # 输出文件路径（存到所选目录，文件名带手臂后缀，避免左右臂互相覆盖）
        fig1_path = os.path.join(out_dir, f'trajectory_combined_3d{out_suffix}.png')
        fig2_path = os.path.join(out_dir, f'position_comparison_aligned{out_suffix}.png')
        fig3_path = os.path.join(out_dir, f'quaternion_comparison_aligned{out_suffix}.png')

        arm_label = _arm_label_from_suffix_or_files(out_suffix, cal_file, real_file)

        # 1. 合并3D轨迹对比图（两个轨迹在同一个坐标系）：总览，叠加全部指标
        print("  Generating combined 3D trajectory comparison...")
        fig1 = plot_combined_3d_trajectory(cal_positions, cal_quaternions, cal_timestamps,
                                            real_positions, real_quaternions, real_timestamps,
                                            ref_label=ref_label, arm_label=arm_label)
        _annotate_metrics(fig1, [cross_track_line, dtw_line, endpoint_line])
        fig1.savefig(fig1_path, dpi=150, bbox_inches='tight')
        print(f"  Saved: {fig1_path}")

        # 2. 位置对比图（时间对齐）：叠加 Cross-track + DTW + Endpoint
        print("  Generating position comparison (time-aligned)...")
        fig2 = plot_comparison_position(cal_timestamps, cal_positions,
                                        real_timestamps, real_positions,
                                        ref_label=ref_label, arm_label=arm_label)
        _annotate_metrics(fig2, [cross_track_line, dtw_line, endpoint_line])
        fig2.savefig(fig2_path, dpi=150, bbox_inches='tight')
        print(f"  Saved: {fig2_path}")

        # 3. 四元数对比图（时间对齐，使用 Slerp）：叠加姿态
        print("  Generating quaternion comparison (time-aligned with Slerp)...")
        fig3 = plot_comparison_quaternion(cal_timestamps, cal_quaternions,
                                          real_timestamps, real_quaternions,
                                          ref_label=ref_label, arm_label=arm_label)
        _annotate_metrics(fig3, [dtw_line, endpoint_line])
        fig3.savefig(fig3_path, dpi=150, bbox_inches='tight')
        print(f"  Saved: {fig3_path}")

        print(f"\n✅ [{out_suffix or 'result'}] plots generated successfully!")
        return True

    except FileNotFoundError:
        print(f"❌ Error: File not found")
        print(f"   Reference ({ref_channel}): {cal_file}")
        print(f"   Actual: {real_file}")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def _resolve_arm_job(run_dir, arm):
    """根据目录与单个手臂，解析出 (cal_file, real_file, ref_channel, out_suffix)。

    缺参考文件或 real 文件时打印警告，返回 None 表示跳过该臂。
    """
    cal_file, ref_channel = select_ref_channel(run_dir, arm)
    real_file = os.path.join(run_dir, f"real_{arm}.csv")
    print(f"\n[{arm}]")
    if cal_file is None:
        print(f"  ⚠ 未找到 pred_{arm}.csv 或 cal_{arm}.csv，跳过")
        return None
    print(f"  {ref_channel}: {cal_file}")
    print(f"  real: {real_file}")
    if not os.path.isfile(real_file):
        print(f"  ⚠ 未找到 {real_file}，跳过")
        return None
    return cal_file, real_file, ref_channel, f"_{arm}"


def main():
    """主函数"""
    # 优先走「选会话 → 选 step → 选手臂 → 选通道」；找不到 record_data 时回退手动输入路径
    run_dirs = select_session_and_steps()

    if run_dirs is not None:
        arm = select_arm()
        arms = ["left", "right"] if arm == "both" else [arm]
        any_ok = False
        for run_dir in run_dirs:
            out_dir = run_dir
            step_name = os.path.basename(run_dir)
            print(f"\n对比目录: {run_dir}")
            print(f"step: {step_name}")
            print(f"对比手臂: {arm}")

            jobs = []
            for a in arms:
                job = _resolve_arm_job(run_dir, a)
                if job is not None:
                    jobs.append(job)

            if not jobs:
                print(f"\n⚠ 跳过（无可对比数据）: {run_dir}")
                continue

            for cal_file, real_file, ref_channel, out_suffix in jobs:
                any_ok = (
                    run_comparison(cal_file, real_file, ref_channel, out_dir, out_suffix)
                    or any_ok
                )

        if not any_ok:
            print("\n❌ 没有可对比的数据。")
            return

        print("\nDisplaying plots (close the windows to exit)...")
        plt.show()
    else:
        # 手动输入路径的回退分支：仅单条对比
        cal_file = prompt_file_path(
            "Enter reference trajectory CSV path (pred or cal): ")
        real_file = prompt_file_path("Enter actual trajectory CSV path: ")
        out_dir = os.getcwd()
        ref_channel = "cal"
        base = os.path.basename(cal_file).lower()
        if base.startswith("pred"):
            ref_channel = "pred"

        if run_comparison(cal_file, real_file, ref_channel, out_dir, ""):
            print("\nDisplaying plots (close the windows to exit)...")
            plt.show()


if __name__ == "__main__":
    main()
