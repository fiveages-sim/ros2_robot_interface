#!/usr/bin/env python3
"""
机械臂轨迹对比可视化脚本
读取两个CSV文件（计算值和实际值），绘制对比图
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from scipy import interpolate

# 设置matplotlib使用英文
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

def quaternion_to_euler(qw, qx, qy, qz):
    """四元数转欧拉角（度）"""
    r = R.from_quat([qx, qy, qz, qw])
    return r.as_euler('xyz', degrees=True)

def load_data_from_csv(file_path, file_type='cal'):
    """从CSV文件加载数据"""
    df = pd.read_csv(file_path)
    
    if file_type == 'cal':
        # 计算数据格式：t,x,y,z,qx,qy,qz,qw
        positions = df[['x', 'y', 'z']].values
        quaternions = df[['qw', 'qx', 'qy', 'qz']].values
        timestamps = df['t'].values
    else:
        # 实际数据格式：timestamp_sec,timestamp_nanosec,...
        positions = df[['position_x', 'position_y', 'position_z']].values
        quaternions = df[['orientation_w', 'orientation_x', 'orientation_y', 'orientation_z']].values
        timestamps = (df['timestamp_sec'] + df['timestamp_nanosec'] * 1e-9).values
        timestamps = timestamps - timestamps[0]
    
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

def plot_combined_3d_trajectory(cal_positions, cal_quaternions, cal_timestamps,
                               real_positions, real_quaternions, real_timestamps):
    """将两个轨迹绘制在同一个3D坐标系中对比"""
    
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制计算轨迹（蓝色实线）
    ax.plot(cal_positions[:, 0], cal_positions[:, 1], cal_positions[:, 2], 
            'b-', linewidth=2, label='Calculated', alpha=0.8)
    
    # 绘制实际轨迹（红色虚线）
    ax.plot(real_positions[:, 0], real_positions[:, 1], real_positions[:, 2], 
            'r--', linewidth=2, label='Actual', alpha=0.8)
    
    # 标记起点和终点
    ax.scatter(*cal_positions[0], color='blue', s=150, marker='o', 
               label='Start (Calculated)', edgecolors='black', linewidth=2)
    ax.scatter(*cal_positions[-1], color='blue', s=150, marker='s', 
               label='End (Calculated)', edgecolors='black', linewidth=2)
    
    ax.scatter(*real_positions[0], color='red', s=150, marker='o', 
               label='Start (Actual)', edgecolors='black', linewidth=2)
    ax.scatter(*real_positions[-1], color='red', s=150, marker='s', 
               label='End (Actual)', edgecolors='black', linewidth=2)
    
    # 每隔N个点绘制姿态坐标系
    step_cal = max(1, len(cal_positions) // 20)  # 大约显示20个姿态
    step_real = max(1, len(real_positions) // 20)
    arrow_length = 0.02
    
    # 计算轨迹的姿态箭头（蓝色实线）
    for i in range(0, len(cal_positions), step_cal):
        pos = cal_positions[i]
        qw, qx, qy, qz = cal_quaternions[i]
        
        r = R.from_quat([qx, qy, qz, qw])
        
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
        qw, qx, qy, qz = real_quaternions[i]
        
        r = R.from_quat([qx, qy, qz, qw])
        
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
        Line2D([0], [0], color='blue', lw=2, label='Calculated Trajectory'),
        Line2D([0], [0], color='red', lw=2, linestyle='--', label='Actual Trajectory'),
        Line2D([0], [0], color='blue', marker='o', lw=0, markersize=8, label='Start (Calculated)'),
        Line2D([0], [0], color='blue', marker='s', lw=0, markersize=8, label='End (Calculated)'),
        Line2D([0], [0], color='red', marker='o', lw=0, markersize=8, label='Start (Actual)'),
        Line2D([0], [0], color='red', marker='s', lw=0, markersize=8, label='End (Actual)'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=9)
    
    # 设置标签
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('3D Trajectory Comparison: Calculated vs Actual', fontsize=14)
    
    # 设置相同的视角
    ax.view_init(elev=20, azim=45)
    
    # 使坐标轴等比例
    all_positions = np.vstack([cal_positions, real_positions])
    max_range = np.ptp(all_positions, axis=0).max() / 2
    mid = np.mean(all_positions, axis=0)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
    
    # 添加网格
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_comparison_position(cal_timestamps, cal_positions, real_timestamps, real_positions):
    """绘制位置对比图（时间轴对齐）"""
    
    # 插值到共同时间轴
    common_time, cal_interp, real_interp = interpolate_to_common_time(
        cal_timestamps, cal_positions, real_timestamps, real_positions)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    labels = ['X', 'Y', 'Z']
    colors = ['red', 'green', 'blue']
    
    for i, (ax, label, color) in enumerate(zip(axes, labels, colors)):
        ax.plot(common_time, cal_interp[:, i], 
               color=color, linestyle='-', linewidth=1.5, label=f'Calculated {label}')
        ax.plot(common_time, real_interp[:, i], 
               color=color, linestyle='--', linewidth=1.5, label=f'Actual {label}')
        ax.set_ylabel(f'{label} Position (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        if i == 0:
            ax.set_title('Position vs Time (Time-aligned)', fontsize=14)
        if i == 2:
            ax.set_xlabel('Time (s)')
    
    # 添加时间范围说明
    fig.text(0.02, 0.02, f'Common time range: [{common_time[0]:.2f}, {common_time[-1]:.2f}] s', 
             fontsize=10, style='italic')
    
    plt.tight_layout()
    return fig

def plot_comparison_orientation(cal_timestamps, cal_quaternions, real_timestamps, real_quaternions):
    """绘制姿态对比图（时间轴对齐）"""
    
    # 计算欧拉角
    cal_euler = np.array([quaternion_to_euler(q[0], q[1], q[2], q[3]) for q in cal_quaternions])
    real_euler = np.array([quaternion_to_euler(q[0], q[1], q[2], q[3]) for q in real_quaternions])
    
    # 插值到共同时间轴
    common_time, cal_interp, real_interp = interpolate_to_common_time(
        cal_timestamps, cal_euler, real_timestamps, real_euler)
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    
    euler_names = ['Roll', 'Pitch', 'Yaw']
    colors = ['red', 'green', 'blue']
    
    for i, (ax, name, color) in enumerate(zip(axes, euler_names, colors)):
        ax.plot(common_time, cal_interp[:, i], 
               color=color, linestyle='-', linewidth=1.5, label=f'Calculated {name}')
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

def plot_comparison_quaternion(cal_timestamps, cal_quaternions, real_timestamps, real_quaternions):
    """绘制四元数对比图（时间轴对齐）"""
    
    # 插值到共同时间轴
    common_time, cal_interp, real_interp = interpolate_to_common_time(
        cal_timestamps, cal_quaternions, real_timestamps, real_quaternions)
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 14))
    
    quat_names = ['qw', 'qx', 'qy', 'qz']
    colors = ['black', 'red', 'green', 'blue']
    
    for i, (ax, name, color) in enumerate(zip(axes, quat_names, colors)):
        ax.plot(common_time, cal_interp[:, i], 
               color=color, linestyle='-', linewidth=1.5, label=f'Calculated {name}')
        ax.plot(common_time, real_interp[:, i], 
               color=color, linestyle='--', linewidth=1.5, label=f'Actual {name}')
        ax.set_ylabel(f'{name}')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        if i == 0:
            ax.set_title('Quaternion vs Time (Time-aligned)', fontsize=14)
        if i == 3:
            ax.set_xlabel('Time (s)')
    
    # 添加时间范围说明
    fig.text(0.02, 0.02, f'Common time range: [{common_time[0]:.2f}, {common_time[-1]:.2f}] s', 
             fontsize=10, style='italic')
    
    plt.tight_layout()
    return fig

def print_comparison_statistics(cal_positions, cal_quaternions, cal_timestamps,
                               real_positions, real_quaternions, real_timestamps):
    """打印对比统计信息"""
    print("=" * 60)
    print("Trajectory Comparison Statistics")
    print("=" * 60)
    
    print(f"\nNumber of data points:")
    print(f"  Calculated: {len(cal_positions)} points")
    print(f"  Actual: {len(real_positions)} points")
    
    print(f"\nTime range:")
    print(f"  Calculated: [{cal_timestamps[0]:.3f}, {cal_timestamps[-1]:.3f}] s")
    print(f"  Actual: [{real_timestamps[0]:.3f}, {real_timestamps[-1]:.3f}] s")
    
    print(f"\nPosition range (Calculated):")
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
    print(f"  Calculated: {cal_total:.3f} m")
    print(f"  Actual: {real_total:.3f} m")
    print(f"  Difference: {cal_total - real_total:.3f} m")

def main():
    """主函数"""
    # 文件路径
    cal_file = "/home/lina/lina/data/cal_movec_data07.csv"
    real_file = "/home/lina/lina/data/left_pose07.csv"
    
    try:
        # 加载数据
        print("Loading calculated data...")
        cal_positions, cal_quaternions, cal_timestamps, cal_df = load_data_from_csv(cal_file, 'cal')
        
        print("Loading actual data...")
        real_positions, real_quaternions, real_timestamps, real_df = load_data_from_csv(real_file, 'real')
        
        # 打印对比统计信息
        print_comparison_statistics(cal_positions, cal_quaternions, cal_timestamps,
                                   real_positions, real_quaternions, real_timestamps)
        
        # 创建图表
        print("\nGenerating comparison plots...")
        
        # 1. 合并3D轨迹对比图（两个轨迹在同一个坐标系）
        print("  Generating combined 3D trajectory comparison...")
        fig1 = plot_combined_3d_trajectory(cal_positions, cal_quaternions, cal_timestamps,
                                          real_positions, real_quaternions, real_timestamps)
        plt.savefig('trajectory_combined_3d.png', dpi=150, bbox_inches='tight')
        print("  Saved: trajectory_combined_3d.png")
        
        # 2. 位置对比图（时间对齐）
        print("  Generating position comparison (time-aligned)...")
        fig2 = plot_comparison_position(cal_timestamps, cal_positions, 
                                       real_timestamps, real_positions)
        plt.savefig('position_comparison_aligned.png', dpi=150, bbox_inches='tight')
        print("  Saved: position_comparison_aligned.png")
        
        # 3. 姿态对比图（时间对齐）
        print("  Generating orientation comparison (time-aligned)...")
        fig3 = plot_comparison_orientation(cal_timestamps, cal_quaternions,
                                          real_timestamps, real_quaternions)
        plt.savefig('orientation_comparison_aligned.png', dpi=150, bbox_inches='tight')
        print("  Saved: orientation_comparison_aligned.png")
        
        # 4. 四元数对比图（时间对齐）
        print("  Generating quaternion comparison (time-aligned)...")
        fig4 = plot_comparison_quaternion(cal_timestamps, cal_quaternions,
                                         real_timestamps, real_quaternions)
        plt.savefig('quaternion_comparison_aligned.png', dpi=150, bbox_inches='tight')
        print("  Saved: quaternion_comparison_aligned.png")
        
        print("\n✅ All plots generated successfully!")
        print("\nFiles created:")
        print("  - trajectory_combined_3d.png")
        print("  - position_comparison_aligned.png (time-aligned)")
        print("  - orientation_comparison_aligned.png (time-aligned)")
        print("  - quaternion_comparison_aligned.png (time-aligned)")
        
        # 显示图表
        print("\nDisplaying plots (close the windows to exit)...")
        plt.show()
        
    except FileNotFoundError as e:
        print(f"❌ Error: File not found")
        print(f"   Please check the file paths:")
        print(f"   Calculated: {cal_file}")
        print(f"   Actual: {real_file}")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()