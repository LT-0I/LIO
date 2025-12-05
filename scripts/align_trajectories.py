#!/usr/bin/env python3
"""
轨迹时间戳对齐脚本
将测试轨迹和基线轨迹按时间戳对齐，并导出 CSV 表格
包含位置、速度、误差信息
"""

import numpy as np
from pathlib import Path
import sys

# 默认文件路径（可通过命令行参数覆盖）
DEFAULT_TEST_TRAJ = '/home/orangepi/ws_livox/src/LIO/logs/OpiMM_3_2.txt'
DEFAULT_BASELINE_TRAJ = '/home/orangepi/ws_livox/optimization_memory/baseline_data/nuchuihs627.txt'
DEFAULT_OUTPUT_CSV = '/home/orangepi/ws_livox/src/LIO/logs/MMvsNUC.csv'

def load_tum_trajectory(filepath):
    """加载 TUM 格式轨迹文件"""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or len(line) == 0:
                continue
            parts = line.split()
            if len(parts) >= 4:
                timestamp = float(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                data.append([timestamp, x, y, z])
    return np.array(data)

def compute_velocity(data):
    """
    计算速度 (通过位置差分)
    返回: [vx, vy, vz, speed_2d, speed_3d] 数组
    """
    n = len(data)
    velocities = np.zeros((n, 5))  # vx, vy, vz, speed_2d, speed_3d
    
    for i in range(1, n):
        dt = data[i, 0] - data[i-1, 0]
        if dt > 0:
            dx = data[i, 1] - data[i-1, 1]
            dy = data[i, 2] - data[i-1, 2]
            dz = data[i, 3] - data[i-1, 3]
            
            vx = dx / dt
            vy = dy / dt
            vz = dz / dt
            speed_2d = np.sqrt(vx**2 + vy**2)
            speed_3d = np.sqrt(vx**2 + vy**2 + vz**2)
            
            velocities[i] = [vx, vy, vz, speed_2d, speed_3d]
    
    # 第一个点的速度用第二个点的值
    velocities[0] = velocities[1]
    
    return velocities

def align_by_nearest_timestamp(test_data, baseline_data, max_time_diff=0.1):
    """按时间戳对齐两个轨迹"""
    aligned_indices = []
    
    test_timestamps = test_data[:, 0]
    bl_timestamps = baseline_data[:, 0]
    
    for i, test_ts in enumerate(test_timestamps):
        diffs = np.abs(bl_timestamps - test_ts)
        nearest_idx = np.argmin(diffs)
        min_diff = diffs[nearest_idx]
        
        if min_diff <= max_time_diff:
            aligned_indices.append((i, nearest_idx))
    
    return aligned_indices

def main():
    # 解析命令行参数
    if len(sys.argv) >= 4:
        test_traj_path = sys.argv[1]
        baseline_traj_path = sys.argv[2]
        output_csv_path = sys.argv[3]
    else:
        test_traj_path = DEFAULT_TEST_TRAJ
        baseline_traj_path = DEFAULT_BASELINE_TRAJ
        output_csv_path = DEFAULT_OUTPUT_CSV
    
    TEST_TRAJ = Path(test_traj_path)
    BASELINE_TRAJ = Path(baseline_traj_path)
    OUTPUT_CSV = Path(output_csv_path)
    
    print("=" * 70)
    print("轨迹时间戳对齐工具 (含速度和误差)")
    print("=" * 70)
    
    # 加载轨迹
    print(f"\n加载测试轨迹: {TEST_TRAJ}")
    test_data = load_tum_trajectory(TEST_TRAJ)
    print(f"  -> 共 {len(test_data)} 个点")
    
    print(f"\n加载基线轨迹: {BASELINE_TRAJ}")
    baseline_data = load_tum_trajectory(BASELINE_TRAJ)
    print(f"  -> 共 {len(baseline_data)} 个点")
    
    # 计算速度
    print("\n计算速度...")
    test_vel = compute_velocity(test_data)
    bl_vel = compute_velocity(baseline_data)
    
    # 对齐
    print("按时间戳对齐...")
    aligned_indices = align_by_nearest_timestamp(test_data, baseline_data, max_time_diff=0.1)
    print(f"  -> 对齐后共 {len(aligned_indices)} 个点")
    
    # 构建输出数据
    print(f"\n导出到: {OUTPUT_CSV}")
    
    with open(OUTPUT_CSV, 'w') as f:
        # 写表头
        header = "timestamp,test_x,test_y,test_z,test_speed_2d,test_speed_3d,"
        header += "bl_x,bl_y,bl_z,bl_speed_2d,bl_speed_3d,"
        header += "error_x,error_y,error_z,error_dist_2d,error_dist_3d\n"
        f.write(header)
        
        # 写数据
        for test_idx, bl_idx in aligned_indices:
            ts = test_data[test_idx, 0]
            
            # 测试轨迹位置和速度
            tx, ty, tz = test_data[test_idx, 1:4]
            t_speed_2d, t_speed_3d = test_vel[test_idx, 3], test_vel[test_idx, 4]
            
            # 基线轨迹位置和速度
            bx, by, bz = baseline_data[bl_idx, 1:4]
            b_speed_2d, b_speed_3d = bl_vel[bl_idx, 3], bl_vel[bl_idx, 4]
            
            # 误差
            ex = tx - bx
            ey = ty - by
            ez = tz - bz
            error_2d = np.sqrt(ex**2 + ey**2)
            error_3d = np.sqrt(ex**2 + ey**2 + ez**2)
            
            f.write(f"{ts:.6f},{tx:.6f},{ty:.6f},{tz:.6f},{t_speed_2d:.6f},{t_speed_3d:.6f},")
            f.write(f"{bx:.6f},{by:.6f},{bz:.6f},{b_speed_2d:.6f},{b_speed_3d:.6f},")
            f.write(f"{ex:.6f},{ey:.6f},{ez:.6f},{error_2d:.6f},{error_3d:.6f}\n")
    
    print("\n✅ 完成！")
    
    # 统计信息
    errors_3d = []
    speeds_2d = []
    for test_idx, bl_idx in aligned_indices:
        tx, ty, tz = test_data[test_idx, 1:4]
        bx, by, bz = baseline_data[bl_idx, 1:4]
        error_3d = np.sqrt((tx-bx)**2 + (ty-by)**2 + (tz-bz)**2)
        errors_3d.append(error_3d)
        speeds_2d.append(bl_vel[bl_idx, 3])
    
    errors_3d = np.array(errors_3d)
    speeds_2d = np.array(speeds_2d)
    
    print("\n" + "=" * 70)
    print("统计信息 (未对齐坐标系)")
    print("-" * 70)
    print(f"  基线速度 (2D): 平均 {np.mean(speeds_2d):.2f} m/s, 最大 {np.max(speeds_2d):.2f} m/s")
    print(f"  位置误差 (3D): 平均 {np.mean(errors_3d):.2f} m, 最大 {np.max(errors_3d):.2f} m")
    print("=" * 70)
    
    # 按速度分段统计误差
    print("\n按速度分段统计误差:")
    print("-" * 70)
    speed_bins = [(0, 2), (2, 5), (5, 10), (10, 20), (20, 100)]
    for low, high in speed_bins:
        mask = (speeds_2d >= low) & (speeds_2d < high)
        if np.sum(mask) > 0:
            avg_err = np.mean(errors_3d[mask])
            max_err = np.max(errors_3d[mask])
            count = np.sum(mask)
            print(f"  速度 {low:2d}-{high:2d} m/s: 点数 {count:5d}, 平均误差 {avg_err:8.2f} m, 最大误差 {max_err:8.2f} m")
    print("=" * 70)

if __name__ == '__main__':
    main()
