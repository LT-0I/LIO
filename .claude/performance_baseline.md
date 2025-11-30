# 性能基线数据

**更新日期**: 2025-11-30

---

## 最终基线 (Iteration 004 + 005 + 006)

**测试设备**: OrangePi 5 MAX (RK3588, 8核, 16GB RAM)
**测试数据**: car隧道.bag (686.1s, 6862帧)

### 时序性能

| 阶段 | 调用次数 | 平均耗时 | 最大耗时 |
|------|----------|----------|----------|
| Estimator::Estimate | 6,862 | 70.81 ms | 726.47 ms |
| Residual build | 12,896 | 21.17 ms | 493.43 ms |
| Ceres solve | 12,896 | 8.23 ms | 56.78 ms |
| MapManager update | 6,862 | 9.28 ms | 27.28 ms |
| Marginalization | 6,767 | 6.76 ms | 44.65 ms |
| RemoveDistortion | 6,863 | 3.76 ms | 34.31 ms |
| IMU_PreIntegration | 6,767 | 0.37 ms | 7.06 ms |
| MapManager snapshot | 6,862 | 0.061 ms | 10.03 ms |

### 实时性指标

| 指标 | 数值 |
|------|------|
| 总帧数 | 6,862 |
| Rosbag 理论时长 | 686.1s |
| 算法处理总时长 | 685.7s |
| 时间差 | **-0.43s (富余)** |
| 平均帧间隔 | **99.77 ms** |
| 实时性比率 | **0.9977x ✓** |

### 内存占用

| 指标 | 数值 |
|------|------|
| 初始 RSS | 67 MB |
| 最终 RSS | 247 MB |
| 峰值 RSS | 284 MB |

---

## 初始化阶段分析 (Iteration 006 待验证)

**问题**: 初始化阶段 (Frame < 20) 耗时是稳定期的 2 倍

| 阶段 | 慢帧数 | 平均间隔 | 最大间隔 |
|------|--------|----------|----------|
| 初始化 (Frame < 20) | 123 | 240.59 ms | 982.80 ms |
| 稳定期 (Frame >= 20) | 26,159 | 122.00 ms | 1181.08 ms |

**InitBoost 预期改善**:
- 初始化阶段: 240ms → ~150ms
- 前7秒卡顿问题改善

---

## 历次迭代对比

| 迭代 | 平均帧间隔 | 实时性比率 | 主要改进 | 状态 |
|------|------------|------------|----------|------|
| 基线 | ~120 ms | ~1.2x | - | - |
| 001 (VoxelIndex) | 111.44 ms | 1.1144x | O(1) 体素索引 | ✓ |
| 002 (增量快照) | 100.18 ms | 1.0018x | 快照 -99.5% | ✓ |
| 003 (Ceres调优) | 102.74 ms | 1.0274x | - | ✗ 失败回滚 |
| **004 (OpenMP)** | **99.77 ms** | **0.9977x** | 并行化 | ✓ 实时达成 |
| 005 (FIFO) | - | - | 保底机制 | ✓ |
| 006 (InitBoost) | - | - | 初始化加速 | ⏳ 待测试 |

---

## 关键优化效果

### Iteration 002 → 004 对比 (同一测试集)

| 指标 | Iteration 002 | Iteration 004 | 改进 |
|------|---------------|---------------|------|
| 平均帧间隔 | 100.18 ms | 99.77 ms | **-0.41 ms** |
| 实时性比率 | 1.0018x (落后) | 0.9977x (富余) | **✓ 实时突破** |
| 时间差 | +1.2s | -0.43s | **+1.6s** |
| Estimator::Estimate | 78.51 ms | 70.81 ms | **-9.8%** |
| Ceres solve | 10.55 ms | 8.23 ms | **-22.0%** |
| Marginalization | 8.94 ms | 6.76 ms | **-24.4%** |
| RemoveDistortion | 4.92 ms | 3.76 ms | **-23.6%** |
| MapManager update | 11.35 ms | 9.28 ms | **-18.2%** |
| 内存峰值 | 260 MB | 284 MB | +9.2% |

---

## obs_livox 障碍物检测性能

**优化后性能** (待实测确认):

| 指标 | 预期值 |
|------|--------|
| 单帧处理 | < 1 ms |
| CPU 占用 | < 2% @10Hz |
| 内存占用 | 极小 |

**优化内容**:
- 直接操作 PointCloud2 原始数据（跳过 PCL 转换）
- 采样步长可配置
- 距离平方比较避免开方
- 乘法代替 atan2
- 提前终止机制

---

## 当前配置 (`horizon_params.yaml`)

```yaml
# VoxelIndex
use_voxel_index_local: true
voxel_index_resolution: 0.5

# 残差限制
max_corner_residuals: 500
max_surf_residuals: 750
max_non_residuals: 350

# FIFO 实时性保护
enable_fifo_drop: true
lidar_queue_max_size: 3
imu_queue_max_size: 2000
max_lidar_lag_seconds: 0.5

# InitBoost 初始化加速 (默认关闭)
enable_init_boost: false
init_residual_ratio: 0.5
init_skip_first_n_frames: 0
init_max_iterations: 2
```

---

## 测试环境

| 项目 | 配置 |
|------|------|
| 硬件 | OrangePi 5 MAX |
| SoC | RK3588 (4×A76 + 4×A55) |
| 内存 | 16GB LPDDR4x |
| 系统 | Linux 5.10.160-rockchip |
| ROS | Noetic |
| 编译器 | GCC 9.4 with -O3 |
