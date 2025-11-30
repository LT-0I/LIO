# LIO-Livox 性能优化进度记录

**设备**: OrangePi 5 MAX (RK3588, 16GB RAM)
**目标**: 单帧处理时间 < 100ms (理想目标 90ms)
**更新日期**: 2025-11-30 (晚间更新)

---

## 优化历程总结

### 基线 (优化前)
- 平均帧间隔: ~120-130ms (估计值)
- 实时性: 远未达标

---

### Iteration 001: VoxelIndex 体素索引优化 ✓

**状态**: 已完成

**改动**:
- 新增 `VoxelIndex.h` 实现 O(1) 空间哈希索引
- 替代局部地图的 KD-Tree 查询
- 配置参数: `use_voxel_index_local: true`, `voxel_index_resolution: 0.5`

**效果**:
- 平均帧间隔: 111.44 ms
- 实时性比率: 1.1144x (落后 78.5s)

---

### Iteration 002: MapManager 增量快照优化 ✓

**状态**: 已完成

**问题**: 每帧全量拷贝 4851 个 cube (21×21×11)

**改动**:
- 实现脏标记 (Dirty Flag) 机制
- 增量拷贝只复制被修改的 cube (~10-50个)
- 文件: `Map_Manager.h`, `Map_Manager.cpp`

**效果**:
| 指标 | 优化前 | 优化后 | 改进 |
|------|--------|--------|------|
| MapManager snapshot | 13.64 ms | 0.067 ms | **-99.5%** |
| 平均帧间隔 | 111.44 ms | 100.18 ms | **-11.26 ms** |
| 实时性比率 | 1.1144x | 1.0018x | 接近实时 |

---

### Iteration 003: Ceres 参数调优 ✗

**状态**: 失败，已回滚

**尝试**:
1. 激进 Ceres 容差 (5e-4) - 失败，迭代次数反增 39%
2. 激进残差削减 (-30%) - 失败，约束变弱导致更多迭代

**教训**:
- Ceres 容差不能过度放宽
- 残差数量不能随意削减
- 单次优化变快 ≠ 总体变快

---

### Iteration 004: RK3588 并行化优化 ✓ 🎉

**状态**: 已完成 - **实时性目标达成！**

**方案演进**:
1. 方案 A: CPU 亲和性绑定大核 - 效果不佳，浪费小核算力
2. 方案 B: OpenMP 动态调度 - **最终采用**

**改动**:
- `RemoveDistortion`: OpenMP `schedule(dynamic, 256) num_threads(8)`
- `Residual Build`: OpenMP `parallel sections num_threads(3)`
- 新增 `CpuAffinity.h` 工具库（保留备用）

**效果**:
| 指标 | Iteration 002 | Iteration 004 | 改进 |
|------|---------------|---------------|------|
| 平均帧间隔 | 100.18 ms | **99.77 ms** | **实时** |
| 实时性比率 | 1.0018x | **0.9977x** | ✓ |

---

### Iteration 005: FIFO 实时性保护 ✓

**状态**: 已完成

**问题**: 队列无限增长可能导致延迟累积

**改动**:
- LiDAR/IMU 队列长度限制
- 过时消息丢弃机制 (`max_lidar_lag_seconds`)
- 新增配置参数

**配置**:
```yaml
enable_fifo_drop: true
lidar_queue_max_size: 3
imu_queue_max_size: 2000
max_lidar_lag_seconds: 0.5
```

**效果**:
| 测试 | 时长 | 帧数 | 实时性 | FIFO丢帧 |
|------|------|------|--------|----------|
| 小 Rosbag | 2分34秒 | 1,547 | 0.9979x ✓ | 0 |
| 大 Rosbag | 11分26秒 | 6,863 | 1.0027x | 0 |

**注意**: FIFO 是保底策略，不是性能优化。播放 rosbag 需使用 `--clock` 参数。

---

### Iteration 006: InitBoost 初始化加速 ✓ (新增)

**状态**: 已完成，待测试

**问题**: 初始化阶段平均帧间隔是稳定期的 2 倍 (240ms vs 122ms)

**改动**:
- 新增 `InitBoostConfig` 配置结构体
- 初始化阶段减少迭代次数 (5 → 2)
- 初始化阶段按比例减少残差数量
- 可选跳过前 N 帧完整优化

**配置** (`horizon_params.yaml`):
```yaml
enable_init_boost: false         # 默认关闭
init_residual_ratio: 0.5        # 残差比例
init_skip_first_n_frames: 0     # 跳过帧数
init_max_iterations: 2          # 最大迭代
```

**预期效果**:
- 初始化阶段帧间隔从 ~240ms 降至 ~150ms
- 解决启动后前 7 秒卡顿问题

---

## 里程碑达成 🎉

| 目标 | 初始值 | 最终值 | 状态 |
|------|--------|--------|------|
| 平均帧间隔 < 100ms | ~120ms | **99.77ms** | ✓ |
| 实时性比率 < 1.0x | ~1.2x | **0.9977x** | ✓ |
| 内存峰值 < 300MB | ~260MB | **284MB** | ✓ |
| FIFO 保护 | 无 | 已实现 | ✓ |
| 初始化加速 | 无 | 已实现 | ✓ |

---

## 当前代码状态

### 已生效的优化

| 文件 | 优化内容 | 迭代 |
|------|----------|------|
| `include/utils/VoxelIndex.h` | O(1) 体素索引 | 001 |
| `include/MapManager/Map_Manager.h` | 增量快照成员 | 002 |
| `src/lio/Map_Manager.cpp` | 增量快照实现 | 002 |
| `include/utils/CpuAffinity.h` | CPU 亲和性工具库 | 004 |
| `src/lio/PoseEstimation.cpp` | OpenMP 并行 + FIFO 保护 + InitBoost 参数 | 004/005/006 |
| `src/lio/Estimator.cpp` | OpenMP 并行 + InitBoost 逻辑 | 004/006 |
| `include/Estimator/Estimator.h` | InitBoostConfig 结构体 | 006 |

### 配置文件 (`config/horizon_params.yaml`)

当前配置:
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

## 相关项目: obs_livox 障碍物检测优化

**位置**: `~/ws_livox/src/obs_livox/`

### 已完成优化:

1. **订阅优化**: 从原始 `/livox/lidar` 改为订阅 SR 节点的 `/livox_full_cloud`
2. **性能优化**:
   - 直接操作 PointCloud2 原始数据（跳过 PCL 转换）
   - 采样步长可配置
   - 使用距离平方避免开方
   - 用乘法代替 atan2
   - 提前终止机制
3. **新增功能**:
   - 高度过滤 (`min_height`, `max_height`)
   - 性能监控 (`log_performance`)
   - 实时点数输出 (`log_point_count`)

### 配置 (`launch/obs_livox.launch`):
```xml
<param name="detection_range" value="3.0" />
<param name="field_of_view" value="26.0" />
<param name="obstacle_threshold" value="200" />
<param name="sample_step" value="2" />
<param name="min_height" value="-0.3" />
<param name="max_height" value="1.5" />
<param name="log_performance" value="true" />
<param name="log_point_count" value="false" />
```

---

## 测试与分析工具

| 工具 | 位置 | 用途 |
|------|------|------|
| 时序日志 | `~/ws_livox/src/LIO/logs/*.log` | 帧间隔统计 |
| 内存监控 | `~/ws_livox/src/LIO/logs/pose_rss_*.csv` | RSS 趋势 |
| 性能分析 | `~/ws_livox/src/LIO/scripts/perf_analyzer.py` | 综合分析 |
| 慢帧分析 | `~/Desktop/实验/analyze_slow_frames.py` | 瓶颈定位 |

---

## 下一步建议

1. **测试 InitBoost**: 启用 `enable_init_boost: true` 验证初始化加速效果
2. **精度验证**: 使用 EVO 工具评估轨迹精度
3. **进一步优化**: 如需达到 90ms 目标，可考虑:
   - 降低残差上限 10%
   - MapManager 异步更新
   - 特征提取并行化

---

## 迭代文档

完整文档位于: `~/ws_livox/src/LIO/算法改进日志1129/`

- `iteration_001_voxel_index.md`
- `iteration_002_incremental_snapshot.md`
- `iteration_003_ceres_tuning.md`
- `iteration_004_rk3588_big_core_parallel.md`
- `iteration_005_fifo_realtime_optimization.md`
- `iteration_006_init_boost_optimization.md`
