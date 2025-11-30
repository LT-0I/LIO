# LIO-Livox 性能优化进度记录

**设备**: OrangePi 5 MAX (RK3588, 16GB RAM)
**目标**: 单帧处理时间 < 100ms (理想目标 90ms)
**更新日期**: 2025-11-30

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
| Estimator::Estimate | 78.51 ms | 72.43 ms | **-7.7%** |
| RemoveDistortion | 4.92 ms | 3.57 ms | **-27.4%** |
| Marginalization | 8.94 ms | 6.92 ms | **-22.6%** |
| MapManager update | 11.35 ms | 9.07 ms | **-20.1%** |
| 平均帧间隔 | 100.18 ms | **99.83 ms** | **-0.35 ms** |
| 实时性比率 | 1.0018x | **0.9983x** | **✓ 实时** |
| 内存峰值 | 260 MB | 214 MB | **-17.7%** |

---

## 里程碑达成 🎉

| 目标 | 初始值 | 最终值 | 状态 |
|------|--------|--------|------|
| 平均帧间隔 < 100ms | ~120ms | **99.83ms** | ✓ |
| 实时性比率 < 1.0x | ~1.2x | **0.9983x** | ✓ |
| 内存峰值 < 300MB | ~260MB | **214MB** | ✓ |

---

## 当前代码状态

### 已生效的优化

| 文件 | 优化内容 | 迭代 |
|------|----------|------|
| `include/VoxelIndex/VoxelIndex.h` | O(1) 体素索引 | 001 |
| `include/MapManager/Map_Manager.h` | 增量快照成员 | 002 |
| `src/lio/Map_Manager.cpp` | 增量快照实现 | 002 |
| `include/utils/CpuAffinity.h` | CPU 亲和性工具库 | 004 |
| `src/lio/PoseEstimation.cpp` | OpenMP 并行畸变校正 | 004 |
| `src/lio/Estimator.cpp` | OpenMP 并行残差构建 | 004 |

### 配置文件 (`config/horizon_params.yaml`)

当前配置（正确）:
```yaml
# VoxelIndex
use_voxel_index_local: true
voxel_index_resolution: 0.5

# 残差限制（保持默认）
max_corner_residuals: 500
max_surf_residuals: 750
max_non_residuals: 350

# AdaptiveBudget（保持默认）
adaptive_budget_target_build_ms: 8.0
adaptive_budget_target_solve_ms: 25.0
```

---

## 关键经验教训

1. **增量快照最有效**: -99.5% 快照时间，单项优化收益最大
2. **OpenMP 动态调度优于手动绑核**: 自动负载均衡，充分利用大小核
3. **Ceres 容差不能激进**: 过度放宽会触发更多优化循环
4. **残差数量不能随意削减**: 约束变弱导致收敛困难

---

## 下一步建议

### 方向 1: 稳定性验证
- 使用更长的 bag 文件 (car隧道.bag 686s) 验证
- 隧道等退化场景测试

### 方向 2: 进一步优化 (目标 90ms)
- Ceres 线程数限制为 4（只用大核）
- MapManager 异步更新
- 特征提取并行化

### 方向 3: 精度验证
- 与原始算法对比轨迹精度
- EVO 工具评估 APE/RPE

---

## 测试日志位置

- 时序日志: `~/ws_livox/src/LIO/logs/*.log`
- 内存监控: `~/ws_livox/src/LIO/logs/pose_rss_*.csv`
- 分析脚本: `~/ws_livox/src/LIO/scripts/perf_analyzer.py`

使用方法:
```bash
python3 scripts/perf_analyzer.py --log logs/XXX.log --rss logs/pose_rss_XXX.csv
```
