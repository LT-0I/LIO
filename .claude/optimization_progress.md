# LIO-Livox 性能优化进度记录

**设备**: OrangePi 5 MAX (RK3588, 8GB RAM)
**目标**: 单帧处理时间 < 100ms (理想目标 90ms)
**测试数据**: car隧道.bag (~686s, 6863帧)
**更新日期**: 2025-11-29

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

**效果**: 减少局部地图搜索耗时

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

### Iteration 003: 90ms 目标优化 (进行中)

**状态**: 测试中，遇到问题

#### 尝试 1: 激进 Ceres 参数 ✗ 失败

**改动**:
```cpp
options.max_num_iterations = 6;      // 原 8
options.function_tolerance = 5e-4;   // 原 1e-4
options.gradient_tolerance = 5e-4;
options.parameter_tolerance = 5e-4;
options.max_consecutive_nonmonotonic_steps = 2;  // 原 3
```

**结果**: 反效果！
- 总迭代次数增加 39% (12,944 → 17,956)
- 帧间隔变慢 (100.18ms → 102.98ms)
- 已回滚

#### 尝试 2: 保守 Ceres + 激进残差削减 ✗ 失败

**改动**:
```yaml
# 残差数量 -30%
max_corner_residuals: 350   # 原 500
max_surf_residuals: 500     # 原 750
max_non_residuals: 250      # 原 350

# AdaptiveBudget 激进目标
adaptive_budget_target_build_ms: 6.0    # 原 8.0
adaptive_budget_target_solve_ms: 18.0   # 原 25.0
```

**结果**: 仍然失败
| 指标 | Iteration 002 | 尝试 2 | 变化 |
|------|---------------|--------|------|
| 平均帧间隔 | 100.18 ms | 102.74 ms | +2.56 ms ✗ |
| Ceres solve | 10.55 ms | 6.46 ms | -4.09 ms ✓ |
| Residual build | 20.51 ms | 16.25 ms | -4.26 ms ✓ |
| 总迭代次数 | 12,944 | 17,920 | +38% ✗ |
| 内存峰值 | 213 MB | 428 MB | +100% ✗ |

**失败原因**: 减少残差导致约束变弱，每帧需要更多迭代才能收敛

---

## 当前代码状态

### 需要回滚的配置 (`config/horizon_params.yaml`)

当前（错误配置）:
```yaml
max_corner_residuals: 350
max_surf_residuals: 500
max_non_residuals: 250
adaptive_budget_target_build_ms: 6.0
adaptive_budget_target_solve_ms: 18.0
```

应恢复为:
```yaml
max_corner_residuals: 500
max_surf_residuals: 750
max_non_residuals: 350
adaptive_budget_target_build_ms: 8.0
adaptive_budget_target_solve_ms: 25.0
```

### Ceres 参数 (`src/lio/Estimator.cpp`)

当前（正确配置，已回滚）:
```cpp
options.max_num_iterations = 8;
options.function_tolerance = 2e-4;
options.gradient_tolerance = 2e-4;
options.parameter_tolerance = 2e-4;
options.max_consecutive_nonmonotonic_steps = 3;
```

---

## 关键经验教训

1. **Ceres 容差不能过度放宽**: 5e-4 导致不收敛，触发更多优化循环
2. **残差数量不能随意削减**: 约束变弱导致每帧迭代次数增加
3. **单次优化变快 ≠ 总体变快**: 需要看总迭代次数
4. **增量快照是最有效的优化**: -99.5% 快照时间

---

## 下一步建议

### 方向 1: 回滚残差配置，保持 Iteration 002 状态
- 恢复 500/750/350 残差限制
- 恢复 8.0/25.0 AdaptiveBudget 目标
- 保持 100.18ms 的接近实时状态

### 方向 2: 优化其他瓶颈
当前各模块耗时:
- Estimator::Estimate: 80.14 ms (主循环)
- Residual build: 16.25 ms
- MapManager update: 11.70 ms ← 可能可优化
- Marginalization: 8.03 ms ← 可能可并行
- Ceres solve: 6.46 ms
- RemoveDistortion: 5.01 ms

### 方向 3: 算法级优化
- 减少每帧迭代次数的其他方法（更好的初值估计）
- 并行化 Marginalization 与下一帧预处理
- 异步 MapManager update

---

## 文件修改清单

| 文件 | 状态 | 说明 |
|------|------|------|
| `include/VoxelIndex/VoxelIndex.h` | 新增 | O(1) 体素索引 |
| `include/MapManager/Map_Manager.h` | 修改 | 增量快照成员 |
| `src/lio/Map_Manager.cpp` | 修改 | 增量快照实现 |
| `src/lio/Estimator.cpp` | 修改 | Ceres 参数（已回滚） |
| `config/horizon_params.yaml` | 修改 | **需要回滚残差配置** |

---

## 测试日志位置

- 时序日志: `~/ws_livox/src/LIO/logs/*.log`
- 内存监控: `~/ws_livox/src/LIO/logs/pose_rss_*.csv`
- 分析脚本: `~/ws_livox/src/LIO/scripts/perf_analyzer.py`

使用方法:
```bash
python3 scripts/perf_analyzer.py --log logs/XXX.log --rss logs/pose_rss_XXX.csv
```
