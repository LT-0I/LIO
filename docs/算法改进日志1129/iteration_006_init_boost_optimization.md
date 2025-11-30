# Iteration 006: InitBoost 初始化阶段加速优化

**日期**: 2025-11-30
**目标**: 通过减少初始化阶段的计算负载，加速算法启动时的帧处理速度

---

## 1. 问题描述

### 1.1 现有问题

在 Iteration 005 的慢帧根因分析中发现：

1. **初始化阶段耗时是稳定期的 2 倍**：
   - 初始化阶段平均帧间隔：240.59 ms
   - 稳定期平均帧间隔：122.00 ms
   - 比值：1.97x

2. **初始化阶段的主要瓶颈**：
   - Residual build：首帧需构建完整 KD-tree
   - 地图初始化开销较大
   - 完整的 5 次迭代优化

3. **影响**：
   - 算法启动后前 7 秒比较卡顿
   - 用户体验不佳
   - 可能导致初始化期间的延迟累积

### 1.2 Iteration 005 慢帧统计

| 阶段 | 慢帧数 | 平均间隔 | 最大间隔 |
|------|--------|----------|----------|
| 初始化 (Frame < 20) | 123 | 240.59 ms | 982.80 ms |
| 稳定期 (Frame >= 20) | 26,159 | 122.00 ms | 1181.08 ms |

---

## 2. 解决方案

### 2.1 InitBoost 策略

实现三层加速机制：

| 策略 | 参数 | 作用 |
|------|------|------|
| 残差比例缩减 | `init_residual_ratio` | 初始化阶段使用更少的残差点 |
| 迭代次数限制 | `init_max_iterations` | 减少优化迭代次数（5 → 2） |
| 跳帧机制 | `init_skip_first_n_frames` | 跳过前 N 帧的完整优化 |

### 2.2 可配置参数

```yaml
# horizon_params.yaml
enable_init_boost: false           # 启用初始化加速（默认关闭）
init_residual_ratio: 0.5          # 残差数量比例 (0.3~0.7)
init_skip_first_n_frames: 0       # 跳过前 N 帧（0=不跳过）
init_max_iterations: 2            # 初始化最大迭代次数（正常为 5）
```

### 2.3 生效条件

InitBoost 仅在以下条件同时满足时生效：
- `enable_init_boost: true`
- `windowSize != SLIDEWINDOWSIZE`（即初始化阶段）

---

## 3. 代码改动

### 3.1 修改的文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `include/Estimator/Estimator.h` | 修改 | 添加 `InitBoostConfig` 结构体 |
| `src/lio/PoseEstimation.cpp` | 修改 | 读取 InitBoost 参数 |
| `src/lio/Estimator.cpp` | 修改 | 应用初始化加速逻辑 |
| `config/horizon_params.yaml` | 修改 | 添加 InitBoost 配置项 |

### 3.2 InitBoostConfig 结构体

```cpp
// Estimator.h
struct InitBoostConfig{
    bool enable = false;                // 启用初始化加速
    double residual_ratio = 0.5;        // 初始化阶段残差数量比例 (0.3~0.7)
    int skip_first_n_frames = 0;        // 跳过前 N 帧的完整优化
    int max_iterations = 2;             // 初始化阶段最大迭代次数 (正常为 4)
};

struct EstimatorResidualConfig{
    // ... 其他配置 ...
    InitBoostConfig init_boost;         // 新增
};
```

### 3.3 迭代次数控制

```cpp
// Estimator.cpp - Estimate 函数
const bool is_init_phase = (windowSize != SLIDEWINDOWSIZE);
const bool init_boost_active = is_init_phase && residual_config_.init_boost.enable;
const int max_iters = init_boost_active
                      ? std::max(1, residual_config_.init_boost.max_iterations)
                      : 5;
```

### 3.4 跳帧机制

```cpp
// Estimator.cpp - 在 Estimate 函数开始处
if(init_boost_active &&
   residual_config_.init_boost.skip_first_n_frames > 0 &&
   static_cast<int>(frame_count) < residual_config_.init_boost.skip_first_n_frames){
    if(log_module_timing_){
        ROS_INFO("[InitBoost] Skipping frame %u (first %d frames)",
                 frame_count, residual_config_.init_boost.skip_first_n_frames);
    }
    frame_count++;
    return;
}
```

### 3.5 残差数量缩减

```cpp
// Estimator.cpp - 残差限制处
int maxCornerResidualsPerFrame = adaptiveCornerLimit;
int maxSurfResidualsPerFrame = std::max(1, runtime_surf_limit_);
int maxNonResidualsPerFrame = std::max(1, runtime_non_limit_);

if(init_boost_active && residual_config_.init_boost.residual_ratio > 0.0 &&
   residual_config_.init_boost.residual_ratio < 1.0){
    const double ratio = residual_config_.init_boost.residual_ratio;
    maxCornerResidualsPerFrame = std::max(50, static_cast<int>(maxCornerResidualsPerFrame * ratio));
    maxSurfResidualsPerFrame = std::max(100, static_cast<int>(maxSurfResidualsPerFrame * ratio));
    maxNonResidualsPerFrame = std::max(50, static_cast<int>(maxNonResidualsPerFrame * ratio));
    if(log_module_timing_ && iterOpt == 0){
        ROS_INFO("[InitBoost] Reduced residuals: corner=%d surf=%d non=%d (ratio=%.2f)",
                 maxCornerResidualsPerFrame, maxSurfResidualsPerFrame, maxNonResidualsPerFrame, ratio);
    }
}
```

---

## 4. 预期效果

### 4.1 理论分析

| 优化项 | 原始值 | InitBoost | 预期加速 |
|--------|--------|-----------|----------|
| 迭代次数 | 5 | 2 | ~2.5x |
| 残差数量 | 100% | 50% | ~1.5x |
| Ceres 求解 | ~25ms | ~10ms | ~2.5x |

### 4.2 初始化阶段预期改善

| 指标 | 原始 | 预期 |
|------|------|------|
| 平均帧间隔 | 240.59 ms | ~150 ms |
| 最大帧间隔 | 982.80 ms | ~500 ms |

---

## 5. 调参指南

### 5.1 保守配置（精度优先）

```yaml
enable_init_boost: true
init_residual_ratio: 0.7          # 保留 70% 残差
init_skip_first_n_frames: 0       # 不跳帧
init_max_iterations: 3            # 3 次迭代
```

### 5.2 激进配置（速度优先）

```yaml
enable_init_boost: true
init_residual_ratio: 0.4          # 保留 40% 残差
init_skip_first_n_frames: 2       # 跳过前 2 帧
init_max_iterations: 2            # 2 次迭代
```

### 5.3 注意事项

1. **精度影响**：InitBoost 会略微降低初始化阶段的定位精度
2. **建议测试**：启用前在目标场景中测试轨迹质量
3. **默认关闭**：`enable_init_boost: false` 以保持向后兼容

---

## 6. 总结

### 6.1 本次优化成果

| 改进项 | 状态 |
|--------|------|
| InitBoostConfig 结构体 | ✓ 实现 |
| 迭代次数限制 | ✓ 实现 |
| 跳帧机制 | ✓ 实现 |
| 残差比例缩减 | ✓ 实现 |
| 可配置参数 | ✓ 实现 |
| 日志输出 | ✓ 实现 |

### 6.2 迭代历程总结

| 迭代 | 主要优化 | 实时性比率 | 状态 |
|------|----------|------------|------|
| 001 | 体素索引 O(1) 查询 | 1.1144x | ✗ |
| 002 | 增量式快照 | 1.0018x | ✗ (接近) |
| 003 | Ceres 求解器调优 | - | 未测试 |
| 004 | RK3588 大核并行 | 0.9977x | ✓ |
| 005 | FIFO 实时性保护 | 0.9979x | ✓ 稳定 |
| **006** | **InitBoost 初始化加速** | **待测试** | **开发完成** |

### 6.3 后续工作

1. 运行实际 rosbag 测试验证 InitBoost 效果
2. 对比启用/禁用 InitBoost 的初始化阶段性能
3. 评估对定位精度的影响
4. 根据测试结果调整默认参数

---

**编译验证**: ✓ 通过
**测试状态**: 待测试
