# Iteration 003: 90ms 目标激进优化

**日期**: 2025-11-29
**目标**: 单帧处理时间 <90ms（从 100.2ms 减少 10.2ms）

---

## 1. 问题描述

Iteration 002 后，平均帧处理时间为 **100.18ms**，距离实时目标 (<100ms) 仅差 **0.18ms**。

需要在不影响精度的前提下，通过参数微调实现最后的性能突破。

## 2. 第一次尝试：激进 Ceres 参数（失败）

### 2.1 修改内容

```cpp
// 激进修改（已回滚）
options.max_num_iterations = 6;      // -25% 迭代
options.function_tolerance = 5e-4;   // 5x 放宽
options.gradient_tolerance = 5e-4;   // 5x 放宽
options.parameter_tolerance = 5e-4;  // 5x 放宽
options.max_consecutive_nonmonotonic_steps = 2;
```

### 2.2 测试结果（反效果！）

| 指标 | Iteration 002 | 激进尝试 | 变化 |
|------|---------------|----------|------|
| Ceres solve | 10.55 ms | **6.48 ms** | -39% ✓ |
| Residual build | 20.51 ms | **16.26 ms** | -21% ✓ |
| 总迭代次数 | 12,944 | **17,956** | **+39% ✗** |
| 平均帧间隔 | 100.18 ms | **102.98 ms** | **+2.8 ms ✗** |
| 内存 RSS | 213 MB | **390 MB** | **+83% ✗** |

### 2.3 失败原因分析

过于放宽的容差（5e-4）导致：
1. **每帧迭代次数增加**：单次求解更快，但不易收敛，需要更多优化循环
2. **总迭代次数从 12,944 增至 17,956 (+39%)**
3. **内存大幅增加**：更多迭代产生更多中间数据

**关键洞察**：Ceres 的收敛容差不能过于放宽，否则会触发更多优化循环。

## 3. 修正方案：保守 Ceres + 激进残差

### 3.1 Ceres 参数（保守）

```cpp
// 修正后的配置（当前）
options.max_num_iterations = 8;      // 保持原值，确保收敛
options.function_tolerance = 2e-4;   // 仅轻微放宽（原 1e-4）
options.gradient_tolerance = 2e-4;
options.parameter_tolerance = 2e-4;
options.max_consecutive_nonmonotonic_steps = 3;  // 保持原值
```

### 3.2 残差数量（激进，保留）

```yaml
# 残差限制 -30%（效果良好，保留）
max_corner_residuals: 350   # 原 500
max_surf_residuals: 500     # 原 750
max_non_residuals: 250      # 原 350
```

### 3.3 AdaptiveBudget（激进，保留）

```yaml
adaptive_budget_target_build_ms: 6.0    # 原 8.0
adaptive_budget_target_solve_ms: 18.0   # 原 25.0
adaptive_budget_tolerance: 0.15
adaptive_budget_adjust_ratio: 0.20
adaptive_budget_min_corner_residuals: 150
adaptive_budget_min_surf_residuals: 300
adaptive_budget_min_non_residuals: 150
```

## 4. 性能测试结果

**测试环境**: OrangePi 5 MAX (RK3588, 8GB RAM)
**测试数据**: car隧道.bag

### 4.1 第一次尝试（激进 Ceres - 失败）

| 阶段 | Iteration 002 | 激进尝试 | 变化 |
|------|---------------|----------|------|
| Ceres solve | 10.55 ms | 6.48 ms | -4.07 ms |
| Residual build | 20.51 ms | 16.26 ms | -4.25 ms |
| 总迭代次数 | 12,944 | 17,956 | +5,012 (+39%) |
| 平均帧间隔 | 100.18 ms | 102.98 ms | **+2.80 ms ✗** |
| 内存 RSS | 213 MB | 390 MB | +177 MB |

**结论**：激进 Ceres 参数导致反效果，已回滚。

### 4.2 修正版测试（待运行）

| 阶段 | Iteration 002 | 修正版 | 预期 |
|------|---------------|--------|------|
| Ceres solve | 10.55 ms | - ms | ~9-10 ms |
| Residual build | 20.51 ms | - ms | ~16-18 ms |
| 平均帧间隔 | 100.18 ms | - ms | <95 ms |
| 实时性比率 | 1.0018x | - | <0.95x |

## 5. 策略总结

| 参数类型 | 策略 | 原因 |
|----------|------|------|
| Ceres 迭代次数 | **保守** (8) | 减少迭代会导致收敛困难 |
| Ceres 容差 | **轻微放宽** (2e-4) | 过度放宽会触发更多循环 |
| 残差数量 | **激进** (-30%) | 直接减少计算量，效果明显 |
| AdaptiveBudget | **激进** | 动态调节配合残差削减 |

## 6. 回滚方法

如需恢复保守配置：

```yaml
# horizon_params.yaml
max_corner_residuals: 500
max_surf_residuals: 750
max_non_residuals: 350
adaptive_budget_target_build_ms: 8.0
adaptive_budget_target_solve_ms: 25.0
```

---

**状态**: 修正完成，待第二次测试
**编译状态**: ✓ 通过
**风险等级**: 中（已回滚激进 Ceres 参数）

**经验教训**：
1. Ceres 容差过度放宽会适得其反
2. 减少残差数量是更有效的优化路径
3. 迭代次数与收敛阈值需要平衡
