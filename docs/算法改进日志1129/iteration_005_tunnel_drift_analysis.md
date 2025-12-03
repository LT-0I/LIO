# Iteration 005: 隧道场景漂移问题分析与优化

**日期**: 2025-12-01
**目标**: 解决隧道场景中里程计严重漂移甚至停止移动的问题

---

## 1. 问题描述

在隧道场景中（车辆以 50-60km/h 行驶），算法出现严重漂移，里程计甚至完全停止移动。

### 1.1 测试数据

| 日志文件 | 总帧数 | 场景描述 |
|----------|--------|----------|
| 1409.log | 627 | 隧道短测试 |
| 1424.log | 1587 | 隧道长测试 |
| 1428.log | 1689 | 隧道长测试 |
| 1431.log | 1061 | 隧道测试 |

### 1.2 实时性指标 (均满足)

| 日志 | 平均帧间隔 | 实时性比率 | 状态 |
|------|-----------|-----------|------|
| 1409 | 99.33 ms | 0.9933x | ✓ 实时 |
| 1424 | 99.34 ms | 0.9934x | ✓ 实时 |
| 1428 | 99.84 ms | 0.9984x | ✓ 实时 |
| 1431 | 99.07 ms | 0.9907x | ✓ 实时 |

**结论**: 实时性不是问题，问题出在定位精度。

---

## 2. 深度分析结果

### 2.1 角点自适应模式分布

| 日志 | balanced | low_feature | high_feature | 漂移程度 |
|------|----------|-------------|--------------|----------|
| 1409 | 99.4% | **0.6%** | 0% | 正常 |
| 1431 | 99.7% | **0.3%** | 0% | 正常 |
| 1424 | 34.9% | **65.1%** | 0% | **严重** |
| 1428 | 8.8% | **90.9%** | 0.3% | **严重** |

**关键发现**: 漂移严重的日志 (1424, 1428) 大部分时间处于 `low_feature` 模式。

### 2.2 全局 KD 匹配趋势

**1428.log 全局 KD 值变化** (采样):

```
帧号    avg_global_kd    模式
1-3     0                low_feature (初始化)
4-10    700-850          high_feature (隧道入口前)
11-50   500-700          balanced
51-100  200-400          balanced (开始下降)
101-140 100-200          balanced (持续下降)
141+    50-80            low_feature (进入隧道深处)
```

**下降原因**: 隧道壁几何结构单一，PCA 特征值比检测失败率高。

### 2.3 Eigenvalue Ratio 检测失败率

| 日志 | 全局 KD 尝试 | 全局通过 | 失败率 |
|------|-------------|---------|--------|
| 1409 | 225 avg | 140 avg | **37.8%** |
| 1424 | 变化大 | - | ~35-40% |
| 1428 | 变化大 | - | ~35-40% |

**关键代码** (`Estimator.cpp:251`):
```cpp
if (saes.eigenvalues()[2] > corner_eigen_ratio_ * saes.eigenvalues()[1]) {
    // 通过: 特征值比 > 3.0
}
```

隧道壁接近平面结构，第一、第二特征值接近，导致角点被误判为非线特征而拒绝。

### 2.4 低特征帧分析

**1424.log 低特征模式时的残差保留**:
```
Frame 1162+: mode=low_feature avg_global_kd=60-80 eigen_ratio=2.50
  corner_kept: 100-130 (global=60-90, local=30-40)
  surf_kept: 700-800
  non_kept: 0-100
```

**1428.log 进入隧道深处后**:
```
Frame 150+: mode=low_feature avg_global_kd=50-70
  corner_kept: 90-130 (global=60-80, local=20-40)
  全局角点占比持续下降
```

---

## 3. 根本原因总结

### 3.1 漂移触发链 (Vicious Cycle)

```
隧道壁几何特征单一
    ↓
Eigenvalue ratio 测试失败率高 (~36%)
    ↓
全局角点匹配数量下降 (avg_global_kd: 130 → 50)
    ↓
触发 low_feature 模式 (threshold: 80)
    ↓
依赖局部匹配 (短时窗口，无绝对位置约束)
    ↓
累积漂移
    ↓
全局匹配更加失败 (位置偏差导致 KD 搜索失效)
    ↓
里程计停止移动
```

### 3.2 核心问题

1. **Eigenvalue ratio 阈值过严** (3.0): 隧道壁的线特征被误判
2. **low_feature 触发阈值过高** (80): 过早进入低特征模式
3. **无漂移检测与恢复机制**: 一旦进入恶性循环无法自动恢复

---

## 4. 优化方案

### 4.1 方案 A: 参数调优 (本次实施)

修改 `config/horizon_params.yaml`:

```yaml
# 放宽 eigenvalue ratio 阈值
corner_adaptive_default_eigen_ratio: 2.5    # 原 3.0
corner_adaptive_low_feature_eigen_ratio: 2.0 # 原 2.5

# 降低 low_feature 触发阈值
corner_adaptive_low_feature_global_kd: 50   # 原 80

# 增加低特征模式下的最小保留数
corner_adaptive_low_feature_min_keep: 250   # 原 200
```

**预期效果**:
- Eigenvalue ratio 放宽后，更多隧道壁角点被接受
- low_feature 触发阈值降低后，更长时间维持 balanced 模式
- 最小保留数增加后，低特征模式下有更多约束

### 4.2 方案 B: 代码增强 (备选)

1. **动态 thres_dist**: 当全局匹配率下降时自动扩大搜索半径
2. **面点权重增强**: 隧道壁是良好平面特征，增加 surf 残差权重
3. **漂移检测**: 监控 IMU 积分与优化位姿差异，异常时触发恢复

---

## 5. 实验记录

### 5.1 Baseline (优化前参数)

```yaml
corner_adaptive_default_eigen_ratio: 3.0
corner_adaptive_low_feature_eigen_ratio: 2.5
corner_adaptive_low_feature_global_kd: 80
corner_adaptive_low_feature_min_keep: 200
```

### 5.2 优化后参数 (待测试)

```yaml
corner_adaptive_default_eigen_ratio: 2.5
corner_adaptive_low_feature_eigen_ratio: 2.0
corner_adaptive_low_feature_global_kd: 50
corner_adaptive_low_feature_min_keep: 250
```

### 5.3 测试结果 - 参数调优 (1444.log)

| 测试项 | Baseline (1428) | 优化后 (1444) | 变化 |
|--------|-----------------|---------------|------|
| low_feature 占比 | 90.9% | **76.1%** | -14.8% ✓ |
| 全局通过率 | ~64% | **83.2%** | +19.2% ✓ |
| 漂移程度 | 严重 | **仍然严重** | ✗ |
| 实时性 | ✓ | ✓ | - |

### 5.4 1444.log 详细分析

**全局 KD 值下降趋势**:
```
帧号        avg_global_kd    说明
1-10        700-850          隧道入口前，特征丰富
30-50       500-600          开始下降
90-100      250-350          急剧下降
150+        45-70            进入隧道深处，低于阈值50
900+        11-29            全局匹配彻底失效，里程计停止
```

**残差保留情况 (帧 880-950)**:
```
corner_kept: 80-135 (global=18-52, local=50-84)
surf_kept: 680-880
non_kept: 0-50
```

**关键发现**:
1. 参数调优确实提升了全局通过率 (64%→83.2%)
2. 但 avg_global_kd 仍然在帧 150 左右降到 50 以下
3. 帧 900 后 avg_global_kd 只有 11-29，完全无法约束位姿
4. **结论: 参数调优无法解决根本问题——隧道深处全局地图匹配失效**

---

## 6. 根因深入分析

### 6.1 为什么全局 KD 会持续下降？

```
初始位置准确
    ↓
进入隧道，特征减少，局部漂移开始
    ↓
位置误差累积 (0.1m → 0.5m → 1m+)
    ↓
全局 KD 搜索半径 thres_dist=1.0m 无法找到对应点
    ↓
avg_global_kd 下降
    ↓
更多依赖局部匹配，漂移加剧
    ↓
恶性循环，里程计停止
```

### 6.2 核心问题: thres_dist 固定为 1.0m

代码 `Estimator.cpp:197`:
```cpp
if (_pointSearchSqDis[4] < thres_dist) {  // thres_dist = 1.0
    // 全局匹配成功
}
```

当位置漂移超过 1m 后，所有全局匹配都会失败。

---

## 7. 下一步方案: 代码增强

### 7.1 方案 B1: 动态扩展 thres_dist (推荐)

当 avg_global_kd 下降时，自动扩大搜索半径:

```cpp
// 动态调整搜索半径
double dynamic_thres_dist = thres_dist;
if (avg_global_kd < 100) {
    dynamic_thres_dist = thres_dist * 2.0;  // 扩大到 2m
}
if (avg_global_kd < 50) {
    dynamic_thres_dist = thres_dist * 4.0;  // 扩大到 4m
}
```

**风险**: 扩大搜索半径可能引入错误匹配
**缓解**: 结合 eigenvalue ratio 和距离权重降低错误匹配影响

### 7.2 方案 B2: 增强面点约束

隧道壁是良好的平面特征，当角点不足时增加面点权重:

```cpp
// 当角点不足时，增加面点权重
if (corner_kept < 100) {
    surf_weight *= 1.5;
}
```

### 7.3 方案 B3: 纯 IMU 退化保护

当全局匹配完全失效时，回退到纯 IMU 积分:

```cpp
if (avg_global_kd < 20 && corner_kept < 50) {
    // 退化模式: 纯 IMU + 局部面点
    use_global_corner = false;
}
```

---

## 8. 实施计划

1. ✓ 参数调优测试 - 效果不足
2. **下一步**: 实施方案 B1 (动态 thres_dist)
3. 如 B1 不足，叠加 B2 (增强面点)
4. 最后考虑 B3 (退化保护)

---

**状态**: 参数调优完成，效果不足，需要代码增强
**编译状态**: 需要修改 Estimator.cpp 并重新编译

---

## 9. 深度根因分析 (Deep Dive)

### 9.1 系统工作机制回顾

LIO系统的位姿估计基于以下约束的联合优化：

```
Ceres优化问题:
  min Σ(IMU残差) + Σ(角点残差) + Σ(面点残差) + Σ(非特征残差) + 边缘化约束
```

各约束的权重由信息矩阵控制：
- IMU残差: 由协方差矩阵逆决定，通常较强
- LiDAR残差: 由 `1/lidar_m` (lidar_m=1.5e-3) 决定，约 666.7

### 9.2 隧道场景的特殊性

**隧道几何特征**:
```
        ┌─────────────────────────┐
        │       天花板            │  ← 平面特征 (surf)
        │                         │
 墙壁 → ├─────────────────────────┤ ← 角点稀少，平面为主
        │                         │
        │       地面              │  ← 平面特征 (surf)
        └─────────────────────────┘
```

隧道壁几乎是连续的平面，缺乏明显的角点特征。PCA分析时：
- 角点要求: λ2 / λ1 > 3.0 (特征值比)
- 隧道壁: λ2 / λ1 ≈ 1.5-2.5 (接近平面，被拒绝)

### 9.3 关键问题：全局与局部地图的不同作用

**全局地图 (Global Map)**:
- 作用: 提供绝对位置约束，防止长期漂移
- 构建: 累积所有历史帧，按cube组织
- 匹配: 使用固定 thres_dist=1.0m 搜索

**局部地图 (Local Map)**:
- 作用: 提供短期相对位置约束
- 构建: 仅保留最近 N 帧 (localMapWindowSize)
- 匹配: 同样使用 thres_dist=1.0m

**核心问题**: 当全局匹配失效后，局部地图也会很快"跟随"漂移：
```
帧 100: 位置准确，局部地图准确
帧 101: 全局匹配失败，依赖局部匹配，位姿微小漂移
帧 102: 漂移后的位姿被加入局部地图
帧 103: 局部地图已包含漂移数据，继续积累误差
...
帧 200: 局部地图完全偏离真实位置
```

**关键洞察**: 局部地图是用当前估计的位姿构建的，如果位姿已经漂移，局部地图也是"漂移"的。这意味着**局部匹配无法阻止漂移，只能保持短期一致性**。

### 9.4 为什么里程计会"停止移动"？

分析 1444.log 中的数据：

```
帧 900 附近:
- avg_global_kd = 11-29 (几乎无全局匹配)
- corner_kept = 80-135 (大部分来自局部)
- surf_kept = 680-880 (面点较多，但也主要是局部)
```

当位姿漂移超过 1m 后：
1. 全局KD搜索在错误位置搜索，找不到对应点
2. 局部地图跟随漂移，提供"虚假"的一致性约束
3. IMU积分提供运动约束，但被"虚假"的LiDAR约束抵消
4. Ceres优化收敛到一个"静止"状态 —— 因为局部地图没有变化

**"停止移动"的本质**: 漂移后的局部地图与当前帧在"漂移后的坐标系"中仍然匹配良好，优化器认为没有运动发生。

### 9.5 面点约束为何无法挽救？

虽然隧道有丰富的面点特征 (surf_kept=680-880)，但：

1. **面点同样存在全局/局部匹配问题**: 代码结构相同，全局面点匹配也使用 thres_dist=1.0m
2. **面点权重被削弱**: `plan_weight_tan = 0.0003`，即切向权重只有法向的 0.03%
3. **面点约束方向性**: 面点只约束法向距离，对沿面滑动不敏感

在隧道中，车辆主要沿隧道轴向运动。面点（隧道壁）提供的约束：
- 强约束: 垂直于隧道壁的方向 (横向/垂向)
- 弱约束: 平行于隧道壁的方向 (纵向) ← **这正是行驶方向！**

### 9.6 问题总结

```
根本原因链:
┌─────────────────────────────────────────────────────────────────┐
│ 1. 隧道几何单一 → 角点特征稀少                                   │
│ 2. 角点稀少 → 全局角点匹配数量下降                               │
│ 3. 全局匹配下降 → 小漂移开始                                     │
│ 4. 漂移 > thres_dist → 全局匹配完全失效                         │
│ 5. 全局失效 → 依赖局部匹配                                       │
│ 6. 局部地图跟随漂移 → 提供虚假一致性                             │
│ 7. 面点约束纵向弱 → 无法约束行驶方向                             │
│ 8. IMU被虚假LiDAR约束抵消 → 优化器认为静止                       │
│ 9. 里程计停止移动                                                │
└─────────────────────────────────────────────────────────────────┘
```

---

## 10. 全面解决方案

基于深度分析，单一方案无法解决问题，需要多管齐下：

### 10.1 方案 A: 动态扩展 thres_dist (打破恶性循环)

当全局匹配率下降时，扩大搜索半径，给系统"自我纠正"的机会：

```cpp
// Estimator.cpp 修改
double dynamic_thres_dist = thres_dist;
if (avg_global_kd < 100 && avg_global_kd > 0) {
    // 线性扩展: avg_global_kd=100 时 1.0m, avg_global_kd=20 时 4.0m
    double scale = 1.0 + 3.0 * (100.0 - avg_global_kd) / 80.0;
    dynamic_thres_dist = thres_dist * std::min(scale, 4.0);
}
```

**原理**: 扩大搜索半径后，即使位姿有1-2m偏差，仍可能找到全局对应点，从而提供正确约束。

### 10.2 方案 B: 增强面点纵向约束权重

当角点不足时，增加面点在所有方向的约束权重：

```cpp
// 当角点严重不足时，提升面点切向权重
if (avg_global_kd < 50) {
    plan_weight_tan = 0.1;  // 从 0.0003 提升到 0.1
}
```

**原理**: 虽然单个面点对纵向约束弱，但大量面点（800+）的联合约束可以提供一定的纵向约束。

### 10.3 方案 C: 检测并响应退化状态

添加退化检测，在极端情况下更信任IMU：

```cpp
// 检测退化: 全局匹配几乎为零
bool degraded = (avg_global_kd < 20 && keptCornerGlobal < 30);
if (degraded) {
    // 增加IMU权重或减少可疑LiDAR约束
    // 记录警告日志
}
```

### 10.4 实施优先级

1. **立即实施**: 方案 A (动态 thres_dist) - 最直接解决匹配失效问题
2. **同时实施**: 方案 B (增强面点权重) - 利用隧道丰富的面点特征
3. **观察效果后**: 方案 C (退化检测) - 作为安全网

---

## 11. 代码修改计划

### 11.1 修改文件

- `src/lio/Estimator.cpp`: 实现动态 thres_dist 和面点权重增强
- `include/Estimator/Estimator.h`: 添加必要的成员变量

### 11.2 关键修改点

1. **行 1071**: thres_dist 赋值处 → 改为动态计算
2. **行 1070**: plan_weight_tan 赋值处 → 改为条件增强
3. **行 197, 287, 292, 602, 670, 675**: 所有使用 thres_dist 的地方 → 使用动态值

---

## 12. 代码修改实施记录

### 12.1 修改状态: ✓ 已完成并编译

### 12.2 Estimator.h 修改

新增成员变量 (行 363-365):
```cpp
// 动态搜索半径 (隧道场景优化)
double last_avg_global_kd_ = 100.0;  // 上一帧的平均全局KD匹配数
double dynamic_thres_dist_ = 1.0;    // 动态调整的搜索半径
```

### 12.3 Estimator.cpp 修改

**修改 1: 动态 thres_dist 计算 (行 1069-1095)**

```cpp
if(windowSize == SLIDEWINDOWSIZE) {
  // 动态搜索半径: 根据上一帧的全局匹配情况调整
  // 当全局匹配下降时扩大搜索范围，打破漂移恶性循环
  double base_thres_dist = 1.0;
  if (last_avg_global_kd_ < 100.0 && last_avg_global_kd_ > 0) {
    // 线性扩展: avg_global_kd=100 时 1.0m, avg_global_kd=20 时 4.0m
    double scale = 1.0 + 3.0 * (100.0 - last_avg_global_kd_) / 80.0;
    scale = std::min(scale, 4.0);
    dynamic_thres_dist_ = base_thres_dist * scale;
  } else {
    dynamic_thres_dist_ = base_thres_dist;
  }
  thres_dist = dynamic_thres_dist_;

  // 动态面点权重: 当角点严重不足时增强面点约束
  // 隧道场景下面点丰富，可以提供更多约束
  if (last_avg_global_kd_ < 50.0) {
    plan_weight_tan = 0.01;  // 低特征时增强 (原0.0003)
  } else if (last_avg_global_kd_ < 100.0) {
    plan_weight_tan = 0.003; // 中等特征时轻微增强
  } else {
    plan_weight_tan = 0.0003; // 正常情况
  }
}
```

**修改 2: 更新历史全局KD值 (行 1199-1202)**

```cpp
// 更新历史全局KD值，用于下一帧的动态搜索半径计算
if(iterOpt == 0 && windowSize == SLIDEWINDOWSIZE){
  last_avg_global_kd_ = avg_global_kd;
}
```

**修改 3: 增强日志输出 (行 1481)**

```cpp
ROS_INFO("Estimator corner adaptive iter %d: mode=%s avg_global_kd=%.1f limit=%d eigen_ratio=%.2f thres_dist=%.2f plan_w=%.4f",
         iterOpt, mode_str, avg_global_kd, maxCornerResidualsPerFrame,
         corner_eigen_ratio_, thres_dist, plan_weight_tan);
```

### 12.4 优化原理

| 参数 | 正常场景 | 中等特征 (kd<100) | 低特征 (kd<50) |
|------|---------|------------------|---------------|
| thres_dist | 1.0m | 1.0-2.5m (线性) | 2.5-4.0m |
| plan_weight_tan | 0.0003 | 0.003 | 0.01 |

**打破恶性循环机制**:
1. 当 avg_global_kd 下降时，自动扩大搜索半径
2. 即使位姿有 1-2m 偏差，仍可能找到全局对应点
3. 全局约束恢复后，位姿被拉回正确位置
4. 同时增强面点约束，利用隧道丰富的平面特征

---

## 13. 测试结果与问题修复

### 13.1 第一次测试 (1518.log)

**问题发现**: 动态 thres_dist 完全没有生效！
- 所有帧 thres_dist = 1.00 或 10.00
- 即使 avg_global_kd 降到 45，thres_dist 仍是 1.0

**根因分析**:
行 1285 `thres_dist = 1.0;` 硬编码覆盖了动态设置的值：
```cpp
if(windowSize == SLIDEWINDOWSIZE) {
    thres_dist = 1.0;  // ← BUG: 覆盖了动态值！
```

### 13.2 Bug 修复

删除行 1285 的硬编码覆盖，改为注释说明：
```cpp
if(windowSize == SLIDEWINDOWSIZE) {
    // 注意: thres_dist 已在函数开头根据 last_avg_global_kd_ 动态设置
    // 此处不再覆盖，保持动态值
```

### 13.3 编译状态: ✓ 成功 (第二次)

下一步: 重新测试隧道 rosbag，预期效果:
1. 当 avg_global_kd < 100 时，thres_dist 应该 > 1.0m
2. 当 avg_global_kd = 50 时，thres_dist ≈ 2.9m
3. 当 avg_global_kd = 20 时，thres_dist = 4.0m (上限)
