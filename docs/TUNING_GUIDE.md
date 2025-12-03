# LIO-Livox 调参实战手册

本手册面向需要在不同场景（隧道、停车场、野外）中快速调试 LIO-Livox 的工程师，围绕地图管理、残差预算、特征自适应与日志诊断四大模块，给出参数原理、可调区间以及推荐的决策流程。

---

## 1. 模块概览

- **MapManager** 负责维护稀疏体素地图与局部 KD-tree，参数可在 `MapManagerConfig` 中找到：  

```14:32:include/MapManager/Map_Manager.h
struct MapManagerConfig{
  int width = 21;
  int height = 11;
  int depth = 21;
  int local_window = 60;
  int map_skip_frame = 2;
  ...
  bool enable_cube_prune = true;
};
```

- **Estimator** 的残差上限、角点自适应与残差预算写在 `EstimatorResidualConfig`：  

```25:54:include/Estimator/Estimator.h
struct CornerAdaptiveConfig{
	bool enable = true;
	double default_eigen_ratio = 3.0;
	...
};
struct ResidualBudgetConfig{
	bool enable = false;
	double target_residual_build_ms = 8.0;
	...
};
```

- **auto_eval.py** 自动评估脚本，对比OPi与NUC基线，生成综合报告并归档：  

```bash
python3 scripts/auto_eval.py              # 分析并归档
python3 scripts/auto_eval.py --no-archive # 仅分析不归档
python3 scripts/auto_eval.py --skip-init 10  # 跳过前10秒初始化漂移
```

EVO轨迹对比默认跳过前5秒初始化期，使用 `--align` 和 `--correct_scale` 进行SE(3)对齐。

---

## 2. MapManager 调参思路

### 2.1 栅格尺寸与更新频率

| 参数 | 原理 | 默认 | 隧道建议 | 开阔场景 |
| --- | --- | --- | --- | --- |
| `map_width` / `map_height` / `map_depth` | 控制 cube 网格数量，越大 RSS 越高 | 21/11/21 | 13/7/9 (约 1/3 cube) | 保持默认或略增 |
| `map_local_window` | MapManager 中为不同方向缓存多少帧以做匹配 | 60 | 40–50 | 60–80 |
| `map_skip_frame` | 每隔 N 帧才写入地图，降低 MapIncrement 频率 | 2 | 2（实时）或 3（降负载） | 2 |

调参流程：优先根据目标场景粗调 `map_width/depth`，然后观察 RSS 曲线；如果 CPU 占用 >80%，先把 `map_skip_frame` +1，再检查轨迹是否受影响。

### 2.2 场景范围裁剪与 cube 清理

- `map_forward_range` 等参数将绝对距离转换为 cube 限幅，配合 `enable_cube_prune` 清理远离本体的体素，实现 RSS 常数级上限。  
- 隧道推荐：`forward/backward=60/20`、`side=4`、`vertical=4`；停车场可放宽到 `80/40/12/12`。  
- 当 ROS 内存曲线持续爬升时，第一步就是缩小这些范围。

### 2.3 局部包围盒与 KD-tree 点数

| 参数 | 影响 | 隧道 | 停车场 |
| --- | --- | --- | --- |
| `local_box_forward/backward` | 从地图拉取局部点的空间窗口 | 35–40 / 8–10 | 45–50 / 12 |
| `local_box_side/vertical` | 横向/竖向窗口 | 4–5 / 3–4 | 8–10 / 6–7 |
| `local_corner/surf/non_max_points` | Voxel 后局部云点数上限，直接影响 RSS | 150k / 200k / 100k 起步 | 180k / 240k / 120k |

当 `auto_eval.py` 报告残差数量下降或 Ceres 粗糙时，适当放宽窗口或点数；当 RSS 平台 >400 MB，则继续降低上限，每次 20k 步长。

---

## 3. Estimator 与 Ceres 残差调参

### 3.1 基础残差上限

- `max_corner_residuals` / `max_surf_residuals` / `max_non_residuals` 控制每帧送入 Ceres 的约束数量。  
- 隧道角点稀疏，可将角点上限降到 350–400，同时保留足够的平面约束（600–700）。  
- 如果 `Ceres solve`（来自 `auto_eval.py`）对齐后仍 >25 ms，可整体按 0.8 缩放三个上限。

### 3.2 CornerAdaptive

角点自适应逻辑控制“特征太少/太多”场景下的保留比例：  

```25:44:include/Estimator/Estimator.h
struct CornerAdaptiveConfig{
	bool enable = true;
	double default_eigen_ratio = 3.0;
	double low_feature_eigen_ratio = 2.5;
	int low_feature_global_kd = 80;
	int low_feature_min_keep = 200;
	int high_feature_global_kd = 800;
	int high_feature_max_keep = 400;
};
```

- `low_feature_global_kd` 越小，越容易触发“角点不足”策略；隧道中可改为 60 并把 `low_feature_min_keep` 提升到 220。  
- 若出现局部抖动，可把 `high_feature_max_keep` 调高至 450–480，避免 Ceres 没有足够约束。

### 3.3 AdaptiveBudget 与求解器原理

Residual Budget 基于实时耗时反馈（`target_residual_build_ms`、`target_ceres_solve_ms`）自动缩放残差数量。当 `build_ms` 或 `solve_ms` 超出容忍区间 (`tolerance_ratio`) 时，就按 `adjust_ratio` 缩放运行时上限，且不会低于 `min_*_residuals`。  

这与 Ceres 信赖域求解过程耦合：根据 Ceres 官方文档，`Solver::Options::max_num_iterations`、`max_solver_time_in_seconds`、`function_tolerance` 等参数将限制求解时间与收敛性（参考 `Ceres Solver` “Solver Options” 文档）。因此减少残差数量能直接降低每次迭代的线性化与求解开销，同时避免 `max_solver_time_in_seconds` 触发的硬停止。

实战建议：

1. 实时性不足（Solve 平均 >25 ms）：把 `target_build_ms` 降到 6.5，`target_solve_ms` 降到 20，观察 2–3 个 bag；若精度下滑，再把 `tolerance_ratio` 放宽到 0.3。
2. 精度下降：先提高 `min_*_residuals`（如角点≥220），再把 `target_*` 拉回默认值。

---

## 4. 日志与诊断

1. 在 `horizon_params.yaml` 中保持 `enable_debug_log: true`，启用benchmark日志输出。  
2. 运行实验后使用 `auto_eval.py` 分析日志并归档：  

```bash
# 分析最新日志，自动归档到 branch_reports/{branch}/
python3 scripts/auto_eval.py

# 仅分析，不归档
python3 scripts/auto_eval.py --no-archive

# 跳过前10秒初始化（用于初始化不稳定的场景）
python3 scripts/auto_eval.py --skip-init 10
```

3. **EVO轨迹对比说明**：
   - 默认跳过前5秒初始化期（`--skip-init 5`）
   - 使用 `--align` 进行SE(3)对齐，消除坐标系差异
   - 使用 `--correct_scale` 进行尺度校正
   - 若初始化漂移严重，可增大 `--skip-init` 值

4. 调参循环：跑 bag → 执行 `python3 scripts/auto_eval.py` → 查看 Timing Performance 和 Optimization Quality 章节 → 决定调参方向。

---

## 5. 场景调参 Cookbook

### 5.1 隧道（长时间、窄空间）

1. 设定范围参数：`map_forward/backward=60/20`、`map_side/vertical=4`、`enable_cube_prune=true`。  
2. 栅格尺寸收紧：`map_width/depth=13/9`，`map_local_window=45`，`map_skip_frame=2`。  
3. 局部窗口：`local_box_forward/backward=38/9`，`local_box_side/vertical=4/3.5`。  
4. 局部点数：`corner/surf/non=150k/200k/100k` 起步，若 RSS 仍>350 MB，再降 20k。  
5. 残差：`max_corner=380`、`max_surf=650`、`max_non=280`，`AdaptiveBudget` 设 `target_build=6.5`、`target_solve=20`，`min_corner=220`。  
6. CornerAdaptive：`low_feature_global_kd=60`、`high_feature_max_keep=450`。

### 5.2 地下/室内停车场

- 空间较宽但结构重复：保持默认地图尺寸，只调范围 `forward/backward=70/30`；局部 box 设 `side=8` 保证车位墙体曲面。  
- Residual Budget 用默认 8/25 ms，但观察 `auto_eval.py` 是否提示 `MapManager update` 过慢；若 >15 ms，可将 `map_skip_frame` 暂时调至 3。

### 5.3 室外开阔场景

- 需要更大视域：`map_forward/backward=100/40`、`map_side/vertical=16/12`，但要注意 RSS；若 RAM 宽裕，可维持 `local_*_max_points` 默认。  
- CornerAdaptive 建议提升 `default_eigen_ratio` 到 3.2，防止保留过多噪声角点。

### 5.4 应急策略

- **RSS 持续增长**：优先缩小 `map_*_range`，其次降低 `local_*_max_points`。  
- **实时性掉到 <8 Hz**：调 `map_skip_frame` +1，降低 `max_*_residuals`，或把 `AdaptiveBudget` 目标时间调小。  
- **精度发散**：恢复 Residual 目标，增加 `min_*_residuals`，再检查 `CornerAdaptive` 是否把角点限制得太紧。

---

## 6. 典型调参流程

1. **建立基线**：在目标场景用默认参数跑一次，记录 `auto_eval.py` 输出（框架、RSS）。  
2. **判断瓶颈**：  
   - `Estimator::Estimate` / `Ceres solve` 过高 → 调 Residual / AdaptiveBudget；  
   - `MapManager update` 过高或 RSS 上升 → 调 `map_*`、`local_*`。  
3. **单变量修改**：一次只改一组（例如局部窗口），跑 3–5 分钟并记录日志。  
4. **对比验证**：使用 `auto_eval.py` 的 avg/max 指标对比前后迭代；若差异显著，再固定该参数进入下一轮。  
5. **长期试跑**：隧道等长包需要至少 15 分钟，确认 RSS 是否稳定；若稳定在 <300 MB，即可固化参数。  
6. **文档回写**：将最终参数填入 `config/horizon_params.yaml`，并在注释中记录“适用场景 + 调参理由”，方便下一次复现。

---

## 7. 参考

- LIO-Livox 源码中的 MapManager / Estimator 参数定义见 `include/MapManager/Map_Manager.h` 与 `include/Estimator/Estimator.h`。  
- Ceres Solver 官方 “Solver Options” 文档（`/ceres-solver/ceres-solver`）提供了信赖域、Dogleg、迭代上限等可调项，可结合 Residual Budget 共同调优。  
- `scripts/auto_eval.py` 提供标准化的日志解析和NUC基线对比，是评估调参效果的主要工具。

---

**提示**：任何代码/配置修改后，请执行 `catkin_make -DCATKIN_WHITELIST_PACKAGES="lio_livox"` 确认通过，再进行实车/实包实验。

