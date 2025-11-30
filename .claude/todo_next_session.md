# 下次会话待办事项

## 紧急: 回滚配置

当前 `config/horizon_params.yaml` 中的残差配置导致性能下降，需要回滚：

```yaml
# 当前（错误）     ->   应恢复为
max_corner_residuals: 350    ->   500
max_surf_residuals: 500      ->   750
max_non_residuals: 250       ->   350

adaptive_budget_target_build_ms: 6.0    ->   8.0
adaptive_budget_target_solve_ms: 18.0   ->   25.0
adaptive_budget_tolerance: 0.15         ->   0.25
adaptive_budget_adjust_ratio: 0.20      ->   0.15
adaptive_budget_min_corner_residuals: 150   ->   200
adaptive_budget_min_surf_residuals: 300     ->   450
adaptive_budget_min_non_residuals: 150      ->   250
```

## 回滚后预期效果

恢复到 Iteration 002 的性能水平：
- 平均帧间隔: ~100.18 ms
- 实时性比率: ~1.0018x
- 内存峰值: ~260 MB

## 进一步优化方向

### 方向 1: MapManager update 优化 (11.7ms)
- 分析 update 中的耗时分布
- 考虑异步更新

### 方向 2: Marginalization 并行化 (8ms)
- 可以与下一帧的预处理并行

### 方向 3: 更好的初值估计
- 减少每帧所需的迭代次数
- 不是通过减少残差，而是提供更好的初始姿态

### 方向 4: 条件性优化
- 在特征丰富区域减少迭代
- 在特征稀疏区域保持完整迭代

## 重要文件位置

- 进度记录: `.cursor/optimization_progress.md`
- 性能基线: `.cursor/performance_baseline.md`
- 配置文件: `config/horizon_params.yaml`
- Ceres 参数: `src/lio/Estimator.cpp:1490-1501`
- 分析脚本: `scripts/perf_analyzer.py`

## 快速命令

```bash
# 编译
cd ~/ws_livox && catkin_make -j8

# 运行测试
source ~/ws_livox/devel/setup.bash
roslaunch lio_livox horizon.launch &
rosbag play ~/car隧道.bag --clock

# 分析日志
cd ~/ws_livox/src/LIO
python3 scripts/perf_analyzer.py --log logs/XXX.log --rss logs/pose_rss_XXX.csv
```
