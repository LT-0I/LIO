# 下次会话待办事项

## 当前状态: 实时性目标已达成 ✓

Iteration 004 通过 OpenMP 动态调度成功实现实时性突破：
- 平均帧间隔: 99.83 ms (< 100ms ✓)
- 实时性比率: 0.9983x (< 1.0x ✓)
- 内存峰值: 214 MB

---

## 下一步可选方向

### 方向 1: 长时间稳定性验证 (推荐)

使用更长的 bag 文件验证算法稳定性：

```bash
# 使用 car隧道.bag (686s) 测试
cd ~/ws_livox
source devel/setup.bash
roslaunch lio_livox horizon.launch &
scripts/capture_pose_bt.sh &
rosbag play ~/Desktop/rosbags/car隧道.bag --clock

# 分析结果
python3 src/LIO/scripts/perf_analyzer.py --log src/LIO/logs/XXX.log --rss src/LIO/logs/pose_rss_XXX.csv
```

### 方向 2: 退化场景测试

在隧道、长走廊等特征稀疏场景验证鲁棒性：
- 观察迭代次数是否剧增
- 检查是否有定位丢失

### 方向 3: 进一步优化 (目标 90ms)

如果需要进一步降低帧处理时间：

1. **Ceres 线程数优化**
   ```cpp
   // src/lio/Estimator.cpp
   options.num_threads = 4;  // 只使用大核
   ```

2. **MapManager 异步更新**
   - 将地图更新移至独立线程
   - 与下一帧预处理并行

3. **特征提取并行化**
   - LidarFeatureExtractor 使用 OpenMP

### 方向 4: 精度验证

使用 EVO 工具评估轨迹精度：

```bash
# 安装 evo
pip3 install evo

# 评估 APE (绝对位姿误差)
evo_ape tum ground_truth.txt estimated.txt -va --plot

# 评估 RPE (相对位姿误差)
evo_rpe tum ground_truth.txt estimated.txt -va --plot
```

---

## 重要文件位置

| 文件 | 说明 |
|------|------|
| `.claude/optimization_progress.md` | 优化进度总览 |
| `.claude/performance_baseline.md` | 性能基线数据 |
| `算法改进日志1129/iteration_004_*.md` | 最新迭代详情 |
| `config/horizon_params.yaml` | 配置文件 |
| `scripts/perf_analyzer.py` | 性能分析脚本 |

---

## 快速命令

```bash
# 编译
cd ~/ws_livox && catkin_make -j8

# 运行测试
source ~/ws_livox/devel/setup.bash
roslaunch lio_livox horizon.launch &
rosbag play ~/Desktop/rosbags/XXX.bag --clock

# 分析日志
cd ~/ws_livox/src/LIO
python3 scripts/perf_analyzer.py --log logs/XXX.log --rss logs/pose_rss_XXX.csv
```

---

## 已完成的优化

| 迭代 | 内容 | 效果 |
|------|------|------|
| 001 | VoxelIndex 体素索引 | 111.44ms |
| 002 | MapManager 增量快照 | 100.18ms |
| 003 | Ceres 参数调优 | ✗ 失败 |
| **004** | **OpenMP 动态调度** | **99.83ms ✓** |
