# LIO-Livox 模块运行时记录

## 测试环境与日志来源
- 硬件：Orangepi 5 Max（RK3588，4×Cortex-A76 + 4×Cortex-A55）
- 系统：Ubuntu 20.04，ROS Noetic，Livox 实测数据，`window=2`，单帧≈2.0–2.4 万点
- 构建方式：
  - **无 ARM 优化**：原始 `catkin_make --pkg lio_livox`，仅添加 `ROS_INFO` 耗时日志
  - **启用 ARM 优化**：在 `CMakeLists.txt` 中开启 AArch64 NEON/LTO/OpenMP 等标志后重新 `catkin_make`
- 日志文件：
  - `src/LIO/运行时间/无ARM优化版/output2.log`
  - `src/LIO/运行时间/有ARM优化版/output.log`
  - `src/LIO/运行时间/有ARM优化版/output1.log`
- 统计方式：使用 `python3` 正则解析 `ROS_INFO`，对 `ScanRegistration(Horizon)`、`Estimator::Estimate`、`Estimator::EstimateLidarPose`、`PoseEstimation` 四类日志提取耗时，计算样本数、最小/最大值、平均值、95% 分位。

## 模块耗时概览

### 无 ARM 优化（基线）
| 模块 | 样本数 | 最小 (ms) | 最大 (ms) | 平均 (ms) | P95 (ms) |
| --- | --- | --- | --- | --- | --- |
| ScanRegistration(Horizon) | 2 689 | 5.33 | 270.68 | 15.16 | 25.02 |
| Estimator::Estimate | 1 544 | 23.32 | 330.44 | 149.95 | 255.13 |
| Estimator::EstimateLidarPose | 1 534 | 0.87 | 349.46 | 168.61 | 280.95 |
| PoseEstimation | 1 544 | 6.89 | 354.53 | 174.73 | 286.71 |

> 注：该日志覆盖长时间运行，尾部存在 250 ms 以上长尾帧，直接推高平均值与 P95。

### 启用 ARM 优化
| 模块 | 样本数 | 最小 (ms) | 最大 (ms) | 平均 (ms) | P95 (ms) |
| --- | --- | --- | --- | --- | --- |
| ScanRegistration(Horizon) | 3 022 | 5.14 | 231.56 | 13.78 | 23.03 |
| Estimator::Estimate | 2 114 | 27.66 | 306.18 | 122.41 | 196.39 |
| Estimator::EstimateLidarPose | 2 108 | 0.67 | 314.36 | 135.06 | 212.48 |
| PoseEstimation | 2 115 | 6.19 | 335.18 | 142.10 | 218.84 |

> 优化后平均和 P95 均明显下降，但缓存/地图波动仍会触发 200 ms 以上的极端帧。

## 对比分析
- 编译层优化对所有模块都有收益：平均耗时降幅依次为 ScanRegistration 9%、Estimator 18%、EstimateLidarPose 20%、PoseEstimation 19%，P95 同样下降 20–30%。
- 长尾帧依旧存在。`Estimator::Estimate` 和 `PoseEstimation` 在地图点数激增或 Ceres 迭代次数上升时仍可能冲到 200–300 ms，需要从算法和并发层面进一步压缩。
- Map 特征数量在优化运行中由约 1 k 增长至 5 k+，对内存拷贝和 KD 树构建造成持续压力；需配合体素下采样、按需拷贝以及 MapIncrement 并行化。

## 观察与建议
- 优先面向地图与优化阶段：引入局部体素筛选、MapIncrement 任务并行、KD 树共享内存以降低 200 ms 以上长尾。
- `Estimator::Estimate` 中的特征线程每帧重建，可考虑复用线程池或 OpenMP，减少线程管理成本。
- 记录更多运行指标：建议保留 `rosconsole` 完整日志，并在后续脚本中输出标准差、峰值帧索引；必要时在关键代码块使用 `ros::WallTime` 区分等待与计算时间。
- 持续维护本文件：每完成一次关键优化或更换数据集，附上新的统计表格和分析，便于纵向评估收益。

后续若进一步引入排序优化、KD 树并行、MapManager 重写等改动，请沿用上述表格格式补充新日期与条件，便于量化收益。

