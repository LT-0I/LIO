# LIO-Livox 模块运行时记录（2025-11-16，未启用 ARM 优化）

## 测试环境
- 硬件：Orangepi 5 Max（RK3588，4×Cortex-A76 + 4×Cortex-A55）
- 系统：Ubuntu 20.04，ROS Noetic
- 数据源：Livox 实测数据，`window=2`，每帧约 2.3 万点
- 构建：`catkin_make --pkg lio_livox`（**未开启 ARM 指令优化**，仅添加 `ROS_INFO` 耗时日志）
- 取得的日志样本见终端输出片段（帧索引 249–262）

## 模块耗时概览（无 ARM 优化基线）
| 模块 | 日志关键字 | 样本数 | 耗时范围 (ms) | 均值 (ms) | 备注 |
| --- | --- | --- | --- | --- | --- |
| 点云特征提取 | `ScanRegistration(Horizon)` | 15 | 7.9 – 26.6 | ≈ 18 | 每帧 24k Livox 点，特征提取模式为 Horizon |
| 后端非线性优化 | `Estimator::Estimate` | 13 | 30.7 – 90.1 | ≈ 54 | 滑窗大小 2，包含 IMU/点云残差与 Ceres 求解 |
| 地图关联与增量更新 | `Estimator::EstimateLidarPose` | 13 | 34.7 – 97.0 | ≈ 57 | 包含特征下采样、MapIncrementLocal 及地图互换 |
| 整帧处理耗时 | `PoseEstimation` | 13 | 39.8 – 169.0 | ≈ 81 | 覆盖 IMU 预积分、优化、发布、点云投影等全流程 |

> 注：均值为简单算术平均，仅用于“无 ARM 优化”场景的基线对比。后续开启优化后可追加新表格进行前后对照。

## 观察与建议
- 当前各模块耗时随帧内容波动较大，PoseEstimation 在个别帧超过 150 ms，可能与地图点数累积或 Ceres 迭代次数增加有关。
- `Estimator::Estimate` 与 `Estimator::EstimateLidarPose` 平均时间接近，说明优化与地图维护占据相当部分 CPU 时间，后续可重点优化特征匹配与 KD 树拷贝流程。
- `ScanRegistration` 已稳定在 10–25 ms 范围，继续优化可聚焦排序/分区或 OpenMP 并行。
- 建议将 `rosconsole` 输出重定向到文件，对更长时间窗口进行统计，或在关键函数处追加 `ros::WallTime` 以区分 CPU 占用与等待时间。

后续如开启 ARM 优化或更换数据集，请继续在本文件追加新的日期/条件/耗时结果，便于对比优化收益。

