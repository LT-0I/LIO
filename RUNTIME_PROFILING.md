# LIO-Livox 模块运行时记录（目标：单帧 < 100 ms）

## 环境与数据
- **硬件/系统**：Orangepi 5 Max（RK3588，Ubuntu 20.04，ROS Noetic），Livox Horizon，`/livox/lidar` 频率固定 10 Hz（帧间隔 100 ms）。
- **算法版本**：当前仍为“未启用 ARM 指令优化”的原始代码，仅在关键环节加入 `ROS_INFO` 时间戳。
- **测试场景**：室外开阔区域，真实激光雷达实时输入；窗口大小 `window=2`。
- **日志**：`output3.log`（总计 1367 帧），已包含前端、后端及帧级耗时打印。
- **统计方式**：使用 `python3` 解析日志，计算样本数、最小/最大、平均值、95% 分位（P95）。

## 模块耗时（output3.log）
| 模块 | 样本数 | 最小 (ms) | 平均 (ms) | P95 (ms) | 最大 (ms) |
| --- | --- | --- | --- | --- | --- |
| ScanRegistration(Horizon) | 2 224 | 6.02 | 16.43 | 27.99 | 95.75 |
| Estimator::Estimate | 1 366 | 32.92 | **138.56** | 225.68 | 343.19 |
| Estimator::EstimateLidarPose | 1 362 | 1.05 | **156.01** | 245.88 | 369.74 |
| PoseEstimation（整帧） | 1 366 | 7.00 | **163.43** | 253.33 | 375.91 |

> 结论：后端估计与地图维护占据绝大部分时间，平均帧耗时 163 ms，远高于 10 Hz 所需的 <100 ms。

## 子阶段耗时（相同日志）
| 环节 | 样本数 | 最小 (ms) | 平均 (ms) | P95 (ms) | 最大 (ms) | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| fetchImuMsgs | 1 368 | 0.003 | 0.50 | 0.022 | 90.75 | 绝大多数帧几乎不等待，偶尔有 90 ms 长尾 |
| IMUIntegration | 1 365 | 0.009 | 0.19 | 0.31 | 3.16 | 不是瓶颈 |
| RemoveLidarDistortion | 1 365 | 2.26 | 5.12 | 7.76 | 24.09 | 固定 O(n) 计算 |
| Estimator::EstimateLidarPose TOTAL | 1 367 | 1.07 | **156.26** | 246.72 | 369.76 | 真正的性能瓶颈 |
| PublishMappedCloud | 1 365 | 0.32 | 0.76 | 1.58 | 8.54 | 可忽略 |
| FrameTotal（PoseEstimation） | 1 366 | 7.00 | **163.43** | 253.33 | 375.91 | 仅 ~22% 的帧能在 100 ms 内完成 |

## 问题定位
1. **后端优化占用 ≥140 ms**：`Estimator::Estimate` + `EstimateLidarPose` 是 10 Hz 无法达标的根本原因，地图特征已达 6.6k(角) / 13.8k(面)；KD 树拷贝、Ceres 求解、MapIncrementLocal 的串行逻辑导致 P95 逼近 250 ms。
2. **IMU 等待不是主矛盾**：大部分帧 `fetchImuMsgs` < 1 ms，仅极少数会拉长；当前重点应放在地图/优化链路。
3. **前端尚可**：`ScanRegistration` 平均 16 ms，处于可接受范围，但后续仍可通过排序优化、OpenMP 等方式留更多余量。

## 优化目标
- **短期**：在现有（未启用 ARM 指令优化）的代码上，通过算法/并发改造，把 `PoseEstimation` 平均耗时压到 <100 ms，P95 控制在 120 ms 内，保证 10 Hz 实时性。
- **长期**：完成 ARM 指令优化、KD 树并行化、MapManager 重构后，再次采集同场景日志，更新本文件并验证是否满足 <100 ms 目标。

## 下一步计划
- **Map 管线**：对 `MapIncrement` / `MapIncrementLocal` 引入任务分片或线程池，减少 4 851 cube 的全量拷贝；同时增加局部体素筛选，限制角点/面点进入 Ceres 的数量。
- **Ceres 优化**：在 `Estimator::Estimate` 内部细分 `vector2double`、特征构建、`ceres::Solve` 耗时，尝试减少迭代次数（例如 5→3），并复用线程池避免频繁创建 `std::thread`。
- **持续观测**：所有新改动都以 `output3.log` 为基线，再采集同场景日志，补充表格并记录是否达到了 “单帧 < 100 ms” 目标。

> 仅当新的日志涌现、并达到/接近 10 Hz 指标时，再考虑将 ARM 指令优化与算法级优化结合，以进一步压缩耗时。