# LIO-Livox 模块运行时记录（目标：单帧 < 100 ms）

## 环境与数据
- **硬件/系统**：Orangepi 5 Max（RK3588，Ubuntu 20.04，ROS Noetic），Livox Horizon，`/livox/lidar` 频率固定 10 Hz（帧间隔 100 ms）。
- **算法版本**：
  - 原始版本：未启用 ARM 指令优化，仅在关键环节加入 `ROS_INFO`。
  - NEON 初步优化版本：`CMakeLists.txt` 启用 `-mcpu=cortex-a76 -mtune=cortex-a76.cortex-a55 -O3 -ffast-math -ftree-vectorize -flto -fopenmp` 等标志后重新编译。
- **测试场景**：室外开阔区域，真实激光雷达实时输入；`window=2`。
- **日志**：
  - `output3.log`：原始版本，1367 帧。
  - `NEON初步优化代码，小环节耗时.log`：NEON 优化版本，738 帧。
- **统计方式**：`python3` 解析日志，计算样本数、最小/最大、平均值、P95。

## 模块耗时对比

### 原始版本（output3.log）
| 模块 | 样本数 | 最小 (ms) | 平均 (ms) | P95 (ms) | 最大 (ms) |
| --- | --- | --- | --- | --- | --- |
| ScanRegistration(Horizon) | 2 224 | 6.02 | 16.43 | 27.99 | 95.75 |
| Estimator::Estimate | 1 366 | 32.92 | **138.56** | 225.68 | 343.19 |
| Estimator::EstimateLidarPose | 1 362 | 1.05 | **156.01** | 245.88 | 369.74 |
| PoseEstimation（整帧） | 1 366 | 7.00 | **163.43** | 253.33 | 375.91 |

> 结论：后端估计与地图维护占据绝大部分时间，平均帧耗时 163 ms，远高于 10 Hz 所需的 <100 ms。

### NEON 初步优化版本
| 模块/环节 | 样本数 | 最小 (ms) | 平均 (ms) | P95 (ms) | 最大 (ms) |
| --- | --- | --- | --- | --- | --- |
| PoseEstimation（整帧） | 738 | 7.03 | **134.54** | 186.89 | 280.88 |
| Estimator::EstimateLidarPose::Estimate | 736 | 35.03 | **113.40** | 162.92 | 251.18 |
| Estimator::Estimate::buildFeatures | 1 188 | 0.88 | **14.69** | 30.20 | 54.63 |
| Estimator::Estimate::ceresSolve | 1 189 | 7.16 | **34.92** | 69.20 | 145.63 |
| Estimator::Estimate::iterTotal | 452 | 25.26 | **75.90** | 118.75 | 168.62 |

> NEON/LTO 将 `buildFeatures`、`ceresSolve` 均值降低 20–30%，整帧平均降至 135 ms，但仍未达到 <100 ms 的目标。

### NEON 深度优化版本（NEON深度优化.log）

| 模块/环节 | 样本数 | 最小 (ms) | 平均 (ms) | P95 (ms) | 最大 (ms) |
| --- | --- | --- | --- | --- | --- |
| PoseEstimation（整帧） | 1 340 | 8.04 | **122.78** | 191.18 | 249.83 |
| Estimator::EstimateLidarPose（TOTAL） | 1 341 | 1.16 | **115.03** | 184.34 | 244.44 |
| Estimator::Estimate | 1 340 | 26.37 | **101.78** | 165.46 | 224.56 |
| Estimator::Estimate::iterTotal | 1 020 | 10.93 | 61.37 | 106.21 | 144.81 |
| Estimator::Estimate::ceresSolve | 2 361 | 4.04 | 22.39 | 55.02 | 87.37 |
| Estimator::Estimate::buildFeatures | 2 360 | 0.98 | 14.23 | 33.82 | 58.30 |
| Estimator::EstimateLidarPose::MapIncrementLocal | 1 339 | 0.27 | 10.93 | 18.00 | 24.22 |
| MapIncrementLocal::downsample | 1 340 | 0.12 | 8.38 | 14.35 | 21.14 |
| ScanRegistration(Horizon) | 1 648 | 7.07 | 15.91 | 23.00 | 34.12 |

> 进一步启用 NEON/并行后端后，整帧平均降至 122 ms、P95 191 ms，`ceresSolve` 均值缩减至 22 ms，但 `iterTotal` 与 `MapIncrementLocal` 仍消耗 70+ ms，距离 <100 ms 目标仍差约 20%。

## 子阶段耗时（原始日志 output3.log）
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
4. **NEON 深度优化后瓶颈更集中**：`PoseEstimation` 平均 122 ms、P95 191 ms，其中 `Estimator::Estimate::iterTotal` 仍需 61 ms，`MapIncrementLocal` 单独就占 11 ms（`downsample` 8.4 ms），说明窗口尺寸与点云复制仍限制整体性能。

## 优化目标
- **短期**：在现有（未启用 ARM 指令优化）的代码上，通过算法/并发改造，把 `PoseEstimation` 平均耗时压到 <100 ms，P95 控制在 120 ms 内，保证 10 Hz 实时性。
- **长期**：完成 ARM 指令优化、KD 树并行化、MapManager 重构后，再次采集同场景日志，更新本文件并验证是否满足 <100 ms 目标。

## 下一步计划
- **Map 管线**：针对 `MapIncrementLocal` 平均 10.9 ms / P95 18 ms 的耗时，将关联、累积、降采样拆成独立 OpenMP 任务，并在 KD-Map 中增量维护体素栅格，避免每帧 8 ms 的重复 `downsample`；同时在 `Map_Manager` 中根据局部点数自适应裁剪 6×6×6 cube。
- **Ceres 优化**：`ceresSolve` 平均 22 ms、`iterTotal` 61 ms，说明求解本身已提速但迭代次数偏多；需要通过约束权重调节、残差筛选（按曲率/信息量限幅）和在 RK3588 上启用 Ceres 的多线程线性求解，力争把迭代次数稳定在 ≤3 次。
- **任务调度**：利用 6 核 CPU，将 `buildFeatures`、`vector2double/double2vector`、`MapIncrementLocal` 等阶段绑定不同核心，必要时引入 TBB/OMP 任务图，减少 OMP 与 ROS 回调抢占导致的 190 ms 长尾。
- **持续观测**：把 `NEON 深度优化.log` 纳入基线，与 `output3.log`、`NEON 初步优化` 一并跟踪；未来每完成一次结构性优化即追加统计，直到 “平均 <100 ms / P95 <120 ms” 达成。

> 仅当新的日志涌现、并达到/接近 10 Hz 指标时，再考虑将 ARM 指令优化与算法级优化结合，以进一步压缩耗时。