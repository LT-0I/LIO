# 版本演进说明（stage2/mapstruct 分支）

以下按时间顺序记录从原仓库代码到当前版本（提交 `1131e9b`）的每一次提交、主要改动点以及带来的实际作用，方便回溯优化决策。

## 1. `262c7c9` 原版
- **改动内容**：同步 Livox 官方开源仓库的基础代码，作为后续优化的起点。
- **作用**：提供可比对的基线数据与行为，便于衡量每一步性能与稳定性提升。

## 2. `42f3756` “jetson上能跑起来的代码（带NEON优化版）”
- **改动内容**：
  - CMake 切换到 C++14，并显式配置 Jetson 上的 Ceres/OpenCV 路径，保证依赖可被正确找到。
  - `LidarFeatureExtractor` 改为使用 `std::vector` 管理分割输入，增加 `try/catch` 捕获 `PCSeg::DoSeg` 的异常，同时在标签回写前做越界保护。
  - `segment.cpp` 在地面点统计异常时进行裁剪，并给 `DoSeg/EncodeFeatures/FreeSeg` 补齐返回值，防止 `std::bad_array_new_length`。
  - `Estimator` 在平面拟合中增加 `isfinite` 判定，同时在输入点云不足时跳过优化。
  - `PoseEstimation` 将所有坐标系统一为 `world`，消除 TF transform 报错。
- **作用**：解决 Jetson 平台上的编译/运行阻塞，根治 `std::bad_array_new_length` 和 KD-Tree 空输入崩溃，为 NEON 平台提供稳定可跑的版本。

## 3. `90c9c25` “每个大模块时间”
- **改动内容**：给 ScanRegistration、LidarFeatureExtractor、PoseEstimation、Estimator 四个主模块增加粗粒度 `ROS_INFO` 打印，并附带第一份整机运行日志。
- **作用**：获得端到端的阶段耗时，定位 100 ms 处理瓶颈位于 Estimator。

## 4. `cbc9fd9` “Estimator optimization模块内部环节耗时”
- **改动内容**：删除之前的整机日志，转而在 `Estimator::Estimate` 内部细化到“数据拉取/残差构建/Ceres 求解/边缘化”等子阶段的起止打印，其它模块恢复安静。
- **作用**：明确 Estimator 内部各子流程的占比，为后续针对性优化（线程、残差数量）提供基准线。

## 5. `c8af6ff` “Refactor Estimator::Estimate…”
- **改动内容**：
  - 将残差构建的线程数量改为根据 `std::thread::hardware_concurrency()` 自适应，避免硬编码 3 线程。
  - 清理冗余的 `std::vector` 预分配与同步逻辑，统一线程收尾流程。
- **作用**：减少线程调度开销并避免在核数不足/充足时出现资源浪费，Estimator 耗时下降 ~10–15 ms。

## 6. `b1282b3` “Refactor Estimator data structures…”
- **改动内容**：
  - 将 `CornerKdMap/SurfKdMap/...` 等地图结构改为指针形式，`Estimator` 通过 `MapManager` 提供的 getter 取地址。
  - 相应调整头文件与实现中的访问方式，隔离 `MapManager` 内部存储布局。
- **作用**：为后续的零拷贝快照和双缓冲打基础，减少 map 数据重复拷贝。

## 7. `a3b1c12` “Refactor Estimator and MapManager…”
- **改动内容**：
  - `MapManager` 引入双缓冲 (`kMatchBufferCount=2`) 与 `MapSnapshot` 结构，增加 `publish/staging` 索引和 `snapshot_ref_count`，用 `std::shared_ptr` 自定义 deleter 管理生命周期。
  - `Estimator` 获取地图时调用 `AcquireSnapshot()`，只持有 `const` 指针，避免与地图更新线程抢锁。
- **作用**：消除地图读写互锁和数据竞争，实现零拷贝快照，稳定了高频更新情况下的性能（map fetch 时间显著下降且无数据错乱）。

## 8. `c9b1756` “cere数量降低”
- **改动内容**：
  - 在残差构建结束后按误差排序，仅保留 `Corner 500 / Surf 750 / Non 350` 个有效特征，并在 Ceres 中跳过其余 residual。
  - 调整 Ceres Solver 参数（最大迭代 8、tolerance 1e-4、非单调步长等），同时将求解线程数与硬件线程数对齐。
- **作用**：显著减少每帧参与优化的约束数量，在不牺牲精度的情况下把 Ceres solve 用时压到 ~60 ms 左右。

## 9. `a125519` “Stage2MapStruct”
- **改动内容**：
  - 在局部地图维护中引入不对称的前向窗口：记录 `localFrameStamp`，仅保留最近 20 帧，并按机体坐标的前/后/侧/上下边界过滤点。
  - `MapIncrementLocal` 只将符合有向包围盒的点推入匹配用局部地图。
- **作用**：紧凑化局部地图（重点保留前向点云），同时避免重复点和远离航线的数据，使 k-d tree 查询更快、缓存命中更高——这是 Stage 2 性能优化的核心。

## 10. `1131e9b` “去打印运行时间版”
- **改动内容**：删除之前为性能压测临时加入的 `ROS_INFO` 打印，保留 `ROS_WARN` 等必要日志。
- **作用**：在确认优化达标后恢复干净的运行输出，避免额外的日志 IO 开销，也方便在飞行中监控真正的异常信息。

## 11. `943b8fe` “MapManagerConfig 参数化 + 内存结构瘦身”
- **改动内容**：
  - 在 `config/*.yaml` 与 `PoseEstimation` 中新增 `map_width/height/depth/local_window` 参数，并通过 `MapManagerConfig` 传入 `Estimator/MapManager` 构造函数，使不同隧道场景可以快速调节地图栅格尺寸。
  - `MapManager` 内的 `laserCloud*Array`、KD-tree 与匹配缓存改为 `std::vector` 动态分配，双缓冲尺寸跟随配置而定；同时引入 `snapshot_ref_count` 自旋锁、条件变量，减少固定 4851 个格子的静态内存占用。
  - `Estimator` 构造函数同步调整，初始化局部地图窗口与 KD-Tree 结构时使用配置尺寸，并在 `MapIncrementLocal` 里复用这些 `std::vector` 容器，避免大块静态数组常驻。
- **作用**：可通过配置文件裁剪地图体素数量，把原先固定网格改为“任务驱动”的动态大小，同时借助 `std::vector`/shared snapshot 降低常驻内存，并使得 `MapManager` 的更新、快照流程在不同栅格尺寸下都能稳定运行。

## 12. `3fee899` “残差筛选参数可配置化”
- **改动内容**：
  - 新增 `EstimatorResidualConfig` 结构，并在 `Estimator` 中实现按残差误差从大到小排序的统一筛选函数，将保留数量（corner/surf/non）与误差阈值作为配置输入，而非写死在代码里。
  - 在 `PoseEstimation` 中从 YAML 读取 `max_corner_residuals`、`max_surf_residuals`、`max_non_residuals`、`feature_error_threshold`，构造 `Estimator` 时传入；三套传感器配置文件均补充了默认值。
  - 残差筛选阶段与 Ceres 求解新增统计日志，便于调参时观察不同配置下的保留数量与收敛情况。
- **作用**：让不同任务/隧道宽度下可以通过改 YAML 即调节残差密度，快速在精度与性能间取舍，同时统一筛选逻辑提升代码可维护性。

## 13. `5293f2f` “Corner 残差回退到均匀抽样 + 构建统计 + 自适应”
- **改动内容**：
  - 将 `selectAndAddResiduals` 回退为旧版均匀抽样逻辑（按 stride 取样），但保留 `EstimatorResidualConfig` 提供的角/面/非特征上限与误差阈值；角点残差在 `FeatureLine` 中记录 `from_global` 标记。
  - 内部仍保留 `FeatureBuildStats` 用于自适应判定，但自 `0f4fdff` 起所有角点/残差统计日志已从命令行移除，仅保留 `Frame` 输出（避免现场调试噪声）。
- **自适应逻辑**：
  - 支持 `corner_adaptive_*` 参数：根据当前窗口的全局 KD 成功数自动切换模式。角点稀少（如隧道）时放宽 eigen 判定并抬高保底数量；角点极多（如室外停车场）时收紧配额以避免估计时延。
  - 动态修改 `corner_eigen_ratio_` 与单帧角点限额，切换信息会在日志中输出 `Estimator corner adaptive iter ...`，便于 post-mortem 分析。
- **验证结论**：
  - **隧道退化场景**（s23 bag）：78% 帧落在 `low_feature` 模式，全局角点通过率 ~66%，开头几帧虽完全依赖局部角点但轨迹不再飞；均匀抽样 + 自适应保证最少 200 个角点进入 Ceres。
  - **金蝶停车场**（diejing bag）：大部分时间处于 `balanced`，在角点暴增时自动切换到 `high_feature`（限额=400），Ceres 时延显著降低但轨迹仍稳定；全局角点通过率 ~60%。
  - **室内球场/常规场景**：仍保持 `balanced` 模式，角点筛选与改动前一致，证明自适应开关不会干扰中等场景。
  - **旧版误差排序的风险**：“开头飞、跑一会儿才稳” 的现象只在误差排序方案中出现（隧道角点误差太小被全部剔除）；回退均匀抽样后，这种隐患消失。
- **作用**：恢复隧道场景的稳定性，同时通过日志量化角点来源与 KD-tree 判定情况，为后续制定“自适应角点保底/阈值放宽”等策略提供依据；也确保其他场景在不调参的情况下继续稳定运行。

## 14. `a791508`统计输出可开关”
- **改动内容**：
  - 在 `EstimatorResidualConfig` 与 `PoseEstimation` 中新增 `log_feature_counts` 参数，可通过 YAML/rosparam 控制是否构建角点/残差统计并在终端输出。
  - 当开关关闭时跳过 `FeatureBuildStats` 的分配与汇总，仅保留 Frame 计数；打开时会恢复 `Estimator corner adaptive`、`build stats` 与残差候选/保留数量的日志。
  - 新增 `log_module_timing` 参数，控制是否输出 RemoveDistortion、IMU 预积分、MapManager 快照/更新、残差构建、Ceres 求解、边缘化等关键阶段的起止时间，方便定位耗时。
  - `horizon_params.yaml` 补充上述两个配置，默认 `false` 以保持安静输出，需要调试时再启用。
- **作用**：让现场运行保持最小日志与计算开销，同时在需要排查角点不足、KD-tree 命中率等问题时可以“即开即用”，避免每次都改代码重新编译。

---

> 如需查看某个提交的具体 diff，可直接在仓库中运行 `git show <commit>`。本文件仅概述改动动机与收益，以便团队成员快速了解版本演进。

---

## 附：`1131e9b` 相比 `42f3756` 的逐文件差异

`git diff --name-status 42f3756 1131e9b` 显示共有 5 个文件被修改，具体如下：

- `include/Estimator/Estimator.h`
  - 将 `CornerKdMap/SurfKdMap/NonFeatureKdMap` 与 `Global*Map` 改为指向 `pcl::PointCloud` / `pcl::KdTreeFLANN` 的 `const` 指针，配合 MapSnapshot 做零拷贝访问。
  - 为局部地图新增 `localFrameId/localFrameStamp` 以及不对称包围盒的尺寸参数（forward/backward/side/vertical）和历史帧窗口常量，用于 Stage2 的方向性局部地图。

- `include/MapManager/Map_Manager.h`
  - 引入 `<condition_variable>`, `<memory>`, `<array>`, `<atomic>`，新增 `MapSnapshot` 结构及 `AcquireSnapshot()` 接口。
  - 将 `laserCloud*_for_match`、`CornerKdMap_last` 等数据改为双缓冲（`kMatchBufferCount=2`），并增加 `publish_idx/staging_idx`、`snapshot_ref_count`、`snapshot_cv`。

- `src/lio/Estimator.cpp`
  - 调整地图访问逻辑，使用 `map_manager->AcquireSnapshot()` 获取 `const` 指针，解决地图并发读写问题。
  - 在 `MapIncrementLocal` 中按照机体系前/后/侧/上下尺寸过滤局部地图点，并仅保留最近 `localMapHistoryFrames` 帧，提高 k-d tree 查询效率。
  - 引入残差裁剪与 Ceres 参数收紧（保留最多 500/750/350 个 corner/surf/non 约束、最大迭代 8，非单调步长开启），减少单帧求解时间。
  - 清理阶段耗时调试日志，仅保留必要警告。

- `src/lio/Map_Manager.cpp`
  - 在 `MapIncrement` 中按 staging buffer 写入地图，并在 copy 完成后切换 `publish_idx`；引用计数确保 Estimator 使用完快照后再复用缓冲区。
  - 新增 `AcquireSnapshot()` / `ReleaseSnapshot()`，以 `shared_ptr` + 自定义 deleter 的方式提供线程安全的地图快照。

- `src/lio/PoseEstimation.cpp`
  - 移除 MAP 初始化阶段的 `std::cout` 日志以及冗余注释，使 PoseEstimation 保持与新的日志策略一致。

除此之外，其它文件在两个提交间保持一致。

