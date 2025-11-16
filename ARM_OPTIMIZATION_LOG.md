# ARM 优化日志

## 可持续优化方向
- **编译与指令集**：继续评估 `-mcpu`、`-mtune`、LTO、OpenMP、链接器（ld.lld）等组合，必要时为特定模块拆分优化级别，并确保第三方库（Eigen/Ceres/PCL/OpenCV）同样开启 NEON/vectorize。
- **算法复杂度**：替换 O(n²) 冒泡排序、低效 KD-tree 访问等瓶颈为 `std::sort`、BVP/voxel 索引或分层搜索，并利用 SIMD/batched 运算降低矩阵求解成本。
- **内存与并行**：对地图维护、点云拷贝、滑窗缓存等环节引入按需拷贝、OpenMP/TBB 并行、NUMA 友好内存布局，减少带宽和锁竞争。
- **调度与系统级**：针对 RK3588 大小核，通过 `taskset`/`cset` 固定重负载线程，结合 `cpufreq performance`、zram/swap 及 I/O 管理提高稳定性。
- **验证与基准**：每次优化后用 `perf`, `htop`, `rostopic hz`, `tegrastats` 等工具记录性能指标，必要时添加微基准代码确认 NEON / cache 层表现。

## 2025-11-16
- **背景**：针对 Orangepi5 Max (RK3588, Ubuntu 20.04, ROS Noetic) 出现的 “Eigen 未启用 NEON 导致矩阵计算退化为标量” 问题，优先落实指令集级别优化。
- **改动**：
  - 在 `CMakeLists.txt` 中检测 `aarch64/arm64`，统一追加 `-mcpu=cortex-a76 -mtune=cortex-a76.cortex-a55 -O3 -ffast-math -ftree-vectorize -funroll-loops -falign-loops=32 -pipe -fopenmp -flto` 等编译/链接标志。
  - 为 Eigen 增加 `-DEIGEN_NO_DEBUG -DEIGEN_STRONG_INLINE=inline -DEIGEN_UNROLLING_LIMIT=256`，确保在 ARM 平台上强制 NEON 内联与展开。
- **验证**：
  - `catkin_make -DCMAKE_BUILD_TYPE=Release` 成功，生成 `devel/lib/lio_livox/PoseEstimation`。
  - `objdump -d devel/lib/lio_livox/PoseEstimation | grep -n "fmla"`，确认二进制包含 AArch64 NEON SIMD 指令（如 `fmla v1.2d, v21.2d, v21.2d`）。
  - `readelf -p .comment devel/lib/lio_livox/PoseEstimation` 显示 GCC 9.4.0，验证编译来自更新后的构建。
- **下一步**：在此日志中持续记录后续排序、KD 树、内存等更深入的 ARM 优化，建议每次增加条目时注明日期、变更范围、验证方法及对性能的观测。


