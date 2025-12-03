# LIO-Livox Benchmark Instrumentation Guide (Extended)

## 概述

本文档详细记录了为LIO-Livox添加的"Golden Benchmark"插桩代码，用于在x86 NUC上建立性能基准，并确保后续在OrangePi嵌入式平台上的代码一致性。

**版本**: Extended v2.0  
**创建日期**: 2025-12-03  
**适用于**: LIO-Livox (Horizon配置)  
**平台**: x86 NUC (Golden Benchmark)

---

## 修改的文件列表

| 文件路径 | 修改类型 | 描述 |
|---------|---------|------|
| `config/horizon_config.yaml` | 修改 | 添加 `enable_debug_log` 配置开关 |
| `include/utils/BenchmarkLogger.h` | **新增** | Benchmark日志工具类（扩展版） |
| `include/Estimator/Estimator.h` | 修改 | 添加优化指标接口 |
| `src/lio/Estimator.cpp` | 修改 | 收集全面的优化过程指标 |
| `src/lio/PoseEstimation.cpp` | 修改 | 帧处理时间测量和日志输出 |
| `launch/horizon.launch` | 修改 | 添加benchmark启动参数 |

---

## 算法流程深度分析

### 完整帧处理流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                    LIO-Livox Frame Processing Pipeline               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────┐     ┌──────────────────┐                      │
│  │   ScanRegistration│     │  PoseEstimation  │                      │
│  │      Node         │────▶│      Node        │                      │
│  └──────────────────┘     └──────────────────┘                      │
│         │                          │                                 │
│  [特征提取]                  [IMU预积分]                             │
│  - detectFeaturePoint()     - GyroIntegration()                     │
│  - detectFeaturePoint2()    - PreIntegration()                      │
│         │                          │                                 │
│         └──────────┬───────────────┘                                │
│                    ▼                                                 │
│           ┌─────────────────┐                                       │
│           │ RemoveLidarDistortion │                                  │
│           │    [去畸变]          │                                   │
│           └─────────────────┘                                       │
│                    │                                                 │
│                    ▼                                                 │
│           ┌─────────────────┐                                       │
│           │ EstimateLidarPose   │                                    │
│           │  [位姿估计主函数]    │                                   │
│           └─────────────────┘                                       │
│                    │                                                 │
│      ┌─────────────┼─────────────┐                                  │
│      ▼             ▼             ▼                                  │
│ [降采样]    [KD-tree构建]   [地图获取]                               │
│      │             │             │                                  │
│      └─────────────┴─────────────┘                                  │
│                    │                                                 │
│                    ▼                                                 │
│           ┌─────────────────┐                                       │
│           │    Estimate()       │  ◀── 核心优化                     │
│           │ [IESKF优化循环]     │                                   │
│           └─────────────────┘                                       │
│                    │                                                 │
│     ┌──────────────┼──────────────┐                                 │
│     ▼              ▼              ▼                                 │
│ [点到线匹配] [点到面匹配]  [非特征ICP]                               │
│ processPointToLine  processPointToPlanVec  processNonFeatureICP     │
│     │              │              │                                 │
│     └──────────────┴──────────────┘                                 │
│                    │                                                 │
│                    ▼                                                 │
│           ┌─────────────────┐                                       │
│           │  Ceres Solver      │                                    │
│           │  [非线性优化]      │                                    │
│           └─────────────────┘                                       │
│                    │                                                 │
│                    ▼                                                 │
│           ┌─────────────────┐                                       │
│           │   pubOdometry()    │                                    │
│           │  [发布里程计]      │                                    │
│           └─────────────────┘                                       │
│                    │                                                 │
│                    ▼                                                 │
│           ┌─────────────────┐                                       │
│           │ MapIncrementLocal  │                                    │
│           │   [地图更新]       │                                    │
│           └─────────────────┘                                       │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 影响处理速度的关键因素

### 1. 点云数据规模

| 变量名 | 位置 | 说明 |
|-------|------|------|
| `raw_cloud_size` | PoseEstimation.cpp | 原始点云点数 |
| `downsampled_corner_size` | Estimator.cpp | 降采样后角点数 |
| `downsampled_surf_size` | Estimator.cpp | 降采样后平面点数 |
| `downsampled_nonfeature_size` | Estimator.cpp | 降采样后非特征点数 |

### 2. 地图规模

| 变量名 | 位置 | 说明 |
|-------|------|------|
| `map_corner_size` | Estimator.cpp | 全局角点地图大小 |
| `map_surf_size` | Estimator.cpp | 全局平面地图大小 |
| `local_corner_size` | Estimator.cpp | 局部角点地图大小 |
| `local_surf_size` | Estimator.cpp | 局部平面地图大小 |

### 3. 优化复杂度

| 变量名 | 位置 | 说明 |
|-------|------|------|
| `window_size` | Estimator.cpp | 滑动窗口大小 (1或2) |
| `iterations` | Estimator.cpp | 外层迭代次数 (max=5) |
| `ceres_iterations` | Estimator.cpp | Ceres内层迭代次数 (max=10) |
| `total_features` | Estimator.cpp | 参与优化的总特征数 |

### 4. 计时分解

| 阶段 | 变量名 | 测量范围 |
|-----|--------|---------|
| 预处理 | `preprocess_time_ms` | IMU获取 + 去畸变 |
| KD-tree构建 | `kdtree_build_time_ms` | 全局+局部地图KD-tree |
| 特征提取 | `feature_extract_time_ms` | 角点/平面分离 |
| 特征匹配 | `feature_match_time_ms` | 点到线/面匹配 |
| 优化 | `optimization_time_ms` | Ceres求解器执行 |
| 地图更新 | `map_update_time_ms` | MapIncrementLocal |
| 发布 | `publish_time_ms` | 点云转换和发布 |

---

## 影响里程计精度的关键因素

### 1. 有效特征数量 (effective_feat_num)

```cpp
// Estimator.cpp 中的特征统计
int cntCorner = 0;  // 角点特征数
int cntSurf = 0;    // 平面特征数  
int cntNon = 0;     // 非特征点数
total_features = cntCorner + cntSurf + cntNon;
```

**影响分析**：特征数量越多，约束越强，精度通常越高，但处理时间也会增加。

### 2. 优化收敛质量

| 指标 | 变量名 | 阈值 | 说明 |
|-----|--------|------|------|
| 初始残差 | `initial_cost` | - | 优化前的代价函数值 |
| 最终残差 | `final_cost` | - | 优化后的代价函数值 |
| 旋转变化 | `delta_rotation` | 0.05° | 迭代间旋转变化 |
| 平移变化 | `delta_translation` | 0.05m | 迭代间平移变化 |
| 提前收敛 | `converged_early` | - | 是否在max_iters前收敛 |

### 3. 特征匹配误差

| 指标 | 变量名 | 说明 |
|-----|--------|------|
| 平均角点误差 | `avg_corner_error` | 点到线距离的平均值 |
| 平均平面误差 | `avg_surf_error` | 点到面距离的平均值 |
| 平均非特征误差 | `avg_nonfeature_error` | 非特征ICP误差平均值 |

### 4. IMU质量参数

```cpp
// IMUIntegrator.h 中的噪声参数
const double acc_n = 0.08;    // 加速度计噪声
const double gyr_n = 0.004;   // 陀螺仪噪声
const double acc_w = 2.0e-4;  // 加速度计随机游走
const double gyr_w = 2.0e-5;  // 陀螺仪随机游走
```

### 5. IMU预积分

| 指标 | 变量名 | 说明 |
|-----|--------|------|
| IMU消息数 | `imu_msg_count` | 本帧使用的IMU消息数量 |
| 积分时间 | `imu_dt` | IMU积分时间跨度 (秒) |
| 初始化状态 | `imu_initialized` | IMU是否已初始化 |

---

## 详细代码修改

### 0. config/horizon_config.yaml - 配置开关

```yaml
%YAML:1.0

# switches
enable_debug_log: 1    # 0-disable, 1-enable benchmark logging
Lidar_Type: 0    # 0-horizon
Used_Line: 6    # lines used for lio, set to 1~6
# ... 其他配置 ...
```

**说明**: 在配置文件开头添加 `enable_debug_log` 开关，用于控制是否启用benchmark日志功能。

---

### 1. BenchmarkLogger.h - 扩展版

#### 1.1 新增头文件引用

```cpp
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
```

#### 1.2 新增私有成员变量

```cpp
private:
    bool enabled_ = false;
    std::ofstream csv_file_;
    std::ofstream tum_file_;
    std::mutex csv_mutex_;
    std::mutex tum_mutex_;
    bool csv_header_written_ = false;
    std::string session_timestamp_;    // 会话时间戳
    std::string output_folder_;        // 输出文件夹路径
```

#### 1.3 新增辅助函数

```cpp
/**
 * @brief Generate timestamp string for current time
 * Format: YYYYMMDD_HHMMSS
 */
static std::string generateTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    
    std::ostringstream oss;
    oss << std::put_time(tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

/**
 * @brief Create directory recursively
 * @param path Directory path to create
 * @return true if successful or already exists
 */
static bool createDirectory(const std::string& path) {
    int result = mkdir(path.c_str(), 0755);
    if (result == 0) {
        return true;  // Successfully created
    }
    struct stat st;
    if (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        return true;  // Already exists
    }
    return false;
}
```

#### 1.4 新增公共接口

```cpp
/**
 * @brief Get the session timestamp string
 */
const std::string& getSessionTimestamp() const { return session_timestamp_; }

/**
 * @brief Get the output folder path
 */
const std::string& getOutputFolder() const { return output_folder_; }
```

#### 1.5 修改后的init函数

```cpp
/**
 * @brief Initialize the logger with automatic timestamped folder and files
 * @param enable Whether logging is enabled
 * @param base_output_dir Base directory for logs (e.g., "/path/to/logs")
 * 
 * Creates folder structure: base_output_dir/YYYYMMDD_HHMMSS/
 * Files created:
 *   - internal_stats_YYYYMMDD_HHMMSS.csv
 *   - benchmark_traj_YYYYMMDD_HHMMSS.txt
 */
void init(bool enable, const std::string& base_output_dir = ".") {
    enabled_ = enable;
    if (!enabled_) return;
    
    // Generate timestamp for this session
    session_timestamp_ = generateTimestamp();
    
    // Create base logs directory if it doesn't exist
    createDirectory(base_output_dir);
    
    // Create timestamped subfolder
    output_folder_ = base_output_dir + "/" + session_timestamp_;
    if (!createDirectory(output_folder_)) {
        output_folder_ = base_output_dir;  // Fallback
    }
    
    // Generate file paths with timestamp
    std::string csv_path = output_folder_ + "/internal_stats_" + session_timestamp_ + ".csv";
    std::string tum_path = output_folder_ + "/benchmark_traj_" + session_timestamp_ + ".txt";
    
    // Open CSV file
    csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
        csv_file_ << std::fixed << std::setprecision(6);
    }
    
    // Open TUM trajectory file
    tum_file_.open(tum_path, std::ios::out | std::ios::trunc);
    if (tum_file_.is_open()) {
        tum_file_ << std::fixed << std::setprecision(6);
        tum_file_ << "# timestamp x y z q_x q_y q_z q_w\n";
        tum_file_ << "# Session started: " << session_timestamp_ << "\n";
    }
}
```

#### 1.6 指标结构体

```cpp
struct OptimizationMetrics {
    // ============== Feature Count Metrics (Accuracy Impact) ==============
    int corner_features = 0;           // Corner features used in optimization
    int surf_features = 0;             // Surface features used in optimization
    int nonfeature_points = 0;         // Non-feature points used
    int total_features = 0;            // Total effective features (effective_feat_num)
    
    // Feature matching statistics
    int corner_from_map = 0;           // Corner features matched from global map
    int corner_from_local = 0;         // Corner features matched from local map
    int surf_from_map = 0;             // Surf features matched from global map
    int surf_from_local = 0;           // Surf features matched from local map
    
    // ============== Optimization Metrics (Accuracy Impact) ==============
    int iterations = 0;                // Outer optimization iterations (max 5)
    int ceres_iterations = 0;          // Inner Ceres solver iterations (max 10)
    double initial_cost = 0.0;         // Initial optimization cost
    double final_cost = 0.0;           // Final optimization cost (residual)
    double delta_rotation = 0.0;       // Rotation change after optimization (degrees)
    double delta_translation = 0.0;    // Translation change after optimization (meters)
    
    // Convergence quality
    bool converged_early = false;      // Whether optimization converged before max iterations
    
    // ============== Point Cloud Metrics (Speed Impact) ==============
    int raw_cloud_size = 0;            // Original point cloud size
    int downsampled_corner_size = 0;   // Downsampled corner cloud size
    int downsampled_surf_size = 0;     // Downsampled surface cloud size
    int downsampled_nonfeature_size = 0; // Downsampled non-feature size
    
    // ============== Map Metrics (Speed & Accuracy Impact) ==============
    int map_corner_size = 0;           // Global corner map size
    int map_surf_size = 0;             // Global surface map size
    int local_corner_size = 0;         // Local corner map size
    int local_surf_size = 0;           // Local surface map size
    int window_size = 0;               // Current sliding window size
    
    // ============== IMU Metrics (Accuracy Impact) ==============
    int imu_msg_count = 0;             // Number of IMU messages in this frame
    double imu_dt = 0.0;               // IMU integration time span (seconds)
    bool imu_initialized = false;      // Whether IMU is initialized
    
    // ============== Timing Metrics (Speed Analysis) ==============
    double preprocess_time_ms = 0.0;        // IMU fetch + distortion removal
    double kdtree_build_time_ms = 0.0;      // KD-tree construction time
    double feature_extract_time_ms = 0.0;   // Feature extraction time
    double feature_match_time_ms = 0.0;     // Point-to-line/plane matching time
    double optimization_time_ms = 0.0;      // Ceres solver execution time
    double map_update_time_ms = 0.0;        // Map increment time
    double publish_time_ms = 0.0;           // Point cloud publishing time
    
    // ============== Average Feature Errors (Accuracy Indicators) ==============
    double avg_corner_error = 0.0;     // Average point-to-line error
    double avg_surf_error = 0.0;       // Average point-to-plane error
    double avg_nonfeature_error = 0.0; // Average non-feature ICP error
};
```

### 2. Estimator.cpp - 指标收集代码

#### 2.1 在Estimate()函数开头

```cpp
// Reset optimization metrics for this frame
optimization_metrics_ = BenchmarkLogger::OptimizationMetrics();

// Start timing for KD-tree building
auto kdtree_start = std::chrono::high_resolution_clock::now();

// BENCHMARK: Record window size
optimization_metrics_.window_size = windowSize;
```

#### 2.2 KD-tree构建后

```cpp
// Record KD-tree build time
auto kdtree_end = std::chrono::high_resolution_clock::now();
optimization_metrics_.kdtree_build_time_ms = 
    std::chrono::duration<double, std::milli>(kdtree_end - kdtree_start).count();

// BENCHMARK: Record map sizes
optimization_metrics_.map_corner_size = map_manager->get_corner_map()->points.size();
optimization_metrics_.map_surf_size = map_manager->get_surf_map()->points.size();
optimization_metrics_.local_corner_size = laserCloudCornerFromLocal->points.size();
optimization_metrics_.local_surf_size = laserCloudSurfFromLocal->points.size();
```

#### 2.3 优化循环中收集指标

```cpp
// Collect optimization metrics for benchmark logging
optimization_metrics_.corner_features = cntCorner;
optimization_metrics_.surf_features = cntSurf;
optimization_metrics_.nonfeature_points = cntNon;
optimization_metrics_.total_features = cntCorner + cntSurf + cntNon;
optimization_metrics_.iterations = iterOpt + 1;
optimization_metrics_.ceres_iterations = summary.iterations.size();
optimization_metrics_.initial_cost = summary.initial_cost;
optimization_metrics_.final_cost = summary.final_cost;
optimization_metrics_.delta_rotation = deltaR;
optimization_metrics_.delta_translation = deltaT;

// BENCHMARK: Calculate average feature errors
double total_corner_error = 0.0;
double total_surf_error = 0.0;
double total_non_error = 0.0;
int valid_corner_count = 0;
int valid_surf_count = 0;
int valid_non_count = 0;

for (int f = 0; f < windowSize; ++f) {
    for (const auto& feat : vLineFeatures[f]) {
        if (feat.valid) {
            total_corner_error += std::fabs(feat.error);
            valid_corner_count++;
        }
    }
    for (const auto& feat : vPlanFeatures[f]) {
        if (feat.valid) {
            total_surf_error += std::fabs(feat.error);
            valid_surf_count++;
        }
    }
    for (const auto& feat : vNonFeatures[f]) {
        if (feat.valid) {
            total_non_error += std::fabs(feat.error);
            valid_non_count++;
        }
    }
}

optimization_metrics_.avg_corner_error = valid_corner_count > 0 ? 
    total_corner_error / valid_corner_count : 0.0;
optimization_metrics_.avg_surf_error = valid_surf_count > 0 ? 
    total_surf_error / valid_surf_count : 0.0;
optimization_metrics_.avg_nonfeature_error = valid_non_count > 0 ? 
    total_non_error / valid_non_count : 0.0;
```

### 3. PoseEstimation.cpp - 修改内容

#### 3.1 新增头文件和全局变量

```cpp
#include "utils/BenchmarkLogger.h"
#include <chrono>

// Benchmark logging globals
bool g_enable_debug_log = false;
```

#### 3.2 main()函数中的Logger初始化

```cpp
// BENCHMARK: Get enable_debug_log parameter
ros::param::param<bool>("~enable_debug_log", g_enable_debug_log, false);

// Initialize benchmark logger if enabled
if (g_enable_debug_log) {
    std::string output_dir;
    ros::param::param<std::string>("~benchmark_output_dir", output_dir, ".");
    
    // Initialize logger with base directory
    // It will automatically create timestamped subfolder and files:
    //   logs/YYYYMMDD_HHMMSS/internal_stats_YYYYMMDD_HHMMSS.csv
    //   logs/YYYYMMDD_HHMMSS/benchmark_traj_YYYYMMDD_HHMMSS.txt
    g_benchmark_logger.init(true, output_dir);
    
    ROS_INFO("Benchmark logging enabled.");
    ROS_INFO("  Session: %s", g_benchmark_logger.getSessionTimestamp().c_str());
    ROS_INFO("  Output folder: %s", g_benchmark_logger.getOutputFolder().c_str());
}
```

#### 3.3 帧级指标收集（process()函数中）

```cpp
// BENCHMARK: Log internal statistics to CSV
if (g_enable_debug_log) {
    auto frame_end_time = std::chrono::high_resolution_clock::now();
    double total_frame_time_ms = std::chrono::duration<double, std::milli>(
        frame_end_time - frame_start_time).count();
    
    // Get optimization metrics from estimator
    BenchmarkLogger::OptimizationMetrics metrics = estimator->getOptimizationMetrics();
    metrics.preprocess_time_ms = preprocess_time_ms;
    
    // Add point cloud and IMU metrics
    metrics.raw_cloud_size = laserCloudFullRes->points.size();
    metrics.imu_msg_count = vimuMsg.size();
    metrics.imu_initialized = LidarIMUInited;
    
    // Calculate IMU time span
    if (!vimuMsg.empty() && vimuMsg.size() > 1) {
        metrics.imu_dt = vimuMsg.back()->header.stamp.toSec() - 
                         vimuMsg.front()->header.stamp.toSec();
    }
    
    // Log to CSV file using message timestamp (bag time)
    g_benchmark_logger.logInternalStats(
        lidar_list->front().timeStamp,  // Message header timestamp
        total_frame_time_ms,
        metrics
    );
}
```

#### 3.4 TUM轨迹日志（pubOdometry()函数中）

```cpp
// BENCHMARK: Log trajectory in TUM format
// Uses message header timestamp (bag time), NOT system wall time
if (g_enable_debug_log) {
    g_benchmark_logger.logTrajectoryTUM(
        timefullCloud,                    // Message header timestamp
        newPosition.x(),                  // x
        newPosition.y(),                  // y  
        newPosition.z(),                  // z
        newQuat.x(),                      // q_x
        newQuat.y(),                      // q_y
        newQuat.z(),                      // q_z
        newQuat.w()                       // q_w
    );
}
```

---

## 输出文件格式

### internal_stats.csv (扩展版)

| 列名 | 类型 | 说明 | 影响因素 |
|------|-----|------|---------|
| timestamp | double | 消息头时间戳 | - |
| **计时指标** | | | |
| total_frame_time_ms | double | 总帧处理时间 | 速度 |
| preprocess_time_ms | double | 预处理时间 | 速度 |
| kdtree_build_time_ms | double | KD-tree构建时间 | 速度 |
| feature_extract_time_ms | double | 特征提取时间 | 速度 |
| feature_match_time_ms | double | 特征匹配时间 | 速度 |
| optimization_time_ms | double | 优化时间 | 速度 |
| map_update_time_ms | double | 地图更新时间 | 速度 |
| publish_time_ms | double | 发布时间 | 速度 |
| **特征数量** | | | |
| corner_features | int | 角点特征数 | 精度 |
| surf_features | int | 平面特征数 | 精度 |
| nonfeature_points | int | 非特征点数 | 精度 |
| total_features | int | 总特征数(effective_feat_num) | 精度 |
| corner_from_map | int | 来自全局地图的角点 | 精度 |
| corner_from_local | int | 来自局部地图的角点 | 精度 |
| surf_from_map | int | 来自全局地图的平面点 | 精度 |
| surf_from_local | int | 来自局部地图的平面点 | 精度 |
| **点云规模** | | | |
| raw_cloud_size | int | 原始点云大小 | 速度 |
| downsampled_corner | int | 降采样角点数 | 速度 |
| downsampled_surf | int | 降采样平面点数 | 速度 |
| downsampled_nonfeature | int | 降采样非特征点数 | 速度 |
| **地图规模** | | | |
| map_corner_size | int | 全局角点地图大小 | 速度/精度 |
| map_surf_size | int | 全局平面地图大小 | 速度/精度 |
| local_corner_size | int | 局部角点地图大小 | 速度/精度 |
| local_surf_size | int | 局部平面地图大小 | 速度/精度 |
| window_size | int | 滑动窗口大小 | 速度/精度 |
| **IMU指标** | | | |
| imu_msg_count | int | IMU消息数量 | 精度 |
| imu_dt_sec | double | IMU积分时间 | 精度 |
| imu_initialized | int | IMU初始化状态 | 精度 |
| **优化指标** | | | |
| outer_iterations | int | 外层迭代次数(iterations) | 精度 |
| ceres_iterations | int | Ceres迭代次数 | 精度 |
| initial_cost | double | 初始代价 | 精度 |
| final_cost | double | 最终代价(residual) | 精度 |
| delta_rotation_deg | double | 旋转变化(度) | 精度 |
| delta_translation_m | double | 平移变化(米) | 精度 |
| converged_early | int | 是否提前收敛 | 精度 |
| **特征误差** | | | |
| avg_corner_error | double | 平均角点误差 | 精度 |
| avg_surf_error | double | 平均平面误差 | 精度 |
| avg_nonfeature_error | double | 平均非特征误差 | 精度 |

### benchmark_traj.txt (TUM格式)

```
# timestamp x y z q_x q_y q_z q_w
1234567890.123456 1.000000 2.000000 3.000000 0.000000 0.000000 0.000000 1.000000
```

---

## Ceres优化代价函数分析

### 1. 点到线代价 (Cost_NavState_IMU_Line)

```cpp
// ceresfunc.h
// 计算点到线的距离误差
T a012 = sqrt(...);  // 三角形面积的2倍
T ld2 = a012 / T(l12);  // 点到线距离
T _weight = T(1) - T(0.9) * abs(ld2) / sqrt(sqrt(...));  // 距离权重
residual[0] = T(sqrt_information(0)) * _weight * ld2;
```

### 2. 点到面代价 (Cost_NavState_IMU_Plan)

```cpp
// ceresfunc.h
// 计算点到平面的距离误差
T pd2 = T(pa) * P_to_Map(0) + T(pb) * P_to_Map(1) + T(pc) * P_to_Map(2) + T(pd);
T _weight = T(1) - T(0.9) * abs(pd2) / sqrt(sqrt(...));
residual[0] = T(sqrt_information(0)) * _weight * pd2;
```

### 3. IMU预积分代价 (Cost_NavState_PRV_Bias)

```cpp
// ceresfunc.h
// 15维残差：位置(3) + 旋转(3) + 速度(3) + 偏置变化(6)
eResiduals.template segment<3>(0) = rPij;    // 位置残差
eResiduals.template segment<3>(3) = rPhiij;  // 旋转残差
eResiduals.template segment<3>(6) = rVij;    // 速度残差
eResiduals.template segment<6>(9) = bias_diff; // 偏置残差
```

---

## 使用方法

### 编译

```bash
cd ~/ws_livox
catkin_make
source devel/setup.bash
```

### 运行（启用benchmark）

```bash
roslaunch lio_livox horizon.launch enable_debug_log:=true
```

### 运行（禁用benchmark）

```bash
roslaunch lio_livox horizon.launch enable_debug_log:=false
```

### 日志输出结构

每次运行会自动创建带时间戳的文件夹和文件：

```
$(find lio_livox)/logs/
└── 20251203_143025/                           # 运行时间戳文件夹
    ├── internal_stats_20251203_143025.csv     # 内部统计CSV
    └── benchmark_traj_20251203_143025.txt     # TUM轨迹文件
```

**时间戳格式**: `YYYYMMDD_HHMMSS`（例如：20251203_143025 = 2025年12月3日 14:30:25）

---

## 数据分析建议

### 速度分析

1. **时间分布饼图**: 分析各阶段时间占比
2. **特征数量与时间相关性**: 绘制scatter plot
3. **地图大小增长曲线**: 追踪地图膨胀

### 精度分析

1. **特征误差分布**: 绘制histogram
2. **残差收敛曲线**: 追踪initial_cost → final_cost
3. **迭代次数分布**: 统计converged_early比例
4. **EVO轨迹评估**: 使用benchmark_traj.txt

### 推荐工具

```bash
# 使用evo工具评估轨迹
evo_ape tum ground_truth.txt benchmark_traj.txt -p -a

# 使用Python分析CSV
python3 analyze_stats.py internal_stats.csv
```

---

## OrangePi移植指南

### 需要复制的文件

1. `include/utils/BenchmarkLogger.h`（整个文件）
2. 本文档中列出的所有代码修改

### 验证一致性

1. **轨迹精度比较**: 使用相同的rosbag，比较两个平台的轨迹误差
2. **时间分布分析**: 比较各阶段时间占比差异
3. **特征匹配质量**: 比较平均特征误差

---

## 关键配置参数参考

### 体素滤波器参数

```cpp
// Estimator.cpp
downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);  // 角点降采样
downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);    // 平面点降采样
downSizeFilterNonFeature.setLeafSize(0.4, 0.4, 0.4); // 非特征降采样
```

### 优化参数

```cpp
// Estimator.cpp
const int max_iters = 5;  // 外层最大迭代
options.max_num_iterations = 10;  // Ceres最大迭代
options.num_threads = 6;  // 并行线程数
```

### 收敛阈值

```cpp
// Estimator.cpp
if (deltaR < 0.05 && deltaT < 0.05)  // 收敛判断
```

---

## Launch文件修改

### horizon.launch

```xml
<launch>
    <!-- Benchmark logging parameters -->
    <arg name="enable_debug_log" default="true" />
    <arg name="benchmark_output_dir" default="$(find lio_livox)/logs" />

    <node pkg="lio_livox" type="ScanRegistration" name="ScanRegistration" output="screen">
        <param name="config_file" value="$(find lio_livox)/config/horizon_config.yaml"/>
    </node>

    <node pkg="lio_livox" type="PoseEstimation" name="PoseEstimation" output="screen">
        <!-- 0-Not Use IMU, 1-Use IMU remove Rotation Distort, 2-Tightly Coupled IMU -->
        <param name="IMU_Mode" type="int" value="2" />
        <!-- Voxel Filter Size Use to Downsize Map Cloud -->
        <param name="filter_parameter_corner" type="double" value="0.2" />
        <param name="filter_parameter_surf" type="double" value="0.4" />
        <!-- Extrinsic Parameter between Lidar & IMU -->
        <rosparam param="Extrinsic_Tlb"> [1.0, 0.0, 0.0, -0.05512,
                                          0.0, 1.0, 0.0, -0.02226,
                                          0.0, 0.0, 1.0,  0.0297,
                                          0.0, 0.0, 0.0,  1.0]</rosparam>
        <!-- Benchmark logging parameters -->
        <param name="enable_debug_log" type="bool" value="$(arg enable_debug_log)" />
        <param name="benchmark_output_dir" type="string" value="$(arg benchmark_output_dir)" />
    </node>
    <node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find lio_livox)/rviz_cfg/lio.rviz" />

</launch>
```

**关键参数说明**:
- `enable_debug_log`: 是否启用benchmark日志（默认: true）
- `benchmark_output_dir`: 日志输出基础目录（默认: `$(find lio_livox)/logs`）

---

## 版本历史

| 版本 | 日期 | 描述 |
|-----|------|------|
| v1.0 | 2025-12-03 | 初始版本：基本timing和feature count |
| v2.0 | 2025-12-03 | 扩展版：全面的速度和精度指标 |
| v2.1 | 2025-12-03 | 自动时间戳文件夹和文件命名 |
