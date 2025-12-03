#ifndef BENCHMARK_LOGGER_H
#define BENCHMARK_LOGGER_H

#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

/**
 * @brief High-performance benchmark logger for LIO-Livox
 * Provides CSV internal stats logging and TUM trajectory logging
 * Extended with comprehensive metrics for speed and accuracy analysis
 */
class BenchmarkLogger {
public:
    // Struct to hold optimization metrics from Estimator
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
        double convergence_threshold_r = 0.05;  // Rotation convergence threshold (deg)
        double convergence_threshold_t = 0.05;  // Translation convergence threshold (m)
        
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
        double feature_extract_time_ms = 0.0;   // Feature extraction time (corner/surf separation)
        double feature_match_time_ms = 0.0;     // Point-to-line/plane matching time
        double optimization_time_ms = 0.0;      // Ceres solver execution time
        double map_update_time_ms = 0.0;        // Map increment time
        double publish_time_ms = 0.0;           // Point cloud publishing time
        
        // ============== Average Feature Errors (Accuracy Indicators) ==============
        double avg_corner_error = 0.0;     // Average point-to-line error
        double avg_surf_error = 0.0;       // Average point-to-plane error
        double avg_nonfeature_error = 0.0; // Average non-feature ICP error
    };

private:
    bool enabled_ = false;
    std::ofstream csv_file_;
    std::ofstream tum_file_;
    std::mutex csv_mutex_;
    std::mutex tum_mutex_;
    bool csv_header_written_ = false;
    std::string session_timestamp_;
    std::string output_folder_;
    
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
        // Try to create the directory
        int result = mkdir(path.c_str(), 0755);
        if (result == 0) {
            return true;  // Successfully created
        }
        // Check if it already exists
        struct stat st;
        if (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            return true;  // Already exists
        }
        return false;
    }
    
public:
    BenchmarkLogger() = default;
    
    ~BenchmarkLogger() {
        close();
    }
    
    /**
     * @brief Get the session timestamp string
     */
    const std::string& getSessionTimestamp() const { return session_timestamp_; }
    
    /**
     * @brief Get the output folder path
     */
    const std::string& getOutputFolder() const { return output_folder_; }
    
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
            // Fallback to base directory if subfolder creation fails
            output_folder_ = base_output_dir;
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
            // TUM format header comment
            tum_file_ << "# timestamp x y z q_x q_y q_z q_w\n";
            tum_file_ << "# Session started: " << session_timestamp_ << "\n";
        }
    }
    
    /**
     * @brief Check if logging is enabled
     */
    bool isEnabled() const { return enabled_; }
    
    /**
     * @brief Log internal statistics to CSV file (Extended Version)
     * @param msg_timestamp Message header timestamp (bag time)
     * @param total_frame_time_ms Total frame processing time
     * @param metrics Optimization metrics from Estimator
     */
    void logInternalStats(double msg_timestamp,
                         double total_frame_time_ms,
                         const OptimizationMetrics& metrics) {
        if (!enabled_ || !csv_file_.is_open()) return;
        
        std::lock_guard<std::mutex> lock(csv_mutex_);
        
        // Write header on first call
        if (!csv_header_written_) {
            csv_file_ << "timestamp,"
                      // Timing metrics (Speed Analysis)
                      << "total_frame_time_ms,"
                      << "preprocess_time_ms,"
                      << "kdtree_build_time_ms,"
                      << "feature_extract_time_ms,"
                      << "feature_match_time_ms,"
                      << "optimization_time_ms,"
                      << "map_update_time_ms,"
                      << "publish_time_ms,"
                      // Feature counts (effective_feat_num breakdown)
                      << "corner_features,"
                      << "surf_features,"
                      << "nonfeature_points,"
                      << "total_features,"
                      // Feature matching details
                      << "corner_from_map,"
                      << "corner_from_local,"
                      << "surf_from_map,"
                      << "surf_from_local,"
                      // Point cloud sizes
                      << "raw_cloud_size,"
                      << "downsampled_corner,"
                      << "downsampled_surf,"
                      << "downsampled_nonfeature,"
                      // Map sizes
                      << "map_corner_size,"
                      << "map_surf_size,"
                      << "local_corner_size,"
                      << "local_surf_size,"
                      << "window_size,"
                      // IMU metrics
                      << "imu_msg_count,"
                      << "imu_dt_sec,"
                      << "imu_initialized,"
                      // Optimization metrics (Accuracy)
                      << "outer_iterations,"
                      << "ceres_iterations,"
                      << "initial_cost,"
                      << "final_cost,"
                      << "delta_rotation_deg,"
                      << "delta_translation_m,"
                      << "converged_early,"
                      // Feature errors (Accuracy indicators)
                      << "avg_corner_error,"
                      << "avg_surf_error,"
                      << "avg_nonfeature_error\n";
            csv_header_written_ = true;
        }
        
        // Write data row - use '\n' instead of std::endl for performance
        csv_file_ << msg_timestamp << ","
                  // Timing metrics
                  << total_frame_time_ms << ","
                  << metrics.preprocess_time_ms << ","
                  << metrics.kdtree_build_time_ms << ","
                  << metrics.feature_extract_time_ms << ","
                  << metrics.feature_match_time_ms << ","
                  << metrics.optimization_time_ms << ","
                  << metrics.map_update_time_ms << ","
                  << metrics.publish_time_ms << ","
                  // Feature counts
                  << metrics.corner_features << ","
                  << metrics.surf_features << ","
                  << metrics.nonfeature_points << ","
                  << metrics.total_features << ","
                  // Feature matching details
                  << metrics.corner_from_map << ","
                  << metrics.corner_from_local << ","
                  << metrics.surf_from_map << ","
                  << metrics.surf_from_local << ","
                  // Point cloud sizes
                  << metrics.raw_cloud_size << ","
                  << metrics.downsampled_corner_size << ","
                  << metrics.downsampled_surf_size << ","
                  << metrics.downsampled_nonfeature_size << ","
                  // Map sizes
                  << metrics.map_corner_size << ","
                  << metrics.map_surf_size << ","
                  << metrics.local_corner_size << ","
                  << metrics.local_surf_size << ","
                  << metrics.window_size << ","
                  // IMU metrics
                  << metrics.imu_msg_count << ","
                  << metrics.imu_dt << ","
                  << (metrics.imu_initialized ? 1 : 0) << ","
                  // Optimization metrics
                  << metrics.iterations << ","
                  << metrics.ceres_iterations << ","
                  << metrics.initial_cost << ","
                  << metrics.final_cost << ","
                  << metrics.delta_rotation << ","
                  << metrics.delta_translation << ","
                  << (metrics.converged_early ? 1 : 0) << ","
                  // Feature errors
                  << metrics.avg_corner_error << ","
                  << metrics.avg_surf_error << ","
                  << metrics.avg_nonfeature_error << "\n";
    }
    
    /**
     * @brief Log trajectory in TUM format
     * Uses message header timestamp (bag time), NOT system wall time
     * @param msg_timestamp Message header timestamp
     * @param x Position x
     * @param y Position y
     * @param z Position z
     * @param qx Quaternion x
     * @param qy Quaternion y
     * @param qz Quaternion z
     * @param qw Quaternion w
     */
    void logTrajectoryTUM(double msg_timestamp,
                          double x, double y, double z,
                          double qx, double qy, double qz, double qw) {
        if (!enabled_ || !tum_file_.is_open()) return;
        
        std::lock_guard<std::mutex> lock(tum_mutex_);
        
        // TUM format: timestamp x y z q_x q_y q_z q_w
        // Use '\n' instead of std::endl for non-blocking I/O
        tum_file_ << msg_timestamp << " "
                  << x << " " << y << " " << z << " "
                  << qx << " " << qy << " " << qz << " " << qw << "\n";
    }
    
    /**
     * @brief Flush and close all log files
     */
    void close() {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
        }
        if (tum_file_.is_open()) {
            tum_file_.flush();
            tum_file_.close();
        }
    }
    
    /**
     * @brief High-resolution timer helper class
     */
    class ScopedTimer {
    public:
        using Clock = std::chrono::high_resolution_clock;
        
        ScopedTimer(double* output_ms) : output_(output_ms) {
            start_ = Clock::now();
        }
        
        ~ScopedTimer() {
            if (output_) {
                auto end = Clock::now();
                *output_ = std::chrono::duration<double, std::milli>(end - start_).count();
            }
        }
        
        double elapsed_ms() const {
            auto now = Clock::now();
            return std::chrono::duration<double, std::milli>(now - start_).count();
        }
        
    private:
        Clock::time_point start_;
        double* output_;
    };
};

// Global benchmark logger instance
extern BenchmarkLogger g_benchmark_logger;

#endif // BENCHMARK_LOGGER_H

