#ifndef LIO_LIVOX_ESTIMATOR_H
#define LIO_LIVOX_ESTIMATOR_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <iterator>
#include <future>
#include <memory>
#include "MapManager/Map_Manager.h"
#include "utils/ceresfunc.h"
#include "utils/VoxelIndex.h"
#include "utils/BenchmarkLogger.h"
#include "IMUIntegrator/IMUIntegrator.h"
#include <chrono>

struct CornerAdaptiveConfig{
	bool enable = true;
	double default_eigen_ratio = 3.0;
	double low_feature_eigen_ratio = 2.5;
	int low_feature_global_kd = 80;
	int low_feature_min_keep = 200;
	int high_feature_global_kd = 800;
	int high_feature_max_keep = 400;
};

struct ResidualBudgetConfig{
	bool enable = false;
	double target_residual_build_ms = 8.0;
	double target_ceres_solve_ms = 25.0;
	double tolerance_ratio = 0.2;
	double adjust_ratio = 0.15;
	int min_corner_residuals = 200;
	int min_surf_residuals = 450;
	int min_non_residuals = 250;
};

struct EstimatorResidualConfig{
	int max_corner_residuals = 500;
	int max_surf_residuals = 750;
	int max_non_residuals = 350;
	double feature_error_threshold = 1e-5;
	bool log_feature_counts = false;
	CornerAdaptiveConfig adaptive_corner;
	ResidualBudgetConfig adaptive_budget;
};

class Estimator{
	typedef pcl::PointXYZINormal PointType;
public:
	/** \brief slide window size */
	static const int SLIDEWINDOWSIZE = 2;

	/** \brief lidar frame struct */
	struct LidarFrame{
		pcl::PointCloud<PointType>::Ptr laserCloud;
		IMUIntegrator imuIntegrator;
		Eigen::Vector3d P;
		Eigen::Vector3d V;
		Eigen::Quaterniond Q;
		Eigen::Vector3d bg;
		Eigen::Vector3d ba;
		double timeStamp;
		LidarFrame(){
			P.setZero();
			V.setZero();
			Q.setIdentity();
			bg.setZero();
			ba.setZero();
			timeStamp = 0;
		}
	};

	/** \brief point to line feature */
	struct FeatureLine{
		Eigen::Vector3d pointOri;
		Eigen::Vector3d lineP1;
		Eigen::Vector3d lineP2;
		double error;
		bool valid;
		bool from_global;
		FeatureLine(Eigen::Vector3d  po, Eigen::Vector3d  p1, Eigen::Vector3d  p2)
						:pointOri(std::move(po)), lineP1(std::move(p1)), lineP2(std::move(p2)){
			valid = false;
			error = 0;
			from_global = false;
		}
		double ComputeError(const Eigen::Matrix4d& pose){
			Eigen::Vector3d P_to_Map = pose.topLeftCorner(3,3) * pointOri + pose.topRightCorner(3,1);
			double l12 = std::sqrt((lineP1(0) - lineP2(0))*(lineP1(0) - lineP2(0)) + (lineP1(1) - lineP2(1))*
																						(lineP1(1) - lineP2(1)) + (lineP1(2) - lineP2(2))*(lineP1(2) - lineP2(2)));
			double a012 = std::sqrt(
							((P_to_Map(0) - lineP1(0)) * (P_to_Map(1) - lineP2(1)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(1) - lineP1(1)))
							* ((P_to_Map(0) - lineP1(0)) * (P_to_Map(1) - lineP2(1)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(1) - lineP1(1)))
							+ ((P_to_Map(0) - lineP1(0)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(2) - lineP1(2)))
								* ((P_to_Map(0) - lineP1(0)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(0) - lineP2(0)) * (P_to_Map(2) - lineP1(2)))
							+ ((P_to_Map(1) - lineP1(1)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(1) - lineP2(1)) * (P_to_Map(2) - lineP1(2)))
								* ((P_to_Map(1) - lineP1(1)) * (P_to_Map(2) - lineP2(2)) - (P_to_Map(1) - lineP2(1)) * (P_to_Map(2) - lineP1(2))));
			if (std::isfinite(l12) && l12 > 1e-6) {
				error = a012 / l12;
				return error;
			}
			error = 0.0;
			return error;
		}
	};

	/** \brief point to plan feature */
	struct FeaturePlan{
		Eigen::Vector3d pointOri;
		double pa;
		double pb;
		double pc;
		double pd;
		double error;
		bool valid;
		FeaturePlan(const Eigen::Vector3d& po, const double& pa_, const double& pb_, const double& pc_, const double& pd_)
						:pointOri(po), pa(pa_), pb(pb_), pc(pc_), pd(pd_){
			valid = false;
			error = 0;
		}
		double ComputeError(const Eigen::Matrix4d& pose){
			Eigen::Vector3d P_to_Map = pose.topLeftCorner(3,3) * pointOri + pose.topRightCorner(3,1);
			error = pa * P_to_Map(0) + pb * P_to_Map(1) + pc * P_to_Map(2) + pd;
			return error;
		}
	};

	/** \brief point to plan feature */
	struct FeaturePlanVec{
		Eigen::Vector3d pointOri;
		Eigen::Vector3d pointProj;
		Eigen::Matrix3d sqrt_info;
		double error;
		bool valid;
		FeaturePlanVec(const Eigen::Vector3d& po, const Eigen::Vector3d& p_proj, Eigen::Matrix3d sqrt_info_)
						:pointOri(po), pointProj(p_proj), sqrt_info(sqrt_info_) {
			valid = false;
			error = 0;
		}
		double ComputeError(const Eigen::Matrix4d& pose){
			Eigen::Vector3d P_to_Map = pose.topLeftCorner(3,3) * pointOri + pose.topRightCorner(3,1);
			error = (P_to_Map - pointProj).norm();
			return error;
		}
	};

	/** \brief non feature */
	struct FeatureNon{
		Eigen::Vector3d pointOri;
		double pa;
		double pb;
		double pc;
		double pd;
		double error;
		bool valid;
		FeatureNon(const Eigen::Vector3d& po, const double& pa_, const double& pb_, const double& pc_, const double& pd_)
						:pointOri(po), pa(pa_), pb(pb_), pc(pc_), pd(pd_){
			valid = false;
			error = 0;
		}
		double ComputeError(const Eigen::Matrix4d& pose){
			Eigen::Vector3d P_to_Map = pose.topLeftCorner(3,3) * pointOri + pose.topRightCorner(3,1);
			error = pa * P_to_Map(0) + pb * P_to_Map(1) + pc * P_to_Map(2) + pd;
			return error;
		}
	};

	struct FeatureBuildStats{
		int points_total = 0;
		int global_region_skipped = 0;
		int global_kd_success = 0;
		int global_eigen_pass = 0;
		int global_eigen_fail = 0;
		int local_kd_success = 0;
		int local_eigen_pass = 0;
		int local_eigen_fail = 0;
	};

public:
	/** \brief constructor of Estimator
	*/
	Estimator(const float& filter_corner,
	          const float& filter_surf,
	          const MapManagerConfig& map_config = MapManagerConfig(),
	          const EstimatorResidualConfig& residual_config = EstimatorResidualConfig(),
	          bool log_module_timing = false);

	~Estimator();

		/** \brief Open a independent thread to increment MAP cloud
		*/
	[[noreturn]] void threadMapIncrement();

	/** \brief construct sharp feature Ceres Costfunctions
	* \param[in] edges: store costfunctions
	* \param[in] m4d: lidar pose, represented by matrix 4X4
	*/
	void processPointToLine(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
							std::vector<FeatureLine>& vLineFeatures,
							const pcl::PointCloud<PointType>::Ptr& laserCloudCorner,
							const pcl::PointCloud<PointType>::Ptr& laserCloudCornerMap,
							const pcl::KdTreeFLANN<PointType>::Ptr& kdtree,
							const Eigen::Matrix4d& exTlb,
							const Eigen::Matrix4d& m4d,
							struct FeatureBuildStats* stats = nullptr);

	/** \brief construct Plan feature Ceres Costfunctions
	* \param[in] edges: store costfunctions
	* \param[in] m4d: lidar pose, represented by matrix 4X4
	*/
	void processPointToPlan(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
							std::vector<FeaturePlan>& vPlanFeatures,
							const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
							const pcl::PointCloud<PointType>::Ptr& laserCloudSurfMap,
							const pcl::KdTreeFLANN<PointType>::Ptr& kdtree,
							const Eigen::Matrix4d& exTlb,
							const Eigen::Matrix4d& m4d);

	void processPointToPlanVec(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
							   std::vector<FeaturePlanVec>& vPlanFeatures,
							   const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
							   const pcl::PointCloud<PointType>::Ptr& laserCloudSurfMap,
							   const pcl::KdTreeFLANN<PointType>::Ptr& kdtree,
							   const Eigen::Matrix4d& exTlb,
							   const Eigen::Matrix4d& m4d);
				
	void processNonFeatureICP(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
							  std::vector<FeatureNon>& vNonFeatures,
							  const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeature,
							  const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureLocal,
							  const pcl::KdTreeFLANN<PointType>::Ptr& kdtreeLocal,
							  const Eigen::Matrix4d& exTlb,
							  const Eigen::Matrix4d& m4d);

	/** \brief Transform Lidar Pose in slidewindow to double array
		* \param[in] lidarFrameList: Lidar Poses in slidewindow
		*/
	void vector2double(const std::list<LidarFrame>& lidarFrameList);

	/** \brief Transform double array to Lidar Pose in slidewindow
		* \param[in] lidarFrameList: Lidar Poses in slidewindow
		*/
	void double2vector(std::list<LidarFrame>& lidarFrameList);

	/** \brief estimate lidar pose by matching current lidar cloud with map cloud and tightly coupled IMU message
		* \param[in] lidarFrameList: multi-frames of lidar cloud and lidar pose
		* \param[in] exTlb: extrinsic matrix between lidar and IMU
		* \param[in] gravity: gravity vector
		*/
	void EstimateLidarPose(std::list<LidarFrame>& lidarFrameList,
						   const Eigen::Matrix4d& exTlb,
						   const Eigen::Vector3d& gravity,
						   nav_msgs::Odometry& debugInfo);

	void Estimate(std::list<LidarFrame>& lidarFrameList,
				  const Eigen::Matrix4d& exTlb,
				  const Eigen::Vector3d& gravity);

	pcl::PointCloud<PointType>::Ptr get_corner_map(){
		return map_manager->get_corner_map();
	}
	pcl::PointCloud<PointType>::Ptr get_surf_map(){
		return map_manager->get_surf_map();
	}
	pcl::PointCloud<PointType>::Ptr get_nonfeature_map(){
		return map_manager->get_nonfeature_map();
	}
	void MapIncrementLocal(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerStack,
						   const pcl::PointCloud<PointType>::Ptr& laserCloudSurfStack,
						   const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureStack,
						   const Eigen::Matrix4d& transformTobeMapped);

	/** \brief Get optimization metrics for benchmark logging */
	const BenchmarkLogger::OptimizationMetrics& getOptimizationMetrics() const {
		return optimization_metrics_;
	}

private:
	// Benchmark optimization metrics collected during Estimate()
	BenchmarkLogger::OptimizationMetrics optimization_metrics_;
	EstimatorResidualConfig residual_config_;
	/** \brief store map points */
	MAP_MANAGER* map_manager;

	double para_PR[SLIDEWINDOWSIZE][6];
	double para_VBias[SLIDEWINDOWSIZE][9];
	MarginalizationInfo *last_marginalization_info = nullptr;
	std::vector<double *> last_marginalization_parameter_blocks;
	std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerLast;
	std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfLast;
	std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudNonFeatureLast;

	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromLocal;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromLocal;
	pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureFromLocal;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerForMap;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfForMap;
	pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureForMap;
	Eigen::Matrix4d transformForMap;
	std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerStack;
	std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfStack;
	std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudNonFeatureStack;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromLocal;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromLocal;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeNonFeatureFromLocal;

	// VoxelIndex for O(1) local map search (space-time tradeoff)
	VoxelIndex voxelCornerLocal_;
	VoxelIndex voxelSurfLocal_;
	VoxelIndex voxelNonLocal_;
	bool use_voxel_index_local_ = true;
	float voxel_index_resolution_ = 0.5f;
	pcl::VoxelGrid<PointType> downSizeFilterCorner;
	pcl::VoxelGrid<PointType> downSizeFilterSurf;
	pcl::VoxelGrid<PointType> downSizeFilterNonFeature;
	std::mutex mtx_Map;
	std::thread threadMap;

	const pcl::KdTreeFLANN<PointType>* CornerKdMap[10000];
	const pcl::KdTreeFLANN<PointType>* SurfKdMap[10000];
	const pcl::KdTreeFLANN<PointType>* NonFeatureKdMap[10000];

	const pcl::PointCloud<PointType>* GlobalSurfMap[10000];
	const pcl::PointCloud<PointType>* GlobalCornerMap[10000];
	const pcl::PointCloud<PointType>* GlobalNonFeatureMap[10000];

	int laserCenWidth_last = 10;
	int laserCenHeight_last = 5;
	int laserCenDepth_last = 10;

	static const int localMapWindowSize = 50;
	int localMapID = 0;
	pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
	pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
	pcl::PointCloud<PointType>::Ptr localNonFeatureMap[localMapWindowSize];
	long localFrameId = 0;
	long localFrameStamp[localMapWindowSize];
	static const int localMapHistoryFrames = 20;
	double localBoxForward = 40.0;
	double localBoxBackward = 8.0;
	double localBoxSide = 8.0;
	double localBoxVertical = 6.0;

	int map_update_ID = 0;

	int map_skip_frame = 2; //every map_skip_frame frame update map
	double plan_weight_tan = 0.0;
	double thres_dist = 1.0;
	double corner_eigen_ratio_ = 3.0;
	bool log_module_timing_ = false;
	int runtime_corner_limit_ = 0;
	int runtime_surf_limit_ = 0;
	int runtime_non_limit_ = 0;
	double last_residual_build_ms_ = 0.0;
	double last_ceres_solve_ms_ = 0.0;
	int local_corner_max_points_ = 0;
	int local_surf_max_points_ = 0;
	int local_non_max_points_ = 0;

	// 动态搜索半径 (隧道场景优化)
	double last_avg_global_kd_ = 100.0;  // 上一帧的平均全局KD匹配数
	double dynamic_thres_dist_ = 1.0;    // 动态调整的搜索半径
	double last_speed_ = 10.0;           // 上一帧的速度估计 (m/s)
	int consecutive_low_speed_frames_ = 0; // 连续低速帧计数

	void UpdateResidualLimits(double build_ms, double solve_ms);
	void EnforceLocalMapLimit(pcl::PointCloud<PointType>::Ptr& cloud, int max_points);
};

#endif //LIO_LIVOX_ESTIMATOR_H
