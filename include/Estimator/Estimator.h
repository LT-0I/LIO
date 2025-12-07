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
#include "MapManager/Map_Manager.h"
#include "utils/ceresfunc.h"
#include "IMUIntegrator/IMUIntegrator.h"
#include <chrono>
#include <unordered_map>
#include <shared_mutex>
#include <array>

// 启用 nanoflann 替换 PCL KdTreeFLANN
#define USE_NANOFLANN 1

#if USE_NANOFLANN
#include "nanoflann/nanoflann_pcl_adaptor.hpp"
#endif

class Estimator{
	typedef pcl::PointXYZINormal PointType;
	
#if USE_NANOFLANN
	// nanoflann KD-tree 类型别名
	using LocalKdTreeType = nanoflann_pcl::KdTreeNano<PointType>;
	using LocalKdTreePtr = std::shared_ptr<LocalKdTreeType>;
#else
	using LocalKdTreeType = pcl::KdTreeFLANN<PointType>;
	using LocalKdTreePtr = pcl::KdTreeFLANN<PointType>::Ptr;
#endif

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
		FeatureLine(Eigen::Vector3d  po, Eigen::Vector3d  p1, Eigen::Vector3d  p2)
						:pointOri(std::move(po)), lineP1(std::move(p1)), lineP2(std::move(p2)){
			valid = false;
			error = 0;
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

public:
	/** \brief constructor of Estimator
	*/
	Estimator(const float& filter_corner, const float& filter_surf,
	          int max_iters = 4, int ceres_max_iters = 10,
	          double conv_r = 0.05, double conv_t = 0.05,
	          float filter_nonfeature = 0.4f);

	~Estimator();

		/** \brief Open a independent thread to increment MAP cloud
		*/
	[[noreturn]] void threadMapIncrement();

	/** \brief construct sharp feature Ceres Costfunctions
	* \param[in] edges: store costfunctions
	* \param[in] m4d: lidar pose, represented by matrix 4X4
	*/
	void processPointToLine(std::vector<ceres::CostFunction *>& edges,
							std::vector<FeatureLine>& vLineFeatures,
							const pcl::PointCloud<PointType>::Ptr& laserCloudCorner,
							const pcl::PointCloud<PointType>::Ptr& laserCloudCornerMap,
							const LocalKdTreePtr& kdtree,
							const Eigen::Matrix4d& exTlb,
							const Eigen::Matrix4d& m4d);

	/** \brief construct Plan feature Ceres Costfunctions
	* \param[in] edges: store costfunctions
	* \param[in] m4d: lidar pose, represented by matrix 4X4
	*/
	void processPointToPlan(std::vector<ceres::CostFunction *>& edges,
							std::vector<FeaturePlan>& vPlanFeatures,
							const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
							const pcl::PointCloud<PointType>::Ptr& laserCloudSurfMap,
							const LocalKdTreePtr& kdtree,
							const Eigen::Matrix4d& exTlb,
							const Eigen::Matrix4d& m4d);

	void processPointToPlanVec(std::vector<ceres::CostFunction *>& edges,
							   std::vector<FeaturePlanVec>& vPlanFeatures,
							   const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
							   const pcl::PointCloud<PointType>::Ptr& laserCloudSurfMap,
							   const LocalKdTreePtr& kdtree,
							   const Eigen::Matrix4d& exTlb,
							   const Eigen::Matrix4d& m4d);
				
	void processNonFeatureICP(std::vector<ceres::CostFunction *>& edges,
							  std::vector<FeatureNon>& vNonFeatures,
							  const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeature,
							  const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureLocal,
							  const LocalKdTreePtr& kdtreeLocal,
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

private:
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
#if USE_NANOFLANN
	// 使用 nanoflann 替换 PCL KdTreeFLANN（局部地图）
	std::shared_ptr<nanoflann_pcl::KdTreeNano<PointType>> kdtreeCornerFromLocal;
	std::shared_ptr<nanoflann_pcl::KdTreeNano<PointType>> kdtreeSurfFromLocal;
	std::shared_ptr<nanoflann_pcl::KdTreeNano<PointType>> kdtreeNonFeatureFromLocal;
#else
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromLocal;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromLocal;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeNonFeatureFromLocal;
#endif
	pcl::VoxelGrid<PointType> downSizeFilterCorner;
	pcl::VoxelGrid<PointType> downSizeFilterSurf;
	pcl::VoxelGrid<PointType> downSizeFilterNonFeature;
	std::mutex mtx_Map;
	std::thread threadMap;

	// 全局地图点云（保留）
	pcl::PointCloud<PointType> GlobalSurfMap[10000];
	pcl::PointCloud<PointType> GlobalCornerMap[10000];
	pcl::PointCloud<PointType> GlobalNonFeatureMap[10000];

	// 全局地图 KD-tree（使用 PCL，nanoflann 缓存方案精度差已回退）
	pcl::KdTreeFLANN<PointType> CornerKdMap[10000];
	pcl::KdTreeFLANN<PointType> SurfKdMap[10000];
	pcl::KdTreeFLANN<PointType> NonFeatureKdMap[10000];

	int laserCenWidth_last = 10;   // 对应 CUBE 网格 21
	int laserCenHeight_last = 5;   // 对应 CUBE 网格 11
	int laserCenDepth_last = 10;   // 对应 CUBE 网格 21

	static const int localMapWindowSize = 50;
	int localMapID = 0;
	pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
	pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
	pcl::PointCloud<PointType>::Ptr localNonFeatureMap[localMapWindowSize];

	int map_update_ID = 0;

	int map_skip_frame = 2; //every map_skip_frame frame update map
	double plan_weight_tan = 0.0;
	double thres_dist = 1.0;
	
	// === 可调参数（从 launch 文件读取） ===
	int max_iterations_ = 4;           // 外层迭代次数
	int ceres_max_iterations_ = 10;    // Ceres 内部迭代次数
	double convergence_threshold_r_ = 0.05;  // 收敛阈值（角度，度）
	double convergence_threshold_t_ = 0.05;  // 收敛阈值（位移，米）
	float filter_nonfeature_ = 0.4f;   // NonFeature 下采样体素
	
	// === 优化：预分配特征存储（避免每帧堆分配） ===
	static constexpr int MAX_FEATURES_PER_FRAME = 3000;
	std::array<std::vector<FeatureLine>, SLIDEWINDOWSIZE> vLineFeatures_;
	std::array<std::vector<FeaturePlanVec>, SLIDEWINDOWSIZE> vPlanFeatures_;
	std::array<std::vector<FeatureNon>, SLIDEWINDOWSIZE> vNonFeatures_;
	std::array<std::vector<ceres::CostFunction*>, SLIDEWINDOWSIZE> edgesLine_;
	std::array<std::vector<ceres::CostFunction*>, SLIDEWINDOWSIZE> edgesPlan_;
	std::array<std::vector<ceres::CostFunction*>, SLIDEWINDOWSIZE> edgesNon_;
	
	// === 优化：退化检测 ===
	bool is_degenerate_ = false;
	int valid_corner_count_ = 0;
	int valid_surf_count_ = 0;
	void checkDegeneracy();
	void initTimeLogger();

	std::ofstream time_log_;
	bool time_log_ready_{false};
};

#endif //LIO_LIVOX_ESTIMATOR_H
