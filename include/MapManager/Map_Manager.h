#ifndef LIO_LIVOX_MAP_MANAGER_H
#define LIO_LIVOX_MAP_MANAGER_H
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <future>
#include <atomic>
#include <memory>

// =========================================================
// MapSnapshot - 零拷贝地图快照，供 Estimator 只读访问
// =========================================================
// 前向声明 CUBE 数量（需要在 MapSnapshot 之前定义）
static const int CUBE_NUM = 21 * 11 * 21;  // 4851 (原始值，回退)

struct MapSnapshot {
    typedef pcl::PointXYZINormal PointType;
    static const int N = CUBE_NUM;
    
    // KD-tree 指针（只读引用）
    pcl::KdTreeFLANN<PointType>* cornerKdMap[N];
    pcl::KdTreeFLANN<PointType>* surfKdMap[N];
    pcl::KdTreeFLANN<PointType>* nonFeatureKdMap[N];
    
    // 点云指针（只读引用）
    pcl::PointCloud<PointType>* cornerPointMap[N];
    pcl::PointCloud<PointType>* surfPointMap[N];
    pcl::PointCloud<PointType>* nonFeaturePointMap[N];
    
    // 地图中心坐标
    int cenWidth = 10;   // 21/2
    int cenHeight = 5;   // 11/2
    int cenDepth = 10;   // 21/2
    
    // 快照有效标志
    bool valid = false;
};

class MAP_MANAGER{
    typedef pcl::PointXYZINormal PointType;
public:

    std::mutex mtx_MapManager;
    /** \brief constructor of MAP_MANAGER */
    MAP_MANAGER(const float& filter_corner, const float& filter_surf);

    static size_t ToIndex(int i, int j, int k);

    /** \brief transform float to int
  */
    static uint32_t _float_as_int(float f){
      union{uint32_t i; float f;} conv{};
      conv.f = f;
      return conv.i;
    }

    /** \brief transform int to float
      */
    static float _int_as_float(uint32_t i){
      union{float f; uint32_t i;} conv{};
      conv.i = i;
      return conv.f;
    }

    /** \brief transform point pi to the MAP coordinate
     * \param[in] pi: point to be transformed
     * \param[in] po: point after transfomation
     * \param[in] _transformTobeMapped: transform matrix between pi and po
     */
    static void pointAssociateToMap(PointType const * const pi,
                                    PointType * const po,
                                    const Eigen::Matrix4d& _transformTobeMapped);

    void featureAssociateToMap(const pcl::PointCloud<PointType>::Ptr& laserCloudCorner,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeature,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudCornerToMap,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudSurfToMap,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureToMap,
                               const Eigen::Matrix4d& transformTobeMapped);
    /** \brief add new lidar points to the map
     * \param[in] laserCloudCornerStack: coner feature points that need to be added to map
     * \param[in] laserCloudSurfStack: surf feature points that need to be added to map
     * \param[in] transformTobeMapped: transform matrix of the lidar pose
     */
    void MapIncrement(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerStack,
                      const pcl::PointCloud<PointType>::Ptr& laserCloudSurfStack,
                      const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureStack,
                      const Eigen::Matrix4d& transformTobeMapped);

    /** \brief retrieve map points according to the lidar pose
     * \param[in] laserCloudCornerFromMap: store coner feature points retrieved from map
     * \param[in] laserCloudSurfFromMap: tore surf feature points retrieved from map
     * \param[in] transformTobeMapped: transform matrix of the lidar pose
     */
    void MapMove(const Eigen::Matrix4d& transformTobeMapped);


    size_t FindUsedCornerMap(const PointType *p,int a,int b,int c);

    size_t FindUsedSurfMap(const PointType *p,int a,int b,int c);

    size_t FindUsedNonFeatureMap(const PointType *p,int a,int b,int c);

    // 只读指针借用（不复制，Map_Manager 持有所有权）
    const pcl::KdTreeFLANN<PointType>* getCornerKdMapPtr(int i){
      return laserCloudCornerKdMap[i].get();
    }
    const pcl::KdTreeFLANN<PointType>* getSurfKdMapPtr(int i){
      return laserCloudSurfKdMap[i].get();
    }
    const pcl::KdTreeFLANN<PointType>* getNonFeatureKdMapPtr(int i){
      return laserCloudNonFeatureKdMap[i].get();
    }
    const pcl::PointCloud<PointType>* getCornerMapPtr(int i){
      return laserCloudCornerArray[i].get();
    }
    const pcl::PointCloud<PointType>* getSurfMapPtr(int i){
      return laserCloudSurfArray[i].get();
    }
    const pcl::PointCloud<PointType>* getNonFeatureMapPtr(int i){
      return laserCloudNonFeatureArray[i].get();
    }
		pcl::PointCloud<PointType>::Ptr get_corner_map(){
			return laserCloudCornerFromMap;
		}
		pcl::PointCloud<PointType>::Ptr get_surf_map(){
			return laserCloudSurfFromMap;
		}
    pcl::PointCloud<PointType>::Ptr get_nonfeature_map(){
			return laserCloudNonFeatureFromMap;
		}
    int get_map_current_pos(){
      return currentUpdatePos;
    }
    int get_laserCloudCenWidth_last(){
      return laserCloudCenWidth_last;
    }
    int get_laserCloudCenHeight_last(){
      return laserCloudCenHeight_last;
    }
    int get_laserCloudCenDepth_last(){
      return laserCloudCenDepth_last;
    }
    pcl::PointCloud<PointType> laserCloudSurf_for_match[CUBE_NUM];
    pcl::PointCloud<PointType> laserCloudCorner_for_match[CUBE_NUM];
    pcl::PointCloud<PointType> laserCloudNonFeature_for_match[CUBE_NUM];
    
    // =========================================================
    // 双缓冲 MapSnapshot 接口（零拷贝）
    // =========================================================
    
    /** \brief 获取当前可读取的地图快照（零拷贝，只读）
     * \return 指向 published 快照的 const 指针
     */
    const MapSnapshot* AcquireSnapshot();
    
    /** \brief 释放快照（当前实现无需显式释放）
     */
    void ReleaseSnapshot();
    
    /** \brief 发布新的快照（由 MapIncrement 内部调用）
     */
    void PublishSnapshot();

private:
    int laserCloudCenWidth = 10;   // 21/2
    int laserCloudCenHeight = 5;   // 11/2
    int laserCloudCenDepth = 10;   // 21/2

    int laserCloudCenWidth_last = 10;
    int laserCloudCenHeight_last = 5;
    int laserCloudCenDepth_last = 10;

    // CUBE 网格大小（原始值，回退）
    static const int laserCloudWidth = 21;
    static const int laserCloudHeight = 11;
    static const int laserCloudDepth = 21;
    static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851
    // 地图滚动边界
    static const int cubeMargin = 8;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureArray[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudCornerArrayStack[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfArrayStack[laserCloudNum];
    pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureArrayStack[laserCloudNum];

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterNonFeature;
    const float filter_corner_leaf_;
    const float filter_surf_leaf_;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr laserCloudCornerKdMap[laserCloudNum];
    pcl::KdTreeFLANN<PointType>::Ptr laserCloudSurfKdMap[laserCloudNum];
    pcl::KdTreeFLANN<PointType>::Ptr laserCloudNonFeatureKdMap[laserCloudNum];

    pcl::KdTreeFLANN<PointType> CornerKdMap_copy[laserCloudNum];
    pcl::KdTreeFLANN<PointType> SurfKdMap_copy[laserCloudNum];
    pcl::KdTreeFLANN<PointType> NonFeatureKdMap_copy[laserCloudNum];

    pcl::KdTreeFLANN<PointType> CornerKdMap_last[laserCloudNum];
    pcl::KdTreeFLANN<PointType> SurfKdMap_last[laserCloudNum];
    pcl::KdTreeFLANN<PointType> NonFeatureKdMap_last[laserCloudNum];

    static const int localMapWindowSize = 60;
    pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
    pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
    pcl::PointCloud<PointType>::Ptr localNonFeatureMap[localMapWindowSize];

    int localMapID = 0;

    int currentUpdatePos = 0;
    int estimatorPos = 0;
    
    // === 增量拷贝优化 ===
    bool cubeNeedSync[laserCloudNum] = {false};  // 脏标记：哪些 cube 需要同步
    int syncedCubeCount = 0;  // 统计：每帧同步的 cube 数量
    
    // === 双缓冲 MapSnapshot ===
    static const int kSnapshotBufferCount = 2;
    MapSnapshot snapshots_[kSnapshotBufferCount];  // 双缓冲快照
    std::atomic<int> publishedIdx_{0};   // 当前可读取的快照索引
    std::atomic<int> stagingIdx_{1};     // 当前写入的快照索引
    std::atomic<int> snapshotRefCount_{0};  // 引用计数（防止覆盖正在读取的快照）
    
    // 双缓冲底层存储（独立的 KD-tree 和点云）
    pcl::KdTreeFLANN<PointType> kdCornerBuffer_[kSnapshotBufferCount][laserCloudNum];
    pcl::KdTreeFLANN<PointType> kdSurfBuffer_[kSnapshotBufferCount][laserCloudNum];
    pcl::KdTreeFLANN<PointType> kdNonBuffer_[kSnapshotBufferCount][laserCloudNum];
    pcl::PointCloud<PointType> pcCornerBuffer_[kSnapshotBufferCount][laserCloudNum];
    pcl::PointCloud<PointType> pcSurfBuffer_[kSnapshotBufferCount][laserCloudNum];
    pcl::PointCloud<PointType> pcNonBuffer_[kSnapshotBufferCount][laserCloudNum];
};

#endif //LIO_LIVOX_MAP_MANAGER_H
