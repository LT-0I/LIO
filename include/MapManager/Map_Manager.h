#ifndef LIO_LIVOX_MAP_MANAGER_H
#define LIO_LIVOX_MAP_MANAGER_H
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <future>
#include <condition_variable>
#include <memory>
#include <array>
#include <vector>
#include <atomic>

struct MapManagerConfig{
  int width = 21;
  int height = 11;
  int depth = 21;
  int local_window = 60;
};

class MAP_MANAGER{
    typedef pcl::PointXYZINormal PointType;
public:

    std::mutex mtx_MapManager;
    /** \brief constructor of MAP_MANAGER */
    MAP_MANAGER(const float& filter_corner,
                const float& filter_surf,
                const MapManagerConfig& config = MapManagerConfig());

    size_t ToIndex(int i, int j, int k) const;

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

    struct MapSnapshot{
      const pcl::KdTreeFLANN<PointType>* corner_kd;
      const pcl::KdTreeFLANN<PointType>* surf_kd;
      const pcl::KdTreeFLANN<PointType>* nonfeature_kd;
      const pcl::PointCloud<PointType>* corner_map;
      const pcl::PointCloud<PointType>* surf_map;
      const pcl::PointCloud<PointType>* nonfeature_map;
      int laserCenWidth_last;
      int laserCenHeight_last;
      int laserCenDepth_last;
      int buffer_idx;
    };

    std::shared_ptr<MapSnapshot> AcquireSnapshot();

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
    static const int kMatchBufferCount = 2;
    const int laserCloudWidth;
    const int laserCloudHeight;
    const int laserCloudDepth;
    const int laserCloudNum;
    const int localMapWindowSize;

    std::array<std::vector<pcl::PointCloud<PointType>>, kMatchBufferCount> laserCloudSurf_for_match;
    std::array<std::vector<pcl::PointCloud<PointType>>, kMatchBufferCount> laserCloudCorner_for_match;
    std::array<std::vector<pcl::PointCloud<PointType>>, kMatchBufferCount> laserCloudNonFeature_for_match;

private:
    int laserCloudCenWidth;
    int laserCloudCenHeight;
    int laserCloudCenDepth;

    int laserCloudCenWidth_last;
    int laserCloudCenHeight_last;
    int laserCloudCenDepth_last;
    std::array<int, kMatchBufferCount> laserCloudCenWidth_last_buf;
    std::array<int, kMatchBufferCount> laserCloudCenHeight_last_buf;
    std::array<int, kMatchBufferCount> laserCloudCenDepth_last_buf;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudNonFeatureArray;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArrayStack;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArrayStack;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudNonFeatureArrayStack;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterNonFeature;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureFromMap;

    std::vector<pcl::KdTreeFLANN<PointType>::Ptr> laserCloudCornerKdMap;
    std::vector<pcl::KdTreeFLANN<PointType>::Ptr> laserCloudSurfKdMap;
    std::vector<pcl::KdTreeFLANN<PointType>::Ptr> laserCloudNonFeatureKdMap;

    std::array<std::vector<pcl::KdTreeFLANN<PointType>>, kMatchBufferCount> CornerKdMap_last;
    std::array<std::vector<pcl::KdTreeFLANN<PointType>>, kMatchBufferCount> SurfKdMap_last;
    std::array<std::vector<pcl::KdTreeFLANN<PointType>>, kMatchBufferCount> NonFeatureKdMap_last;

    std::vector<pcl::PointCloud<PointType>::Ptr> localCornerMap;
    std::vector<pcl::PointCloud<PointType>::Ptr> localSurfMap;
    std::vector<pcl::PointCloud<PointType>::Ptr> localNonFeatureMap;

    int localMapID = 0;

    int currentUpdatePos = 0;
    int estimatorPos = 0;
    int publish_idx = 0;
    int staging_idx = 1;
    std::array<std::atomic<int>, kMatchBufferCount> snapshot_ref_count;
    std::condition_variable snapshot_cv;

    void ReleaseSnapshot(int idx);
};

#endif //LIO_LIVOX_MAP_MANAGER_H
