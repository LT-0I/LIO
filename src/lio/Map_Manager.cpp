#include "MapManager/Map_Manager.h"
#include <fstream>
#include <algorithm>

MAP_MANAGER::MAP_MANAGER(const float& filter_corner,
                         const float& filter_surf,
                         const MapManagerConfig& config)
    : laserCloudWidth(std::max(8, config.width)),
      laserCloudHeight(std::max(3, config.height)),
      laserCloudDepth(std::max(8, config.depth)),
      laserCloudNum(laserCloudWidth * laserCloudHeight * laserCloudDepth),
      localMapWindowSize(std::max(10, config.local_window)),
      laserCloudCenWidth(laserCloudWidth / 2),
      laserCloudCenHeight(laserCloudHeight / 2),
      laserCloudCenDepth(laserCloudDepth / 2),
      laserCloudCenWidth_last(laserCloudCenWidth),
      laserCloudCenHeight_last(laserCloudCenHeight),
      laserCloudCenDepth_last(laserCloudCenDepth),
      laserCloudCornerArray(laserCloudNum),
      laserCloudSurfArray(laserCloudNum),
      laserCloudNonFeatureArray(laserCloudNum),
      laserCloudCornerArrayStack(laserCloudNum),
      laserCloudSurfArrayStack(laserCloudNum),
      laserCloudNonFeatureArrayStack(laserCloudNum),
      laserCloudCornerKdMap(laserCloudNum),
      laserCloudSurfKdMap(laserCloudNum),
      laserCloudNonFeatureKdMap(laserCloudNum),
      localCornerMap(localMapWindowSize),
      localSurfMap(localMapWindowSize),
      localNonFeatureMap(localMapWindowSize){

  laserCloudCenWidth_last_buf.fill(laserCloudCenWidth);
  laserCloudCenHeight_last_buf.fill(laserCloudCenHeight);
  laserCloudCenDepth_last_buf.fill(laserCloudCenDepth);

  for(auto& buf : laserCloudCorner_for_match){
    buf.resize(laserCloudNum);
  }
  for(auto& buf : laserCloudSurf_for_match){
    buf.resize(laserCloudNum);
  }
  for(auto& buf : laserCloudNonFeature_for_match){
    buf.resize(laserCloudNum);
  }
  for(auto& buf : CornerKdMap_last){
    buf.resize(laserCloudNum);
  }
  for(auto& buf : SurfKdMap_last){
    buf.resize(laserCloudNum);
  }
  for(auto& buf : NonFeatureKdMap_last){
    buf.resize(laserCloudNum);
  }

  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudNonFeatureArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudCornerArrayStack[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArrayStack[i].reset(new pcl::PointCloud<PointType>());
    laserCloudNonFeatureArrayStack[i].reset(new pcl::PointCloud<PointType>());

    laserCloudCornerKdMap[i].reset(new pcl::KdTreeFLANN<PointType>);
    laserCloudSurfKdMap[i].reset(new pcl::KdTreeFLANN<PointType>);
    laserCloudNonFeatureKdMap[i].reset(new pcl::KdTreeFLANN<PointType>);
  }
  for (int i = 0; i < localMapWindowSize; i++) {
    localCornerMap[i].reset(new pcl::PointCloud<PointType>());
    localSurfMap[i].reset(new pcl::PointCloud<PointType>());
    localNonFeatureMap[i].reset(new pcl::PointCloud<PointType>());
  }
  laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudNonFeatureFromMap.reset(new pcl::PointCloud<PointType>());
  downSizeFilterCorner.setLeafSize(0.4, 0.4, 0.4);
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
  downSizeFilterNonFeature.setLeafSize(0.4, 0.4, 0.4);

  snapshot_ref_count[0].store(0);
  snapshot_ref_count[1].store(0);
}

size_t MAP_MANAGER::ToIndex(int i, int j, int k) const {
  return i + laserCloudDepth * j + laserCloudDepth * laserCloudWidth * k;
}

/** \brief transform point pi to the MAP coordinate
 * \param[in] pi: point to be transformed
 * \param[in] po: point after transfomation
 * \param[in] _transformTobeMapped: transform matrix between pi and po
 */
void MAP_MANAGER::pointAssociateToMap(PointType const * const pi,
                                      PointType * const po,
                                      const Eigen::Matrix4d& _transformTobeMapped){
        Eigen::Vector3d pin, pout;
        pin.x() = pi->x;
        pin.y() = pi->y;
        pin.z() = pi->z;
        pout = _transformTobeMapped.topLeftCorner(3,3) * pin + _transformTobeMapped.topRightCorner(3,1);
        po->x = pout.x();
        po->y = pout.y();
        po->z = pout.z();
        po->intensity = pi->intensity;
        po->normal_z = pi->normal_z;
      }
void MAP_MANAGER::featureAssociateToMap(const pcl::PointCloud<PointType>::Ptr& laserCloudCorner,
                                        const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
                                        const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeature,
                                        const pcl::PointCloud<PointType>::Ptr& laserCloudCornerToMap,
                                        const pcl::PointCloud<PointType>::Ptr& laserCloudSurfToMap,
                                        const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureToMap,
                                        const Eigen::Matrix4d& transformTobeMapped){

  int laserCloudCornerNum = laserCloudCorner->points.size();
  int laserCloudSurfNum = laserCloudSurf->points.size();
  int laserCloudNonFeatureNum = laserCloudNonFeature->points.size();
  PointType pointSel1,pointSel2,pointSel3;
  for (int i = 0; i < laserCloudCornerNum; i++) {
    pointAssociateToMap(&laserCloudCorner->points[i], &pointSel1, transformTobeMapped);
    laserCloudCornerToMap->push_back(pointSel1);
  }
  for (int i = 0; i < laserCloudSurfNum; i++) {
    pointAssociateToMap(&laserCloudSurf->points[i], &pointSel2, transformTobeMapped);
    laserCloudSurfToMap->push_back(pointSel2);
  }
  for (int i = 0; i < laserCloudNonFeatureNum; i++) {
    pointAssociateToMap(&laserCloudNonFeature->points[i], &pointSel3, transformTobeMapped);
    laserCloudNonFeatureToMap->push_back(pointSel3);
  }
  
}
/** \brief add new lidar points to the map
 * \param[in] laserCloudCornerStack: coner feature points that need to be added to map
 * \param[in] laserCloudSurfStack: surf feature points that need to be added to map
 * \param[in] transformTobeMapped: transform matrix of the lidar pose
 */
void MAP_MANAGER::MapIncrement(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerStack,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudSurfStack,
                               const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureStack,
                               const Eigen::Matrix4d& transformTobeMapped){
  
  clock_t t0,t1,t2,t3,t4,t5;
  t0 = clock();
  std::unique_lock<std::mutex> locker2(mtx_MapManager);
  const int write_idx = staging_idx;
  snapshot_cv.wait(locker2, [this, write_idx](){ return snapshot_ref_count[write_idx].load() == 0; });
  for(int i = 0; i < laserCloudNum; i++){
    CornerKdMap_last[write_idx][i] = *laserCloudCornerKdMap[i];
    SurfKdMap_last[write_idx][i] = *laserCloudSurfKdMap[i];
    NonFeatureKdMap_last[write_idx][i] = *laserCloudNonFeatureKdMap[i];
    laserCloudSurf_for_match[write_idx][i] = *laserCloudSurfArray[i];
    laserCloudCorner_for_match[write_idx][i] = *laserCloudCornerArray[i];
    laserCloudNonFeature_for_match[write_idx][i] = *laserCloudNonFeatureArray[i];
  }

  laserCloudCenWidth_last_buf[write_idx] = laserCloudCenWidth;
  laserCloudCenHeight_last_buf[write_idx] = laserCloudCenHeight;
  laserCloudCenDepth_last_buf[write_idx] = laserCloudCenDepth;

  publish_idx = write_idx;
  staging_idx = 1 - publish_idx;
  laserCloudCenWidth_last = laserCloudCenWidth_last_buf[publish_idx];
  laserCloudCenHeight_last = laserCloudCenHeight_last_buf[publish_idx];
  laserCloudCenDepth_last = laserCloudCenDepth_last_buf[publish_idx];

  locker2.unlock();
  snapshot_cv.notify_all();
  
  t1 = clock();
  MapMove(transformTobeMapped);

  t2 = clock();
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
  int laserCloudNonFeatureStackNum = laserCloudNonFeatureStack->points.size();
  std::vector<uint8_t> CornerChangeFlag(laserCloudNum, 0);
  std::vector<uint8_t> SurfChangeFlag(laserCloudNum, 0);
  std::vector<uint8_t> NonFeatureChangeFlag(laserCloudNum, 0);
  PointType pointSel;
  for (int i = 0; i < laserCloudCornerStackNum; i++) {

    pointSel = laserCloudCornerStack->points[i];

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenDepth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenHeight;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudDepth &&
        cubeJ >= 0 && cubeJ < laserCloudWidth &&
        cubeK >= 0 &&
        cubeK < laserCloudHeight) {
      size_t cubeInd = ToIndex(cubeI, cubeJ, cubeK);
      laserCloudCornerArray[cubeInd]->push_back(pointSel);
      CornerChangeFlag[cubeInd] = 1;
    }
  }

  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    pointSel = laserCloudSurfStack->points[i];
    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenDepth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenHeight;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudDepth &&
        cubeJ >= 0 && cubeJ < laserCloudWidth &&
        cubeK >= 0 && cubeK < laserCloudHeight) {
      size_t cubeInd = ToIndex(cubeI, cubeJ, cubeK);
      laserCloudSurfArray[cubeInd]->push_back(pointSel);
      SurfChangeFlag[cubeInd] = 1;
    }
  }

  for (int i = 0; i < laserCloudNonFeatureStackNum; i++) {
    pointSel = laserCloudNonFeatureStack->points[i];
    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenDepth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenHeight;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudDepth &&
        cubeJ >= 0 && cubeJ < laserCloudWidth &&
        cubeK >= 0 && cubeK < laserCloudHeight) {
      size_t cubeInd = ToIndex(cubeI, cubeJ, cubeK);
      laserCloudNonFeatureArray[cubeInd]->push_back(pointSel);
      NonFeatureChangeFlag[cubeInd] = 1;
    }
  }

  t3 = clock();
  
  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  laserCloudNonFeatureFromMap->clear();
  for(int i = 0; i < laserCloudNum; i++){
    if(CornerChangeFlag[i]){
      if(laserCloudCornerArray[i]->points.size() > 300){
        downSizeFilterCorner.setInputCloud(laserCloudCornerArray[i]);
        laserCloudCornerArrayStack[i]->clear();
        downSizeFilterCorner.filter(*laserCloudCornerArrayStack[i]);
        pcl::PointCloud<PointType>::Ptr tmp = laserCloudCornerArrayStack[i];
        laserCloudCornerArrayStack[i] = laserCloudCornerArray[i];
        laserCloudCornerArray[i] = tmp;
      }

      laserCloudCornerKdMap[i]->setInputCloud(laserCloudCornerArray[i]); 
      *laserCloudCornerFromMap += *laserCloudCornerKdMap[i]->getInputCloud();
    }

    if(SurfChangeFlag[i]){
      if(laserCloudSurfArray[i]->points.size() > 300){
        downSizeFilterSurf.setInputCloud(laserCloudSurfArray[i]);
        laserCloudSurfArrayStack[i]->clear();
        downSizeFilterSurf.filter(*laserCloudSurfArrayStack[i]);
        pcl::PointCloud<PointType>::Ptr tmp = laserCloudSurfArrayStack[i];
        laserCloudSurfArrayStack[i] = laserCloudSurfArray[i];
        laserCloudSurfArray[i] = tmp;
      }

      laserCloudSurfKdMap[i]->setInputCloud(laserCloudSurfArray[i]);
      *laserCloudSurfFromMap += *laserCloudSurfKdMap[i]->getInputCloud();
    }

    if(NonFeatureChangeFlag[i]){
      if(laserCloudNonFeatureArray[i]->points.size() > 300){
        downSizeFilterNonFeature.setInputCloud(laserCloudNonFeatureArray[i]);
        laserCloudNonFeatureArrayStack[i]->clear();
        downSizeFilterNonFeature.filter(*laserCloudNonFeatureArrayStack[i]);
        pcl::PointCloud<PointType>::Ptr tmp = laserCloudNonFeatureArrayStack[i];
        laserCloudNonFeatureArrayStack[i] = laserCloudNonFeatureArray[i];
        laserCloudNonFeatureArray[i] = tmp;
      }

      laserCloudNonFeatureKdMap[i]->setInputCloud(laserCloudNonFeatureArray[i]);
      *laserCloudNonFeatureFromMap += *laserCloudNonFeatureKdMap[i]->getInputCloud();
    }
      
  }

  t4 = clock();
  // Removed historical copy buffers to reduce memory footprint
  t5 = clock();

  currentUpdatePos ++;

}

/** \brief move the map index if need
 * \param[in] transformTobeMapped: transform matrix of the lidar pose
 */
void MAP_MANAGER::MapMove(const Eigen::Matrix4d& transformTobeMapped){
  const Eigen::Matrix3d transformTobeMapped_R = transformTobeMapped.topLeftCorner(3, 3);
  const Eigen::Vector3d transformTobeMapped_t = transformTobeMapped.topRightCorner(3, 1);

  PointType pointOnYAxis;
  pointOnYAxis.x = 0.0;
  pointOnYAxis.y = 0.0;
  pointOnYAxis.z = 10.0;

  pointAssociateToMap(&pointOnYAxis, &pointOnYAxis, transformTobeMapped);

  int centerCubeI = int((transformTobeMapped_t.x() + 25.0) / 50.0) + laserCloudCenDepth;
  int centerCubeJ = int((transformTobeMapped_t.y() + 25.0) / 50.0) + laserCloudCenWidth;
  int centerCubeK = int((transformTobeMapped_t.z() + 25.0) / 50.0) + laserCloudCenHeight;

  if (transformTobeMapped_t.x() + 25.0 < 0) centerCubeI--;
  if (transformTobeMapped_t.y() + 25.0 < 0) centerCubeJ--;
  if (transformTobeMapped_t.z() + 25.0 < 0) centerCubeK--;

  while (centerCubeI < 8) {
    for (int j = 0; j < laserCloudWidth; j++) {
      for (int k = 0; k < laserCloudHeight; k++) {
        int i = laserCloudDepth - 1;
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeCornerPointerKd =
                laserCloudCornerKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeSurfPointerKd =
                laserCloudSurfKdMap[ToIndex(i, j, k)];

        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeNonFeaturePointerKd =
                laserCloudNonFeatureKdMap[ToIndex(i, j, k)];

        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeNonFeaturePointer =
                laserCloudNonFeatureArray[ToIndex(i, j, k)];

        for (; i >= 1; i--) {
          const size_t index_a = ToIndex(i, j, k);
          const size_t index_b = ToIndex(i - 1, j, k);
          laserCloudCornerKdMap[index_a] = laserCloudCornerKdMap[index_b];
          laserCloudSurfKdMap[index_a] = laserCloudSurfKdMap[index_b];
          laserCloudNonFeatureKdMap[index_a] = laserCloudNonFeatureKdMap[index_b];

          laserCloudCornerArray[index_a] = laserCloudCornerArray[index_b];
          laserCloudSurfArray[index_a] = laserCloudSurfArray[index_b];
          laserCloudNonFeatureArray[index_a] = laserCloudNonFeatureArray[index_b];
        }
        //此时i已经移动至0
        laserCloudCornerKdMap[ToIndex(i, j, k)] = laserCloudCubeCornerPointerKd;
        laserCloudSurfKdMap[ToIndex(i, j, k)] = laserCloudCubeSurfPointerKd;
        laserCloudNonFeatureKdMap[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointerKd;

        laserCloudCornerArray[ToIndex(i, j, k)] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[ToIndex(i, j, k)] = laserCloudCubeSurfPointer;
        laserCloudNonFeatureArray[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
        laserCloudCubeNonFeaturePointer->clear();
      }
    }

    centerCubeI++;
    laserCloudCenDepth++;
  }

  while (centerCubeI >= laserCloudDepth - 8) {
    for (int j = 0; j < laserCloudWidth; j++) {
      for (int k = 0; k < laserCloudHeight; k++) {
        int i = 0;
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeCornerPointerKd =
                laserCloudCornerKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeSurfPointerKd =
                laserCloudSurfKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeNonFeaturePointerKd =
                laserCloudNonFeatureKdMap[ToIndex(i, j, k)];

        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeNonFeaturePointer =
                laserCloudNonFeatureArray[ToIndex(i, j, k)];
        
        for (; i < laserCloudDepth - 1; i++) {
          const size_t index_a = ToIndex(i, j, k);
          const size_t index_b = ToIndex(i + 1, j, k);
          laserCloudCornerKdMap[index_a] = laserCloudCornerKdMap[index_b];
          laserCloudSurfKdMap[index_a] = laserCloudSurfKdMap[index_b];
          laserCloudNonFeatureKdMap[index_a] = laserCloudNonFeatureKdMap[index_b];

          laserCloudCornerArray[index_a] = laserCloudCornerArray[index_b];
          laserCloudSurfArray[index_a] = laserCloudSurfArray[index_b];
          laserCloudNonFeatureArray[index_a] = laserCloudNonFeatureArray[index_b];
        }
        laserCloudCornerKdMap[ToIndex(i, j, k)] = laserCloudCubeCornerPointerKd;
        laserCloudSurfKdMap[ToIndex(i, j, k)] = laserCloudCubeSurfPointerKd;
        laserCloudNonFeatureKdMap[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointerKd;

        laserCloudCornerArray[ToIndex(i, j, k)] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[ToIndex(i, j, k)] = laserCloudCubeSurfPointer;
        laserCloudNonFeatureArray[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
        laserCloudCubeNonFeaturePointer->clear();
      }
    }

    centerCubeI--;
    laserCloudCenDepth--;
  }

  while (centerCubeJ < 8) {
    for (int i = 0; i < laserCloudDepth; i++) {
      for (int k = 0; k < laserCloudHeight; k++) {
        int j = laserCloudWidth - 1;
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeCornerPointerKd =
                laserCloudCornerKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeSurfPointerKd =
                laserCloudSurfKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeNonFeaturePointerKd =
                laserCloudNonFeatureKdMap[ToIndex(i, j, k)];

        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeNonFeaturePointer =
                laserCloudNonFeatureArray[ToIndex(i, j, k)];
        for (; j >= 1; j--) {
          const size_t index_a = ToIndex(i, j, k);
          const size_t index_b = ToIndex(i, j - 1, k);
          laserCloudCornerKdMap[index_a] = laserCloudCornerKdMap[index_b];
          laserCloudSurfKdMap[index_a] = laserCloudSurfKdMap[index_b];
          laserCloudNonFeatureKdMap[index_a] = laserCloudNonFeatureKdMap[index_b];

          laserCloudCornerArray[index_a] = laserCloudCornerArray[index_b];
          laserCloudSurfArray[index_a] = laserCloudSurfArray[index_b];
          laserCloudNonFeatureArray[index_a] = laserCloudNonFeatureArray[index_b];
        }
        laserCloudCornerKdMap[ToIndex(i, j, k)] = laserCloudCubeCornerPointerKd;
        laserCloudSurfKdMap[ToIndex(i, j, k)] = laserCloudCubeSurfPointerKd;
        laserCloudNonFeatureKdMap[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointerKd;

        laserCloudCornerArray[ToIndex(i, j, k)] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[ToIndex(i, j, k)] = laserCloudCubeSurfPointer;
        laserCloudNonFeatureArray[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
        laserCloudCubeNonFeaturePointer->clear();
      }
    }

    centerCubeJ++;
    laserCloudCenWidth++;
  }

  while (centerCubeJ >= laserCloudWidth - 8) {
    for (int i = 0; i < laserCloudDepth; i++) {
      for (int k = 0; k < laserCloudHeight; k++) {
        int j = 0;
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeCornerPointerKd =
                laserCloudCornerKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeSurfPointerKd =
                laserCloudSurfKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeNonFeaturePointerKd =
                laserCloudNonFeatureKdMap[ToIndex(i, j, k)];

        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeNonFeaturePointer =
                laserCloudNonFeatureArray[ToIndex(i, j, k)];
        for (; j < laserCloudWidth - 1; j++) {
          const size_t index_a = ToIndex(i, j, k);
          const size_t index_b = ToIndex(i, j + 1, k);
          laserCloudCornerKdMap[index_a] = laserCloudCornerKdMap[index_b];
          laserCloudSurfKdMap[index_a] = laserCloudSurfKdMap[index_b];
          laserCloudNonFeatureKdMap[index_a] = laserCloudNonFeatureKdMap[index_b];

          laserCloudCornerArray[index_a] = laserCloudCornerArray[index_b];
          laserCloudSurfArray[index_a] = laserCloudSurfArray[index_b];
          laserCloudNonFeatureArray[index_a] = laserCloudNonFeatureArray[index_b];
        }
        laserCloudCornerKdMap[ToIndex(i, j, k)] = laserCloudCubeCornerPointerKd;
        laserCloudSurfKdMap[ToIndex(i, j, k)] = laserCloudCubeSurfPointerKd;
        laserCloudNonFeatureKdMap[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointerKd;

        laserCloudCornerArray[ToIndex(i, j, k)] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[ToIndex(i, j, k)] = laserCloudCubeSurfPointer;
        laserCloudNonFeatureArray[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
        laserCloudCubeNonFeaturePointer->clear();
      }
    }

    centerCubeJ--;
    laserCloudCenWidth--;
  }

  while (centerCubeK < 8) {
    for (int i = 0; i < laserCloudDepth; i++) {
      for (int j = 0; j < laserCloudWidth; j++) {
        int k = laserCloudHeight - 1;
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeCornerPointerKd =
                laserCloudCornerKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeSurfPointerKd =
                laserCloudSurfKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeNonFeaturePointerKd =
                laserCloudNonFeatureKdMap[ToIndex(i, j, k)];

        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeNonFeaturePointer =
                laserCloudNonFeatureArray[ToIndex(i, j, k)];
        for (; k >= 1; k--) {
          const size_t index_a = ToIndex(i, j, k);
          const size_t index_b = ToIndex(i, j, k - 1);
          laserCloudCornerKdMap[index_a] = laserCloudCornerKdMap[index_b];
          laserCloudSurfKdMap[index_a] = laserCloudSurfKdMap[index_b];
          laserCloudNonFeatureKdMap[index_a] = laserCloudNonFeatureKdMap[index_b];

          laserCloudCornerArray[index_a] = laserCloudCornerArray[index_b];
          laserCloudSurfArray[index_a] = laserCloudSurfArray[index_b];
          laserCloudNonFeatureArray[index_a] = laserCloudNonFeatureArray[index_b];
        }
        laserCloudCornerKdMap[ToIndex(i, j, k)] = laserCloudCubeCornerPointerKd;
        laserCloudSurfKdMap[ToIndex(i, j, k)] = laserCloudCubeSurfPointerKd;
        laserCloudNonFeatureKdMap[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointerKd;

        laserCloudCornerArray[ToIndex(i, j, k)] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[ToIndex(i, j, k)] = laserCloudCubeSurfPointer;
        laserCloudNonFeatureArray[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
        laserCloudCubeNonFeaturePointer->clear();
      }
    }

    centerCubeK++;
    laserCloudCenHeight++;
  }

  while (centerCubeK >= laserCloudHeight - 8) {
    for (int i = 0; i < laserCloudDepth; i++) {
      for (int j = 0; j < laserCloudWidth; j++) {
        int k = 0;
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeCornerPointerKd =
                laserCloudCornerKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeSurfPointerKd =
                laserCloudSurfKdMap[ToIndex(i, j, k)];
        pcl::KdTreeFLANN<PointType>::Ptr laserCloudCubeNonFeaturePointerKd =
                laserCloudNonFeatureKdMap[ToIndex(i, j, k)];

        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[ToIndex(i, j, k)];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeNonFeaturePointer =
                laserCloudNonFeatureArray[ToIndex(i, j, k)];
        for (; k < laserCloudHeight - 1; k++) {
          const size_t index_a = ToIndex(i, j, k);
          const size_t index_b = ToIndex(i, j, k + 1);
          laserCloudCornerKdMap[index_a] = laserCloudCornerKdMap[index_b];
          laserCloudSurfKdMap[index_a] = laserCloudSurfKdMap[index_b];
          laserCloudNonFeatureKdMap[index_a] = laserCloudNonFeatureKdMap[index_b];

          laserCloudCornerArray[index_a] = laserCloudCornerArray[index_b];
          laserCloudSurfArray[index_a] = laserCloudSurfArray[index_b];
          laserCloudNonFeatureArray[index_a] = laserCloudNonFeatureArray[index_b];
        }
        laserCloudCornerKdMap[ToIndex(i, j, k)] = laserCloudCubeCornerPointerKd;
        laserCloudSurfKdMap[ToIndex(i, j, k)] = laserCloudCubeSurfPointerKd;
        laserCloudNonFeatureKdMap[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointerKd;

        laserCloudCornerArray[ToIndex(i, j, k)] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[ToIndex(i, j, k)] = laserCloudCubeSurfPointer;
        laserCloudNonFeatureArray[ToIndex(i, j, k)] = laserCloudCubeNonFeaturePointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
        laserCloudCubeNonFeaturePointer->clear();
      }
    }

    centerCubeK--;
    laserCloudCenHeight--;
  }

}

std::shared_ptr<MAP_MANAGER::MapSnapshot> MAP_MANAGER::AcquireSnapshot(){
  std::unique_lock<std::mutex> lock(mtx_MapManager);
  const int idx = publish_idx;
  snapshot_ref_count[idx]++;
  MapSnapshot* snapshot = new MapSnapshot{
    CornerKdMap_last[idx].data(),
    SurfKdMap_last[idx].data(),
    NonFeatureKdMap_last[idx].data(),
    laserCloudCorner_for_match[idx].data(),
    laserCloudSurf_for_match[idx].data(),
    laserCloudNonFeature_for_match[idx].data(),
    laserCloudCenWidth_last_buf[idx],
    laserCloudCenHeight_last_buf[idx],
    laserCloudCenDepth_last_buf[idx],
    idx
  };
  auto deleter = [this](MapSnapshot* snap){
    ReleaseSnapshot(snap->buffer_idx);
    delete snap;
  };
  return std::shared_ptr<MapSnapshot>(snapshot, deleter);
}

void MAP_MANAGER::ReleaseSnapshot(int idx){
  std::unique_lock<std::mutex> lock(mtx_MapManager);
  snapshot_ref_count[idx]--;
  if(snapshot_ref_count[idx] == 0){
    snapshot_cv.notify_all();
  }
}

size_t MAP_MANAGER::FindUsedCornerMap(const PointType *p,int a,int b, int c)
{
    int cubeI = int((p->x + 25.0) / 50.0) + c;
    int cubeJ = int((p->y + 25.0) / 50.0) + a;
    int cubeK = int((p->z + 25.0) / 50.0) + b;

    size_t cubeInd = 0;

    if (p->x + 25.0 < 0) cubeI--;
    if (p->y + 25.0 < 0) cubeJ--;
    if (p->z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudDepth &&
        cubeJ >= 0 && cubeJ < laserCloudWidth &&
        cubeK >= 0 && cubeK < laserCloudHeight) {
      cubeInd = ToIndex(cubeI, cubeJ, cubeK);
    }
    else{
      cubeInd = 5000;
    }

    return cubeInd;  
}
size_t MAP_MANAGER::FindUsedSurfMap(const PointType *p,int a,int b, int c)
{
    int cubeI = int((p->x + 25.0) / 50.0) + c;
    int cubeJ = int((p->y + 25.0) / 50.0) + a;
    int cubeK = int((p->z + 25.0) / 50.0) + b;

    size_t cubeInd = 0;

    if (p->x + 25.0 < 0) cubeI--;
    if (p->y + 25.0 < 0) cubeJ--;
    if (p->z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudDepth &&
        cubeJ >= 0 && cubeJ < laserCloudWidth &&
        cubeK >= 0 && cubeK < laserCloudHeight) {
      cubeInd = ToIndex(cubeI, cubeJ, cubeK);
    }
    else{
      cubeInd = 5000;
    }

    return cubeInd;
}

size_t MAP_MANAGER::FindUsedNonFeatureMap(const PointType *p,int a,int b, int c)
{
    int cubeI = int((p->x + 25.0) / 50.0) + c;
    int cubeJ = int((p->y + 25.0) / 50.0) + a;
    int cubeK = int((p->z + 25.0) / 50.0) + b;

    size_t cubeInd = 0;

    if (p->x + 25.0 < 0) cubeI--;
    if (p->y + 25.0 < 0) cubeJ--;
    if (p->z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudDepth &&
        cubeJ >= 0 && cubeJ < laserCloudWidth &&
        cubeK >= 0 && cubeK < laserCloudHeight) {
      cubeInd = ToIndex(cubeI, cubeJ, cubeK);
    }
    else{
      cubeInd = 5000;
    }

    return cubeInd; 
}
