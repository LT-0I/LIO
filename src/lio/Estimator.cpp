#include "Estimator/Estimator.h"
#include <omp.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

Estimator::Estimator(const float& filter_corner,
                     const float& filter_surf,
                     const MapManagerConfig& map_config,
                     const EstimatorResidualConfig& residual_config,
                     bool log_module_timing)
: residual_config_(residual_config),
  log_module_timing_(log_module_timing),
  voxelCornerLocal_(map_config.voxel_index_resolution),
  voxelSurfLocal_(map_config.voxel_index_resolution),
  voxelNonLocal_(map_config.voxel_index_resolution){
  corner_eigen_ratio_ = residual_config_.adaptive_corner.default_eigen_ratio;
  localBoxForward = map_config.local_box_forward;
  localBoxBackward = map_config.local_box_backward;
  localBoxSide = map_config.local_box_side;
  localBoxVertical = map_config.local_box_vertical;
  map_skip_frame = std::max(1, map_config.map_skip_frame);
  local_corner_max_points_ = std::max(0, map_config.local_corner_max_points);
  local_surf_max_points_ = std::max(0, map_config.local_surf_max_points);
  local_non_max_points_ = std::max(0, map_config.local_non_max_points);
  use_voxel_index_local_ = map_config.use_voxel_index_local;
  voxel_index_resolution_ = map_config.voxel_index_resolution;
  runtime_corner_limit_ = std::max(1, residual_config_.max_corner_residuals);
  runtime_surf_limit_ = std::max(1, residual_config_.max_surf_residuals);
  runtime_non_limit_ = std::max(1, residual_config_.max_non_residuals);
  laserCloudCornerFromLocal.reset(new pcl::PointCloud<PointType>);
  laserCloudSurfFromLocal.reset(new pcl::PointCloud<PointType>);
  laserCloudNonFeatureFromLocal.reset(new pcl::PointCloud<PointType>);
  laserCloudCornerLast.resize(SLIDEWINDOWSIZE);
  for(auto& p:laserCloudCornerLast)
    p.reset(new pcl::PointCloud<PointType>);
  laserCloudSurfLast.resize(SLIDEWINDOWSIZE);
  for(auto& p:laserCloudSurfLast)
    p.reset(new pcl::PointCloud<PointType>);
  laserCloudNonFeatureLast.resize(SLIDEWINDOWSIZE);
  for(auto& p:laserCloudNonFeatureLast)
    p.reset(new pcl::PointCloud<PointType>);
  laserCloudCornerStack.resize(SLIDEWINDOWSIZE);
  for(auto& p:laserCloudCornerStack)
    p.reset(new pcl::PointCloud<PointType>);
  laserCloudSurfStack.resize(SLIDEWINDOWSIZE);
  for(auto& p:laserCloudSurfStack)
    p.reset(new pcl::PointCloud<PointType>);
  laserCloudNonFeatureStack.resize(SLIDEWINDOWSIZE);
  for(auto& p:laserCloudNonFeatureStack)
    p.reset(new pcl::PointCloud<PointType>);
  laserCloudCornerForMap.reset(new pcl::PointCloud<PointType>);
  laserCloudSurfForMap.reset(new pcl::PointCloud<PointType>);
  laserCloudNonFeatureForMap.reset(new pcl::PointCloud<PointType>);
  transformForMap.setIdentity();
  kdtreeCornerFromLocal.reset(new pcl::KdTreeFLANN<PointType>);
  kdtreeSurfFromLocal.reset(new pcl::KdTreeFLANN<PointType>);
  kdtreeNonFeatureFromLocal.reset(new pcl::KdTreeFLANN<PointType>);
  std::fill(std::begin(localFrameStamp), std::end(localFrameStamp), 0L);

  std::fill(std::begin(CornerKdMap), std::end(CornerKdMap), nullptr);
  std::fill(std::begin(SurfKdMap), std::end(SurfKdMap), nullptr);
  std::fill(std::begin(NonFeatureKdMap), std::end(NonFeatureKdMap), nullptr);
  std::fill(std::begin(GlobalSurfMap), std::end(GlobalSurfMap), nullptr);
  std::fill(std::begin(GlobalCornerMap), std::end(GlobalCornerMap), nullptr);
  std::fill(std::begin(GlobalNonFeatureMap), std::end(GlobalNonFeatureMap), nullptr);

  for(int i = 0; i < localMapWindowSize; i++){
    localCornerMap[i].reset(new pcl::PointCloud<PointType>);
    localSurfMap[i].reset(new pcl::PointCloud<PointType>);
    localNonFeatureMap[i].reset(new pcl::PointCloud<PointType>);
  }

  downSizeFilterCorner.setLeafSize(filter_corner, filter_corner, filter_corner);
  downSizeFilterSurf.setLeafSize(filter_surf, filter_surf, filter_surf);
  downSizeFilterNonFeature.setLeafSize(0.4, 0.4, 0.4);
  map_manager = new MAP_MANAGER(filter_corner, filter_surf, map_config);
  threadMap = std::thread(&Estimator::threadMapIncrement, this);
}

Estimator::~Estimator(){
  delete map_manager;
}

[[noreturn]] void Estimator::threadMapIncrement(){
  pcl::PointCloud<PointType>::Ptr laserCloudCorner(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr laserCloudSurf(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr laserCloudNonFeature(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr laserCloudCorner_to_map(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr laserCloudSurf_to_map(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr laserCloudNonFeature_to_map(new pcl::PointCloud<PointType>);
  Eigen::Matrix4d transform;
  while(true){
    std::unique_lock<std::mutex> locker(mtx_Map);
    if(!laserCloudCornerForMap->empty()){

      map_update_ID ++;

      map_manager->featureAssociateToMap(laserCloudCornerForMap,
                                         laserCloudSurfForMap,
                                         laserCloudNonFeatureForMap,
                                         laserCloudCorner,
                                         laserCloudSurf,
                                         laserCloudNonFeature,
                                         transformForMap);
      laserCloudCornerForMap->clear();
      laserCloudSurfForMap->clear();
      laserCloudNonFeatureForMap->clear();
      transform = transformForMap;
      locker.unlock();

      *laserCloudCorner_to_map += *laserCloudCorner;
      *laserCloudSurf_to_map += *laserCloudSurf;
      *laserCloudNonFeature_to_map += *laserCloudNonFeature;

      laserCloudCorner->clear();
      laserCloudSurf->clear();
      laserCloudNonFeature->clear();

      if(map_update_ID % map_skip_frame == 0){
        map_manager->MapIncrement(laserCloudCorner_to_map, 
                                  laserCloudSurf_to_map, 
                                  laserCloudNonFeature_to_map,
                                  transform);

        laserCloudCorner_to_map->clear();
        laserCloudSurf_to_map->clear();
        laserCloudNonFeature_to_map->clear();
      }
      
    }else
      locker.unlock();

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void Estimator::processPointToLine(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
                                   std::vector<FeatureLine>& vLineFeatures,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudCorner,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudCornerLocal,
                                   const pcl::KdTreeFLANN<PointType>::Ptr& kdtreeLocal,
                                   const Eigen::Matrix4d& exTlb,
                                   const Eigen::Matrix4d& m4d,
                                   FeatureBuildStats* stats){

  Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  Tbl.topLeftCorner(3,3) = exTlb.topLeftCorner(3,3).transpose();
  Tbl.topRightCorner(3,1) = -1.0 * Tbl.topLeftCorner(3,3) * exTlb.topRightCorner(3,1);
  if(!vLineFeatures.empty()){
    for(const auto& l : vLineFeatures){
      edges.emplace_back(Cost_NavState_IMU_Line::Create(l.pointOri,
                                                        l.lineP1,
                                                        l.lineP2,
                                                        Tbl,
                                                        Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
    }
    return;
  }
  if(stats){
    stats->points_total = laserCloudCorner->points.size();
  }
  PointType _pointOri, _pointSel, _coeff;
  std::vector<int> _pointSearchInd;
  std::vector<float> _pointSearchSqDis;
  std::vector<int> _pointSearchInd2;
  std::vector<float> _pointSearchSqDis2;

  Eigen::Matrix< double, 3, 3 > _matA1;
  _matA1.setZero();

  int laserCloudCornerStackNum = laserCloudCorner->points.size();
  pcl::PointCloud<PointType>::Ptr kd_pointcloud(new pcl::PointCloud<PointType>);
  int debug_num1 = 0;
  int debug_num2 = 0;
  int debug_num12 = 0;
  int debug_num22 = 0;
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    _pointOri = laserCloudCorner->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);
    int id = map_manager->FindUsedCornerMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);

    if(id == 5000){
      if(stats) stats->global_region_skipped++;
      continue;
    }

    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

    const auto* globalCornerMap = GlobalCornerMap[id];
    const auto* globalCornerKd = CornerKdMap[id];
    if(globalCornerMap && globalCornerKd && globalCornerMap->points.size() > 100) {
      globalCornerKd->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);
      
      if (_pointSearchSqDis[4] < thres_dist) {
        if(stats) stats->global_kd_success++;

        debug_num1 ++;
      float cx = 0;
      float cy = 0;
      float cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += globalCornerMap->points[_pointSearchInd[j]].x;
          cy += globalCornerMap->points[_pointSearchInd[j]].y;
          cz += globalCornerMap->points[_pointSearchInd[j]].z;
      }
      cx /= 5;
      cy /= 5;
      cz /= 5;

      float a11 = 0;
      float a12 = 0;
      float a13 = 0;
      float a22 = 0;
      float a23 = 0;
      float a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax = globalCornerMap->points[_pointSearchInd[j]].x - cx;
          float ay = globalCornerMap->points[_pointSearchInd[j]].y - cy;
          float az = globalCornerMap->points[_pointSearchInd[j]].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      _matA1(0, 0) = a11;
      _matA1(0, 1) = a12;
      _matA1(0, 2) = a13;
      _matA1(1, 0) = a12;
      _matA1(1, 1) = a22;
      _matA1(1, 2) = a23;
      _matA1(2, 0) = a13;
      _matA1(2, 1) = a23;
      _matA1(2, 2) = a33;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

      if (saes.eigenvalues()[2] > corner_eigen_ratio_ * saes.eigenvalues()[1]) {
        if(stats) stats->global_eigen_pass++;
        debug_num12 ++;
        float x1 = cx + 0.1 * unit_direction[0];
        float y1 = cy + 0.1 * unit_direction[1];
        float z1 = cz + 0.1 * unit_direction[2];
        float x2 = cx - 0.1 * unit_direction[0];
        float y2 = cy - 0.1 * unit_direction[1];
        float z2 = cz - 0.1 * unit_direction[2];

        Eigen::Vector3d tripod1(x1, y1, z1);
        Eigen::Vector3d tripod2(x2, y2, z2);
        edges.emplace_back(Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                          tripod1,
                                                          tripod2,
                                                          Tbl,
                                                          Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
        vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                   tripod1,
                                   tripod2);
        vLineFeatures.back().from_global = true;
        vLineFeatures.back().ComputeError(m4d);

        continue;
      }
      else if(stats){
        stats->global_eigen_fail++;
      }

    }
    
    }

    if(laserCloudCornerLocal->points.size() > 20 ){
      kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
      if (_pointSearchSqDis2[4] < thres_dist) {
        if(stats) stats->local_kd_success++;

        debug_num2 ++;
        float cx = 0;
        float cy = 0;
        float cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += laserCloudCornerLocal->points[_pointSearchInd2[j]].x;
          cy += laserCloudCornerLocal->points[_pointSearchInd2[j]].y;
          cz += laserCloudCornerLocal->points[_pointSearchInd2[j]].z;
        }
        cx /= 5;
        cy /= 5;
        cz /= 5;

        float a11 = 0;
        float a12 = 0;
        float a13 = 0;
        float a22 = 0;
        float a23 = 0;
        float a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax = laserCloudCornerLocal->points[_pointSearchInd2[j]].x - cx;
          float ay = laserCloudCornerLocal->points[_pointSearchInd2[j]].y - cy;
          float az = laserCloudCornerLocal->points[_pointSearchInd2[j]].z - cz;

          a11 += ax * ax;
          a12 += ax * ay;
          a13 += ax * az;
          a22 += ay * ay;
          a23 += ay * az;
          a33 += az * az;
        }
        a11 /= 5;
        a12 /= 5;
        a13 /= 5;
        a22 /= 5;
        a23 /= 5;
        a33 /= 5;

        _matA1(0, 0) = a11;
        _matA1(0, 1) = a12;
        _matA1(0, 2) = a13;
        _matA1(1, 0) = a12;
        _matA1(1, 1) = a22;
        _matA1(1, 2) = a23;
        _matA1(2, 0) = a13;
        _matA1(2, 1) = a23;
        _matA1(2, 2) = a33;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

        if (saes.eigenvalues()[2] > corner_eigen_ratio_ * saes.eigenvalues()[1]) {
          if(stats) stats->local_eigen_pass++;
          debug_num22++;
          float x1 = cx + 0.1 * unit_direction[0];
          float y1 = cy + 0.1 * unit_direction[1];
          float z1 = cz + 0.1 * unit_direction[2];
          float x2 = cx - 0.1 * unit_direction[0];
          float y2 = cy - 0.1 * unit_direction[1];
          float z2 = cz - 0.1 * unit_direction[2];

          Eigen::Vector3d tripod1(x1, y1, z1);
          Eigen::Vector3d tripod2(x2, y2, z2);
          edges.emplace_back(Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                            tripod1,
                                                            tripod2,
                                                            Tbl,
                                                            Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
          vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    tripod1,
                                    tripod2);
          vLineFeatures.back().from_global = false;
          vLineFeatures.back().ComputeError(m4d);
        }
        else if(stats){
          stats->local_eigen_fail++;
        }
      }
    }
     
  }
}

void Estimator::processPointToPlan(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
                                   std::vector<FeaturePlan>& vPlanFeatures,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLocal,
                                   const pcl::KdTreeFLANN<PointType>::Ptr& kdtreeLocal,
                                   const Eigen::Matrix4d& exTlb,
                                   const Eigen::Matrix4d& m4d){
  Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  Tbl.topLeftCorner(3,3) = exTlb.topLeftCorner(3,3).transpose();
  Tbl.topRightCorner(3,1) = -1.0 * Tbl.topLeftCorner(3,3) * exTlb.topRightCorner(3,1);
  if(!vPlanFeatures.empty()){
    for(const auto& p : vPlanFeatures){
      edges.emplace_back(Cost_NavState_IMU_Plan::Create(p.pointOri,
                                                        p.pa,
                                                        p.pb,
                                                        p.pc,
                                                        p.pd,
                                                        Tbl,
                                                        Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
    }
    return;
  }
  PointType _pointOri, _pointSel, _coeff;
  std::vector<int> _pointSearchInd;
  std::vector<float> _pointSearchSqDis;
  std::vector<int> _pointSearchInd2;
  std::vector<float> _pointSearchSqDis2;

  Eigen::Matrix< double, 5, 3 > _matA0;
  _matA0.setZero();
  Eigen::Matrix< double, 5, 1 > _matB0;
  _matB0.setOnes();
  _matB0 *= -1;
  Eigen::Matrix< double, 3, 1 > _matX0;
  _matX0.setZero();
  int laserCloudSurfStackNum = laserCloudSurf->points.size();

  int debug_num1 = 0;
  int debug_num2 = 0;
  int debug_num12 = 0;
  int debug_num22 = 0;
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    _pointOri = laserCloudSurf->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);

    int id = map_manager->FindUsedSurfMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);

    if(id == 5000) continue;

    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

    const auto* globalSurfMap = GlobalSurfMap[id];
    const auto* globalSurfKd = SurfKdMap[id];
    if(globalSurfMap && globalSurfKd && globalSurfMap->points.size() > 50) {
      globalSurfKd->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);

      if (_pointSearchSqDis[4] < 1.0) {
        debug_num1 ++;
        for (int j = 0; j < 5; j++) {
          _matA0(j, 0) = globalSurfMap->points[_pointSearchInd[j]].x;
          _matA0(j, 1) = globalSurfMap->points[_pointSearchInd[j]].y;
          _matA0(j, 2) = globalSurfMap->points[_pointSearchInd[j]].z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

        float pa = _matX0(0, 0);
        float pb = _matX0(1, 0);
        float pc = _matX0(2, 0);
        float pd = 1.0f;

        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
        if (!std::isfinite(ps) || ps < 1e-6f) {
          continue;
        }
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (std::fabs(pa * globalSurfMap->points[_pointSearchInd[j]].x +
                        pb * globalSurfMap->points[_pointSearchInd[j]].y +
                        pc * globalSurfMap->points[_pointSearchInd[j]].z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
          debug_num12 ++;
          edges.emplace_back(Cost_NavState_IMU_Plan::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                            pa,
                                                            pb,
                                                            pc,
                                                            pd,
                                                            Tbl,
                                                            Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
          vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    pa,
                                    pb,
                                    pc,
                                    pd);
          vPlanFeatures.back().ComputeError(m4d);

          continue;
        }
        
      }
    }
    if(laserCloudSurfLocal->points.size() > 20 ){
    kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
    if (_pointSearchSqDis2[4] < 1.0) {
      debug_num2++;
      for (int j = 0; j < 5; j++) {
        _matA0(j, 0) = laserCloudSurfLocal->points[_pointSearchInd2[j]].x;
        _matA0(j, 1) = laserCloudSurfLocal->points[_pointSearchInd2[j]].y;
        _matA0(j, 2) = laserCloudSurfLocal->points[_pointSearchInd2[j]].z;
      }
      _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

      float pa = _matX0(0, 0);
      float pb = _matX0(1, 0);
      float pc = _matX0(2, 0);
      float pd = 1;

      float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (std::fabs(pa * laserCloudSurfLocal->points[_pointSearchInd2[j]].x +
                      pb * laserCloudSurfLocal->points[_pointSearchInd2[j]].y +
                      pc * laserCloudSurfLocal->points[_pointSearchInd2[j]].z + pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        debug_num22 ++;
        edges.emplace_back(Cost_NavState_IMU_Plan::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                          pa,
                                                          pb,
                                                          pc,
                                                          pd,
                                                          Tbl,
                                                          Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
        vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                  pa,
                                  pb,
                                  pc,
                                  pd);
        vPlanFeatures.back().ComputeError(m4d);
      }
    }
  }

  }

}

void Estimator::processPointToPlanVec(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
                                   std::vector<FeaturePlanVec>& vPlanFeatures,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLocal,
                                   const pcl::KdTreeFLANN<PointType>::Ptr& kdtreeLocal,
                                   const Eigen::Matrix4d& exTlb,
                                   const Eigen::Matrix4d& m4d){
  Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  Tbl.topLeftCorner(3,3) = exTlb.topLeftCorner(3,3).transpose();
  Tbl.topRightCorner(3,1) = -1.0 * Tbl.topLeftCorner(3,3) * exTlb.topRightCorner(3,1);
  if(!vPlanFeatures.empty()){
    for(const auto& p : vPlanFeatures){
      edges.emplace_back(Cost_NavState_IMU_Plan_Vec::Create(p.pointOri,
                                                            p.pointProj,
                                                            Tbl,
                                                            p.sqrt_info));
    }
    return;
  }
  PointType _pointOri, _pointSel, _coeff;
  std::vector<int> _pointSearchInd;
  std::vector<float> _pointSearchSqDis;
  std::vector<int> _pointSearchInd2;
  std::vector<float> _pointSearchSqDis2;

  Eigen::Matrix< double, 5, 3 > _matA0;
  _matA0.setZero();
  Eigen::Matrix< double, 5, 1 > _matB0;
  _matB0.setOnes();
  _matB0 *= -1;
  Eigen::Matrix< double, 3, 1 > _matX0;
  _matX0.setZero();
  int laserCloudSurfStackNum = laserCloudSurf->points.size();

  int debug_num1 = 0;
  int debug_num2 = 0;
  int debug_num12 = 0;
  int debug_num22 = 0;
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    _pointOri = laserCloudSurf->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);

    int id = map_manager->FindUsedSurfMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);

    if(id == 5000) continue;

    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

    const auto* globalSurfMapPlanVec = GlobalSurfMap[id];
    const auto* globalSurfKdPlanVec = SurfKdMap[id];
    if(globalSurfMapPlanVec && globalSurfKdPlanVec && globalSurfMapPlanVec->points.size() > 50) {
      globalSurfKdPlanVec->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);

      if (_pointSearchSqDis[4] < thres_dist) {
        debug_num1 ++;
        for (int j = 0; j < 5; j++) {
          _matA0(j, 0) = globalSurfMapPlanVec->points[_pointSearchInd[j]].x;
          _matA0(j, 1) = globalSurfMapPlanVec->points[_pointSearchInd[j]].y;
          _matA0(j, 2) = globalSurfMapPlanVec->points[_pointSearchInd[j]].z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

        float pa = _matX0(0, 0);
        float pb = _matX0(1, 0);
        float pc = _matX0(2, 0);
        float pd = 1.0f;

        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
        if (!std::isfinite(ps) || ps < 1e-6f) {
          continue;
        }
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (std::fabs(pa * globalSurfMapPlanVec->points[_pointSearchInd[j]].x +
                        pb * globalSurfMapPlanVec->points[_pointSearchInd[j]].y +
                        pc * globalSurfMapPlanVec->points[_pointSearchInd[j]].z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
          debug_num12 ++;
          double dist = pa * _pointSel.x +
                        pb * _pointSel.y +
                        pc * _pointSel.z + pd;
          Eigen::Vector3d omega(pa, pb, pc);
          Eigen::Vector3d point_proj = Eigen::Vector3d(_pointSel.x,_pointSel.y,_pointSel.z) - (dist * omega);
          Eigen::Vector3d e1(1, 0, 0);
          Eigen::Matrix3d J = e1 * omega.transpose();
          Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
          Eigen::Matrix3d R_svd = svd.matrixV() * svd.matrixU().transpose();
          Eigen::Matrix3d info = (1.0/IMUIntegrator::lidar_m) * Eigen::Matrix3d::Identity();
          info(1, 1) *= plan_weight_tan;
          info(2, 2) *= plan_weight_tan;
          Eigen::Matrix3d sqrt_info = info * R_svd.transpose();

          edges.emplace_back(Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                                point_proj,
                                                                Tbl,
                                                                sqrt_info));
          vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                     point_proj,
                                     sqrt_info);
          vPlanFeatures.back().ComputeError(m4d);

          continue;
        }
        
      }
    }


    if(laserCloudSurfLocal->points.size() > 20 ){
    kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
    if (_pointSearchSqDis2[4] < thres_dist) {
      debug_num2++;
      for (int j = 0; j < 5; j++) {
        _matA0(j, 0) = laserCloudSurfLocal->points[_pointSearchInd2[j]].x;
        _matA0(j, 1) = laserCloudSurfLocal->points[_pointSearchInd2[j]].y;
        _matA0(j, 2) = laserCloudSurfLocal->points[_pointSearchInd2[j]].z;
      }
      _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

      float pa = _matX0(0, 0);
      float pb = _matX0(1, 0);
      float pc = _matX0(2, 0);
      float pd = 1;

      float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (std::fabs(pa * laserCloudSurfLocal->points[_pointSearchInd2[j]].x +
                      pb * laserCloudSurfLocal->points[_pointSearchInd2[j]].y +
                      pc * laserCloudSurfLocal->points[_pointSearchInd2[j]].z + pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        debug_num22 ++;
        double dist = pa * _pointSel.x +
                      pb * _pointSel.y +
                      pc * _pointSel.z + pd;
        Eigen::Vector3d omega(pa, pb, pc);
        Eigen::Vector3d point_proj = Eigen::Vector3d(_pointSel.x,_pointSel.y,_pointSel.z) - (dist * omega);
        Eigen::Vector3d e1(1, 0, 0);
        Eigen::Matrix3d J = e1 * omega.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d R_svd = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Matrix3d info = (1.0/IMUIntegrator::lidar_m) * Eigen::Matrix3d::Identity();
        info(1, 1) *= plan_weight_tan;
        info(2, 2) *= plan_weight_tan;
        Eigen::Matrix3d sqrt_info = info * R_svd.transpose();

        edges.emplace_back(Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                              point_proj,
                                                              Tbl,
                                                              sqrt_info));
        vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    point_proj,
                                    sqrt_info);
        vPlanFeatures.back().ComputeError(m4d);
      }
    }
  }

  }

}


void Estimator::processNonFeatureICP(std::vector<std::unique_ptr<ceres::CostFunction>>& edges,
                                     std::vector<FeatureNon>& vNonFeatures,
                                     const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeature,
                                     const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureLocal,
                                     const pcl::KdTreeFLANN<PointType>::Ptr& kdtreeLocal,
                                     const Eigen::Matrix4d& exTlb,
                                     const Eigen::Matrix4d& m4d){
  Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  Tbl.topLeftCorner(3,3) = exTlb.topLeftCorner(3,3).transpose();
  Tbl.topRightCorner(3,1) = -1.0 * Tbl.topLeftCorner(3,3) * exTlb.topRightCorner(3,1);
  if(!vNonFeatures.empty()){
    for(const auto& p : vNonFeatures){
      edges.emplace_back(Cost_NonFeature_ICP::Create(p.pointOri,
                                                     p.pa,
                                                     p.pb,
                                                     p.pc,
                                                     p.pd,
                                                     Tbl,
                                                     Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
    }
    return;
  }

  PointType _pointOri, _pointSel, _coeff;
  std::vector<int> _pointSearchInd;
  std::vector<float> _pointSearchSqDis;
  std::vector<int> _pointSearchInd2;
  std::vector<float> _pointSearchSqDis2;

  Eigen::Matrix< double, 5, 3 > _matA0;
  _matA0.setZero();
  Eigen::Matrix< double, 5, 1 > _matB0;
  _matB0.setOnes();
  _matB0 *= -1;
  Eigen::Matrix< double, 3, 1 > _matX0;
  _matX0.setZero();

  int laserCloudNonFeatureStackNum = laserCloudNonFeature->points.size();
  for (int i = 0; i < laserCloudNonFeatureStackNum; i++) {
    _pointOri = laserCloudNonFeature->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);
    int id = map_manager->FindUsedNonFeatureMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);

    if(id == 5000) continue;

    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

    const auto* globalNonFeatureMap = GlobalNonFeatureMap[id];
    const auto* globalNonFeatureKd = NonFeatureKdMap[id];
    if(globalNonFeatureMap && globalNonFeatureKd && globalNonFeatureMap->points.size() > 100) {
      globalNonFeatureKd->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);
      if (_pointSearchSqDis[4] < 1 * thres_dist) {
        for (int j = 0; j < 5; j++) {
          _matA0(j, 0) = globalNonFeatureMap->points[_pointSearchInd[j]].x;
          _matA0(j, 1) = globalNonFeatureMap->points[_pointSearchInd[j]].y;
          _matA0(j, 2) = globalNonFeatureMap->points[_pointSearchInd[j]].z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

        float pa = _matX0(0, 0);
        float pb = _matX0(1, 0);
        float pc = _matX0(2, 0);
        float pd = 1;

        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (std::fabs(pa * globalNonFeatureMap->points[_pointSearchInd[j]].x +
                        pb * globalNonFeatureMap->points[_pointSearchInd[j]].y +
                        pc * globalNonFeatureMap->points[_pointSearchInd[j]].z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if(planeValid) {

          edges.emplace_back(Cost_NonFeature_ICP::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                         pa,
                                                         pb,
                                                         pc,
                                                         pd,
                                                         Tbl,
                                                         Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
          vNonFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    pa,
                                    pb,
                                    pc,
                                    pd);
          vNonFeatures.back().ComputeError(m4d);

          continue;
        }
      }
    
    }

    if(laserCloudNonFeatureLocal->points.size() > 20 ){
      kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
      if (_pointSearchSqDis2[4] < thres_dist) {
        for (int j = 0; j < 5; j++) {
          _matA0(j, 0) = laserCloudNonFeatureLocal->points[_pointSearchInd2[j]].x;
          _matA0(j, 1) = laserCloudNonFeatureLocal->points[_pointSearchInd2[j]].y;
          _matA0(j, 2) = laserCloudNonFeatureLocal->points[_pointSearchInd2[j]].z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

        float pa = _matX0(0, 0);
        float pb = _matX0(1, 0);
        float pc = _matX0(2, 0);
        float pd = 1;

        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (std::fabs(pa * laserCloudNonFeatureLocal->points[_pointSearchInd2[j]].x +
                        pb * laserCloudNonFeatureLocal->points[_pointSearchInd2[j]].y +
                        pc * laserCloudNonFeatureLocal->points[_pointSearchInd2[j]].z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if(planeValid) {

          edges.emplace_back(Cost_NonFeature_ICP::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                         pa,
                                                         pb,
                                                         pc,
                                                         pd,
                                                         Tbl,
                                                         Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m)));
          vNonFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    pa,
                                    pb,
                                    pc,
                                    pd);
          vNonFeatures.back().ComputeError(m4d);
        }
      }
    }
  }

}


void Estimator::vector2double(const std::list<LidarFrame>& lidarFrameList){
  int i = 0;
  for(const auto& l : lidarFrameList){
    Eigen::Map<Eigen::Matrix<double, 6, 1>> PR(para_PR[i]);
    PR.segment<3>(0) = l.P;
    PR.segment<3>(3) = Sophus::SO3d(l.Q).log();

    Eigen::Map<Eigen::Matrix<double, 9, 1>> VBias(para_VBias[i]);
    VBias.segment<3>(0) = l.V;
    VBias.segment<3>(3) = l.bg;
    VBias.segment<3>(6) = l.ba;
    i++;
  }
}

void Estimator::double2vector(std::list<LidarFrame>& lidarFrameList){
  int i = 0;
  for(auto& l : lidarFrameList){
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> PR(para_PR[i]);
    Eigen::Map<const Eigen::Matrix<double, 9, 1>> VBias(para_VBias[i]);
    l.P = PR.segment<3>(0);
    l.Q = Sophus::SO3d::exp(PR.segment<3>(3)).unit_quaternion();
    l.V = VBias.segment<3>(0);
    l.bg = VBias.segment<3>(3);
    l.ba = VBias.segment<3>(6);
    i++;
  }
}

void Estimator::EstimateLidarPose(std::list<LidarFrame>& lidarFrameList,
                           const Eigen::Matrix4d& exTlb,
                           const Eigen::Vector3d& gravity,
                           nav_msgs::Odometry& debugInfo){
  
  Eigen::Matrix3d exRbl = exTlb.topLeftCorner(3,3).transpose();
  Eigen::Vector3d exPbl = -1.0 * exRbl * exTlb.topRightCorner(3,1);
  Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
  transformTobeMapped.topLeftCorner(3,3) = lidarFrameList.back().Q * exRbl;
  transformTobeMapped.topRightCorner(3,1) = lidarFrameList.back().Q * exPbl + lidarFrameList.back().P;

  int laserCloudCornerFromMapNum = map_manager->get_corner_map()->points.size();
  int laserCloudSurfFromMapNum = map_manager->get_surf_map()->points.size();
  int laserCloudCornerFromLocalNum = laserCloudCornerFromLocal->points.size();
  int laserCloudSurfFromLocalNum = laserCloudSurfFromLocal->points.size();
  int stack_count = 0;
  for(const auto& l : lidarFrameList){
    laserCloudCornerLast[stack_count]->clear();
    for(const auto& p : l.laserCloud->points){
      if(std::fabs(p.normal_z - 1.0) < 1e-5)
        laserCloudCornerLast[stack_count]->push_back(p);
    }
    laserCloudSurfLast[stack_count]->clear();
    for(const auto& p : l.laserCloud->points){
      if(std::fabs(p.normal_z - 2.0) < 1e-5)
        laserCloudSurfLast[stack_count]->push_back(p);
    }

    laserCloudNonFeatureLast[stack_count]->clear();
    for(const auto& p : l.laserCloud->points){
      if(std::fabs(p.normal_z - 3.0) < 1e-5)
        laserCloudNonFeatureLast[stack_count]->push_back(p);
    }

    laserCloudCornerStack[stack_count]->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast[stack_count]);
    downSizeFilterCorner.filter(*laserCloudCornerStack[stack_count]);

    laserCloudSurfStack[stack_count]->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast[stack_count]);
    downSizeFilterSurf.filter(*laserCloudSurfStack[stack_count]);

    laserCloudNonFeatureStack[stack_count]->clear();
    downSizeFilterNonFeature.setInputCloud(laserCloudNonFeatureLast[stack_count]);
    downSizeFilterNonFeature.filter(*laserCloudNonFeatureStack[stack_count]);
    stack_count++;
  }
  if ( ((laserCloudCornerFromMapNum >= 0 && laserCloudSurfFromMapNum > 100) || 
       (laserCloudCornerFromLocalNum >= 0 && laserCloudSurfFromLocalNum > 100))) {
    if(log_module_timing_){
      ROS_INFO("[Timing] Estimator::Estimate start %.6f", ros::Time::now().toSec());
    }
    Estimate(lidarFrameList, exTlb, gravity);
    if(log_module_timing_){
      ROS_INFO("[Timing] Estimator::Estimate end   %.6f", ros::Time::now().toSec());
    }
  }

  transformTobeMapped = Eigen::Matrix4d::Identity();
  transformTobeMapped.topLeftCorner(3,3) = lidarFrameList.front().Q * exRbl;
  transformTobeMapped.topRightCorner(3,1) = lidarFrameList.front().Q * exPbl + lidarFrameList.front().P;

  std::unique_lock<std::mutex> locker(mtx_Map);
  *laserCloudCornerForMap = *laserCloudCornerStack[0];
  *laserCloudSurfForMap = *laserCloudSurfStack[0];
  *laserCloudNonFeatureForMap = *laserCloudNonFeatureStack[0];
  transformForMap = transformTobeMapped;
  laserCloudCornerFromLocal->clear();
  laserCloudSurfFromLocal->clear();
  laserCloudNonFeatureFromLocal->clear();
  if(log_module_timing_){
    ROS_INFO("[Timing] MapManager update start %.6f", ros::Time::now().toSec());
  }
  MapIncrementLocal(laserCloudCornerForMap,laserCloudSurfForMap,laserCloudNonFeatureForMap,transformTobeMapped);
  if(log_module_timing_){
    ROS_INFO("[Timing] MapManager update end   %.6f", ros::Time::now().toSec());
  }
  locker.unlock();
}

void Estimator::Estimate(std::list<LidarFrame>& lidarFrameList,
                         const Eigen::Matrix4d& exTlb,
                         const Eigen::Vector3d& gravity){

  static uint32_t frame_count = 0;
  int num_corner_map = 0;
  int num_surf_map = 0;

  int windowSize = lidarFrameList.size();
  Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d exRbl = exTlb.topLeftCorner(3,3).transpose();
  Eigen::Vector3d exPbl = -1.0 * exRbl * exTlb.topRightCorner(3,1);

  if(laserCloudCornerFromLocal->empty() || laserCloudSurfFromLocal->empty()){
    ROS_WARN_STREAM_THROTTLE(1.0, "[Estimator] Skip optimization: corner=" <<
      laserCloudCornerFromLocal->size() << ", surf=" << laserCloudSurfFromLocal->size());
    return;
  }

  kdtreeCornerFromLocal->setInputCloud(laserCloudCornerFromLocal);
  kdtreeSurfFromLocal->setInputCloud(laserCloudSurfFromLocal);
  kdtreeNonFeatureFromLocal->setInputCloud(laserCloudNonFeatureFromLocal);

  if(log_module_timing_){
    ROS_INFO("[Timing] MapManager snapshot start %.6f", ros::Time::now().toSec());
  }
  auto map_snapshot = map_manager->AcquireSnapshot();
  if(log_module_timing_){
    ROS_INFO("[Timing] MapManager snapshot end   %.6f", ros::Time::now().toSec());
  }
  for(int i = 0; i < 4851; i++){
    CornerKdMap[i] = map_snapshot->corner_kd + i;
    SurfKdMap[i] = map_snapshot->surf_kd + i;
    NonFeatureKdMap[i] = map_snapshot->nonfeature_kd + i;

    GlobalSurfMap[i] = map_snapshot->surf_map + i;
    GlobalCornerMap[i] = map_snapshot->corner_map + i;
    GlobalNonFeatureMap[i] = map_snapshot->nonfeature_map + i;
  }
  laserCenWidth_last = map_snapshot->laserCenWidth_last;
  laserCenHeight_last = map_snapshot->laserCenHeight_last;
  laserCenDepth_last = map_snapshot->laserCenDepth_last;

  // store point to line features
  std::vector<std::vector<FeatureLine>> vLineFeatures(windowSize);
  for(auto& v : vLineFeatures){
    v.reserve(2000);
  }

  // store point to plan features
  std::vector<std::vector<FeaturePlanVec>> vPlanFeatures(windowSize);
  for(auto& v : vPlanFeatures){
    v.reserve(2000);
  }

  std::vector<std::vector<FeatureNon>> vNonFeatures(windowSize);
  for(auto& v : vNonFeatures){
    v.reserve(2000);
  }

  // 恢复到稳定版本的固定值
  // 稳定版本不使用动态 thres_dist 和 plan_weight_tan
  if(windowSize == SLIDEWINDOWSIZE) {
    plan_weight_tan = 0.0003;
    thres_dist = 1.0;
  } else {
    plan_weight_tan = 0.0;
    thres_dist = 25.0;
  }

  // excute optimize process
  const int max_iters = 5;
  for(int iterOpt=0; iterOpt<max_iters; ++iterOpt){

    vector2double(lidarFrameList);

    std::vector<std::vector<std::unique_ptr<ceres::CostFunction>>> edgesLine(windowSize);
    std::vector<std::vector<std::unique_ptr<ceres::CostFunction>>> edgesPlan(windowSize);
    std::vector<std::vector<std::unique_ptr<ceres::CostFunction>>> edgesNon(windowSize);
    std::vector<FeatureBuildStats> line_feature_stats;
    if(residual_config_.log_feature_counts){
      line_feature_stats.resize(windowSize);
    }

    // 恢复到稳定版本的并行策略: 按帧并行，而不是按特征类型并行
    // 稳定版本使用 std::thread 按帧分配工作，每个worker处理一个帧的所有特征类型
    // 新版本使用 OpenMP parallel sections 按特征类型并行，可能导致共享KD-Tree的竞争问题
    const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
    const int worker_count = std::min<int>(windowSize, std::max(1u, hw_threads));
    const int frames_per_worker = std::max(1, (windowSize + worker_count - 1) / worker_count);
    ros::WallTime residual_build_start = ros::WallTime::now();
    if(log_module_timing_){
      ROS_INFO("[Timing] Residual build start %.6f", ros::Time::now().toSec());
    }
    auto process_frames = [&](int start, int end){
      for(int f=start; f<end; ++f) {
        edgesLine[f].clear();
        edgesPlan[f].clear();
        edgesNon[f].clear();
        auto frame_curr = lidarFrameList.begin();
        std::advance(frame_curr, f);
        Eigen::Matrix4d localTransform = Eigen::Matrix4d::Identity();
        localTransform.topLeftCorner(3,3) = frame_curr->Q * exRbl;
        localTransform.topRightCorner(3,1) = frame_curr->Q * exPbl + frame_curr->P;

        FeatureBuildStats* stats_ptr = (residual_config_.log_feature_counts && iterOpt == 0 && !line_feature_stats.empty())
                                         ? &line_feature_stats[f]
                                         : nullptr;
        processPointToLine(edgesLine[f],
                           vLineFeatures[f],
                           laserCloudCornerStack[f],
                           laserCloudCornerFromLocal,
                           kdtreeCornerFromLocal,
                           exTlb,
                           localTransform,
                           stats_ptr);

        processPointToPlanVec(edgesPlan[f],
                              vPlanFeatures[f],
                              laserCloudSurfStack[f],
                              laserCloudSurfFromLocal,
                              kdtreeSurfFromLocal,
                              exTlb,
                              localTransform);

        processNonFeatureICP(edgesNon[f],
                             vNonFeatures[f],
                             laserCloudNonFeatureStack[f],
                             laserCloudNonFeatureFromLocal,
                             kdtreeNonFeatureFromLocal,
                             exTlb,
                             localTransform);
      }
    };

    std::vector<std::thread> residual_workers;
    residual_workers.reserve(worker_count);
    for(int w=0; w<worker_count; ++w){
      int start = w * frames_per_worker;
      if(start >= windowSize) break;
      int end = std::min(windowSize, start + frames_per_worker);
      residual_workers.emplace_back(process_frames, start, end);
    }
    for(auto& worker : residual_workers){
      worker.join();
    }
    if(log_module_timing_){
      ROS_INFO("[Timing] Residual build end   %.6f", ros::Time::now().toSec());
    }
    const double residual_build_ms =
        (ros::WallTime::now() - residual_build_start).toSec() * 1000.0;
    if(iterOpt == 0){
      last_residual_build_ms_ = residual_build_ms;
    }

    double avg_global_kd = residual_config_.adaptive_corner.low_feature_global_kd + 1.0;
    if(residual_config_.log_feature_counts && !line_feature_stats.empty()){
      long total_global_kd = 0;
      for(const auto& stats : line_feature_stats){
        total_global_kd += stats.global_kd_success;
      }
      avg_global_kd = static_cast<double>(total_global_kd) / static_cast<double>(line_feature_stats.size());
    }
    // 更新历史全局KD值，用于下一帧的动态搜索半径计算
    if(iterOpt == 0 && windowSize == SLIDEWINDOWSIZE){
      last_avg_global_kd_ = avg_global_kd;
    }

    // Loss Function: 恢复到稳定版本的逻辑
    // 稳定版本 (commit 5e0281584f3ac8eaaacc3f8d55c8b5667e0a302d) 中:
    // - 初始化阶段 (windowSize != SLIDEWINDOWSIZE): 使用 Huber Loss
    // - 正常运行阶段 (windowSize == SLIDEWINDOWSIZE): loss_function = nullptr
    // 使用 nullptr 意味着完整的 LiDAR 约束权重，不降权
    ceres::LossFunction* loss_function = nullptr;
    if(windowSize != SLIDEWINDOWSIZE){
      loss_function = new ceres::HuberLoss(0.1 / IMUIntegrator::lidar_m);
    }

    Eigen::Quaterniond q_before_opti = lidarFrameList.back().Q;
    Eigen::Vector3d t_before_opti = lidarFrameList.back().P;

    ceres::Solver::Summary summary;
    {
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);
      MarginalizationFactor* previous_margin_factor = nullptr;

      for(int i=0; i<windowSize; ++i) {
        problem.AddParameterBlock(para_PR[i], 6);
      }

      for(int i=0; i<windowSize; ++i)
        problem.AddParameterBlock(para_VBias[i], 9);

      // add IMU CostFunction
      for(int f=1; f<windowSize; ++f){
        auto frame_curr = lidarFrameList.begin();
        std::advance(frame_curr, f);
        problem.AddResidualBlock(Cost_NavState_PRV_Bias::Create(frame_curr->imuIntegrator,
                                                                const_cast<Eigen::Vector3d&>(gravity),
                                                                Eigen::LLT<Eigen::Matrix<double, 15, 15>>
                                                                        (frame_curr->imuIntegrator.GetCovariance().inverse())
                                                                        .matrixL().transpose()),
                                 nullptr,
                                 para_PR[f-1],
                                 para_VBias[f-1],
                                 para_PR[f],
                                 para_VBias[f]);
      }

      if (last_marginalization_info){
        previous_margin_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(previous_margin_factor, nullptr,
                                 last_marginalization_parameter_blocks);
      }

    int cntSurf = 0;
    int cntCorner = 0;
    int cntNon = 0;
    int candidateCorner = 0;
    int candidateSurf = 0;
    int candidateNon = 0;
    int candidateCornerGlobal = 0;
    int candidateCornerLocal = 0;
    int keptCornerGlobal = 0;
    int keptCornerLocal = 0;
    bool low_feature_mode = false;
    bool high_feature_mode = false;
    const auto& adaptive_corner = residual_config_.adaptive_corner;
    int adaptiveCornerLimit = runtime_corner_limit_;
    double eigen_ratio_target = adaptive_corner.default_eigen_ratio;
    if(adaptive_corner.enable){
      if(avg_global_kd < adaptive_corner.low_feature_global_kd){
        low_feature_mode = true;
        adaptiveCornerLimit = std::max(adaptiveCornerLimit, adaptive_corner.low_feature_min_keep);
        eigen_ratio_target = adaptive_corner.low_feature_eigen_ratio;
      }else if(avg_global_kd > adaptive_corner.high_feature_global_kd){
        high_feature_mode = true;
        adaptiveCornerLimit = std::min(adaptiveCornerLimit, adaptive_corner.high_feature_max_keep);
      }
    }
    adaptiveCornerLimit = std::max(1, adaptiveCornerLimit);
    corner_eigen_ratio_ = eigen_ratio_target;

    // 残差数量限制: 恢复到稳定版本的逻辑
    // 不再使用退化模式激进削减残差，让 LiDAR 约束保持完整
    const int maxCornerResidualsPerFrame = adaptiveCornerLimit;
    const int maxSurfResidualsPerFrame = std::max(1, runtime_surf_limit_);
    const int maxNonResidualsPerFrame = std::max(1, runtime_non_limit_);
    const double featureErrorThreshold = residual_config_.feature_error_threshold;
    auto shouldKeepFeature = [](int idx, int total, int kept, int limit) -> bool {
      if(limit <= 0 || total <= limit) return true;
      if(kept >= limit) return false;
      const int stride = (total + limit - 1) / limit;
      return (idx % stride) == 0;
    };
      if(windowSize == SLIDEWINDOWSIZE) {
        thres_dist = 1.0;
        if(iterOpt == 0){
          for(int f=0; f<windowSize; ++f){
          candidateCorner += static_cast<int>(edgesLine[f].size());
          candidateSurf += static_cast<int>(edgesPlan[f].size());
          candidateNon += static_cast<int>(edgesNon[f].size());
          const int totalCorner = edgesLine[f].size();
          int keptCornerLocalFrame = 0;
          for(size_t idx=0; idx<edgesLine[f].size(); ++idx){
            if(edgesLine[f][idx]){
              if(vLineFeatures[f][idx].from_global){
                candidateCornerGlobal++;
              }else{
                candidateCornerLocal++;
              }
            }
            if(edgesLine[f][idx] && std::fabs(vLineFeatures[f][idx].error) > featureErrorThreshold &&
               shouldKeepFeature(static_cast<int>(idx), totalCorner, keptCornerLocalFrame, maxCornerResidualsPerFrame)){
              problem.AddResidualBlock(edgesLine[f][idx].get(), loss_function, para_PR[f]);
              edgesLine[f][idx].release();
              vLineFeatures[f][idx].valid = true;
              ++keptCornerLocalFrame;
              ++cntCorner;
              if(vLineFeatures[f][idx].from_global){
                ++keptCornerGlobal;
              }else{
                ++keptCornerLocal;
              }
            }else{
              vLineFeatures[f][idx].valid = false;
              edgesLine[f][idx].reset();
            }
          }

          const int totalSurf = edgesPlan[f].size();
          int keptSurfLocal = 0;
          for(size_t idx=0; idx<edgesPlan[f].size(); ++idx){
            if(edgesPlan[f][idx] && std::fabs(vPlanFeatures[f][idx].error) > featureErrorThreshold &&
               shouldKeepFeature(static_cast<int>(idx), totalSurf, keptSurfLocal, maxSurfResidualsPerFrame)){
              problem.AddResidualBlock(edgesPlan[f][idx].get(), loss_function, para_PR[f]);
              edgesPlan[f][idx].release();
              vPlanFeatures[f][idx].valid = true;
              ++keptSurfLocal;
              ++cntSurf;
            }else{
              vPlanFeatures[f][idx].valid = false;
              edgesPlan[f][idx].reset();
            }
          }

          const int totalNon = edgesNon[f].size();
          int keptNonLocal = 0;
          for(size_t idx=0; idx<edgesNon[f].size(); ++idx){
            if(edgesNon[f][idx] && std::fabs(vNonFeatures[f][idx].error) > featureErrorThreshold &&
               shouldKeepFeature(static_cast<int>(idx), totalNon, keptNonLocal, maxNonResidualsPerFrame)){
              problem.AddResidualBlock(edgesNon[f][idx].get(), loss_function, para_PR[f]);
              edgesNon[f][idx].release();
              vNonFeatures[f][idx].valid = true;
              ++keptNonLocal;
              ++cntNon;
            }else{
              vNonFeatures[f][idx].valid = false;
              edgesNon[f][idx].reset();
            }
          }
          }
        }else{
          for(int f=0; f<windowSize; ++f){
            candidateCorner += static_cast<int>(edgesLine[f].size());
            candidateSurf += static_cast<int>(edgesPlan[f].size());
            candidateNon += static_cast<int>(edgesNon[f].size());
            int cntFtu = 0;
            for (auto &edge_ptr : edgesLine[f]) {
              if(edge_ptr){
                if(vLineFeatures[f][cntFtu].from_global){
                  candidateCornerGlobal++;
                }else{
                  candidateCornerLocal++;
                }
              }
              if(vLineFeatures[f][cntFtu].valid && edge_ptr) {
                problem.AddResidualBlock(edge_ptr.get(), loss_function, para_PR[f]);
                edge_ptr.release();
                if(vLineFeatures[f][cntFtu].from_global){
                  ++keptCornerGlobal;
                }else{
                  ++keptCornerLocal;
                }
              } else{
                edge_ptr.reset();
              }
              cntFtu++;
              cntCorner++;
            }
            cntFtu = 0;
            for (auto &edge_ptr : edgesPlan[f]) {
              if(vPlanFeatures[f][cntFtu].valid && edge_ptr){
                problem.AddResidualBlock(edge_ptr.get(), loss_function, para_PR[f]);
                edge_ptr.release();
              }else{
                edge_ptr.reset();
              }
              cntFtu++;
              cntSurf++;
            }

            cntFtu = 0;
            for (auto &edge_ptr : edgesNon[f]) {
              if(vNonFeatures[f][cntFtu].valid && edge_ptr){
                problem.AddResidualBlock(edge_ptr.get(), loss_function, para_PR[f]);
                edge_ptr.release();
              }else{
                edge_ptr.reset();
              }
              cntFtu++;
              cntNon++;
            }
          }
        }
      } else {
          if(iterOpt == 0) {
            thres_dist = 10.0;
          } else {
            thres_dist = 1.0;
          }
          for(int f=0; f<windowSize; ++f){
          candidateCorner += static_cast<int>(edgesLine[f].size());
          candidateSurf += static_cast<int>(edgesPlan[f].size());
          candidateNon += static_cast<int>(edgesNon[f].size());

          const int totalCorner = edgesLine[f].size();
          int keptCornerLocalFrame = 0;
          for(size_t idx=0; idx<edgesLine[f].size(); ++idx){
            if(edgesLine[f][idx]){
              if(vLineFeatures[f][idx].from_global){
                candidateCornerGlobal++;
              }else{
                candidateCornerLocal++;
              }
            }
            if(edgesLine[f][idx] && std::fabs(vLineFeatures[f][idx].error) > featureErrorThreshold &&
               shouldKeepFeature(static_cast<int>(idx), totalCorner, keptCornerLocalFrame, maxCornerResidualsPerFrame)){
              problem.AddResidualBlock(edgesLine[f][idx].get(), loss_function, para_PR[f]);
              edgesLine[f][idx].release();
              vLineFeatures[f][idx].valid = true;
              ++keptCornerLocalFrame;
              ++cntCorner;
              if(vLineFeatures[f][idx].from_global){
                ++keptCornerGlobal;
              }else{
                ++keptCornerLocal;
              }
            }else{
              vLineFeatures[f][idx].valid = false;
              edgesLine[f][idx].reset();
            }
          }

          const int totalSurf = edgesPlan[f].size();
          int keptSurfLocalFrame = 0;
          for(size_t idx=0; idx<edgesPlan[f].size(); ++idx){
            if(edgesPlan[f][idx] && std::fabs(vPlanFeatures[f][idx].error) > featureErrorThreshold &&
               shouldKeepFeature(static_cast<int>(idx), totalSurf, keptSurfLocalFrame, maxSurfResidualsPerFrame)){
              problem.AddResidualBlock(edgesPlan[f][idx].get(), loss_function, para_PR[f]);
              edgesPlan[f][idx].release();
              vPlanFeatures[f][idx].valid = true;
              ++keptSurfLocalFrame;
              ++cntSurf;
            }else{
              vPlanFeatures[f][idx].valid = false;
              edgesPlan[f][idx].reset();
            }
          }

          const int totalNon = edgesNon[f].size();
          int keptNonLocalFrame = 0;
          for(size_t idx=0; idx<edgesNon[f].size(); ++idx){
            if(edgesNon[f][idx] && std::fabs(vNonFeatures[f][idx].error) > featureErrorThreshold &&
               shouldKeepFeature(static_cast<int>(idx), totalNon, keptNonLocalFrame, maxNonResidualsPerFrame)){
              problem.AddResidualBlock(edgesNon[f][idx].get(), loss_function, para_PR[f]);
              edgesNon[f][idx].release();
              vNonFeatures[f][idx].valid = true;
              ++keptNonLocalFrame;
              ++cntNon;
            }else{
              vNonFeatures[f][idx].valid = false;
              edgesNon[f][idx].reset();
            }
          }
          }
      }

      if(residual_config_.log_feature_counts){
        const char* mode_str = "balanced";
        if(low_feature_mode) mode_str = "low_feature";
        else if(high_feature_mode) mode_str = "high_feature";
        ROS_INFO("Estimator corner adaptive iter %d: mode=%s avg_global_kd=%.1f limit=%d eigen_ratio=%.2f",
                 iterOpt,
                 mode_str,
                 avg_global_kd,
                 maxCornerResidualsPerFrame,
                 corner_eigen_ratio_);
        if(!line_feature_stats.empty()){
          FeatureBuildStats total_stats;
          for(const auto& s : line_feature_stats){
            total_stats.points_total += s.points_total;
            total_stats.global_region_skipped += s.global_region_skipped;
            total_stats.global_kd_success += s.global_kd_success;
            total_stats.global_eigen_pass += s.global_eigen_pass;
            total_stats.global_eigen_fail += s.global_eigen_fail;
            total_stats.local_kd_success += s.local_kd_success;
            total_stats.local_eigen_pass += s.local_eigen_pass;
            total_stats.local_eigen_fail += s.local_eigen_fail;
          }
          if(total_stats.points_total > 0){
            ROS_INFO("Estimator corner build stats iter %d: points=%d region_skip=%d global_kd=%d global_pass=%d global_fail=%d local_kd=%d local_pass=%d local_fail=%d",
                     iterOpt,
                     total_stats.points_total,
                     total_stats.global_region_skipped,
                     total_stats.global_kd_success,
                     total_stats.global_eigen_pass,
                     total_stats.global_eigen_fail,
                     total_stats.local_kd_success,
                     total_stats.local_eigen_pass,
                     total_stats.local_eigen_fail);
          }
        }
        ROS_INFO("Estimator residual candidates iter %d: corner=%d (global=%d local=%d) surf=%d non=%d",
                 iterOpt, candidateCorner, candidateCornerGlobal, candidateCornerLocal, candidateSurf, candidateNon);
        ROS_INFO("Estimator residual kept iter %d: corner=%d (global=%d local=%d) surf=%d non=%d",
                 iterOpt, cntCorner, keptCornerGlobal, keptCornerLocal, cntSurf, cntNon);
        if(iterOpt == 0){
          ROS_INFO("Estimator residual timing iter %d: build=%.2f ms solve=%.2f ms runtime_limit(c/s/n)=(%d/%d/%d)",
                   iterOpt,
                   last_residual_build_ms_,
                   last_ceres_solve_ms_,
                   runtime_corner_limit_,
                   runtime_surf_limit_,
                   runtime_non_limit_);
        }
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.trust_region_strategy_type = ceres::DOGLEG;
      options.max_num_iterations = 8;
      options.function_tolerance = 1e-4;
      options.gradient_tolerance = 1e-4;
      options.parameter_tolerance = 1e-4;
      options.use_nonmonotonic_steps = true;
      options.max_consecutive_nonmonotonic_steps = 3;
      options.minimizer_progress_to_stdout = false;
    const unsigned int ceres_threads = std::max(1u, std::thread::hardware_concurrency());
    options.num_threads = static_cast<int>(ceres_threads);
    ros::WallTime ceres_start = ros::WallTime::now();
    if(log_module_timing_){
      ROS_INFO("[Timing] Ceres solve start %.6f", ros::Time::now().toSec());
    }
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if(log_module_timing_){
      ROS_INFO("[Timing] Ceres solve end   %.6f", ros::Time::now().toSec());
    }
    const double ceres_solve_ms =
        (ros::WallTime::now() - ceres_start).toSec() * 1000.0;
    if(iterOpt == 0){
      last_ceres_solve_ms_ = ceres_solve_ms;
      UpdateResidualLimits(last_residual_build_ms_, last_ceres_solve_ms_);
    }
    } // problem scope

    double2vector(lidarFrameList);

    Eigen::Quaterniond q_after_opti = lidarFrameList.back().Q;
    Eigen::Vector3d t_after_opti = lidarFrameList.back().P;
    Eigen::Vector3d V_after_opti = lidarFrameList.back().V;
    double deltaR = (q_before_opti.angularDistance(q_after_opti)) * 180.0 / M_PI;
    double deltaT = (t_before_opti - t_after_opti).norm();
    double speed = V_after_opti.norm();

    if (deltaR < 0.05 && deltaT < 0.05 || (iterOpt+1) == max_iters){
      if(log_module_timing_){
        ROS_INFO("[Timing] Marginalization start %.6f", ros::Time::now().toSec());
      }
      // 诊断日志: 位姿变化、速度、迭代次数
      if(residual_config_.log_feature_counts && windowSize == SLIDEWINDOWSIZE){
        ROS_INFO("Estimator pose_delta: deltaR=%.4f deg deltaT=%.4f m speed=%.2f m/s iters=%d",
                 deltaR, deltaT, speed, iterOpt + 1);
      }
      ROS_INFO("Frame: %u", frame_count++);
      if(windowSize != SLIDEWINDOWSIZE) break;
      // apply marginalization
      auto *marginalization_info = new MarginalizationInfo();
      if (last_marginalization_info){
        std::vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
        {
          if (last_marginalization_parameter_blocks[i] == para_PR[0] ||
              last_marginalization_parameter_blocks[i] == para_VBias[0])
            drop_set.push_back(i);
        }

        auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        auto *residual_block_info = new ResidualBlockInfo(marginalization_factor, nullptr,
                                                          last_marginalization_parameter_blocks,
                                                          drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
      
      auto frame_curr = lidarFrameList.begin();
      std::advance(frame_curr, 1);
      ceres::CostFunction* IMU_Cost = Cost_NavState_PRV_Bias::Create(frame_curr->imuIntegrator,
                                                                     const_cast<Eigen::Vector3d&>(gravity),
                                                                     Eigen::LLT<Eigen::Matrix<double, 15, 15>>
                                                                             (frame_curr->imuIntegrator.GetCovariance().inverse())
                                                                             .matrixL().transpose());
      auto *residual_block_info = new ResidualBlockInfo(IMU_Cost, nullptr,
                                                        std::vector<double *>{para_PR[0], para_VBias[0], para_PR[1], para_VBias[1]},
                                                        std::vector<int>{0, 1});
      marginalization_info->addResidualBlockInfo(residual_block_info);

      int f = 0;
      transformTobeMapped = Eigen::Matrix4d::Identity();
      transformTobeMapped.topLeftCorner(3,3) = frame_curr->Q * exRbl;
      transformTobeMapped.topRightCorner(3,1) = frame_curr->Q * exPbl + frame_curr->P;
      edgesLine[f].clear();
      edgesPlan[f].clear();
      edgesNon[f].clear();
      std::thread marginal_threads[3];
      marginal_threads[0] = std::thread(&Estimator::processPointToLine, this,
                                        std::ref(edgesLine[f]),
                                        std::ref(vLineFeatures[f]),
                                        std::ref(laserCloudCornerStack[f]),
                                        std::ref(laserCloudCornerFromLocal),
                                        std::ref(kdtreeCornerFromLocal),
                                        std::ref(exTlb),
                                        std::ref(transformTobeMapped),
                                        nullptr);

      marginal_threads[1] = std::thread(&Estimator::processPointToPlanVec, this,
                                        std::ref(edgesPlan[f]),
                                        std::ref(vPlanFeatures[f]),
                                        std::ref(laserCloudSurfStack[f]),
                                        std::ref(laserCloudSurfFromLocal),
                                        std::ref(kdtreeSurfFromLocal),
                                        std::ref(exTlb),
                                        std::ref(transformTobeMapped));

      marginal_threads[2] = std::thread(&Estimator::processNonFeatureICP, this,
                                        std::ref(edgesNon[f]),
                                        std::ref(vNonFeatures[f]),
                                        std::ref(laserCloudNonFeatureStack[f]),
                                        std::ref(laserCloudNonFeatureFromLocal),
                                        std::ref(kdtreeNonFeatureFromLocal),
                                        std::ref(exTlb),
                                        std::ref(transformTobeMapped));      
                    
      marginal_threads[0].join();
      marginal_threads[1].join();
      marginal_threads[2].join();
      int cntFtu = 0;
      for (auto &edge_ptr : edgesLine[f]) {
        if(vLineFeatures[f][cntFtu].valid && edge_ptr){
          auto *residual_block_info = new ResidualBlockInfo(edge_ptr.release(), nullptr,
                                                            std::vector<double *>{para_PR[0]},
                                                            std::vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        cntFtu++;
      }
      cntFtu = 0;
      for (auto &edge_ptr : edgesPlan[f]) {
        if(vPlanFeatures[f][cntFtu].valid && edge_ptr){
          auto *residual_block_info = new ResidualBlockInfo(edge_ptr.release(), nullptr,
                                                            std::vector<double *>{para_PR[0]},
                                                            std::vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        cntFtu++;
      }

      cntFtu = 0;
      for (auto &edge_ptr : edgesNon[f]) {
        if(vNonFeatures[f][cntFtu].valid && edge_ptr){
          auto *residual_block_info = new ResidualBlockInfo(edge_ptr.release(), nullptr,
                                                            std::vector<double *>{para_PR[0]},
                                                            std::vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        cntFtu++;
      }

      marginalization_info->preMarginalize();
      marginalization_info->marginalize();

      std::unordered_map<long, double *> addr_shift;
      for (int i = 1; i < SLIDEWINDOWSIZE; i++)
      {
        addr_shift[reinterpret_cast<long>(para_PR[i])] = para_PR[i - 1];
        addr_shift[reinterpret_cast<long>(para_VBias[i])] = para_VBias[i - 1];
      }
      std::vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

      delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;
      if(log_module_timing_){
        ROS_INFO("[Timing] Marginalization end   %.6f", ros::Time::now().toSec());
      }
      break;
    }

    if(windowSize != SLIDEWINDOWSIZE) {
      for(int f=0; f<windowSize; ++f){
        edgesLine[f].clear();
        edgesPlan[f].clear();
        edgesNon[f].clear();
        vLineFeatures[f].clear();
        vPlanFeatures[f].clear();
        vNonFeatures[f].clear();
      }
    }
  }

}

void Estimator::EnforceLocalMapLimit(pcl::PointCloud<PointType>::Ptr& cloud, int max_points){
  if(!cloud || max_points <= 0) return;
  if(static_cast<int>(cloud->points.size()) <= max_points) return;
  pcl::PointCloud<PointType>::Ptr limited(new pcl::PointCloud<PointType>());
  limited->points.reserve(max_points);
  const double step = static_cast<double>(cloud->points.size()) / static_cast<double>(max_points);
  double cursor = 0.0;
  for(size_t i = 0; i < cloud->points.size() &&
                     limited->points.size() < static_cast<size_t>(max_points); ++i){
    if(static_cast<double>(i) >= cursor){
      limited->points.push_back(cloud->points[i]);
      cursor += step;
    }
  }
  limited->width = limited->points.size();
  limited->height = 1;
  limited->is_dense = cloud->is_dense;
  cloud = limited;
}

void Estimator::UpdateResidualLimits(double build_ms, double solve_ms){
  auto clamp_limit = [](int value, int min_limit, int max_limit){
    return std::max(min_limit, std::min(max_limit, value));
  };
  const int cfg_corner_max = std::max(1, residual_config_.max_corner_residuals);
  const int cfg_surf_max = std::max(1, residual_config_.max_surf_residuals);
  const int cfg_non_max = std::max(1, residual_config_.max_non_residuals);
  const auto& budget = residual_config_.adaptive_budget;
  if(!budget.enable){
    runtime_corner_limit_ = cfg_corner_max;
    runtime_surf_limit_ = cfg_surf_max;
    runtime_non_limit_ = cfg_non_max;
    return;
  }
  int min_corner = std::max(1, budget.min_corner_residuals);
  int min_surf = std::max(1, budget.min_surf_residuals);
  int min_non = std::max(1, budget.min_non_residuals);
  runtime_corner_limit_ = clamp_limit(runtime_corner_limit_, min_corner, cfg_corner_max);
  runtime_surf_limit_ = clamp_limit(runtime_surf_limit_, min_surf, cfg_surf_max);
  runtime_non_limit_ = clamp_limit(runtime_non_limit_, min_non, cfg_non_max);
  double pressure_sum = 0.0;
  int pressure_count = 0;
  auto accumulate_pressure = [&](double observed, double target){
    if(target <= 0.0) return;
    pressure_sum += (observed - target) / target;
    ++pressure_count;
  };
  accumulate_pressure(build_ms, budget.target_residual_build_ms);
  accumulate_pressure(solve_ms, budget.target_ceres_solve_ms);
  if(pressure_count == 0) return;
  double avg_pressure = pressure_sum / static_cast<double>(pressure_count);
  if(std::fabs(avg_pressure) < budget.tolerance_ratio){
    return;
  }
  int direction = avg_pressure > 0.0 ? -1 : 1;
  auto adjust_limit = [&](int& current, int min_limit, int max_limit){
    int delta = std::max(1, static_cast<int>(current * budget.adjust_ratio));
    current = clamp_limit(current + direction * delta, min_limit, max_limit);
  };
  adjust_limit(runtime_corner_limit_, min_corner, cfg_corner_max);
  adjust_limit(runtime_surf_limit_, min_surf, cfg_surf_max);
  adjust_limit(runtime_non_limit_, min_non, cfg_non_max);
}

void Estimator::MapIncrementLocal(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerStack,
                                  const pcl::PointCloud<PointType>::Ptr& laserCloudSurfStack,
                                  const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureStack,
                                  const Eigen::Matrix4d& transformTobeMapped){
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
  int laserCloudNonFeatureStackNum = laserCloudNonFeatureStack->points.size();
  PointType pointSel;
  PointType pointSel2;
  size_t Id = localMapID % localMapWindowSize;
  localCornerMap[Id]->clear();
  localSurfMap[Id]->clear();
  localNonFeatureMap[Id]->clear();
  localFrameId++;
  localFrameStamp[Id] = localFrameId;
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    MAP_MANAGER::pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel, transformTobeMapped);
    localCornerMap[Id]->push_back(pointSel);
  }
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    MAP_MANAGER::pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel2, transformTobeMapped);
    localSurfMap[Id]->push_back(pointSel2);
  }
  for (int i = 0; i < laserCloudNonFeatureStackNum; i++) {
    MAP_MANAGER::pointAssociateToMap(&laserCloudNonFeatureStack->points[i], &pointSel2, transformTobeMapped);
    localNonFeatureMap[Id]->push_back(pointSel2);
  }

  Eigen::Vector3d anchor = transformTobeMapped.topRightCorner(3,1);
  Eigen::Matrix3d rot = transformTobeMapped.topLeftCorner(3,3);
  Eigen::Vector3d forward = rot.col(0);
  Eigen::Vector3d left = rot.col(1);
  Eigen::Vector3d up = rot.col(2);
  const double forwardLimit = localBoxForward;
  const double backwardLimit = localBoxBackward;
  const double sideLimit = localBoxSide;
  const double verticalLimit = localBoxVertical;
  const long minFrameStamp = std::max(0L, localFrameId - static_cast<long>(localMapHistoryFrames) + 1);
  auto pointInLocalBox = [&](const PointType& pt)->bool{
    Eigen::Vector3d rel(pt.x - anchor.x(), pt.y - anchor.y(), pt.z - anchor.z());
    double dForward = rel.dot(forward);
    if(dForward > forwardLimit || dForward < -backwardLimit) return false;
    double dSide = rel.dot(left);
    if(std::fabs(dSide) > sideLimit) return false;
    double dUp = rel.dot(up);
    if(std::fabs(dUp) > verticalLimit) return false;
    return true;
  };

  for (int i = 0; i < localMapWindowSize; i++) {
    if(localFrameStamp[i] < minFrameStamp) continue;
    for(const auto& pt : *localCornerMap[i]) {
      if(pointInLocalBox(pt)) laserCloudCornerFromLocal->push_back(pt);
    }
    for(const auto& pt : *localSurfMap[i]) {
      if(pointInLocalBox(pt)) laserCloudSurfFromLocal->push_back(pt);
    }
    for(const auto& pt : *localNonFeatureMap[i]) {
      if(pointInLocalBox(pt)) laserCloudNonFeatureFromLocal->push_back(pt);
    }
  }
  pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
  downSizeFilterCorner.setInputCloud(laserCloudCornerFromLocal);
  downSizeFilterCorner.filter(*temp);
  laserCloudCornerFromLocal = temp;
  pcl::PointCloud<PointType>::Ptr temp2(new pcl::PointCloud<PointType>());
  downSizeFilterSurf.setInputCloud(laserCloudSurfFromLocal);
  downSizeFilterSurf.filter(*temp2);
  laserCloudSurfFromLocal = temp2;
  pcl::PointCloud<PointType>::Ptr temp3(new pcl::PointCloud<PointType>());
  downSizeFilterNonFeature.setInputCloud(laserCloudNonFeatureFromLocal);
  downSizeFilterNonFeature.filter(*temp3);
  laserCloudNonFeatureFromLocal = temp3;
  EnforceLocalMapLimit(laserCloudCornerFromLocal, local_corner_max_points_);
  EnforceLocalMapLimit(laserCloudSurfFromLocal, local_surf_max_points_);
  EnforceLocalMapLimit(laserCloudNonFeatureFromLocal, local_non_max_points_);
  localMapID ++;
}