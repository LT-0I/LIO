#include "Estimator/Estimator.h"
#include <algorithm>
#include <thread>

Estimator::Estimator(const float& filter_corner, const float& filter_surf){
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
  map_manager = new MAP_MANAGER(filter_corner, filter_surf);
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

void Estimator::processPointToLine(std::vector<ceres::CostFunction *>& edges,
                                   std::vector<FeatureLine>& vLineFeatures,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudCorner,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudCornerLocal,
                                   const pcl::KdTreeFLANN<PointType>::Ptr& kdtreeLocal,
                                   const Eigen::Matrix4d& exTlb,
                                   const Eigen::Matrix4d& m4d){

  Eigen::Matrix4d Tbl = Eigen::Matrix4d::Identity();
  Tbl.topLeftCorner(3,3) = exTlb.topLeftCorner(3,3).transpose();
  Tbl.topRightCorner(3,1) = -1.0 * Tbl.topLeftCorner(3,3) * exTlb.topRightCorner(3,1);
  if(!vLineFeatures.empty()){
    for(const auto& l : vLineFeatures){
      auto* e = Cost_NavState_IMU_Line::Create(l.pointOri,
                                               l.lineP1,
                                               l.lineP2,
                                               Tbl,
                                               Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
      edges.push_back(e);
    }
    return;
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

    if(id == 5000) continue;

    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

    const auto* globalCornerMap = GlobalCornerMap[id];
    const auto* globalCornerKd = CornerKdMap[id];
    if(globalCornerMap && globalCornerKd && globalCornerMap->points.size() > 100) {
      globalCornerKd->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);
      
      if (_pointSearchSqDis[4] < thres_dist) {

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

      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        debug_num12 ++;
        float x1 = cx + 0.1 * unit_direction[0];
        float y1 = cy + 0.1 * unit_direction[1];
        float z1 = cz + 0.1 * unit_direction[2];
        float x2 = cx - 0.1 * unit_direction[0];
        float y2 = cy - 0.1 * unit_direction[1];
        float z2 = cz - 0.1 * unit_direction[2];

        Eigen::Vector3d tripod1(x1, y1, z1);
        Eigen::Vector3d tripod2(x2, y2, z2);
        auto* e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                 tripod1,
                                                 tripod2,
                                                 Tbl,
                                                 Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
        edges.push_back(e);
        vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                   tripod1,
                                   tripod2);
        vLineFeatures.back().ComputeError(m4d);

        continue;
      }
      
    }
    
    }

    if(laserCloudCornerLocal->points.size() > 20 ){
      kdtreeLocal->nearestKSearch(_pointSel, 5, _pointSearchInd2, _pointSearchSqDis2);
      if (_pointSearchSqDis2[4] < thres_dist) {

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

        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          debug_num22++;
          float x1 = cx + 0.1 * unit_direction[0];
          float y1 = cy + 0.1 * unit_direction[1];
          float z1 = cz + 0.1 * unit_direction[2];
          float x2 = cx - 0.1 * unit_direction[0];
          float y2 = cy - 0.1 * unit_direction[1];
          float z2 = cz - 0.1 * unit_direction[2];

          Eigen::Vector3d tripod1(x1, y1, z1);
          Eigen::Vector3d tripod2(x2, y2, z2);
          auto* e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                  tripod1,
                                                  tripod2,
                                                  Tbl,
                                                  Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
          edges.push_back(e);
          vLineFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    tripod1,
                                    tripod2);
          vLineFeatures.back().ComputeError(m4d);
        }
      }
    }
     
  }
}

void Estimator::processPointToPlan(std::vector<ceres::CostFunction *>& edges,
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
      auto* e = Cost_NavState_IMU_Plan::Create(p.pointOri,
                                               p.pa,
                                               p.pb,
                                               p.pc,
                                               p.pd,
                                               Tbl,
                                               Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
      edges.push_back(e);
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
          auto* e = Cost_NavState_IMU_Plan::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                  pa,
                                                  pb,
                                                  pc,
                                                  pd,
                                                  Tbl,
                                                  Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
          edges.push_back(e);
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
        auto* e = Cost_NavState_IMU_Plan::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                pa,
                                                pb,
                                                pc,
                                                pd,
                                                Tbl,
                                                Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
        edges.push_back(e);
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

void Estimator::processPointToPlanVec(std::vector<ceres::CostFunction *>& edges,
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
      auto* e = Cost_NavState_IMU_Plan_Vec::Create(p.pointOri,
                                                   p.pointProj,
                                                   Tbl,
                                                   p.sqrt_info);
      edges.push_back(e);
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

          auto* e = Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                       point_proj,
                                                       Tbl,
                                                       sqrt_info);
          edges.push_back(e);
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

        auto* e = Cost_NavState_IMU_Plan_Vec::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                      point_proj,
                                                      Tbl,
                                                      sqrt_info);
        edges.push_back(e);
        vPlanFeatures.emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                    point_proj,
                                    sqrt_info);
        vPlanFeatures.back().ComputeError(m4d);
      }
    }
  }

  }

}


void Estimator::processNonFeatureICP(std::vector<ceres::CostFunction *>& edges,
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
      auto* e = Cost_NonFeature_ICP::Create(p.pointOri,
                                            p.pa,
                                            p.pb,
                                            p.pc,
                                            p.pd,
                                            Tbl,
                                            Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
      edges.push_back(e);
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

          auto* e = Cost_NonFeature_ICP::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                pa,
                                                pb,
                                                pc,
                                                pd,
                                                Tbl,
                                                Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
          edges.push_back(e);
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
      if (_pointSearchSqDis2[4] < 1 * thres_dist) {
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

          auto* e = Cost_NonFeature_ICP::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                pa,
                                                pb,
                                                pc,
                                                pd,
                                                Tbl,
                                                Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
          edges.push_back(e);
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
    Estimate(lidarFrameList, exTlb, gravity);
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
  MapIncrementLocal(laserCloudCornerForMap,laserCloudSurfForMap,laserCloudNonFeatureForMap,transformTobeMapped);
  locker.unlock();
}

void Estimator::Estimate(std::list<LidarFrame>& lidarFrameList,
                         const Eigen::Matrix4d& exTlb,
                         const Eigen::Vector3d& gravity){

  ROS_INFO("Estimator optimization start %.6f", ros::Time::now().toSec());
  int num_corner_map = 0;
  int num_surf_map = 0;

  static uint32_t frame_count = 0;
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

  ROS_INFO("Estimator map data fetch start %.6f", ros::Time::now().toSec());
  auto map_snapshot = map_manager->AcquireSnapshot();
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
  ROS_INFO("Estimator map data fetch end %.6f", ros::Time::now().toSec());

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

    std::vector<std::vector<ceres::CostFunction *>> edgesLine(windowSize);
    std::vector<std::vector<ceres::CostFunction *>> edgesPlan(windowSize);
    std::vector<std::vector<ceres::CostFunction *>> edgesNon(windowSize);

    ROS_INFO("Estimator residual construction start %.6f", ros::Time::now().toSec());
    const unsigned int hw_threads = std::max(1u, std::thread::hardware_concurrency());
    const int worker_count = std::min<int>(windowSize, std::max(1u, hw_threads));
    const int frames_per_worker = std::max(1, (windowSize + worker_count - 1) / worker_count);
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

        processPointToLine(edgesLine[f],
                           vLineFeatures[f],
                           laserCloudCornerStack[f],
                           laserCloudCornerFromLocal,
                           kdtreeCornerFromLocal,
                           exTlb,
                           localTransform);

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
    ROS_INFO("Estimator residual construction end %.6f", ros::Time::now().toSec());

    //create huber loss function
    ceres::LossFunction* loss_function = NULL;
    loss_function = new ceres::HuberLoss(0.1 / IMUIntegrator::lidar_m);
    if(windowSize == SLIDEWINDOWSIZE) {
      loss_function = NULL;
    } else {
      loss_function = new ceres::HuberLoss(0.1 / IMUIntegrator::lidar_m);
    }

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

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
      // construct new marginlization_factor
      auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
      problem.AddResidualBlock(marginalization_factor, nullptr,
                               last_marginalization_parameter_blocks);
    }

    Eigen::Quaterniond q_before_opti = lidarFrameList.back().Q;
    Eigen::Vector3d t_before_opti = lidarFrameList.back().P;

    int cntSurf = 0;
    int cntCorner = 0;
    int cntNon = 0;
    const int maxCornerResidualsPerFrame = 600;
    const int maxSurfResidualsPerFrame = 900;
    const int maxNonResidualsPerFrame = 450;
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
          const int totalCorner = edgesLine[f].size();
          int keptCornerLocal = 0;
          for(size_t idx=0; idx<edgesLine[f].size(); ++idx){
            if(std::fabs(vLineFeatures[f][idx].error) > 1e-5 &&
               shouldKeepFeature(static_cast<int>(idx), totalCorner, keptCornerLocal, maxCornerResidualsPerFrame)){
              problem.AddResidualBlock(edgesLine[f][idx], loss_function, para_PR[f]);
              vLineFeatures[f][idx].valid = true;
              ++keptCornerLocal;
              ++cntCorner;
            }else{
              vLineFeatures[f][idx].valid = false;
            }
          }

          const int totalSurf = edgesPlan[f].size();
          int keptSurfLocal = 0;
          for(size_t idx=0; idx<edgesPlan[f].size(); ++idx){
            if(std::fabs(vPlanFeatures[f][idx].error) > 1e-5 &&
               shouldKeepFeature(static_cast<int>(idx), totalSurf, keptSurfLocal, maxSurfResidualsPerFrame)){
              problem.AddResidualBlock(edgesPlan[f][idx], loss_function, para_PR[f]);
              vPlanFeatures[f][idx].valid = true;
              ++keptSurfLocal;
              ++cntSurf;
            }else{
              vPlanFeatures[f][idx].valid = false;
            }
          }

          const int totalNon = edgesNon[f].size();
          int keptNonLocal = 0;
          for(size_t idx=0; idx<edgesNon[f].size(); ++idx){
            if(std::fabs(vNonFeatures[f][idx].error) > 1e-5 &&
               shouldKeepFeature(static_cast<int>(idx), totalNon, keptNonLocal, maxNonResidualsPerFrame)){
              problem.AddResidualBlock(edgesNon[f][idx], loss_function, para_PR[f]);
              vNonFeatures[f][idx].valid = true;
              ++keptNonLocal;
              ++cntNon;
            }else{
              vNonFeatures[f][idx].valid = false;
            }
          }
        }
      }else{
        for(int f=0; f<windowSize; ++f){
          int cntFtu = 0;
          for (auto &e : edgesLine[f]) {
            if(vLineFeatures[f][cntFtu].valid) {
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
            }
            cntFtu++;
            cntCorner++;
          }
          cntFtu = 0;
          for (auto &e : edgesPlan[f]) {
            if(vPlanFeatures[f][cntFtu].valid){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
            }
            cntFtu++;
            cntSurf++;
          }

          cntFtu = 0;
          for (auto &e : edgesNon[f]) {
            if(vNonFeatures[f][cntFtu].valid){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
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
          const int totalCorner = edgesLine[f].size();
          int keptCornerLocal = 0;
          for(size_t idx=0; idx<edgesLine[f].size(); ++idx){
            if(std::fabs(vLineFeatures[f][idx].error) > 1e-5 &&
               shouldKeepFeature(static_cast<int>(idx), totalCorner, keptCornerLocal, maxCornerResidualsPerFrame)){
              problem.AddResidualBlock(edgesLine[f][idx], loss_function, para_PR[f]);
              vLineFeatures[f][idx].valid = true;
              ++keptCornerLocal;
              ++cntCorner;
            }else{
              vLineFeatures[f][idx].valid = false;
            }
          }
          const int totalSurf = edgesPlan[f].size();
          int keptSurfLocal = 0;
          for(size_t idx=0; idx<edgesPlan[f].size(); ++idx){
            if(std::fabs(vPlanFeatures[f][idx].error) > 1e-5 &&
               shouldKeepFeature(static_cast<int>(idx), totalSurf, keptSurfLocal, maxSurfResidualsPerFrame)){
              problem.AddResidualBlock(edgesPlan[f][idx], loss_function, para_PR[f]);
              vPlanFeatures[f][idx].valid = true;
              ++keptSurfLocal;
              ++cntSurf;
            }else{
              vPlanFeatures[f][idx].valid = false;
            }
          }

          const int totalNon = edgesNon[f].size();
          int keptNonLocal = 0;
          for(size_t idx=0; idx<edgesNon[f].size(); ++idx){
            if(std::fabs(vNonFeatures[f][idx].error) > 1e-5 &&
               shouldKeepFeature(static_cast<int>(idx), totalNon, keptNonLocal, maxNonResidualsPerFrame)){
              problem.AddResidualBlock(edgesNon[f][idx], loss_function, para_PR[f]);
              vNonFeatures[f][idx].valid = true;
              ++keptNonLocal;
              ++cntNon;
            }else{
              vNonFeatures[f][idx].valid = false;
            }
          }
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
    ROS_INFO("Estimator ceres solve start %.6f", ros::Time::now().toSec());
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_INFO("Estimator ceres solve end %.6f", ros::Time::now().toSec());

    double2vector(lidarFrameList);

    Eigen::Quaterniond q_after_opti = lidarFrameList.back().Q;
    Eigen::Vector3d t_after_opti = lidarFrameList.back().P;
    Eigen::Vector3d V_after_opti = lidarFrameList.back().V;
    double deltaR = (q_before_opti.angularDistance(q_after_opti)) * 180.0 / M_PI;
    double deltaT = (t_before_opti - t_after_opti).norm();

    if (deltaR < 0.05 && deltaT < 0.05 || (iterOpt+1) == max_iters){
      ROS_INFO("Frame: %d\n",frame_count++);
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
                                        std::ref(transformTobeMapped));

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
      for (auto &e : edgesLine[f]) {
        if(vLineFeatures[f][cntFtu].valid){
          auto *residual_block_info = new ResidualBlockInfo(e, nullptr,
                                                            std::vector<double *>{para_PR[0]},
                                                            std::vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        cntFtu++;
      }
      cntFtu = 0;
      for (auto &e : edgesPlan[f]) {
        if(vPlanFeatures[f][cntFtu].valid){
          auto *residual_block_info = new ResidualBlockInfo(e, nullptr,
                                                            std::vector<double *>{para_PR[0]},
                                                            std::vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        cntFtu++;
      }

      cntFtu = 0;
      for (auto &e : edgesNon[f]) {
        if(vNonFeatures[f][cntFtu].valid){
          auto *residual_block_info = new ResidualBlockInfo(e, nullptr,
                                                            std::vector<double *>{para_PR[0]},
                                                            std::vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        cntFtu++;
      }

      ROS_INFO("Estimator marginalization start %.6f", ros::Time::now().toSec());
      marginalization_info->preMarginalize();
      marginalization_info->marginalize();
      ROS_INFO("Estimator marginalization end %.6f", ros::Time::now().toSec());

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

  ROS_INFO("Estimator optimization end %.6f", ros::Time::now().toSec());
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

  for (int i = 0; i < localMapWindowSize; i++) {
    *laserCloudCornerFromLocal += *localCornerMap[i];
    *laserCloudSurfFromLocal += *localSurfMap[i];
    *laserCloudNonFeatureFromLocal += *localNonFeatureMap[i];
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
  localMapID ++;
}