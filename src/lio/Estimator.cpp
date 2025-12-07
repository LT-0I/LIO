#include "Estimator/Estimator.h"
#include <omp.h>

#ifndef LIKELY
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#endif

Estimator::Estimator(const float& filter_corner, const float& filter_surf,
                     int max_iters, int ceres_max_iters,
                     double conv_r, double conv_t,
                     float filter_nonfeature)
  : max_iterations_(max_iters), ceres_max_iterations_(ceres_max_iters),
    convergence_threshold_r_(conv_r), convergence_threshold_t_(conv_t),
    filter_nonfeature_(filter_nonfeature) {
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
#if USE_NANOFLANN
  kdtreeCornerFromLocal = std::make_shared<nanoflann_pcl::KdTreeNano<PointType>>();
  kdtreeSurfFromLocal = std::make_shared<nanoflann_pcl::KdTreeNano<PointType>>();
  kdtreeNonFeatureFromLocal = std::make_shared<nanoflann_pcl::KdTreeNano<PointType>>();
#else
  kdtreeCornerFromLocal.reset(new pcl::KdTreeFLANN<PointType>);
  kdtreeSurfFromLocal.reset(new pcl::KdTreeFLANN<PointType>);
  kdtreeNonFeatureFromLocal.reset(new pcl::KdTreeFLANN<PointType>);
#endif

  for(int i = 0; i < localMapWindowSize; i++){
    localCornerMap[i].reset(new pcl::PointCloud<PointType>);
    localSurfMap[i].reset(new pcl::PointCloud<PointType>);
    localNonFeatureMap[i].reset(new pcl::PointCloud<PointType>);
  }

  downSizeFilterCorner.setLeafSize(filter_corner, filter_corner, filter_corner);
  downSizeFilterSurf.setLeafSize(filter_surf, filter_surf, filter_surf);
  downSizeFilterNonFeature.setLeafSize(filter_nonfeature_, filter_nonfeature_, filter_nonfeature_);
  map_manager = new MAP_MANAGER(filter_corner, filter_surf);
  threadMap = std::thread(&Estimator::threadMapIncrement, this);
  
  // === 优化点1：激进预分配特征存储（16GB内存，不吝啬空间）===
  for(int i = 0; i < SLIDEWINDOWSIZE; ++i) {
    vLineFeatures_[i].reserve(MAX_FEATURES_PER_FRAME);
    vPlanFeatures_[i].reserve(MAX_FEATURES_PER_FRAME);
    vNonFeatures_[i].reserve(MAX_FEATURES_PER_FRAME);
    edgesLine_[i].reserve(MAX_FEATURES_PER_FRAME);
    edgesPlan_[i].reserve(MAX_FEATURES_PER_FRAME);
    edgesNon_[i].reserve(MAX_FEATURES_PER_FRAME);
  }
  ROS_INFO("Estimator: Pre-allocated %d features per frame x %d frames", 
           MAX_FEATURES_PER_FRAME, SLIDEWINDOWSIZE);
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
                                   const LocalKdTreePtr& kdtreeLocal,
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

  int laserCloudCornerStackNum = laserCloudCorner->points.size();
  int localCloudSize = laserCloudCornerLocal->points.size();
  
  // 预分配线程局部结果数组
  int max_threads = omp_get_max_threads();
  std::vector<std::vector<ceres::CostFunction*>> thread_edges(max_threads);
  std::vector<std::vector<FeatureLine>> thread_features(max_threads);
  for(int t = 0; t < max_threads; t++){
    thread_edges[t].reserve(laserCloudCornerStackNum / max_threads + 100);
    thread_features[t].reserve(laserCloudCornerStackNum / max_threads + 100);
  }

  #pragma omp parallel
  {
    int tid = omp_get_thread_num();
    // 线程私有变量
    PointType _pointOri, _pointSel;
    std::array<int,5>   _pointSearchInd{};
    std::array<float,5> _pointSearchSqDis{};
    std::array<int,5>   _pointSearchInd2{};
    std::array<float,5> _pointSearchSqDis2{};
    // KdTree 接口仍要求 std::vector，使用小向量接收后拷贝到 std::array
    std::vector<int> knn_idx_global(5);
    std::vector<float> knn_dist_global(5);
    std::vector<int> knn_idx_local(5);
    std::vector<float> knn_dist_local(5);
    Eigen::Matrix3d _matA1;

    // === 优化点4：RK3588 big.LITTLE 架构适配，chunk=16 更好负载均衡 ===
    #pragma omp for schedule(dynamic, 16) nowait
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    if (i + 8 < laserCloudCornerStackNum) {
      __builtin_prefetch(&laserCloudCorner->points[i + 8], 0, 1);
    }
    _pointOri = laserCloudCorner->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);
    int id = map_manager->FindUsedCornerMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);

      if(UNLIKELY(id == 5000)) continue;
    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

      bool found_global = false;
      if(GlobalCornerMap[id].points.size() > 100) {
        CornerKdMap[id].nearestKSearch(_pointSel, 5, knn_idx_global, knn_dist_global);
        for(int j=0;j<5;++j){ _pointSearchInd[j]=knn_idx_global[j]; _pointSearchSqDis[j]=knn_dist_global[j]; }
      
      if (_pointSearchSqDis[4] < thres_dist) {
          float cx = 0, cy = 0, cz = 0;
          const PointType* __restrict__ gpts = GlobalCornerMap[id].points.data();
          #pragma GCC unroll 5
        for (int j = 0; j < 5; j++) {
            const auto& pj = gpts[_pointSearchInd[j]];
            cx += pj.x;
            cy += pj.y;
            cz += pj.z;
      }
          cx /= 5; cy /= 5; cz /= 5;

          float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
          #pragma GCC unroll 5
        for (int j = 0; j < 5; j++) {
            const auto& pj = gpts[_pointSearchInd[j]];
            float ax = pj.x - cx;
            float ay = pj.y - cy;
            float az = pj.z - cz;
            a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
            a22 += ay * ay; a23 += ay * az; a33 += az * az;
      }
          a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

          _matA1 << a11, a12, a13, a12, a22, a23, a13, a23, a33;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            found_global = true;
        float x1 = cx + 0.1 * unit_direction[0];
        float y1 = cy + 0.1 * unit_direction[1];
        float z1 = cz + 0.1 * unit_direction[2];
        float x2 = cx - 0.1 * unit_direction[0];
        float y2 = cy - 0.1 * unit_direction[1];
        float z2 = cz - 0.1 * unit_direction[2];

        Eigen::Vector3d tripod1(x1, y1, z1);
        Eigen::Vector3d tripod2(x2, y2, z2);
            auto* e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                     tripod1, tripod2, Tbl,
                                                     Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
            thread_edges[tid].push_back(e);
            thread_features[tid].emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z), tripod1, tripod2);
            thread_features[tid].back().ComputeError(m4d);
          }
        }
      }

      if(!found_global && localCloudSize > 20){
      kdtreeLocal->nearestKSearch(_pointSel, 5, knn_idx_local, knn_dist_local);
      for(int j=0;j<5;++j){ _pointSearchInd2[j]=knn_idx_local[j]; _pointSearchSqDis2[j]=knn_dist_local[j]; }
      if (_pointSearchSqDis2[4] < thres_dist) {
          float cx = 0, cy = 0, cz = 0;
          const PointType* __restrict__ lpts = laserCloudCornerLocal->points.data();
          #pragma GCC unroll 5
        for (int j = 0; j < 5; j++) {
          const auto& pj = lpts[_pointSearchInd2[j]];
          cx += pj.x;
          cy += pj.y;
          cz += pj.z;
        }
          cx /= 5; cy /= 5; cz /= 5;

          float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
          #pragma GCC unroll 5
        for (int j = 0; j < 5; j++) {
          const auto& pj = lpts[_pointSearchInd2[j]];
          float ax = pj.x - cx;
          float ay = pj.y - cy;
          float az = pj.z - cz;
            a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
            a22 += ay * ay; a23 += ay * az; a33 += az * az;
        }
          a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

          _matA1 << a11, a12, a13, a12, a22, a23, a13, a23, a33;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);

          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          float x1 = cx + 0.1 * unit_direction[0];
          float y1 = cy + 0.1 * unit_direction[1];
          float z1 = cz + 0.1 * unit_direction[2];
          float x2 = cx - 0.1 * unit_direction[0];
          float y2 = cy - 0.1 * unit_direction[1];
          float z2 = cz - 0.1 * unit_direction[2];

          Eigen::Vector3d tripod1(x1, y1, z1);
          Eigen::Vector3d tripod2(x2, y2, z2);
            auto* e = Cost_NavState_IMU_Line::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                    tripod1, tripod2, Tbl,
                                                    Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
            thread_edges[tid].push_back(e);
            thread_features[tid].emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z), tripod1, tripod2);
            thread_features[tid].back().ComputeError(m4d);
        }
        }
      }
    }
  } // end parallel

  // 串行合并结果
  for(int t = 0; t < max_threads; t++){
    edges.insert(edges.end(), thread_edges[t].begin(), thread_edges[t].end());
    vLineFeatures.insert(vLineFeatures.end(), thread_features[t].begin(), thread_features[t].end());
  }
}

void Estimator::processPointToPlan(std::vector<ceres::CostFunction *>& edges,
                                   std::vector<FeaturePlan>& vPlanFeatures,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudSurf,
                                   const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLocal,
                                   const LocalKdTreePtr& kdtreeLocal,
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
  std::array<int,5>   _pointSearchInd{};
  std::array<float,5> _pointSearchSqDis{};
  std::array<int,5>   _pointSearchInd2{};
  std::array<float,5> _pointSearchSqDis2{};
  std::vector<int> knn_idx_global(5);
  std::vector<float> knn_dist_global(5);
  std::vector<int> knn_idx_local(5);
  std::vector<float> knn_dist_local(5);

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

    if(GlobalSurfMap[id].points.size() > 50) {
      SurfKdMap[id].nearestKSearch(_pointSel, 5, knn_idx_global, knn_dist_global);
      for(int j=0;j<5;++j){ _pointSearchInd[j]=knn_idx_global[j]; _pointSearchSqDis[j]=knn_dist_global[j]; }

      if (_pointSearchSqDis[4] < 1.0) {
        debug_num1 ++;
        const PointType* __restrict__ gpts = GlobalSurfMap[id].points.data();
        for (int j = 0; j < 5; j++) {
          const auto& pj = gpts[_pointSearchInd[j]];
          _matA0(j, 0) = pj.x;
          _matA0(j, 1) = pj.y;
          _matA0(j, 2) = pj.z;
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
          const auto& pj = gpts[_pointSearchInd[j]];
          if (std::fabs(pa * pj.x + pb * pj.y + pc * pj.z + pd) > 0.2) {
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
    kdtreeLocal->nearestKSearch(_pointSel, 5, knn_idx_local, knn_dist_local);
    for(int j=0;j<5;++j){ _pointSearchInd2[j]=knn_idx_local[j]; _pointSearchSqDis2[j]=knn_dist_local[j]; }
    if (_pointSearchSqDis2[4] < 1.0) {
      debug_num2++;
      const PointType* __restrict__ lpts = laserCloudSurfLocal->points.data();
      for (int j = 0; j < 5; j++) { 
        const auto& pj = lpts[_pointSearchInd2[j]];
        _matA0(j, 0) = pj.x;
        _matA0(j, 1) = pj.y;
        _matA0(j, 2) = pj.z;
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
          const auto& pj = lpts[_pointSearchInd2[j]];
          if (std::fabs(pa * pj.x + pb * pj.y + pc * pj.z + pd) > 0.2) {
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
                                   const LocalKdTreePtr& kdtreeLocal,
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

  int laserCloudSurfStackNum = laserCloudSurf->points.size();
  int localCloudSize = laserCloudSurfLocal->points.size();

  // 预分配线程局部结果数组
  int max_threads = omp_get_max_threads();
  std::vector<std::vector<ceres::CostFunction*>> thread_edges(max_threads);
  std::vector<std::vector<FeaturePlanVec>> thread_features(max_threads);
  for(int t = 0; t < max_threads; t++){
    thread_edges[t].reserve(laserCloudSurfStackNum / max_threads + 100);
    thread_features[t].reserve(laserCloudSurfStackNum / max_threads + 100);
  }

  #pragma omp parallel
  {
    int tid = omp_get_thread_num();
    // 线程私有变量
    PointType _pointOri, _pointSel;
    std::array<int,5>   _pointSearchInd{};
    std::array<float,5> _pointSearchSqDis{};
    std::array<int,5>   _pointSearchInd2{};
    std::array<float,5> _pointSearchSqDis2{};
    std::vector<int> knn_idx_global(5);
    std::vector<float> knn_dist_global(5);
    std::vector<int> knn_idx_local(5);
    std::vector<float> knn_dist_local(5);
    Eigen::Matrix<double, 5, 3> _matA0;
    Eigen::Matrix<double, 5, 1> _matB0;
    _matB0.setOnes(); _matB0 *= -1;
    Eigen::Matrix<double, 3, 1> _matX0;

    // === 优化点4：RK3588 big.LITTLE 架构适配 ===
    #pragma omp for schedule(dynamic, 16) nowait
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    _pointOri = laserCloudSurf->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);

    int id = map_manager->FindUsedSurfMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);
    if(id == 5000) continue;
    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

      bool found_global = false;
      if(GlobalSurfMap[id].points.size() > 50) {
        SurfKdMap[id].nearestKSearch(_pointSel, 5, knn_idx_global, knn_dist_global);
        for(int j=0;j<5;++j){ _pointSearchInd[j]=knn_idx_global[j]; _pointSearchSqDis[j]=knn_dist_global[j]; }

      if (_pointSearchSqDis[4] < thres_dist) {
          const PointType* __restrict__ gpts = GlobalSurfMap[id].points.data();
        for (int j = 0; j < 5; j++) {
            const auto& pj = gpts[_pointSearchInd[j]];
            _matA0(j, 0) = pj.x;
            _matA0(j, 1) = pj.y;
            _matA0(j, 2) = pj.z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0), pb = _matX0(1, 0), pc = _matX0(2, 0), pd = 1.0f;
        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          if (!std::isfinite(ps) || ps < 1e-6f) continue;
          pa /= ps; pb /= ps; pc /= ps; pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
            const auto& pj = gpts[_pointSearchInd[j]];
            if (std::fabs(pa * pj.x +
                          pb * pj.y +
                          pc * pj.z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
            found_global = true;
            double dist = pa * _pointSel.x + pb * _pointSel.y + pc * _pointSel.z + pd;
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
                                                         point_proj, Tbl, sqrt_info);
            thread_edges[tid].push_back(e);
            thread_features[tid].emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z), point_proj, sqrt_info);
            thread_features[tid].back().ComputeError(m4d);
          }
      }
    }

      if(!found_global && localCloudSize > 20){
    kdtreeLocal->nearestKSearch(_pointSel, 5, knn_idx_local, knn_dist_local);
    for(int j=0;j<5;++j){ _pointSearchInd2[j]=knn_idx_local[j]; _pointSearchSqDis2[j]=knn_dist_local[j]; }
    if (_pointSearchSqDis2[4] < thres_dist) {
      const PointType* __restrict__ lpts = laserCloudSurfLocal->points.data();
      for (int j = 0; j < 5; j++) { 
        const auto& pj = lpts[_pointSearchInd2[j]];
        _matA0(j, 0) = pj.x;
        _matA0(j, 1) = pj.y;
        _matA0(j, 2) = pj.z;
      }
      _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0), pb = _matX0(1, 0), pc = _matX0(2, 0), pd = 1.0f;
      float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          if (!std::isfinite(ps) || ps < 1e-6f) continue;
          pa /= ps; pb /= ps; pc /= ps; pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        const auto& pj = lpts[_pointSearchInd2[j]];
        if (std::fabs(pa * pj.x +
                      pb * pj.y +
                      pc * pj.z + pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
            double dist = pa * _pointSel.x + pb * _pointSel.y + pc * _pointSel.z + pd;
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
                                                          point_proj, Tbl, sqrt_info);
            thread_edges[tid].push_back(e);
            thread_features[tid].emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z), point_proj, sqrt_info);
            thread_features[tid].back().ComputeError(m4d);
      }
    }
  }
    }
  } // end parallel

  // 串行合并结果
  for(int t = 0; t < max_threads; t++){
    edges.insert(edges.end(), thread_edges[t].begin(), thread_edges[t].end());
    vPlanFeatures.insert(vPlanFeatures.end(), thread_features[t].begin(), thread_features[t].end());
  }
}


void Estimator::processNonFeatureICP(std::vector<ceres::CostFunction *>& edges,
                                     std::vector<FeatureNon>& vNonFeatures,
                                     const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeature,
                                     const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureLocal,
                                     const LocalKdTreePtr& kdtreeLocal,
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

  int laserCloudNonFeatureStackNum = laserCloudNonFeature->points.size();
  int localCloudSize = laserCloudNonFeatureLocal->points.size();

  // 预分配线程局部结果数组
  int max_threads = omp_get_max_threads();
  std::vector<std::vector<ceres::CostFunction*>> thread_edges(max_threads);
  std::vector<std::vector<FeatureNon>> thread_features(max_threads);
  for(int t = 0; t < max_threads; t++){
    thread_edges[t].reserve(laserCloudNonFeatureStackNum / max_threads + 100);
    thread_features[t].reserve(laserCloudNonFeatureStackNum / max_threads + 100);
  }

  #pragma omp parallel
  {
    int tid = omp_get_thread_num();
    // 线程私有变量
    PointType _pointOri, _pointSel;
    std::array<int,5>   _pointSearchInd{};
    std::array<float,5> _pointSearchSqDis{};
    std::array<int,5>   _pointSearchInd2{};
    std::array<float,5> _pointSearchSqDis2{};
    std::vector<int> knn_idx_global(5);
    std::vector<float> knn_dist_global(5);
    std::vector<int> knn_idx_local(5);
    std::vector<float> knn_dist_local(5);
    Eigen::Matrix<double, 5, 3> _matA0;
    Eigen::Matrix<double, 5, 1> _matB0;
    _matB0.setOnes(); _matB0 *= -1;
    Eigen::Matrix<double, 3, 1> _matX0;

    // === 优化点4：RK3588 big.LITTLE 架构适配 ===
    #pragma omp for schedule(dynamic, 16) nowait
  for (int i = 0; i < laserCloudNonFeatureStackNum; i++) {
    _pointOri = laserCloudNonFeature->points[i];
    MAP_MANAGER::pointAssociateToMap(&_pointOri, &_pointSel, m4d);
    int id = map_manager->FindUsedNonFeatureMap(&_pointSel,laserCenWidth_last,laserCenHeight_last,laserCenDepth_last);

    if(id == 5000) continue;
    if(std::isnan(_pointSel.x) || std::isnan(_pointSel.y) ||std::isnan(_pointSel.z)) continue;

      bool found_global = false;
      if(GlobalNonFeatureMap[id].points.size() > 100) {
        NonFeatureKdMap[id].nearestKSearch(_pointSel, 5, knn_idx_global, knn_dist_global);
        for(int j=0;j<5;++j){ _pointSearchInd[j]=knn_idx_global[j]; _pointSearchSqDis[j]=knn_dist_global[j]; }
        if (_pointSearchSqDis[4] < thres_dist) {
          const PointType* __restrict__ gpts = GlobalNonFeatureMap[id].points.data();
        for (int j = 0; j < 5; j++) {
            const auto& pj = gpts[_pointSearchInd[j]];
            _matA0(j, 0) = pj.x;
            _matA0(j, 1) = pj.y;
            _matA0(j, 2) = pj.z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0), pb = _matX0(1, 0), pc = _matX0(2, 0), pd = 1.0f;
        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          if (!std::isfinite(ps) || ps < 1e-6f) continue;
          pa /= ps; pb /= ps; pc /= ps; pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
            const auto& pj = gpts[_pointSearchInd[j]];
            if (std::fabs(pa * pj.x +
                          pb * pj.y +
                          pc * pj.z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if(planeValid) {
            found_global = true;
            auto* e = Cost_NonFeature_ICP::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                  pa, pb, pc, pd, Tbl,
                                                  Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
            thread_edges[tid].push_back(e);
            thread_features[tid].emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z), pa, pb, pc, pd);
            thread_features[tid].back().ComputeError(m4d);
        }
      }
    }

      if(!found_global && localCloudSize > 20){
      kdtreeLocal->nearestKSearch(_pointSel, 5, knn_idx_local, knn_dist_local);
      for(int j=0;j<5;++j){ _pointSearchInd2[j]=knn_idx_local[j]; _pointSearchSqDis2[j]=knn_dist_local[j]; }
        if (_pointSearchSqDis2[4] < thres_dist) {
        const PointType* __restrict__ lpts = laserCloudNonFeatureLocal->points.data();
        for (int j = 0; j < 5; j++) { 
          const auto& pj = lpts[_pointSearchInd2[j]];
          _matA0(j, 0) = pj.x;
          _matA0(j, 1) = pj.y;
          _matA0(j, 2) = pj.z;
        }
        _matX0 = _matA0.colPivHouseholderQr().solve(_matB0);

          float pa = _matX0(0, 0), pb = _matX0(1, 0), pc = _matX0(2, 0), pd = 1.0f;
        float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
          if (!std::isfinite(ps) || ps < 1e-6f) continue;
          pa /= ps; pb /= ps; pc /= ps; pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          const auto& pj = lpts[_pointSearchInd2[j]];
          if (std::fabs(pa * pj.x +
                        pb * pj.y +
                        pc * pj.z + pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if(planeValid) {
            auto* e = Cost_NonFeature_ICP::Create(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z),
                                                  pa, pb, pc, pd, Tbl,
                                                  Eigen::Matrix<double, 1, 1>(1/IMUIntegrator::lidar_m));
            thread_edges[tid].push_back(e);
            thread_features[tid].emplace_back(Eigen::Vector3d(_pointOri.x,_pointOri.y,_pointOri.z), pa, pb, pc, pd);
            thread_features[tid].back().ComputeError(m4d);
        }
      }
    }
  }
  } // end parallel

  // 串行合并结果
  for(int t = 0; t < max_threads; t++){
    edges.insert(edges.end(), thread_edges[t].begin(), thread_edges[t].end());
    vNonFeatures.insert(vNonFeatures.end(), thread_features[t].begin(), thread_features[t].end());
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

  // === 零拷贝 MapSnapshot：获取快照指针（无需持锁）===
  const int cubeNum = CUBE_NUM;  // 使用全局常量
  const MapSnapshot* snapshot = map_manager->AcquireSnapshot();
  if (!snapshot) {
    // 第一帧还没有有效快照，使用旧方法
    std::unique_lock<std::mutex> locker3(map_manager->mtx_MapManager);
    #pragma omp parallel for schedule(static, 256)
    for(int i = 0; i < cubeNum; i++){
      CornerKdMap[i] = map_manager->getCornerKdMap(i);
      SurfKdMap[i] = map_manager->getSurfKdMap(i);
      NonFeatureKdMap[i] = map_manager->getNonFeatureKdMap(i);
      GlobalSurfMap[i] = map_manager->laserCloudSurf_for_match[i];
      GlobalCornerMap[i] = map_manager->laserCloudCorner_for_match[i];
      GlobalNonFeatureMap[i] = map_manager->laserCloudNonFeature_for_match[i];
  }
    laserCenWidth_last = map_manager->get_laserCloudCenWidth_last();
    laserCenHeight_last = map_manager->get_laserCloudCenHeight_last();
    laserCenDepth_last = map_manager->get_laserCloudCenDepth_last();
    locker3.unlock();
  } else {
    // 使用 MapSnapshot（无锁并行拷贝）
    #pragma omp parallel for schedule(static, 256)
    for(int i = 0; i < cubeNum; i++){
      CornerKdMap[i] = *(snapshot->cornerKdMap[i]);
      SurfKdMap[i] = *(snapshot->surfKdMap[i]);
      NonFeatureKdMap[i] = *(snapshot->nonFeatureKdMap[i]);
      GlobalSurfMap[i] = *(snapshot->surfPointMap[i]);
      GlobalCornerMap[i] = *(snapshot->cornerPointMap[i]);
      GlobalNonFeatureMap[i] = *(snapshot->nonFeaturePointMap[i]);
  }
    laserCenWidth_last = snapshot->cenWidth;
    laserCenHeight_last = snapshot->cenHeight;
    laserCenDepth_last = snapshot->cenDepth;
    
    // 释放快照
    map_manager->ReleaseSnapshot();
  }

  // === 优化点1：使用预分配的成员变量，只 clear 不重新分配 ===
  for(int f = 0; f < windowSize; ++f) {
    vLineFeatures_[f].clear();
    vPlanFeatures_[f].clear();
    vNonFeatures_[f].clear();
    edgesLine_[f].clear();
    edgesPlan_[f].clear();
    edgesNon_[f].clear();
  }
  // 使用成员变量引用（避免修改后面大量代码）
  auto& vLineFeatures = vLineFeatures_;
  auto& vPlanFeatures = vPlanFeatures_;
  auto& vNonFeatures = vNonFeatures_;
  auto& edgesLine = edgesLine_;
  auto& edgesPlan = edgesPlan_;
  auto& edgesNon = edgesNon_;

  if(windowSize == SLIDEWINDOWSIZE) {
    plan_weight_tan = 0.0003;
    thres_dist = 1.0;
  } else {
    plan_weight_tan = 0.0;
    thres_dist = 25.0;
  }

  // excute optimize process
  // 动态迭代次数：初始化阶段少迭代，稳定后多迭代
  const int max_iters = (windowSize == SLIDEWINDOWSIZE) ? max_iterations_ : std::max(1, max_iterations_ - 1);
  bool converged = false;
  for(int iterOpt=0; iterOpt<max_iters && !converged; ++iterOpt){

    vector2double(lidarFrameList);

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

    std::vector<std::vector<ceres::CostFunction *>> edgesLine(windowSize);
    std::vector<std::vector<ceres::CostFunction *>> edgesPlan(windowSize);
    std::vector<std::vector<ceres::CostFunction *>> edgesNon(windowSize);
    
    // === 多帧并行优化：同时处理所有帧的所有特征类型 ===
    // windowSize=2 时，6个线程同时运行（2帧 × 3特征类型）
    const int totalThreads = windowSize * 3;
    std::vector<std::thread> threads(totalThreads);
    std::vector<Eigen::Matrix4d> transforms(windowSize);  // 每帧独立的变换矩阵
    
    // 预计算所有帧的变换矩阵
    for(int f=0; f<windowSize; ++f) {
      auto frame_curr = lidarFrameList.begin();
      std::advance(frame_curr, f);
      transforms[f] = Eigen::Matrix4d::Identity();
      transforms[f].topLeftCorner(3,3) = frame_curr->Q * exRbl;
      transforms[f].topRightCorner(3,1) = frame_curr->Q * exPbl + frame_curr->P;
    }
    
    // 同时启动所有线程（多帧并行）
    for(int f=0; f<windowSize; ++f) {
      threads[f*3 + 0] = std::thread(&Estimator::processPointToLine, this,
                               std::ref(edgesLine[f]),
                               std::ref(vLineFeatures[f]),
                               std::ref(laserCloudCornerStack[f]),
                               std::ref(laserCloudCornerFromLocal),
                               std::ref(kdtreeCornerFromLocal),
                               std::ref(exTlb),
                               std::ref(transforms[f]));

      threads[f*3 + 1] = std::thread(&Estimator::processPointToPlanVec, this,
                               std::ref(edgesPlan[f]),
                               std::ref(vPlanFeatures[f]),
                               std::ref(laserCloudSurfStack[f]),
                               std::ref(laserCloudSurfFromLocal),
                               std::ref(kdtreeSurfFromLocal),
                               std::ref(exTlb),
                               std::ref(transforms[f]));

      threads[f*3 + 2] = std::thread(&Estimator::processNonFeatureICP, this,
                               std::ref(edgesNon[f]),
                               std::ref(vNonFeatures[f]),
                               std::ref(laserCloudNonFeatureStack[f]),
                               std::ref(laserCloudNonFeatureFromLocal),
                               std::ref(kdtreeNonFeatureFromLocal),
                               std::ref(exTlb),
                               std::ref(transforms[f]));
    }
    
    // 等待所有线程完成
    for(int i=0; i<totalThreads; ++i) {
      threads[i].join();
      }

    int cntSurf = 0;
    int cntCorner = 0;
    int cntNon = 0;
      if(windowSize == SLIDEWINDOWSIZE) {
        thres_dist = 1.0;
        if(iterOpt == 0){
          for(int f=0; f<windowSize; ++f){
          int cntFtu = 0;
          for (auto &e : edgesLine[f]) {
            if(std::fabs(vLineFeatures[f][cntFtu].error) > 1e-5){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vLineFeatures[f][cntFtu].valid = true;
              }else{
              vLineFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
            cntCorner++;
              }

          cntFtu = 0;
          for (auto &e : edgesPlan[f]) {
            if(std::fabs(vPlanFeatures[f][cntFtu].error) > 1e-5){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vPlanFeatures[f][cntFtu].valid = true;
            }else{
              vPlanFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
            cntSurf++;
          }

          cntFtu = 0;
          for (auto &e : edgesNon[f]) {
            if(std::fabs(vNonFeatures[f][cntFtu].error) > 1e-5){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vNonFeatures[f][cntFtu].valid = true;
            }else{
              vNonFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
            cntNon++;
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
          int cntFtu = 0;
          for (auto &e : edgesLine[f]) {
            if(std::fabs(vLineFeatures[f][cntFtu].error) > 1e-5){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vLineFeatures[f][cntFtu].valid = true;
              }else{
              vLineFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
            cntCorner++;
              }
          cntFtu = 0;
          for (auto &e : edgesPlan[f]) {
            if(std::fabs(vPlanFeatures[f][cntFtu].error) > 1e-5){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vPlanFeatures[f][cntFtu].valid = true;
              }else{
              vPlanFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
            cntSurf++;
          }

          cntFtu = 0;
          for (auto &e : edgesNon[f]) {
            if(std::fabs(vNonFeatures[f][cntFtu].error) > 1e-5){
              problem.AddResidualBlock(e, loss_function, para_PR[f]);
              vNonFeatures[f][cntFtu].valid = true;
            }else{
              vNonFeatures[f][cntFtu].valid = false;
            }
            cntFtu++;
            cntNon++;
          }
            }
          }

      // === 优化点5：退化检测（仅首次迭代统计）===
      if (iterOpt == 0) {
        valid_corner_count_ = cntCorner;
        valid_surf_count_ = cntSurf;
        checkDegeneracy();
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.trust_region_strategy_type = ceres::DOGLEG;
      options.max_num_iterations = ceres_max_iterations_;
      options.minimizer_progress_to_stdout = false;
      options.num_threads = 8;  // RK3588: 4大核 + 4小核
      // === 优化：Ceres 配置加速（不影响精度）===
      options.function_tolerance = 1e-5;        // 函数值收敛容差
      options.gradient_tolerance = 1e-8;        // 梯度收敛容差
      options.parameter_tolerance = 1e-6;       // 参数收敛容差
      options.use_nonmonotonic_steps = true;    // 允许非单调步长（加速收敛）
      options.max_consecutive_nonmonotonic_steps = 4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double2vector(lidarFrameList);

    Eigen::Quaterniond q_after_opti = lidarFrameList.back().Q;
    Eigen::Vector3d t_after_opti = lidarFrameList.back().P;
    Eigen::Vector3d V_after_opti = lidarFrameList.back().V;
    double deltaR = (q_before_opti.angularDistance(q_after_opti)) * 180.0 / M_PI;
    double deltaT = (t_before_opti - t_after_opti).norm();

    // 动态收敛检测：位姿变化足够小则提前终止
    if (deltaR < convergence_threshold_r_ && deltaT < convergence_threshold_t_) {
      converged = true;
    }

    if (converged || (iterOpt+1) == max_iters){
      ROS_INFO("Frame: %d, iters: %d\n", frame_count++, iterOpt+1);
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
      
      // 边缘化阶段的并行处理
      std::thread marg_threads[3];
      marg_threads[0] = std::thread(&Estimator::processPointToLine, this,
                                        std::ref(edgesLine[f]),
                                        std::ref(vLineFeatures[f]),
                                        std::ref(laserCloudCornerStack[f]),
                                        std::ref(laserCloudCornerFromLocal),
                                        std::ref(kdtreeCornerFromLocal),
                                        std::ref(exTlb),
                               std::ref(transformTobeMapped));

      marg_threads[1] = std::thread(&Estimator::processPointToPlanVec, this,
                                        std::ref(edgesPlan[f]),
                                        std::ref(vPlanFeatures[f]),
                                        std::ref(laserCloudSurfStack[f]),
                                        std::ref(laserCloudSurfFromLocal),
                                        std::ref(kdtreeSurfFromLocal),
                                        std::ref(exTlb),
                                        std::ref(transformTobeMapped));

      marg_threads[2] = std::thread(&Estimator::processNonFeatureICP, this,
                                        std::ref(edgesNon[f]),
                                        std::ref(vNonFeatures[f]),
                                        std::ref(laserCloudNonFeatureStack[f]),
                                        std::ref(laserCloudNonFeatureFromLocal),
                                        std::ref(kdtreeNonFeatureFromLocal),
                                        std::ref(exTlb),
                                        std::ref(transformTobeMapped));      
                    
      marg_threads[0].join();
      marg_threads[1].join();
      marg_threads[2].join();
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
void Estimator::MapIncrementLocal(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerStack,
                                  const pcl::PointCloud<PointType>::Ptr& laserCloudSurfStack,
                                  const pcl::PointCloud<PointType>::Ptr& laserCloudNonFeatureStack,
                                  const Eigen::Matrix4d& transformTobeMapped){
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
  int laserCloudNonFeatureStackNum = laserCloudNonFeatureStack->points.size();
  PointType pointSel, pointSel2;
  size_t Id = localMapID % localMapWindowSize;
  localCornerMap[Id]->clear();
  localSurfMap[Id]->clear();
  localNonFeatureMap[Id]->clear();
  
  // 点云变换（保持原始串行逻辑，避免线程问题）
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

  // === 合并局部地图（串行，因为 += 操作不是线程安全的）===
  for (int i = 0; i < localMapWindowSize; i++) {
    *laserCloudCornerFromLocal += *localCornerMap[i];
    *laserCloudSurfFromLocal += *localSurfMap[i];
    *laserCloudNonFeatureFromLocal += *localNonFeatureMap[i];
    }
  
  // === 下采样（串行以避免线程问题）===
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

// === 优化点5：隧道/退化场景检测 ===
void Estimator::checkDegeneracy() {
  // 计算特征丰富度比率
  const float corner_ratio = (valid_corner_count_ > 0) ? 
      static_cast<float>(valid_corner_count_) / MAX_FEATURES_PER_FRAME : 0.0f;
  const float surf_ratio = (valid_surf_count_ > 0) ? 
      static_cast<float>(valid_surf_count_) / MAX_FEATURES_PER_FRAME : 0.0f;
  
  // 退化条件：角点 < 10% 或 面点 < 15%
  bool was_degenerate = is_degenerate_;
  is_degenerate_ = (corner_ratio < 0.10f) || (surf_ratio < 0.15f);
  
  if (is_degenerate_ && !was_degenerate) {
    ROS_WARN("[Degeneracy] Detected! Corner: %.1f%%, Surf: %.1f%% - Switching to IMU-primary mode",
             corner_ratio * 100.0f, surf_ratio * 100.0f);
  } else if (!is_degenerate_ && was_degenerate) {
    ROS_INFO("[Degeneracy] Recovered. Corner: %.1f%%, Surf: %.1f%%",
             corner_ratio * 100.0f, surf_ratio * 100.0f);
  }
}