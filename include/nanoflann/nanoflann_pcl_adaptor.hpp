/**
 * @file nanoflann_pcl_adaptor.hpp
 * @brief nanoflann adaptor for PCL PointCloud
 * 
 * 将 PCL PointCloud 适配到 nanoflann KD-tree 接口
 */

#pragma once

#include <nanoflann/nanoflann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

namespace nanoflann_pcl {

/**
 * @brief PCL PointCloud 适配器模板
 * @tparam PointT PCL 点类型 (需要有 x, y, z 成员)
 */
template <typename PointT>
struct PointCloudAdaptor {
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    const PointCloud* cloud;

    PointCloudAdaptor() : cloud(nullptr) {}
    
    explicit PointCloudAdaptor(const PointCloud* cloud_) : cloud(cloud_) {}
    
    void setInputCloud(const PointCloud* cloud_) { cloud = cloud_; }
    void setInputCloud(const PointCloudPtr& cloud_) { cloud = cloud_.get(); }

    // nanoflann 必需接口：返回点数
    inline size_t kdtree_get_point_count() const {
        return cloud ? cloud->points.size() : 0;
    }

    // nanoflann 必需接口：返回第 idx 点的第 dim 维坐标
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return cloud->points[idx].x;
        if (dim == 1) return cloud->points[idx].y;
        return cloud->points[idx].z;
    }

    // nanoflann 可选接口：边界框（返回 false 让 nanoflann 自己计算）
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

/**
 * @brief 基于 nanoflann 的 KD-tree，兼容 PCL KdTreeFLANN 接口
 * @tparam PointT PCL 点类型
 */
template <typename PointT>
class KdTreeNano {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;
    
    // nanoflann KD-tree 类型定义
    using Adaptor = PointCloudAdaptor<PointT>;
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, Adaptor>,
        Adaptor,
        3,      // 3维
        size_t  // 索引类型
    >;

    KdTreeNano() : adaptor_(), tree_(nullptr) {}
    
    ~KdTreeNano() = default;

    /**
     * @brief 设置输入点云并构建 KD-tree
     * @param cloud 输入点云
     */
    void setInputCloud(const PointCloudPtr& cloud) {
        if (!cloud || cloud->empty()) {
            tree_.reset();
            return;
        }
        
        adaptor_.setInputCloud(cloud.get());
        
        // 构建 KD-tree (叶子节点最大点数 = 10)
        tree_ = std::make_unique<KDTree>(
            3, adaptor_, 
            nanoflann::KDTreeSingleIndexAdaptorParams(10)
        );
        tree_->buildIndex();
    }

    /**
     * @brief K 近邻搜索（兼容 PCL KdTreeFLANN 接口）
     * @param point 查询点
     * @param k 近邻数量
     * @param k_indices 输出：近邻点索引
     * @param k_sqr_distances 输出：平方距离
     * @return 找到的近邻数量
     */
    int nearestKSearch(const PointT& point, int k,
                       std::vector<int>& k_indices,
                       std::vector<float>& k_sqr_distances) const {
        if (!tree_ || k <= 0) {
            k_indices.clear();
            k_sqr_distances.clear();
            return 0;
        }
        
        // 准备查询点
        float query_pt[3] = {point.x, point.y, point.z};
        
        // 准备结果容器（nanoflann 使用 size_t 索引）
        std::vector<size_t> ret_indices(k);
        std::vector<float> out_dists_sqr(k);
        
        // 执行 KNN 搜索
        nanoflann::KNNResultSet<float> resultSet(k);
        resultSet.init(ret_indices.data(), out_dists_sqr.data());
        
        tree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(32));
        
        // 转换结果（size_t -> int）
        size_t found = resultSet.size();
        k_indices.resize(found);
        k_sqr_distances.resize(found);
        
        for (size_t i = 0; i < found; ++i) {
            k_indices[i] = static_cast<int>(ret_indices[i]);
            k_sqr_distances[i] = out_dists_sqr[i];
        }
        
        return static_cast<int>(found);
    }

    /**
     * @brief 检查 KD-tree 是否已构建
     */
    bool isValid() const { return tree_ != nullptr; }

    /**
     * @brief 获取点云大小
     */
    size_t size() const { return adaptor_.kdtree_get_point_count(); }

private:
    Adaptor adaptor_;
    std::unique_ptr<KDTree> tree_;
};

}  // namespace nanoflann_pcl

