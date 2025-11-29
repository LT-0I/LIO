#ifndef LIO_LIVOX_VOXEL_INDEX_H
#define LIO_LIVOX_VOXEL_INDEX_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

/**
 * @brief VoxelIndex - O(1) approximate nearest neighbor search using spatial hashing
 *
 * This class provides a space-time tradeoff optimization for KD-Tree searches.
 * Instead of O(log N) KD-Tree searches, it uses O(1) hash table lookups.
 *
 * Designed for OrangePi 5 MAX (RK3588) with 16GB RAM.
 */
class VoxelIndex {
public:
    typedef pcl::PointXYZINormal PointType;
    typedef int64_t VoxelKey;

    /**
     * @brief Constructor
     * @param resolution Voxel size in meters (default 0.5m)
     */
    explicit VoxelIndex(float resolution = 0.5f);

    /**
     * @brief Build index from point cloud
     * @param cloud Input point cloud
     */
    void buildIndex(const pcl::PointCloud<PointType>::Ptr& cloud);

    /**
     * @brief Find k-nearest neighbors within radius
     * @param query Query point
     * @param radius Search radius in meters
     * @param indices Output indices of found points
     * @param sqr_distances Output squared distances to found points
     * @param max_neighbors Maximum number of neighbors to return
     * @return Number of neighbors found
     */
    int radiusSearch(const PointType& query,
                     float radius,
                     std::vector<int>& indices,
                     std::vector<float>& sqr_distances,
                     int max_neighbors = 5) const;

    /**
     * @brief Clear the index
     */
    void clear();

    /**
     * @brief Check if index is empty
     */
    bool empty() const { return cloud_ == nullptr || cloud_->empty(); }

    /**
     * @brief Get the number of indexed points
     */
    size_t size() const { return cloud_ ? cloud_->size() : 0; }

    /**
     * @brief Get the number of non-empty voxels
     */
    size_t numVoxels() const { return voxel_map_.size(); }

private:
    float resolution_;
    float inv_resolution_;
    pcl::PointCloud<PointType>::Ptr cloud_;
    std::unordered_map<VoxelKey, std::vector<int>> voxel_map_;

    // Offset for converting signed coordinates to unsigned keys
    static constexpr int KEY_OFFSET = 0x100000;  // 2^20, supports ±500km at 0.5m resolution

    /**
     * @brief Compute voxel key from 3D coordinates
     */
    VoxelKey computeKey(float x, float y, float z) const;

    /**
     * @brief Decode voxel key to grid indices
     */
    void decodeKey(VoxelKey key, int& ix, int& iy, int& iz) const;

    /**
     * @brief Get 27-neighborhood voxel keys (center + 26 neighbors)
     */
    void getNeighborKeys(VoxelKey center_key, std::vector<VoxelKey>& keys) const;
};

#endif // LIO_LIVOX_VOXEL_INDEX_H
