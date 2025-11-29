#include "utils/VoxelIndex.h"

VoxelIndex::VoxelIndex(float resolution)
    : resolution_(resolution),
      inv_resolution_(1.0f / resolution),
      cloud_(nullptr) {
}

void VoxelIndex::buildIndex(const pcl::PointCloud<PointType>::Ptr& cloud) {
    clear();
    if (!cloud || cloud->empty()) {
        return;
    }

    cloud_ = cloud;

    // Reserve approximate capacity to reduce rehashing
    // Assuming average ~10 points per voxel
    voxel_map_.reserve(cloud->size() / 10 + 1000);

    // Insert all points into voxel grid
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        VoxelKey key = computeKey(pt.x, pt.y, pt.z);
        voxel_map_[key].push_back(static_cast<int>(i));
    }
}

int VoxelIndex::radiusSearch(const PointType& query,
                              float radius,
                              std::vector<int>& indices,
                              std::vector<float>& sqr_distances,
                              int max_neighbors) const {
    indices.clear();
    sqr_distances.clear();

    if (!cloud_ || cloud_->empty()) {
        return 0;
    }

    // Get center voxel and determine search extent
    const float radius_sq = radius * radius;

    // Calculate how many voxels we need to search in each direction
    // For radius search, we need ceil(radius / resolution) voxels
    const int search_extent = static_cast<int>(std::ceil(radius * inv_resolution_));

    // Get center voxel indices
    int cx = static_cast<int>(std::floor(query.x * inv_resolution_));
    int cy = static_cast<int>(std::floor(query.y * inv_resolution_));
    int cz = static_cast<int>(std::floor(query.z * inv_resolution_));

    // Collect candidates from nearby voxels
    std::vector<std::pair<float, int>> candidates;
    candidates.reserve(max_neighbors * 4);  // Pre-allocate for typical case

    // Search in cubic neighborhood
    for (int dx = -search_extent; dx <= search_extent; ++dx) {
        for (int dy = -search_extent; dy <= search_extent; ++dy) {
            for (int dz = -search_extent; dz <= search_extent; ++dz) {
                int ix = cx + dx;
                int iy = cy + dy;
                int iz = cz + dz;

                // Compute key directly to avoid function call overhead
                VoxelKey key = (static_cast<int64_t>(ix + KEY_OFFSET) << 40) |
                               (static_cast<int64_t>(iy + KEY_OFFSET) << 20) |
                               static_cast<int64_t>(iz + KEY_OFFSET);

                auto it = voxel_map_.find(key);
                if (it == voxel_map_.end()) {
                    continue;
                }

                for (int idx : it->second) {
                    const auto& pt = cloud_->points[idx];
                    float dx_pt = pt.x - query.x;
                    float dy_pt = pt.y - query.y;
                    float dz_pt = pt.z - query.z;
                    float sqr_dist = dx_pt * dx_pt + dy_pt * dy_pt + dz_pt * dz_pt;

                    if (sqr_dist <= radius_sq) {
                        candidates.emplace_back(sqr_dist, idx);
                    }
                }
            }
        }
    }

    if (candidates.empty()) {
        return 0;
    }

    // Sort by distance and take top k
    const size_t result_count = std::min(static_cast<size_t>(max_neighbors), candidates.size());

    if (candidates.size() > result_count) {
        std::partial_sort(candidates.begin(),
                          candidates.begin() + result_count,
                          candidates.end());
    } else {
        std::sort(candidates.begin(), candidates.end());
    }

    // Copy results
    indices.reserve(result_count);
    sqr_distances.reserve(result_count);

    for (size_t i = 0; i < result_count; ++i) {
        indices.push_back(candidates[i].second);
        sqr_distances.push_back(candidates[i].first);
    }

    return static_cast<int>(result_count);
}

void VoxelIndex::clear() {
    voxel_map_.clear();
    cloud_.reset();
}

VoxelIndex::VoxelKey VoxelIndex::computeKey(float x, float y, float z) const {
    int ix = static_cast<int>(std::floor(x * inv_resolution_));
    int iy = static_cast<int>(std::floor(y * inv_resolution_));
    int iz = static_cast<int>(std::floor(z * inv_resolution_));

    // Pack three 20-bit signed integers into a 64-bit key
    // Each dimension supports range [-KEY_OFFSET, KEY_OFFSET-1] * resolution
    return (static_cast<int64_t>(ix + KEY_OFFSET) << 40) |
           (static_cast<int64_t>(iy + KEY_OFFSET) << 20) |
           static_cast<int64_t>(iz + KEY_OFFSET);
}

void VoxelIndex::decodeKey(VoxelKey key, int& ix, int& iy, int& iz) const {
    iz = static_cast<int>(key & 0xFFFFF) - KEY_OFFSET;
    iy = static_cast<int>((key >> 20) & 0xFFFFF) - KEY_OFFSET;
    ix = static_cast<int>((key >> 40) & 0xFFFFF) - KEY_OFFSET;
}

void VoxelIndex::getNeighborKeys(VoxelKey center_key, std::vector<VoxelKey>& keys) const {
    keys.clear();
    keys.reserve(27);

    int cx, cy, cz;
    decodeKey(center_key, cx, cy, cz);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                int ix = cx + dx;
                int iy = cy + dy;
                int iz = cz + dz;
                VoxelKey key = (static_cast<int64_t>(ix + KEY_OFFSET) << 40) |
                               (static_cast<int64_t>(iy + KEY_OFFSET) << 20) |
                               static_cast<int64_t>(iz + KEY_OFFSET);
                keys.push_back(key);
            }
        }
    }
}
