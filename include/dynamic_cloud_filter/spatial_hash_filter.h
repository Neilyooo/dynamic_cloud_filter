/****************************************************************************************
 * Spatial Hash Point Cloud Filter
 *
 * Copyright (c) 2025, AutoCity
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * Uses spatial hashing to accelerate point-bbox queries
 *
 * Expected speedup: 2-5x over KD-tree for sparse bboxes
 *
 * License: MIT
 ****************************************************************************************/

#ifndef DYNAMIC_CLOUD_FILTER_SPATIAL_HASH_FILTER_H_
#define DYNAMIC_CLOUD_FILTER_SPATIAL_HASH_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <unordered_map>
#include <cmath>

namespace dynamic_cloud_filter {
namespace spatial_hash {

/**
 * @brief Cached bounding box data
 */
struct BBoxCached {
    float x, y, z;
    float cos_yaw, sin_yaw;
    float half_l, half_w, half_h;
    bool should_filter;
    int class_id;
};

/**
 * @brief 3D grid cell key for spatial hashing
 */
struct GridCell {
    int x, y, z;

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for GridCell
struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        // Use prime numbers for better distribution
        return ((cell.x * 73856093) ^ (cell.y * 19349663) ^ (cell.z * 83492791));
    }
};

/**
 * @brief Spatial hash filter using grid-based acceleration
 *
 * Divides space into uniform grid cells
 * Each bbox is registered to overlapping cells
 * Points only check bboxes in their cell + neighbors
 *
 * Time complexity: O(N*k) where k = avg bboxes per cell (typically k << M)
 */
class SpatialHashFilter {
public:
    SpatialHashFilter();

    void setDynamicClasses(const std::vector<int>& classes);
    void setParameters(float bbox_margin, float search_radius);

    /**
     * @brief Filter points using spatial hashing
     */
    template<typename BBoxType>
    void filterDynamicPoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        const std::vector<BBoxType>& bboxes,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud);

private:
    template<typename BBoxType>
    std::vector<BBoxCached> cacheBoundingBoxes(const std::vector<BBoxType>& bboxes);

    inline bool shouldFilterClass(int class_id) const {
        return (class_filter_mask_ & (1 << class_id)) != 0;
    }

    inline bool isPointInBBox(
        const pcl::PointXYZI& point,
        const BBoxCached& cached) const {

        float dx = point.x - cached.x;
        float dy = point.y - cached.y;
        float dz = point.z - cached.z;

        // OBB (Oriented Bounding Box) - More accurate but considers rotation
        // Commented out to match TRLO's AABB behavior
        // float local_x = cached.cos_yaw * dx - cached.sin_yaw * dy;
        // float local_y = cached.sin_yaw * dx + cached.cos_yaw * dy;
        // return (std::abs(local_x) <= cached.half_l &&
        //         std::abs(local_y) <= cached.half_w &&
        //         std::abs(dz) <= cached.half_h);

        // AABB (Axis-Aligned Bounding Box) - Matches TRLO, ignores rotation
        return (std::abs(dx) <= cached.half_l &&
                std::abs(dy) <= cached.half_w &&
                std::abs(dz) <= cached.half_h);
    }

    // Convert point to grid cell
    inline GridCell pointToCell(float x, float y, float z) const {
        return {
            static_cast<int>(std::floor(x / cell_size_)),
            static_cast<int>(std::floor(y / cell_size_)),
            static_cast<int>(std::floor(z / cell_size_))
        };
    }

    // Build spatial hash from bboxes
    void buildSpatialHash(const std::vector<BBoxCached>& bboxes);

    // Get bbox indices in cell and its 26 neighbors
    std::vector<int> getBBoxesNearPoint(float x, float y, float z) const;

    uint16_t class_filter_mask_;
    float bbox_margin_;
    float bbox_search_radius_;
    float search_radius_sq_;
    float cell_size_;  // Grid cell size

    // Spatial hash: cell -> list of bbox indices
    std::unordered_map<GridCell, std::vector<int>, GridCellHash> spatial_hash_;
    std::vector<BBoxCached> cached_bboxes_;
};

// Template implementation (must be in header)
template<typename BBoxType>
std::vector<BBoxCached> SpatialHashFilter::cacheBoundingBoxes(
    const std::vector<BBoxType>& bboxes) {

    std::vector<BBoxCached> cached;
    cached.reserve(bboxes.size());

    for (const auto& bbox : bboxes) {
        BBoxCached c;
        c.x = bbox.x;
        c.y = bbox.y;
        c.z = bbox.z;
        c.class_id = bbox.id;

        float yaw = -bbox.rt;
        c.cos_yaw = std::cos(yaw);
        c.sin_yaw = std::sin(yaw);

        c.half_l = (bbox.l * 0.5f) + bbox_margin_;
        c.half_w = (bbox.w * 0.5f) + bbox_margin_;
        c.half_h = (bbox.h * 0.5f) + bbox_margin_;

        c.should_filter = shouldFilterClass(bbox.id);

        cached.push_back(c);
    }

    return cached;
}

template<typename BBoxType>
void SpatialHashFilter::filterDynamicPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    const std::vector<BBoxType>& bboxes,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud) {

    static_cloud->clear();
    dynamic_cloud->clear();

    if (bboxes.empty()) {
        *static_cloud = *input_cloud;
        return;
    }

    // Pre-compute bbox data
    cached_bboxes_ = cacheBoundingBoxes(bboxes);

    // Build spatial hash
    buildSpatialHash(cached_bboxes_);

    // Pre-allocate memory
    static_cloud->reserve(input_cloud->size());
    dynamic_cloud->reserve(input_cloud->size() / 10);

    // Filter each point
    for (const auto& point : input_cloud->points) {
        bool is_dynamic = false;

        // Get nearby bboxes using spatial hash
        auto nearby_bbox_indices = getBBoxesNearPoint(point.x, point.y, point.z);

        // Check only nearby bboxes
        for (int idx : nearby_bbox_indices) {
            const BBoxCached& bbox = cached_bboxes_[idx];

            // Skip if class should not be filtered
            if (!bbox.should_filter) continue;

            // Fast distance check
            float dx = point.x - bbox.x;
            float dy = point.y - bbox.y;
            float dz = point.z - bbox.z;
            float dist_sq = dx*dx + dy*dy + dz*dz;

            if (dist_sq > search_radius_sq_) continue;

            // OBB test
            if (isPointInBBox(point, bbox)) {
                is_dynamic = true;
                break;
            }
        }

        if (is_dynamic) {
            dynamic_cloud->push_back(point);
        } else {
            static_cloud->push_back(point);
        }
    }
}

} // namespace spatial_hash
} // namespace dynamic_cloud_filter

#endif // DYNAMIC_CLOUD_FILTER_SPATIAL_HASH_FILTER_H_
