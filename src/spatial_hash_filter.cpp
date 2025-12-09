/****************************************************************************************
 * Spatial Hash Point Cloud Filter - Implementation
 *
 * Copyright (c) 2025, AutoCity
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * License: MIT
 ****************************************************************************************/

#include "dynamic_cloud_filter/spatial_hash_filter.h"
#include <cmath>

namespace dynamic_cloud_filter {
namespace spatial_hash {

SpatialHashFilter::SpatialHashFilter()
    : class_filter_mask_(0),
      bbox_margin_(0.2f),
      bbox_search_radius_(2.0f),
      search_radius_sq_(4.0f),
      cell_size_(5.0f) {  // 5m grid cells
}

void SpatialHashFilter::setDynamicClasses(const std::vector<int>& classes) {
    class_filter_mask_ = 0;
    for (int cls : classes) {
        if (cls >= 0 && cls < 16) {
            class_filter_mask_ |= (1 << cls);
        }
    }
}

void SpatialHashFilter::setParameters(float bbox_margin, float search_radius) {
    bbox_margin_ = bbox_margin;
    bbox_search_radius_ = search_radius;
    search_radius_sq_ = search_radius * search_radius;

    // Set cell size to 2x search radius for efficiency
    cell_size_ = search_radius * 2.0f;
}

// Template implementation moved to header

void SpatialHashFilter::buildSpatialHash(const std::vector<BBoxCached>& bboxes) {
    spatial_hash_.clear();

    for (int i = 0; i < bboxes.size(); ++i) {
        const auto& bbox = bboxes[i];

        // Calculate bbox extent (conservative: use max dimension + search radius)
        float max_dim = std::max({bbox.half_l, bbox.half_w, bbox.half_h});
        float extent = max_dim + bbox_search_radius_;

        // Register bbox to all cells it might overlap
        GridCell min_cell = pointToCell(bbox.x - extent, bbox.y - extent, bbox.z - extent);
        GridCell max_cell = pointToCell(bbox.x + extent, bbox.y + extent, bbox.z + extent);

        for (int x = min_cell.x; x <= max_cell.x; ++x) {
            for (int y = min_cell.y; y <= max_cell.y; ++y) {
                for (int z = min_cell.z; z <= max_cell.z; ++z) {
                    GridCell cell{x, y, z};
                    spatial_hash_[cell].push_back(i);
                }
            }
        }
    }
}

std::vector<int> SpatialHashFilter::getBBoxesNearPoint(float x, float y, float z) const {
    std::vector<int> bbox_indices;
    GridCell center_cell = pointToCell(x, y, z);

    // Check center cell and 26 neighbors
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                GridCell cell{center_cell.x + dx, center_cell.y + dy, center_cell.z + dz};

                auto it = spatial_hash_.find(cell);
                if (it != spatial_hash_.end()) {
                    bbox_indices.insert(bbox_indices.end(), it->second.begin(), it->second.end());
                }
            }
        }
    }

    return bbox_indices;
}

} // namespace spatial_hash
} // namespace dynamic_cloud_filter
