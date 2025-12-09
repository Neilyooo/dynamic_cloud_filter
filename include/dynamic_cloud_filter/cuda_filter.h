/****************************************************************************************
 * CUDA Parallel Point Cloud Filter
 *
 * Copyright (c) 2025, AutoCity
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * GPU-accelerated dynamic point filtering using CUDA
 *
 * Expected speedup: 5-10x over CPU version
 *
 * License: MIT
 ****************************************************************************************/

#ifndef DYNAMIC_CLOUD_FILTER_CUDA_FILTER_H_
#define DYNAMIC_CLOUD_FILTER_CUDA_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cuda_runtime.h>

namespace dynamic_cloud_filter {
namespace cuda {

/**
 * @brief Cached bounding box data for GPU (same as CPU version)
 */
struct BBoxCached {
    float x, y, z;
    float cos_yaw, sin_yaw;
    float half_l, half_w, half_h;
    bool should_filter;
    int class_id;
};

/**
 * @brief CUDA-accelerated point cloud filter
 *
 * Uses GPU parallelization to filter points
 * Each point is processed by a separate CUDA thread
 */
class CudaFilter {
public:
    CudaFilter();
    ~CudaFilter();

    void setDynamicClasses(const std::vector<int>& classes);
    void setParameters(float bbox_margin, float search_radius);

    /**
     * @brief CUDA-accelerated dynamic point filtering
     *
     * Processes all points in parallel on GPU
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

    uint16_t class_filter_mask_;
    float bbox_margin_;
    float bbox_search_radius_;
    float search_radius_sq_;
};

// CUDA kernel launcher (implemented in .cu file)
void launchFilterKernel(
    const float* points,          // Input points [x, y, z, intensity]
    const BBoxCached* bboxes,     // Cached bounding boxes
    int* labels,                  // Output: 0=static, 1=dynamic
    int num_points,
    int num_bboxes,
    float search_radius_sq);

// Template implementation (must be in header)
template<typename BBoxType>
std::vector<BBoxCached> CudaFilter::cacheBoundingBoxes(const std::vector<BBoxType>& bboxes) {
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
void CudaFilter::filterDynamicPoints(
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

    int num_points = input_cloud->size();

    // Pre-compute bbox data
    auto cached_bboxes = cacheBoundingBoxes(bboxes);
    int num_bboxes = cached_bboxes.size();

    // Allocate host memory
    std::vector<float> h_points(num_points * 4);
    std::vector<int> h_labels(num_points);

    // Pack point data
    for (int i = 0; i < num_points; ++i) {
        h_points[i*4 + 0] = input_cloud->points[i].x;
        h_points[i*4 + 1] = input_cloud->points[i].y;
        h_points[i*4 + 2] = input_cloud->points[i].z;
        h_points[i*4 + 3] = input_cloud->points[i].intensity;
    }

    // Allocate GPU memory
    float* d_points;
    BBoxCached* d_bboxes;
    int* d_labels;

    cudaMalloc(&d_points, num_points * 4 * sizeof(float));
    cudaMalloc(&d_bboxes, num_bboxes * sizeof(BBoxCached));
    cudaMalloc(&d_labels, num_points * sizeof(int));

    // Copy to GPU
    cudaMemcpy(d_points, h_points.data(), num_points * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_bboxes, cached_bboxes.data(), num_bboxes * sizeof(BBoxCached), cudaMemcpyHostToDevice);

    // Launch kernel
    launchFilterKernel(d_points, d_bboxes, d_labels, num_points, num_bboxes, search_radius_sq_);

    // Copy results back
    cudaMemcpy(h_labels.data(), d_labels, num_points * sizeof(int), cudaMemcpyDeviceToHost);

    // Free GPU memory
    cudaFree(d_points);
    cudaFree(d_bboxes);
    cudaFree(d_labels);

    // Split points based on labels
    static_cloud->reserve(num_points);
    dynamic_cloud->reserve(num_points / 10);

    for (int i = 0; i < num_points; ++i) {
        if (h_labels[i] == 1) {
            dynamic_cloud->push_back(input_cloud->points[i]);
        } else {
            static_cloud->push_back(input_cloud->points[i]);
        }
    }
}

} // namespace cuda
} // namespace dynamic_cloud_filter

#endif // DYNAMIC_CLOUD_FILTER_CUDA_FILTER_H_
