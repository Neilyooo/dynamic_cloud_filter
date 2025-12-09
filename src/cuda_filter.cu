/****************************************************************************************
 * CUDA Parallel Point Cloud Filter - Implementation
 *
 * Copyright (c) 2025, AutoCity
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * License: MIT
 ****************************************************************************************/

#include "dynamic_cloud_filter/cuda_filter.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>

namespace dynamic_cloud_filter {
namespace cuda {

// CUDA kernel: Check if point is in BBox (runs on GPU)
__device__ bool isPointInBBox(
    float px, float py, float pz,
    const BBoxCached& bbox) {

    // Translate to bbox local frame
    float dx = px - bbox.x;
    float dy = py - bbox.y;
    float dz = pz - bbox.z;

    // OBB (Oriented Bounding Box) - More accurate but considers rotation
    // Commented out to match TRLO's AABB behavior
    // float local_x = bbox.cos_yaw * dx - bbox.sin_yaw * dy;
    // float local_y = bbox.sin_yaw * dx + bbox.cos_yaw * dy;
    // return (fabsf(local_x) <= bbox.half_l &&
    //         fabsf(local_y) <= bbox.half_w &&
    //         fabsf(dz) <= bbox.half_h);

    // AABB (Axis-Aligned Bounding Box) - Matches TRLO, ignores rotation
    return (fabsf(dx) <= bbox.half_l &&
            fabsf(dy) <= bbox.half_w &&
            fabsf(dz) <= bbox.half_h);
}

// CUDA kernel: Filter points in parallel
__global__ void filterPointsKernel(
    const float* points,          // [x, y, z, intensity] × num_points
    const BBoxCached* bboxes,     // Cached bounding boxes
    int* labels,                  // Output: 0=static, 1=dynamic
    int num_points,
    int num_bboxes,
    float search_radius_sq) {

    // Each thread processes one point
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx >= num_points) return;

    // Load point data
    float px = points[idx * 4 + 0];
    float py = points[idx * 4 + 1];
    float pz = points[idx * 4 + 2];

    // Check against all bboxes
    bool is_dynamic = false;

    for (int i = 0; i < num_bboxes; ++i) {
        const BBoxCached& bbox = bboxes[i];

        // Skip if class should not be filtered
        if (!bbox.should_filter) continue;

        // Fast distance check (squared distance)
        float dx = px - bbox.x;
        float dy = py - bbox.y;
        float dz = pz - bbox.z;
        float dist_sq = dx*dx + dy*dy + dz*dz;

        if (dist_sq > search_radius_sq) continue;

        // OBB test
        if (isPointInBBox(px, py, pz, bbox)) {
            is_dynamic = true;
            break;
        }
    }

    labels[idx] = is_dynamic ? 1 : 0;
}

// Host function to launch CUDA kernel
void launchFilterKernel(
    const float* points,
    const BBoxCached* bboxes,
    int* labels,
    int num_points,
    int num_bboxes,
    float search_radius_sq) {

    // Launch configuration
    int threads_per_block = 256;
    int num_blocks = (num_points + threads_per_block - 1) / threads_per_block;

    filterPointsKernel<<<num_blocks, threads_per_block>>>(
        points, bboxes, labels, num_points, num_bboxes, search_radius_sq);

    // Wait for GPU to finish
    cudaDeviceSynchronize();
}

// ============================================================================
// CudaFilter Implementation
// ============================================================================

CudaFilter::CudaFilter()
    : class_filter_mask_(0),
      bbox_margin_(0.2f),
      bbox_search_radius_(2.0f),
      search_radius_sq_(4.0f) {
}

CudaFilter::~CudaFilter() {
}

void CudaFilter::setDynamicClasses(const std::vector<int>& classes) {
    class_filter_mask_ = 0;
    for (int cls : classes) {
        if (cls >= 0 && cls < 16) {
            class_filter_mask_ |= (1 << cls);
        }
    }
}

void CudaFilter::setParameters(float bbox_margin, float search_radius) {
    bbox_margin_ = bbox_margin;
    bbox_search_radius_ = search_radius;
    search_radius_sq_ = search_radius * search_radius;
}

// Template implementations moved to header

} // namespace cuda
} // namespace dynamic_cloud_filter
