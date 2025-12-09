/****************************************************************************************
 * Performance Benchmark Tool
 *
 * Copyright (c) 2025, AutoCity
 * Authors: Sonny
 *
 * Benchmark tool to compare original vs optimized filtering performance
 ****************************************************************************************/

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <random>
#include <iomanip>

#include "dynamic_cloud_filter/point_cloud_filter.h"
#include "dynamic_cloud_filter/optimized_filter.h"
#include "dynamic_cloud_filter/spatial_hash_filter.h"
#include "dynamic_cloud_filter/cuda_filter.h"

// Dummy bbox structure for testing
struct TestBBox {
    float x, y, z;
    float l, w, h;
    float rt;  // rotation (yaw)
    int id;
    float score;
};

class PerformanceBenchmark {
public:
    void run() {
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║     Dynamic Cloud Filter - Performance Benchmark            ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

        // Test different scenarios
        runBenchmark(56600, 5, "Small");      // 10K points, 5 boxes
        runBenchmark(56600, 10, "Medium");    // 50K points, 10 boxes
        // runBenchmark(56600, 20, "32-Line LiDAR");  // Real 32-line lidar
        runBenchmark(56600, 20, "Large");    // 120K points, 20 boxes
        runBenchmark(56600, 50, "XLarge");   // 300K points, 50 boxes

        printSummary();
    }

private:
    struct BenchmarkResult {
        std::string scenario;
        int num_points;
        int num_boxes;
        double time_trlo_ms;
        double time_obb_ms;
        double time_optimized_ms;
        double time_spatial_hash_ms;
        double time_cuda_ms;
        double speedup_vs_trlo;
        double speedup_vs_obb;
        double speedup_spatial_hash;
        double speedup_cuda;
        size_t static_trlo;
        size_t dynamic_trlo;
        size_t static_obb;
        size_t dynamic_obb;
        size_t static_opt;
        size_t dynamic_opt;
        size_t static_spatial;
        size_t dynamic_spatial;
        size_t static_cuda;
        size_t dynamic_cuda;
    };

    std::vector<BenchmarkResult> results_;

    void runBenchmark(int num_points, int num_boxes, const std::string& name) {
        std::cout << std::string(66, '=') << "\n";
        std::cout << "Scenario: " << name << "\n";
        std::cout << "  Points: " << num_points << ", Boxes: " << num_boxes << "\n";
        std::cout << std::string(66, '=') << "\n";

        // Generate test data
        auto cloud = generatePointCloud(num_points);
        auto bboxes = generateBBoxes(num_boxes);

        // Warm up
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // === Benchmark 1: TRLO Original (AABB, no rotation) ===
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_trlo(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_trlo(new pcl::PointCloud<pcl::PointXYZI>());

        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 10; ++i) {
            filterTRLO(cloud, bboxes, static_trlo, dynamic_trlo);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        double time_trlo = std::chrono::duration<double, std::milli>(t2 - t1).count() / 10.0;

        // === Benchmark 2: OBB without optimization (repeated cos/sin) ===
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_obb(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_obb(new pcl::PointCloud<pcl::PointXYZI>());

        auto t3 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 10; ++i) {
            filterOBB(cloud, bboxes, static_obb, dynamic_obb);
        }
        auto t4 = std::chrono::high_resolution_clock::now();
        double time_obb = std::chrono::duration<double, std::milli>(t4 - t3).count() / 10.0;

        // === Benchmark 3: Optimized OBB (pre-computed cos/sin) ===
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_opt(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_opt(new pcl::PointCloud<pcl::PointXYZI>());

        dynamic_cloud_filter::optimized::OptimizedFilter opt_filter;
        opt_filter.setDynamicClasses({0, 6, 8});
        opt_filter.setParameters(0.2f, 2.0f);

        auto t5 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 10; ++i) {
            opt_filter.filterDynamicPoints(cloud, bboxes, static_opt, dynamic_opt);
        }
        auto t6 = std::chrono::high_resolution_clock::now();
        double time_optimized = std::chrono::duration<double, std::milli>(t6 - t5).count() / 10.0;

        // === Benchmark 4: Spatial Hash ===
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_spatial(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_spatial(new pcl::PointCloud<pcl::PointXYZI>());

        dynamic_cloud_filter::spatial_hash::SpatialHashFilter spatial_filter;
        spatial_filter.setDynamicClasses({0, 6, 8});
        spatial_filter.setParameters(0.2f, 2.0f);

        auto t7 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 10; ++i) {
            spatial_filter.filterDynamicPoints(cloud, bboxes, static_spatial, dynamic_spatial);
        }
        auto t8 = std::chrono::high_resolution_clock::now();
        double time_spatial_hash = std::chrono::duration<double, std::milli>(t8 - t7).count() / 10.0;

        // === Benchmark 5: CUDA ===
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_cuda(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cuda(new pcl::PointCloud<pcl::PointXYZI>());

        dynamic_cloud_filter::cuda::CudaFilter cuda_filter;
        cuda_filter.setDynamicClasses({0, 6, 8});
        cuda_filter.setParameters(0.2f, 2.0f);

        auto t9 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 10; ++i) {
            cuda_filter.filterDynamicPoints(cloud, bboxes, static_cuda, dynamic_cuda);
        }
        auto t10 = std::chrono::high_resolution_clock::now();
        double time_cuda = std::chrono::duration<double, std::milli>(t10 - t9).count() / 10.0;

        // Calculate speedup
        double speedup_vs_trlo = time_trlo / time_optimized;
        double speedup_vs_obb = time_obb / time_optimized;
        double speedup_spatial_hash = time_trlo / time_spatial_hash;
        double speedup_cuda = time_trlo / time_cuda;

        // Store result
        BenchmarkResult result;
        result.scenario = name;
        result.num_points = num_points;
        result.num_boxes = num_boxes;
        result.time_trlo_ms = time_trlo;
        result.time_obb_ms = time_obb;
        result.time_optimized_ms = time_optimized;
        result.time_spatial_hash_ms = time_spatial_hash;
        result.time_cuda_ms = time_cuda;
        result.speedup_vs_trlo = speedup_vs_trlo;
        result.speedup_vs_obb = speedup_vs_obb;
        result.speedup_spatial_hash = speedup_spatial_hash;
        result.speedup_cuda = speedup_cuda;
        result.static_trlo = static_trlo->size();
        result.dynamic_trlo = dynamic_trlo->size();
        result.static_obb = static_obb->size();
        result.dynamic_obb = dynamic_obb->size();
        result.static_opt = static_opt->size();
        result.dynamic_opt = dynamic_opt->size();
        result.static_spatial = static_spatial->size();
        result.dynamic_spatial = dynamic_spatial->size();
        result.static_cuda = static_cuda->size();
        result.dynamic_cuda = dynamic_cuda->size();
        results_.push_back(result);

        // Print result
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  [1] TRLO (AABB):       " << std::setw(8) << time_trlo << " ms"
                  << " (static: " << result.static_trlo << ", dynamic: " << result.dynamic_trlo << ")\n";
        std::cout << "  [2] OBB (unopt):       " << std::setw(8) << time_obb << " ms"
                  << " (static: " << result.static_obb << ", dynamic: " << result.dynamic_obb << ")\n";
        std::cout << "  [3] OBB (optimized):   " << std::setw(8) << time_optimized << " ms"
                  << " (static: " << result.static_opt << ", dynamic: " << result.dynamic_opt << ")\n";
        std::cout << "  [4] Spatial Hash:      " << std::setw(8) << time_spatial_hash << " ms"
                  << " (static: " << result.static_spatial << ", dynamic: " << result.dynamic_spatial << ")\n";
        std::cout << "  [5] CUDA Parallel:     " << std::setw(8) << time_cuda << " ms"
                  << " (static: " << result.static_cuda << ", dynamic: " << result.dynamic_cuda << ")\n";
        std::cout << "\n";
        std::cout << "  Speedup vs TRLO:       " << std::setw(8) << speedup_vs_trlo << "x (OBB opt)";
        if (speedup_vs_trlo > 1.0) {
            std::cout << " [Faster]\n";
        } else {
            std::cout << " [Slower]\n";
        }
        std::cout << "  Speedup vs TRLO:       " << std::setw(8) << speedup_spatial_hash << "x (Spatial Hash)";
        if (speedup_spatial_hash > 2.0) {
            std::cout << " [Excellent!]\n";
        } else if (speedup_spatial_hash > 1.5) {
            std::cout << " [Good]\n";
        } else if (speedup_spatial_hash > 1.0) {
            std::cout << " [Marginal]\n";
        } else {
            std::cout << " [No improvement]\n";
        }
        std::cout << "  Speedup vs TRLO:       " << std::setw(8) << speedup_cuda << "x (CUDA)";
        if (speedup_cuda > 5.0) {
            std::cout << " [Excellent!]\n";
        } else if (speedup_cuda > 2.0) {
            std::cout << " [Good]\n";
        } else if (speedup_cuda > 1.0) {
            std::cout << " [Marginal]\n";
        } else {
            std::cout << " [No improvement]\n";
        }

        // Verify OBB consistency
        if (result.static_obb != result.static_opt || result.dynamic_obb != result.dynamic_opt) {
            std::cout << "  [WARNING] OBB results differ!\n";
        }

        std::cout << "\n";
    }

    void printSummary() {
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    BENCHMARK SUMMARY                         ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

        std::cout << std::setw(12) << "Scenario"
                  << std::setw(9) << "Points"
                  << std::setw(7) << "Boxes"
                  << std::setw(9) << "TRLO"
                  << std::setw(9) << "OBB"
                  << std::setw(9) << "Spatial"
                  << std::setw(9) << "CUDA" << "\n";
        std::cout << std::string(64, '-') << "\n";

        double avg_spatial = 0.0;
        double avg_cuda = 0.0;
        for (const auto& r : results_) {
            std::cout << std::setw(12) << r.scenario
                      << std::setw(9) << r.num_points
                      << std::setw(7) << r.num_boxes
                      << std::setw(9) << r.time_trlo_ms
                      << std::setw(9) << r.time_obb_ms
                      << std::setw(9) << r.time_spatial_hash_ms
                      << std::setw(9) << r.time_cuda_ms << "\n";
            avg_spatial += r.speedup_spatial_hash;
            avg_cuda += r.speedup_cuda;
        }

        avg_spatial /= results_.size();
        avg_cuda /= results_.size();

        std::cout << std::string(64, '-') << "\n";
        std::cout << "Average Speedup vs TRLO:\n";
        std::cout << "  Spatial Hash: " << std::fixed << std::setprecision(2)
                  << avg_spatial << "x\n";
        std::cout << "  CUDA:         " << std::fixed << std::setprecision(2)
                  << avg_cuda << "x\n\n";

        std::cout << "Algorithm Comparison:\n";
        std::cout << "  [1] TRLO (AABB):       Fast but ignores rotation\n";
        std::cout << "  [2] OBB (unopt):       Accurate but slow (repeated cos/sin)\n";
        std::cout << "  [3] OBB (optimized):   Pre-computed cos/sin\n";
        std::cout << "  [4] Spatial Hash:      Grid-based acceleration\n";
        std::cout << "  [5] CUDA:              GPU parallelization\n\n";
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr generatePointCloud(int num_points) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->reserve(num_points);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-50.0f, 50.0f);

        for (int i = 0; i < num_points; ++i) {
            pcl::PointXYZI p;
            p.x = dis(gen);
            p.y = dis(gen);
            p.z = dis(gen) * 0.1f;  // Flatten z
            p.intensity = 100.0f;
            cloud->push_back(p);
        }

        return cloud;
    }

    std::vector<TestBBox> generateBBoxes(int num_boxes) {
        std::vector<TestBBox> bboxes;
        bboxes.reserve(num_boxes);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> pos_dis(-40.0f, 40.0f);
        std::uniform_real_distribution<float> size_dis(2.0f, 8.0f);
        std::uniform_real_distribution<float> rot_dis(-3.14f, 3.14f);
        std::uniform_int_distribution<int> class_dis(0, 9);

        for (int i = 0; i < num_boxes; ++i) {
            TestBBox box;
            box.x = pos_dis(gen);
            box.y = pos_dis(gen);
            box.z = 0.0f;
            box.l = size_dis(gen);
            box.w = size_dis(gen) * 0.5f;
            box.h = 2.0f;
            box.rt = rot_dis(gen);
            box.id = class_dis(gen);
            box.score = 0.8f;
            bboxes.push_back(box);
        }

        return bboxes;
    }

    // Original algorithm from TRLO: KD-tree + AABB (no rotation, no cos/sin)
    void filterTRLO(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
        const std::vector<TestBBox>& bboxes,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& static_out,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_out) {

        static_out->clear();
        dynamic_out->clear();

        if (bboxes.empty()) {
            *static_out = *input;
            return;
        }

        // Build KD-tree for bbox centers
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_centers(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& box : bboxes) {
            pcl::PointXYZ p;
            p.x = box.x; p.y = box.y; p.z = box.z;
            bbox_centers->push_back(p);
        }
        kdtree.setInputCloud(bbox_centers);

        std::vector<int> k_indices(1);
        std::vector<float> k_sqr_distances(1);

        for (const auto& point : input->points) {
            pcl::PointXYZ query;
            query.x = point.x; query.y = point.y; query.z = point.z;

            if (kdtree.nearestKSearch(query, 1, k_indices, k_sqr_distances) > 0) {
                // Check distance
                if (k_sqr_distances[0] > 2.0) {  // squared distance threshold
                    static_out->push_back(point);
                    continue;
                }

                const TestBBox& box = bboxes[k_indices[0]];

                // AABB test (Axis-Aligned Bounding Box) - no rotation!
                // This is TRLO's original method: simple, fast, but ignores rotation
                if ((point.x < box.x - box.l / 2 - 0.2f || point.x > box.x + box.l / 2 + 0.2f) ||
                    (point.y < box.y - box.w / 2 - 0.2f || point.y > box.y + box.w / 2 + 0.2f) ||
                    (point.z < box.z - box.h / 2 - 0.2f || point.z > box.z + box.h / 2 + 0.2f)) {
                    static_out->push_back(point);
                } else {
                    dynamic_out->push_back(point);
                }
            } else {
                static_out->push_back(point);
            }
        }
    }

    // OBB without optimization: KD-tree + OBB with REPEATED cos/sin computation
    void filterOBB(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
        const std::vector<TestBBox>& bboxes,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& static_out,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_out) {

        static_out->clear();
        dynamic_out->clear();

        if (bboxes.empty()) {
            *static_out = *input;
            return;
        }

        // Build KD-tree for bbox centers
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_centers(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& box : bboxes) {
            pcl::PointXYZ p;
            p.x = box.x; p.y = box.y; p.z = box.z;
            bbox_centers->push_back(p);
        }
        kdtree.setInputCloud(bbox_centers);

        std::vector<int> k_indices(1);
        std::vector<float> k_sqr_distances(1);

        for (const auto& point : input->points) {
            pcl::PointXYZ query;
            query.x = point.x; query.y = point.y; query.z = point.z;

            if (kdtree.nearestKSearch(query, 1, k_indices, k_sqr_distances) > 0) {
                // Check distance
                if (k_sqr_distances[0] > 4.0f) {  // 2.0^2
                    static_out->push_back(point);
                    continue;
                }

                const TestBBox& box = bboxes[k_indices[0]];

                // Check class filter
                bool should_filter = (box.id == 0 || box.id == 6 || box.id == 8);
                if (!should_filter) {
                    static_out->push_back(point);
                    continue;
                }

                // OBB test - REPEATEDLY compute cos/sin every time! (SLOW)
                float dx = point.x - box.x;
                float dy = point.y - box.y;
                float dz = point.z - box.z;

                float cos_yaw = std::cos(-box.rt);  // ~100-200 CPU cycles
                float sin_yaw = std::sin(-box.rt);  // ~100-200 CPU cycles
                float local_x = cos_yaw * dx - sin_yaw * dy;
                float local_y = sin_yaw * dx + cos_yaw * dy;

                float half_l = box.l / 2.0f + 0.2f;  // Repeated division
                float half_w = box.w / 2.0f + 0.2f;
                float half_h = box.h / 2.0f + 0.2f;

                if (std::abs(local_x) <= half_l &&
                    std::abs(local_y) <= half_w &&
                    std::abs(dz) <= half_h) {
                    dynamic_out->push_back(point);
                } else {
                    static_out->push_back(point);
                }
            } else {
                static_out->push_back(point);
            }
        }
    }
};

int main(int argc, char** argv) {
    std::cout << "\n";
    PerformanceBenchmark benchmark;
    benchmark.run();
    return 0;
}
