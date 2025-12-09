/****************************************************************************************
 * Point Cloud Utilities
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * Helper functions for point cloud format conversion and manipulation
 *
 * License: MIT
 ****************************************************************************************/

#ifndef DYNAMIC_CLOUD_FILTER_POINT_CLOUD_UTILS_H_
#define DYNAMIC_CLOUD_FILTER_POINT_CLOUD_UTILS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace dynamic_cloud_filter {
namespace utils {

/**
 * @brief Custom point type with ring and time information (e.g., for Velodyne, Ouster)
 */
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace utils
} // namespace dynamic_cloud_filter

// Register custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(dynamic_cloud_filter::utils::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace dynamic_cloud_filter {
namespace utils {

/**
 * @brief Convert various point cloud formats to XYZI
 */
template<typename PointT>
void convertToXYZI(
    const typename pcl::PointCloud<PointT>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output);

/**
 * @brief Convert XYZI back to custom format (preserving ring/timestamp if possible)
 */
template<typename PointT>
void convertFromXYZI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    typename pcl::PointCloud<PointT>::Ptr& output,
    const typename pcl::PointCloud<PointT>::Ptr& original_cloud = nullptr);

/**
 * @brief Remove points outside range [min_range, max_range]
 */
void filterByRange(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float min_range,
    float max_range);

/**
 * @brief Check if a ROS message contains specific fields
 */
bool hasField(const sensor_msgs::PointCloud2& msg, const std::string& field_name);

/**
 * @brief Get point cloud format type from ROS message
 */
std::string getPointCloudFormat(const sensor_msgs::PointCloud2& msg);

} // namespace utils
} // namespace dynamic_cloud_filter

#endif // DYNAMIC_CLOUD_FILTER_POINT_CLOUD_UTILS_H_
