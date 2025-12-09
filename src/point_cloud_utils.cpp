/****************************************************************************************
 * Point Cloud Utilities - Implementation
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * License: MIT
 ****************************************************************************************/

#include "dynamic_cloud_filter/point_cloud_utils.h"
#include <pcl/filters/extract_indices.h>

namespace dynamic_cloud_filter {
namespace utils {

// Template specializations for different point types

template<>
void convertToXYZI<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {

    output->resize(input->size());
    output->header = input->header;
    output->is_dense = input->is_dense;

    for (size_t i = 0; i < input->size(); ++i) {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].z;
        output->points[i].intensity = 0.0;
    }
}

template<>
void convertToXYZI<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {
    *output = *input;
}

template<>
void convertToXYZI<PointXYZIRT>(
    const pcl::PointCloud<PointXYZIRT>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output) {

    output->resize(input->size());
    output->header = input->header;
    output->is_dense = input->is_dense;

    for (size_t i = 0; i < input->size(); ++i) {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].z;
        output->points[i].intensity = input->points[i].intensity;
    }
}

template<>
void convertFromXYZI<PointXYZIRT>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    pcl::PointCloud<PointXYZIRT>::Ptr& output,
    const pcl::PointCloud<PointXYZIRT>::Ptr& original_cloud) {

    output->resize(input->size());
    output->header = input->header;
    output->is_dense = input->is_dense;

    for (size_t i = 0; i < input->size(); ++i) {
        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].z;
        output->points[i].intensity = input->points[i].intensity;

        // Try to preserve ring and timestamp from original cloud
        if (original_cloud && i < original_cloud->size()) {
            output->points[i].ring = original_cloud->points[i].ring;
            output->points[i].timestamp = original_cloud->points[i].timestamp;
        } else {
            output->points[i].ring = 0;
            output->points[i].timestamp = 0.0;
        }
    }
}

void filterByRange(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float min_range,
    float max_range) {

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (size_t i = 0; i < cloud->size(); ++i) {
        float range = std::sqrt(cloud->points[i].x * cloud->points[i].x +
                               cloud->points[i].y * cloud->points[i].y +
                               cloud->points[i].z * cloud->points[i].z);

        if (range >= min_range && range <= max_range) {
            inliers->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
}

bool hasField(const sensor_msgs::PointCloud2& msg, const std::string& field_name) {
    for (const auto& field : msg.fields) {
        if (field.name == field_name) {
            return true;
        }
    }
    return false;
}

std::string getPointCloudFormat(const sensor_msgs::PointCloud2& msg) {
    bool has_x = hasField(msg, "x");
    bool has_y = hasField(msg, "y");
    bool has_z = hasField(msg, "z");
    bool has_intensity = hasField(msg, "intensity");
    bool has_ring = hasField(msg, "ring");
    bool has_time = hasField(msg, "time") || hasField(msg, "timestamp") || hasField(msg, "t");

    if (!has_x || !has_y || !has_z) {
        return "UNKNOWN";
    }

    if (has_intensity && has_ring && has_time) {
        return "XYZIRT";  // Velodyne, Ouster format
    } else if (has_intensity && has_ring) {
        return "XYZIR";
    } else if (has_intensity) {
        return "XYZI";
    } else {
        return "XYZ";
    }
}

} // namespace utils
} // namespace dynamic_cloud_filter
