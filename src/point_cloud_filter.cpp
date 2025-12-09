/****************************************************************************************
 * Dynamic Point Cloud Filter - Implementation
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * License: MIT
 ****************************************************************************************/

#include "dynamic_cloud_filter/point_cloud_filter.h"
#include "dynamic_cloud_filter/point_cloud_utils.h"
#include <tf/transform_datatypes.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

namespace dynamic_cloud_filter {

PointCloudFilter::PointCloudFilter(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize point clouds
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    static_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    dynamic_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // Initialize KD-tree
    kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    // Load parameters
    loadParameters();

    // Setup filters
    crop_filter_.setNegative(true);
    crop_filter_.setMin(Eigen::Vector4f(-params_.crop_size, -params_.crop_size,
                                        -params_.crop_size, 1.0));
    crop_filter_.setMax(Eigen::Vector4f(params_.crop_size, params_.crop_size,
                                        params_.crop_size, 1.0));

    voxel_filter_.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size,
                              params_.voxel_leaf_size);

    // Setup buffers (deque doesn't have set_capacity, will manage size manually)

    // Publishers
    pub_static_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("output/static_cloud", 10);

    if (params_.publish_dynamic_cloud) {
        pub_dynamic_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("output/dynamic_cloud", 10);
    }

    pub_filtered_bbox_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
        "output/filtered_boxes", 10);

    if (params_.publish_visualization) {
        pub_visualization_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "output/visualization", 10);
    }

    // Subscribers
    sub_pointcloud_ = nh_.subscribe("input/pointcloud", 1,
                                    &PointCloudFilter::pointCloudCallback, this);
    sub_bbox_ = nh_.subscribe("input/bounding_boxes", 10,
                              &PointCloudFilter::bboxCallback, this);
    sub_odom_ = nh_.subscribe("input/odometry", 50,
                              &PointCloudFilter::odomCallback, this);

    ROS_INFO("Dynamic Cloud Filter Node Initialized");
    ROS_INFO("  Subscribed to: %s", sub_pointcloud_.getTopic().c_str());
    ROS_INFO("  Publishing to: %s", pub_static_cloud_.getTopic().c_str());
}

PointCloudFilter::~PointCloudFilter() {
}

void PointCloudFilter::loadParameters() {
    // Frame IDs
    nh_.param<std::string>("frames/odom", params_.odom_frame, "odom");
    nh_.param<std::string>("frames/lidar", params_.lidar_frame, "base_link");

    // Preprocessing
    nh_.param<bool>("preprocessing/enable_crop_filter", params_.enable_crop_filter, false);
    nh_.param<double>("preprocessing/crop_size", params_.crop_size, 1.0);
    nh_.param<bool>("preprocessing/enable_voxel_filter", params_.enable_voxel_filter, true);
    nh_.param<double>("preprocessing/voxel_leaf_size", params_.voxel_leaf_size, 0.05);
    nh_.param<double>("preprocessing/min_range", params_.min_range, 0.5);
    nh_.param<double>("preprocessing/max_range", params_.max_range, 80.0);

    // Dynamic filtering
    nh_.param<double>("filtering/bbox_margin", params_.bbox_margin, 0.2);
    nh_.param<double>("filtering/bbox_search_radius", params_.bbox_search_radius, 2.0);
    nh_.param<bool>("filtering/filter_by_velocity", params_.filter_by_velocity, false);
    nh_.param<double>("filtering/velocity_threshold", params_.velocity_threshold, 0.1);
    nh_.param<bool>("filtering/filter_by_class", params_.filter_by_class, true);

    // Dynamic classes (default: car=0, cyclist=6, pedestrian=8)
    std::vector<int> default_classes = {0, 6, 8};
    nh_.param("filtering/dynamic_classes", params_.dynamic_classes, default_classes);

    // Tracking
    nh_.param<int>("tracking/bbox_buffer_size", params_.bbox_buffer_size, 3);
    nh_.param<bool>("tracking/use_tracking", params_.use_tracking, false);
    nh_.param<int>("tracking/min_track_age", params_.min_track_age, 3);

    // Output
    nh_.param<bool>("output/publish_dynamic_cloud", params_.publish_dynamic_cloud, true);
    nh_.param<bool>("output/publish_visualization", params_.publish_visualization, true);
    nh_.param<bool>("output/verbose", params_.verbose, false);
}

void PointCloudFilter::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert to PCL
    pcl::fromROSMsg(*msg, *current_cloud_);

    if (current_cloud_->empty()) {
        ROS_WARN("Received empty point cloud!");
        return;
    }

    // Preprocess
    preprocessPointCloud(current_cloud_);

    // Get latest bounding boxes
    std::vector<BoundingBox3D> bboxes;
    if (!bbox_buffer_.empty()) {
        convertBBoxMsg(bbox_buffer_.front(), bboxes);
    }

    // Filter dynamic points
    filterDynamicPoints(current_cloud_, bboxes, static_cloud_, dynamic_cloud_);

    // Publish results
    publishResults(msg->header, static_cloud_, dynamic_cloud_, bboxes);

    if (params_.verbose) {
        ROS_INFO("Filtered: %lu -> %lu static points, %lu dynamic points",
                 current_cloud_->size(), static_cloud_->size(), dynamic_cloud_->size());
    }
}

void PointCloudFilter::bboxCallback(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) {
    bbox_buffer_.push_front(*msg);
    // Manually limit buffer size (deque doesn't have set_capacity)
    while (bbox_buffer_.size() > static_cast<size_t>(params_.bbox_buffer_size)) {
        bbox_buffer_.pop_back();
    }
}

void PointCloudFilter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_buffer_.push_front(*msg);
    // Manually limit buffer size
    while (odom_buffer_.size() > static_cast<size_t>(params_.bbox_buffer_size)) {
        odom_buffer_.pop_back();
    }
}

void PointCloudFilter::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // Remove NaNs
    std::vector<int> indices;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Range filter
    utils::filterByRange(cloud, params_.min_range, params_.max_range);

    // Crop box filter
    if (params_.enable_crop_filter) {
        crop_filter_.setInputCloud(cloud);
        crop_filter_.filter(*cloud);
    }

    // Voxel grid filter
    if (params_.enable_voxel_filter) {
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.filter(*cloud);
    }
}

void PointCloudFilter::filterDynamicPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    const std::vector<BoundingBox3D>& bboxes,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud) {

    static_cloud->clear();
    dynamic_cloud->clear();

    // If no bounding boxes, all points are static
    if (bboxes.empty()) {
        *static_cloud = *input_cloud;
        return;
    }

    // Build KD-tree for bounding box centers
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_centers(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& bbox : bboxes) {
        pcl::PointXYZ p;
        p.x = bbox.x;
        p.y = bbox.y;
        p.z = bbox.z;
        bbox_centers->push_back(p);
    }
    kdtree_->setInputCloud(bbox_centers);

    // Process each point
    for (const auto& point : input_cloud->points) {
        pcl::PointXYZ query_point;
        query_point.x = point.x;
        query_point.y = point.y;
        query_point.z = point.z;

        // Find nearest bounding box
        std::vector<int> k_indices(1);
        std::vector<float> k_sqr_distances(1);

        if (kdtree_->nearestKSearch(query_point, 1, k_indices, k_sqr_distances) > 0) {
            // If point is far from all bounding boxes, it's static
            if (k_sqr_distances[0] > params_.bbox_search_radius * params_.bbox_search_radius) {
                static_cloud->push_back(point);
                continue;
            }

            // Check if point is inside the nearest bounding box
            const BoundingBox3D& nearest_bbox = bboxes[k_indices[0]];

            // Apply filters
            bool should_filter = true;

            // Class filter
            if (params_.filter_by_class) {
                bool is_dynamic_class = std::find(params_.dynamic_classes.begin(),
                                                  params_.dynamic_classes.end(),
                                                  nearest_bbox.class_id) != params_.dynamic_classes.end();
                if (!is_dynamic_class) {
                    should_filter = false;
                }
            }

            // Velocity filter
            if (should_filter && params_.filter_by_velocity) {
                float velocity = std::sqrt(nearest_bbox.velocity_x * nearest_bbox.velocity_x +
                                          nearest_bbox.velocity_y * nearest_bbox.velocity_y);
                if (velocity < params_.velocity_threshold) {
                    should_filter = false;
                }
            }

            // Check if point is in bounding box
            if (should_filter && isPointInBoundingBox(point, nearest_bbox, params_.bbox_margin)) {
                dynamic_cloud->push_back(point);
            } else {
                static_cloud->push_back(point);
            }
        } else {
            // No nearby bbox found, point is static
            static_cloud->push_back(point);
        }
    }

    static_cloud->header = input_cloud->header;
    dynamic_cloud->header = input_cloud->header;
}

bool PointCloudFilter::isPointInBoundingBox(
    const pcl::PointXYZI& point,
    const BoundingBox3D& bbox,
    double margin) const {

    // Simple AABB check (axis-aligned bounding box)
    // For oriented bounding box, we need to rotate the point

    // Calculate half dimensions with margin
    float half_length = (bbox.length / 2.0) + margin;
    float half_width = (bbox.width / 2.0) + margin;
    float half_height = (bbox.height / 2.0) + margin;

    // Transform point to bounding box local coordinates
    float dx = point.x - bbox.x;
    float dy = point.y - bbox.y;
    float dz = point.z - bbox.z;

    // Rotate point by -yaw angle
    float cos_yaw = std::cos(-bbox.rotation);
    float sin_yaw = std::sin(-bbox.rotation);

    float local_x = cos_yaw * dx - sin_yaw * dy;
    float local_y = sin_yaw * dx + cos_yaw * dy;
    float local_z = dz;

    // Check if point is inside the box
    return (std::abs(local_x) <= half_length &&
            std::abs(local_y) <= half_width &&
            std::abs(local_z) <= half_height);
}

void PointCloudFilter::convertBBoxMsg(
    const jsk_recognition_msgs::BoundingBoxArray& bbox_msg,
    std::vector<BoundingBox3D>& bboxes) {

    bboxes.clear();

    for (const auto& box : bbox_msg.boxes) {
        BoundingBox3D bbox;

        bbox.x = box.pose.position.x;
        bbox.y = box.pose.position.y;
        bbox.z = box.pose.position.z;

        bbox.length = box.dimensions.y;  // Note: JSK convention
        bbox.width = box.dimensions.x;
        bbox.height = box.dimensions.z;

        // Extract yaw from quaternion
        tf::Quaternion q(box.pose.orientation.x,
                        box.pose.orientation.y,
                        box.pose.orientation.z,
                        box.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        bbox.rotation = yaw;

        bbox.class_id = box.label;
        bbox.confidence = box.value;

        // Velocity (if available in custom message)
        bbox.velocity_x = 0.0;
        bbox.velocity_y = 0.0;

        bboxes.push_back(bbox);
    }
}

void PointCloudFilter::publishResults(
    const std_msgs::Header& header,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud,
    const std::vector<BoundingBox3D>& filtered_bboxes) {

    // Publish static cloud
    sensor_msgs::PointCloud2 static_msg;
    pcl::toROSMsg(*static_cloud, static_msg);
    static_msg.header = header;
    static_msg.header.frame_id = params_.lidar_frame;
    pub_static_cloud_.publish(static_msg);

    // Publish dynamic cloud
    if (params_.publish_dynamic_cloud && !dynamic_cloud->empty()) {
        sensor_msgs::PointCloud2 dynamic_msg;
        pcl::toROSMsg(*dynamic_cloud, dynamic_msg);
        dynamic_msg.header = header;
        dynamic_msg.header.frame_id = params_.lidar_frame;
        pub_dynamic_cloud_.publish(dynamic_msg);
    }

    // Publish visualization
    if (params_.publish_visualization && !filtered_bboxes.empty()) {
        publishVisualization(header, filtered_bboxes);
    }
}

void PointCloudFilter::publishVisualization(
    const std_msgs::Header& header,
    const std::vector<BoundingBox3D>& bboxes) {

    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < bboxes.size(); ++i) {
        const auto& bbox = bboxes[i];

        visualization_msgs::Marker marker;
        marker.header = header;
        marker.header.frame_id = params_.lidar_frame;
        marker.ns = "dynamic_objects";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = bbox.x;
        marker.pose.position.y = bbox.y;
        marker.pose.position.z = bbox.z;

        tf::Quaternion q;
        q.setRPY(0, 0, bbox.rotation);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = bbox.length;
        marker.scale.y = bbox.width;
        marker.scale.z = bbox.height;

        // Color based on class
        marker.color.a = 0.5;
        if (bbox.class_id == 0) {  // Car - red
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (bbox.class_id == 8) {  // Pedestrian - green
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else {  // Other - yellow
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        marker.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(marker);
    }

    pub_visualization_.publish(marker_array);
}

void PointCloudFilter::spin() {
    ros::spin();
}

} // namespace dynamic_cloud_filter
