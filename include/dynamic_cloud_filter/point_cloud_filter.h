/****************************************************************************************
 * Dynamic Point Cloud Filter
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * A standalone module for filtering dynamic objects from point clouds using
 * 3D object detection (PointPillar/CenterPoint) and Multi-Object Tracking (MOT)
 *
 * License: MIT
 ****************************************************************************************/

#ifndef DYNAMIC_CLOUD_FILTER_POINT_CLOUD_FILTER_H_
#define DYNAMIC_CLOUD_FILTER_POINT_CLOUD_FILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <vector>
#include <memory>
#include <string>

namespace dynamic_cloud_filter {

/**
 * @brief Structure to represent a 3D bounding box detection
 */
struct BoundingBox3D {
    float x, y, z;           // Center position
    float length, width, height;  // Dimensions (l, w, h)
    float rotation;          // Yaw angle
    float velocity_x, velocity_y;  // Velocity
    int class_id;            // Object class
    float confidence;        // Detection confidence score
    int track_id;            // Tracking ID (-1 if not tracked)

    BoundingBox3D() : x(0), y(0), z(0), length(0), width(0), height(0),
                      rotation(0), velocity_x(0), velocity_y(0),
                      class_id(-1), confidence(0), track_id(-1) {}
};

/**
 * @brief Main class for dynamic point cloud filtering
 *
 * This class subscribes to:
 * - Point cloud topic (any format with XYZ)
 * - Bounding box detections (from PointPillar/CenterPoint)
 * - Odometry (optional, for transformation)
 *
 * And publishes:
 * - Filtered static point cloud
 * - Dynamic point cloud (optional)
 * - Visualization markers
 */
class PointCloudFilter {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    explicit PointCloudFilter(ros::NodeHandle& nh);

    /**
     * @brief Destructor
     */
    ~PointCloudFilter();

    /**
     * @brief Start processing
     */
    void spin();

private:
    // ROS node handle
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber sub_pointcloud_;
    ros::Subscriber sub_bbox_;
    ros::Subscriber sub_odom_;

    // Publishers
    ros::Publisher pub_static_cloud_;
    ros::Publisher pub_dynamic_cloud_;
    ros::Publisher pub_filtered_bbox_;
    ros::Publisher pub_visualization_;

    // Point cloud types with different formats support
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud_;

    // Bounding box buffer
    std::deque<jsk_recognition_msgs::BoundingBoxArray> bbox_buffer_;
    std::deque<nav_msgs::Odometry> odom_buffer_;

    // KD-tree for fast spatial queries
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;

    // Filters
    pcl::CropBox<pcl::PointXYZI> crop_filter_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;

    // Parameters
    struct FilterParams {
        // Frame IDs
        std::string odom_frame;
        std::string lidar_frame;

        // Point cloud preprocessing
        bool enable_crop_filter;
        double crop_size;
        bool enable_voxel_filter;
        double voxel_leaf_size;
        double min_range;
        double max_range;

        // Dynamic object filtering
        double bbox_margin;          // Safety margin around bounding box (meters)
        double bbox_search_radius;   // KD-tree search radius (meters)
        bool filter_by_velocity;     // Only filter objects with velocity > threshold
        double velocity_threshold;   // Minimum velocity to consider dynamic (m/s)
        bool filter_by_class;        // Filter specific classes
        std::vector<int> dynamic_classes;  // Class IDs to filter (e.g., car=0, pedestrian=8)

        // Tracking parameters
        int bbox_buffer_size;
        bool use_tracking;           // Use tracking info if available
        int min_track_age;           // Minimum tracking age to trust

        // Output options
        bool publish_dynamic_cloud;
        bool publish_visualization;
        bool verbose;

        FilterParams() :
            odom_frame("odom"),
            lidar_frame("base_link"),
            enable_crop_filter(false),
            crop_size(1.0),
            enable_voxel_filter(true),
            voxel_leaf_size(0.05),
            min_range(0.5),
            max_range(80.0),
            bbox_margin(0.2),
            bbox_search_radius(2.0),
            filter_by_velocity(false),
            velocity_threshold(0.1),
            filter_by_class(true),
            dynamic_classes({0, 6, 8}),  // car, cyclist, pedestrian
            bbox_buffer_size(3),
            use_tracking(false),
            min_track_age(3),
            publish_dynamic_cloud(true),
            publish_visualization(true),
            verbose(false) {}
    } params_;

    // Callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void bboxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Core processing functions
    void filterDynamicPoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        const std::vector<BoundingBox3D>& bboxes,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud);

    void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    bool isPointInBoundingBox(
        const pcl::PointXYZI& point,
        const BoundingBox3D& bbox,
        double margin = 0.0) const;

    void convertBBoxMsg(
        const jsk_recognition_msgs::BoundingBoxArray& bbox_msg,
        std::vector<BoundingBox3D>& bboxes);

    // Utilities
    void loadParameters();
    void publishResults(
        const std_msgs::Header& header,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud,
        const std::vector<BoundingBox3D>& filtered_bboxes);

    void publishVisualization(
        const std_msgs::Header& header,
        const std::vector<BoundingBox3D>& bboxes);
};

} // namespace dynamic_cloud_filter

#endif // DYNAMIC_CLOUD_FILTER_POINT_CLOUD_FILTER_H_
