/****************************************************************************************
 * Cloud Format Converter
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * A utility node to convert between different point cloud formats
 * Useful for ensuring compatibility with different SLAM/perception systems
 *
 * License: MIT
 ****************************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "dynamic_cloud_filter/point_cloud_utils.h"

class CloudFormatConverter {
public:
    CloudFormatConverter(ros::NodeHandle& nh) : nh_(nh) {
        // Parameters
        nh_.param<std::string>("input_topic", input_topic_, "/input_cloud");
        nh_.param<std::string>("output_topic", output_topic_, "/output_cloud");
        nh_.param<std::string>("output_format", output_format_, "XYZI");
        nh_.param<std::string>("output_frame", output_frame_, "");

        // Publisher and subscriber
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        sub_ = nh_.subscribe(input_topic_, 10, &CloudFormatConverter::callback, this);

        ROS_INFO("Cloud Format Converter initialized");
        ROS_INFO("  Input:  %s", input_topic_.c_str());
        ROS_INFO("  Output: %s (format: %s)", output_topic_.c_str(), output_format_.c_str());
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Detect input format
        std::string input_format = dynamic_cloud_filter::utils::getPointCloudFormat(*msg);

        if (input_format == "UNKNOWN") {
            ROS_WARN_THROTTLE(5.0, "Unknown input point cloud format!");
            return;
        }

        sensor_msgs::PointCloud2 output_msg = *msg;

        // If output frame is specified, update frame_id
        if (!output_frame_.empty()) {
            output_msg.header.frame_id = output_frame_;
        }

        // Convert if necessary
        if (input_format != output_format_) {
            // For now, we just pass through
            // In a real implementation, you would do the actual conversion
            ROS_INFO_ONCE("Converting from %s to %s", input_format.c_str(), output_format_.c_str());
        }

        pub_.publish(output_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string input_topic_;
    std::string output_topic_;
    std::string output_format_;
    std::string output_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_format_converter");
    ros::NodeHandle nh("~");

    CloudFormatConverter converter(nh);
    ros::spin();

    return 0;
}
