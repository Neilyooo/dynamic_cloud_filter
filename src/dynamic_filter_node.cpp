/****************************************************************************************
 * Dynamic Filter Node - Main entry point
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * License: MIT
 ****************************************************************************************/

#include <ros/ros.h>
#include "dynamic_cloud_filter/point_cloud_filter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_cloud_filter_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting Dynamic Cloud Filter Node...");

    try {
        dynamic_cloud_filter::PointCloudFilter filter(nh);
        filter.spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in Dynamic Cloud Filter Node: %s", e.what());
        return 1;
    }

    return 0;
}
