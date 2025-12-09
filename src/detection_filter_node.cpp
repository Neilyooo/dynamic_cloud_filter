/****************************************************************************************
 * Detection and Filter Node - Complete Pipeline
 *
 * Copyright (c) 2025, AutoCity
 *
 * Authors: Sonny
 * Contact: sonnygonnarich@gmail.com
 *
 * This node integrates:
 * 1. PointPillar 3D object detection
 * 2. Dynamic point cloud filtering
 * 3. Static point cloud output for SLAM
 *
 * License: MIT
 ****************************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

// PointPillar detection
#include "centerpoint/centerpoint.h"
#include "centerpoint/common.h"

#include <cuda_runtime.h>
#include <fstream>

#define GPU_CHECK(ans) { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char *file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}

class DetectionFilterNode {
public:
    DetectionFilterNode(ros::NodeHandle& nh) : nh_(nh) {
        // Load parameters
        loadParameters();

        // Initialize CUDA
        initCUDA();

        // Initialize CenterPoint detector
        centerpoint_ptr_.reset(new CenterPoint(model_path_, verbose_));
        centerpoint_ptr_->prepare();

        // Initialize KD-tree for filtering
        kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

        // Publishers
        pub_static_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "output/static_cloud", 10);
        pub_dynamic_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "output/dynamic_cloud", 10);
        pub_bboxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
            "output/bounding_boxes", 10);

        // Subscriber
        sub_pointcloud_ = nh_.subscribe("input/pointcloud", 1,
            &DetectionFilterNode::pointcloudCallback, this);

        GPU_CHECK(cudaStreamCreate(&stream_));

        ROS_INFO("Detection & Filter Node Initialized");
        ROS_INFO("  Model path: %s", model_path_.c_str());
        ROS_INFO("  Input topic: %s", sub_pointcloud_.getTopic().c_str());
    }

    ~DetectionFilterNode() {
        GPU_CHECK(cudaStreamDestroy(stream_));
    }

private:
    void loadParameters() {
        // Get package path for models
        std::string pkg_path = ros::package::getPath("dynamic_cloud_filter");
        std::string default_model_path = pkg_path + "/models/";

        nh_.param<std::string>("model_path", model_path_, default_model_path);
        nh_.param<std::string>("lidar_frame", lidar_frame_, "base_link");
        nh_.param<bool>("verbose", verbose_, false);
        nh_.param<double>("min_range", min_range_, 0.5);
        nh_.param<double>("max_range", max_range_, 80.0);
        nh_.param<double>("bbox_margin", bbox_margin_, 0.2);
        nh_.param<double>("bbox_search_radius", bbox_search_radius_, 2.0);
        nh_.param<bool>("filter_by_class", filter_by_class_, true);
        nh_.param<bool>("publish_dynamic_cloud", publish_dynamic_cloud_, true);

        // Dynamic classes: 0=car, 6=cyclist, 8=pedestrian
        std::vector<int> default_classes = {0, 6, 8};
        nh_.param("dynamic_classes", dynamic_classes_, default_classes);
    }

    void initCUDA() {
        int dev = 0;
        cudaDeviceProp deviceProp;
        GPU_CHECK(cudaGetDeviceProperties(&deviceProp, dev));
        ROS_INFO("Using GPU: %s", deviceProp.name);
        GPU_CHECK(cudaSetDevice(dev));
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        double t_start = ros::Time::now().toSec();

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(
            new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud_in);

        if (cloud_in->empty()) {
            ROS_WARN("Received empty point cloud!");
            return;
        }

        // Save to bin file for CenterPoint
        std::string temp_bin = "/tmp/pointcloud_temp.bin";
        saveToBin(*cloud_in, temp_bin);

        // Load and run CenterPoint detection
        unsigned int length = 0;
        void *pc_data = NULL;
        loadData(temp_bin.c_str(), &pc_data, &length);
        size_t points_num = length / (5 * sizeof(float));

        float *d_points = nullptr;
        GPU_CHECK(cudaMallocManaged((void **)&d_points, MAX_POINTS_NUM * 5 * sizeof(float)));
        GPU_CHECK(cudaMemcpy(d_points, pc_data, length, cudaMemcpyHostToDevice));
        GPU_CHECK(cudaDeviceSynchronize());

        // Run detection
        centerpoint_ptr_->doinfer((void *)d_points, points_num, stream_);

        GPU_CHECK(cudaFree(d_points));
        delete[] (char*)pc_data;

        // Filter boxes based on score and class
        std::vector<Bndbox> filtered_boxes;
        for (const auto& box : centerpoint_ptr_->nms_pred_) {
            if ((box.id == 0 && box.score > 0.5) ||  // car
                (box.id == 6 && box.score > 0.75) || // cyclist
                (box.id == 8 && box.score > 0.5)) {  // pedestrian
                filtered_boxes.push_back(box);
            }
        }

        // Filter dynamic points
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());

        filterDynamicPoints(cloud_in, filtered_boxes, static_cloud, dynamic_cloud);

        // Publish results
        publishResults(msg->header, static_cloud, dynamic_cloud, filtered_boxes);

        double t_end = ros::Time::now().toSec();
        if (verbose_) {
            ROS_INFO("Total time: %.2f ms | Detections: %lu | Static points: %lu | Dynamic points: %lu",
                     (t_end - t_start) * 1000, filtered_boxes.size(),
                     static_cloud->size(), dynamic_cloud->size());
        }
    }

    void filterDynamicPoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        const std::vector<Bndbox>& bboxes,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud) {

        static_cloud->clear();
        dynamic_cloud->clear();

        if (bboxes.empty()) {
            *static_cloud = *input_cloud;
            return;
        }

        // Build KD-tree for bbox centers
        pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_centers(
            new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& bbox : bboxes) {
            pcl::PointXYZ p;
            p.x = bbox.x; p.y = bbox.y; p.z = bbox.z;
            bbox_centers->push_back(p);
        }
        kdtree_->setInputCloud(bbox_centers);

        // Filter each point
        for (const auto& point : input_cloud->points) {
            pcl::PointXYZ query;
            query.x = point.x; query.y = point.y; query.z = point.z;

            std::vector<int> k_indices(1);
            std::vector<float> k_sqr_distances(1);

            if (kdtree_->nearestKSearch(query, 1, k_indices, k_sqr_distances) > 0) {
                if (k_sqr_distances[0] > bbox_search_radius_ * bbox_search_radius_) {
                    static_cloud->push_back(point);
                    continue;
                }

                const Bndbox& nearest_bbox = bboxes[k_indices[0]];

                // Check class filter
                bool should_filter = true;
                if (filter_by_class_) {
                    should_filter = std::find(dynamic_classes_.begin(),
                                             dynamic_classes_.end(),
                                             nearest_bbox.id) != dynamic_classes_.end();
                }

                // Check if point is in bbox
                if (should_filter && isPointInBBox(point, nearest_bbox)) {
                    dynamic_cloud->push_back(point);
                } else {
                    static_cloud->push_back(point);
                }
            } else {
                static_cloud->push_back(point);
            }
        }
    }

    bool isPointInBBox(const pcl::PointXYZI& point, const Bndbox& bbox) {
        float dx = point.x - bbox.x;
        float dy = point.y - bbox.y;
        float dz = point.z - bbox.z;

        float cos_yaw = std::cos(-bbox.rt);
        float sin_yaw = std::sin(-bbox.rt);

        float local_x = cos_yaw * dx - sin_yaw * dy;
        float local_y = sin_yaw * dx + cos_yaw * dy;

        float half_l = (bbox.l / 2.0) + bbox_margin_;
        float half_w = (bbox.w / 2.0) + bbox_margin_;
        float half_h = (bbox.h / 2.0) + bbox_margin_;

        return (std::abs(local_x) <= half_l &&
                std::abs(local_y) <= half_w &&
                std::abs(dz) <= half_h);
    }

    void publishResults(
        const std_msgs::Header& header,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& static_cloud,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& dynamic_cloud,
        const std::vector<Bndbox>& bboxes) {

        // Publish static cloud
        sensor_msgs::PointCloud2 static_msg;
        pcl::toROSMsg(*static_cloud, static_msg);
        static_msg.header = header;
        static_msg.header.frame_id = lidar_frame_;
        pub_static_cloud_.publish(static_msg);

        // Publish dynamic cloud
        if (publish_dynamic_cloud_ && !dynamic_cloud->empty()) {
            sensor_msgs::PointCloud2 dynamic_msg;
            pcl::toROSMsg(*dynamic_cloud, dynamic_msg);
            dynamic_msg.header = header;
            dynamic_msg.header.frame_id = lidar_frame_;
            pub_dynamic_cloud_.publish(dynamic_msg);
        }

        // Publish bounding boxes
        if (!bboxes.empty()) {
            jsk_recognition_msgs::BoundingBoxArray bbox_array;
            bbox_array.header = header;
            bbox_array.header.frame_id = lidar_frame_;

            for (const auto& box : bboxes) {
                jsk_recognition_msgs::BoundingBox bbox;
                bbox.header = header;
                bbox.header.frame_id = lidar_frame_;
                bbox.pose.position.x = box.x;
                bbox.pose.position.y = box.y;
                bbox.pose.position.z = box.z;
                bbox.dimensions.x = box.w;
                bbox.dimensions.y = box.l;
                bbox.dimensions.z = box.h;

                tf::Quaternion q;
                q.setRPY(0, 0, -box.rt);
                bbox.pose.orientation.x = q.x();
                bbox.pose.orientation.y = q.y();
                bbox.pose.orientation.z = q.z();
                bbox.pose.orientation.w = q.w();

                bbox.value = box.score;
                bbox.label = box.id;

                bbox_array.boxes.push_back(bbox);
            }
            pub_bboxes_.publish(bbox_array);
        }
    }

    void saveToBin(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                   const std::string& path) {
        std::ofstream out(path, std::ios::out | std::ios::binary);
        float zero = 0.0f;
        for (const auto& point : cloud.points) {
            out.write((char*)&point.x, 3 * sizeof(float));
            out.write((char*)&zero, sizeof(float));
            out.write((char*)&zero, sizeof(float));
        }
        out.close();
    }

    int loadData(const char *file, void **data, unsigned int *length) {
        std::fstream dataFile(file, std::ifstream::in);
        if (!dataFile.is_open()) {
            ROS_ERROR("Can't open file: %s", file);
            return -1;
        }

        dataFile.seekg(0, dataFile.end);
        unsigned int len = dataFile.tellg();
        dataFile.seekg(0, dataFile.beg);

        char *buffer = new char[len];
        dataFile.read(buffer, len);
        dataFile.close();

        *data = (void*)buffer;
        *length = len;
        return 0;
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_pointcloud_;
    ros::Publisher pub_static_cloud_;
    ros::Publisher pub_dynamic_cloud_;
    ros::Publisher pub_bboxes_;

    std::unique_ptr<CenterPoint> centerpoint_ptr_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;

    cudaStream_t stream_;

    std::string model_path_;
    std::string lidar_frame_;
    bool verbose_;
    double min_range_;
    double max_range_;
    double bbox_margin_;
    double bbox_search_radius_;
    bool filter_by_class_;
    bool publish_dynamic_cloud_;
    std::vector<int> dynamic_classes_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_filter_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting Detection & Filter Node...");

    try {
        DetectionFilterNode node(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
