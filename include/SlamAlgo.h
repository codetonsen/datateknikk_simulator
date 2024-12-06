// SlamAlgo.h

#ifndef SLAM_ALGO_H
#define SLAM_ALGO_H


#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Sphero.h"
// SlamAlgo.h
#include <cmath>
#include <mutex>
#include <atomic>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// Include this for aligned allocator
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Sphero.h"
// Forward declaration for g2o classes


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

class Slam {
public:
    Slam();
    ~Slam();

    void enqueueFrame(const std::pair<std::vector<LidarScan>, std::vector<float>>& data);
    void stop();
    void updateGlobalMap();
    void visualizeOptimizedMap();
    void visualizeOccupancyGrid();
    void updateOccupancyGrid();

    std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> parseDataset(
    const std::string &filename, float offsetX, float offsetY, float offsetHeading);

    void visualizeOdometryRawMap();

    std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> offlineData_;

private:
    void slamThreadFunc();
    void loopClosureThreadFunc();

    std::vector<Eigen::Matrix4f> graph_poses_; // Optimized poses of keyframes
    std::vector<std::tuple<int, int, Eigen::Matrix4f>> graph_edges_; // Edges: (node1, node2, relative_transform)

    bool optimized_poses_available_ = false; // Indicates if optimized poses are ready
    float grid_resolution_ = 0.1f; // 0.1 meter per cell
    int grid_size_ = 1000;         // Grid dimensions (100x100 meters for 0.1m resolution)
    std::vector<std::vector<int>> occupancy_grid_; // 2D grid for occupancy

    std::thread slamThread_;
    std::thread loopClosureThread_;
    std::mutex queueMutex_;
    std::mutex keyframeMutex_; // Protect keyframes during optimization
    std::condition_variable condition_;
    std::queue<std::pair<std::vector<LidarScan>, std::vector<float>>> frameQueue_;
    bool running_;
    bool optimizing_; // Indicates if optimization is in progress

    PointCloudT::Ptr global_map_;
    PointCloudT::Ptr raw_map_;
    PointCloudT::Ptr previous_map_;
    std::vector<std::pair<PointCloudT::Ptr, Eigen::Matrix4f>> keyframes_; // Stores keyframe point clouds and poses
    std::vector<Eigen::Matrix4f> odometry_poses_;





    float normalizeAngle(float angle);

    void addFrame(const std::pair<std::vector<LidarScan>, std::vector<float>>& data);
    void detectLoopClosure();
    void optimizeGraph();
    void visualizeMap();
    void visualizeRawMap();




};



/*
class SLAM {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    std::mutex data_mutex_;
    cv::Mat map_image_;
    std::atomic<bool> running_;

    void visualize() {
        while (running_) {
            cv::Mat display_map;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (map_image_.empty()) {
                    continue; // Skip visualization if the map is empty
                }
                display_map = map_image_.clone();
            }
            cv::imshow("Finished Map", display_map);
            if (cv::waitKey(30) == 27) { // Exit on ESC key
                running_ = false;
            }
        }
    }


    void updateMapImage() {
        map_image_ = cv::Mat::zeros(1000, 1000, CV_8UC3); // Reset the map image

        if (map_cloud_->points.empty()) {
            return; // No points to display
        }

        for (const auto& point : map_cloud_->points) {
            int px = static_cast<int>(point.x * 10 + map_image_.cols / 2);
            int py = static_cast<int>(-point.y * 10 + map_image_.rows / 2);

            if (px >= 0 && px < map_image_.cols && py >= 0 && py < map_image_.rows) {
                map_image_.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 255, 0); // Draw green points
            }
        }
    }


public:
    SLAM() : map_cloud_(new pcl::PointCloud<pcl::PointXYZ>), running_(true) {}

    ~SLAM() {
        running_ = false;
    }

    void update(const std::pair<std::vector<LidarScan>, std::vector<float>>& input) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Extract point cloud and transform
        const auto& lidar_points = input.first;
        const auto& transform = input.second;

        // Convert lidar_points to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& scan : lidar_points) {
            input_cloud->points.emplace_back(scan.start.x, scan.start.z, 0.0f);
        }

        // Transform the input cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
        transformation.translation() << transform[0], 0.0f, transform[1];
        transformation.rotate(Eigen::AngleAxisf(transform[2], Eigen::Vector3f::UnitY()));
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, transformation);

        // Perform ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(transformed_cloud);
        icp.setInputTarget(map_cloud_);
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud);

        // Merge aligned cloud into the map
        *map_cloud_ += aligned_cloud;

        // Update the map image
        updateMapImage();
    }

    void startVisualization() {
        std::thread vis_thread(&SLAM::visualize, this);
        vis_thread.detach();
    }
};
*/

#endif // SLAM_ALGO_H

