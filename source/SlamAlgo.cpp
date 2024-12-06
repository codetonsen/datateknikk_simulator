#include "SlamAlgo.h"
#include <iostream>
#include <pcl/registration/gicp.h> // Include for Generalized ICP
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>

// Definitions for PI and normalization
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*

#include "SlamAlgo.h"
#include <iostream>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>  // For std::vector
#include <mutex>   // For std::mutex, std::lock_guard
#include <utility> // For std::pair
#include <cmath>   // For std::sqrt
#include <iostream> // For std::cout, std::cerr

// Definitions for PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath> // For std::sqrt

float distance(const Eigen::Matrix4f& pose1, const Eigen::Matrix4f& pose2) {
    float dx = pose1(0, 3) - pose2(0, 3);
    float dz = pose1(2, 3) - pose2(2, 3);
    return std::sqrt(dx * dx + dz * dz);
}
Slam::Slam()
    : global_map_(new PointCloudT),
      raw_map_(new PointCloudT),
      previous_map_(new PointCloudT),
      running_(true),
      optimizing_(false) {
    slamThread_ = std::thread(&Slam::slamThreadFunc, this);
    loopClosureThread_ = std::thread(&Slam::loopClosureThreadFunc, this);

    // Initialize occupancy grid with unknown values (-1)
    occupancy_grid_ = std::vector<std::vector<int>>(grid_size_, std::vector<int>(grid_size_, -1));
}


Slam::~Slam() {
    stop();
}

void Slam::stop() {
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        running_ = false;
    }
    condition_.notify_all();
    if (slamThread_.joinable()) {
        slamThread_.join();
    }
    if (loopClosureThread_.joinable()) {
        loopClosureThread_.join();
    }
}


// Normalize angle to [-PI, PI)
float Slam::normalizeAngle(float angle) {
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    while (angle < 0.0f) angle += 2.0f * M_PI;
    return angle;
}

void Slam::enqueueFrame(const std::pair<std::vector<LidarScan>, std::vector<float>>& data) {
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        frameQueue_.push(data);
    }
    condition_.notify_one();
}
void Slam::slamThreadFunc() {
    while (running_) {
        std::pair<std::vector<LidarScan>, std::vector<float>> frame;

        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            condition_.wait(lock, [&]() { return !frameQueue_.empty() || !running_; });

            if (!running_ && frameQueue_.empty()) {
                return;
            }

            frame = frameQueue_.front();
            frameQueue_.pop();
        }

        // Process the frame
        addFrame(frame);
    }
}

void Slam::updateOccupancyGrid() {
    // Clear the grid
    for (auto& row : occupancy_grid_) {
        std::fill(row.begin(), row.end(), 0); // Default to free cells (0)
    }

    // Define the grid origin (center of the grid)
    int grid_origin = grid_size_ / 2;

    // Populate the grid based on global map points
    for (const auto& point : global_map_->points) {
        int x_idx = static_cast<int>(point.x / grid_resolution_) + grid_origin;
        int y_idx = static_cast<int>(-point.z / grid_resolution_) + grid_origin;

        // Ensure indices are within bounds
        if (x_idx >= 0 && x_idx < grid_size_ && y_idx >= 0 && y_idx < grid_size_) {
            occupancy_grid_[y_idx][x_idx] = 1; // Mark as occupied
        }
    }
}




void Slam::visualizeOccupancyGrid() {
    // Create an image for the grid
    cv::Mat grid_image(grid_size_, grid_size_, CV_8UC1, cv::Scalar(128)); // Unknown cells (gray)

    // Fill the grid image based on occupancy values
    for (int y = 0; y < grid_size_; ++y) {
        for (int x = 0; x < grid_size_; ++x) {
            if (occupancy_grid_[y][x] == 1) {
                grid_image.at<uchar>(y, x) = 0; // Occupied cells (black)
            } else if (occupancy_grid_[y][x] == 0) {
                grid_image.at<uchar>(y, x) = 255; // Free cells (white)
            }
        }
    }

    // Resize for better visualization
    cv::Mat resized_image;
    cv::resize(grid_image, resized_image, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);

    // Show the grid
    cv::imshow("Occupancy Grid Map", resized_image);
    cv::waitKey(1);
}







void Slam::addFrame(const std::pair<std::vector<LidarScan>, std::vector<float>>& data) {
    PointCloudT::Ptr current_scan(new PointCloudT);

    // Convert LidarScan data to PointCloudT
    for (const auto& scan : data.first) {
        PointT point;
        point.x = scan.range * std::cos(scan.angle);
        point.z = scan.range * std::sin(scan.angle);
        point.y = 0.0f; // Assume 2D LiDAR for simplicity
        if (std::isfinite(point.x) && std::isfinite(point.z)) {
            current_scan->points.push_back(point);
        }
    }

    if (current_scan->empty()) {
        std::cerr << "Warning: Empty scan received. Skipping frame." << std::endl;
        return;
    }

    // Convert odometry data (x, z, theta) to Matrix4f
    Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
    if (data.second.size() == 3) {
        float x = data.second[0];
        float z = data.second[1];
        float theta = normalizeAngle(data.second[2]);

        odometry_matrix(0, 3) = x;
        odometry_matrix(2, 3) = z;
        odometry_matrix(0, 0) = std::cos(theta);
        odometry_matrix(0, 2) = std::sin(theta);
        odometry_matrix(2, 0) = -std::sin(theta);
        odometry_matrix(2, 2) = std::cos(theta);


    } else {
        std::cerr << "Error: Odometry data must have 3 entries (x, z, theta)." << std::endl;
        return;
    }

    // Save the odometry pose
    odometry_poses_.push_back(odometry_matrix);



    // Add the raw scan to the raw map
    *raw_map_ += *current_scan;

    // Transform the point cloud using the odometry pose
    PointCloudT::Ptr transformed_scan(new PointCloudT);
    pcl::transformPointCloud(*current_scan, *transformed_scan, odometry_matrix);
    // Check if a new keyframe is needed
    {
        std::lock_guard<std::mutex> lock(keyframeMutex_);
        if (keyframes_.empty() || (distance(odometry_matrix, keyframes_.back().second) > 0.5)) {
            keyframes_.emplace_back(transformed_scan, odometry_matrix);
        }
    }
    // Perform GICP refinement
    if (!previous_map_->empty()) {
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setInputSource(transformed_scan);
        gicp.setInputTarget(previous_map_); // Compare with the last processed map
        gicp.setMaxCorrespondenceDistance(1.0); // Adjust as needed
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);

        PointCloudT aligned_scan;
        gicp.align(aligned_scan);

        if (gicp.hasConverged()) {
            std::cout << "GICP converged. Fitness score: " << gicp.getFitnessScore() << std::endl;

            // Replace transformed_scan with the aligned scan
            *transformed_scan = aligned_scan;
        } else {
            std::cerr << "GICP failed to converge!" << std::endl;
        }
    }

    // Add the transformed (and optionally refined) scan to the global map
    *global_map_ += *transformed_scan;

    // Update the previous map
    *previous_map_ = *transformed_scan;

    // Visualize the transformed map
    visualizeMap();

    visualizeOccupancyGrid();

}

void Slam::loopClosureThreadFunc() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Check for loops every 5 seconds

        if (!running_) break;

        // Detect loop closure
        detectLoopClosure();

        // Optimize the graph if a loop is detected
        optimizeGraph();
    }
}

void Slam::detectLoopClosure() {
    std::lock_guard<std::mutex> lock(keyframeMutex_);

    if (keyframes_.size() < 2) return;

    auto& current = keyframes_.back();
    for (size_t i = 0; i < keyframes_.size() - 1; ++i) {
        // Compare current keyframe with past keyframes
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setInputSource(current.first);
        gicp.setInputTarget(keyframes_[i].first);

        PointCloudT aligned_scan;
        gicp.align(aligned_scan);

        if (gicp.hasConverged() && gicp.getFitnessScore() < 0.5) {
            std::cout << "Loop closure detected between keyframes " << i << " and " << keyframes_.size() - 1 << std::endl;

            Eigen::Matrix4f relative_transform = gicp.getFinalTransformation();
            graph_edges_.emplace_back(i, keyframes_.size() - 1, relative_transform);

        }
    }
}


void Slam::optimizeGraph() {
    std::lock_guard<std::mutex> lock(keyframeMutex_);
    if (keyframes_.size() < 2 || graph_edges_.empty()) return;

    std::cout << "Starting graph optimization..." << std::endl;

    // Initialize poses to current keyframe poses
    graph_poses_ = std::vector<Eigen::Matrix4f>(keyframes_.size());
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        graph_poses_[i] = keyframes_[i].second; // Start with the current poses
    }


    // Simple gradient descent optimization
    constexpr int max_iterations = 50;
    constexpr float step_size = 0.01f;

    for (int iter = 0; iter < max_iterations; ++iter) {
        for (const auto& edge : graph_edges_) {
            int idx1 = std::get<0>(edge);
            int idx2 = std::get<1>(edge);
            Eigen::Matrix4f relative_transform = std::get<2>(edge);

            // Calculate current relative transform between poses
            Eigen::Matrix4f current_transform = graph_poses_[idx1].inverse() * graph_poses_[idx2];

            // Compute error
            Eigen::Matrix4f error = relative_transform.inverse() * current_transform;

            // Update poses to minimize error
            graph_poses_[idx1] = graph_poses_[idx1] * (step_size * error);
            graph_poses_[idx2] = graph_poses_[idx2] * (step_size * error.inverse());
        }
    }

    std::cout << "Graph optimization complete." << std::endl;

    // Update keyframe poses with optimized poses
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        keyframes_[i].second = graph_poses_[i];
    }

    // Update global map
    updateGlobalMap();
    optimized_poses_available_ = true; // Optimized poses are now available

}

void Slam::updateGlobalMap() {
    global_map_->clear();
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        PointCloudT::Ptr transformed_scan(new PointCloudT);
        pcl::transformPointCloud(*keyframes_[i].first, *transformed_scan, graph_poses_[i]);
        *global_map_ += *transformed_scan;
    }
    // Update the occupancy grid
    updateOccupancyGrid();
}


/*
void Slam::visualizeMap() {
    // Define the map image
    int image_size = 1000;         // Adjust for resolution
    float map_scale = 50.0f;       // Pixels per meter
    cv::Mat map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f);

    // Draw the global map points
    for (const auto& point : global_map_->points) {
        int u = static_cast<int>(point.x * map_scale + map_origin.x);
        int v = static_cast<int>(-point.z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            map_image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255); // White points
        }
    }

    // Draw odometry poses and headings (red markers, green lines)
    for (const auto& pose : odometry_poses_) {
        float x = pose(0, 3);
        float z = pose(2, 3);
        float theta = std::atan2(pose(2, 0), pose(0, 0)) + M_PI_2;

        int u = static_cast<int>(x * map_scale + map_origin.x);
        int v = static_cast<int>(-z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            // Draw a circle for the odometry position
            cv::circle(map_image, cv::Point(u, v), 5, cv::Scalar(0, 0, 255), -1); // Red circle

            // Draw a line indicating heading
            int line_length = 15;
            int end_u = static_cast<int>(u + line_length * std::cos(theta));
            int end_v = static_cast<int>(v - line_length * std::sin(theta));
            cv::line(map_image, cv::Point(u, v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 0), 2); // Green line
        }
    }

    // Draw optimized poses and headings (blue markers, yellow lines)
    for (const auto& pose : graph_poses_) {
        float x = pose(0, 3);
        float z = pose(2, 3);
        float theta = std::atan2(pose(2, 0), pose(0, 0)) + M_PI_2;

        int u = static_cast<int>(x * map_scale + map_origin.x);
        int v = static_cast<int>(-z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            // Draw a circle for the optimized position
            cv::circle(map_image, cv::Point(u, v), 5, cv::Scalar(255, 0, 0), -1); // Blue circle

            // Draw a line indicating heading
            int line_length = 15;
            int end_u = static_cast<int>(u + line_length * std::cos(theta));
            int end_v = static_cast<int>(v - line_length * std::sin(theta));
            cv::line(map_image, cv::Point(u, v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 255), 2); // Yellow line
        }
    }

    // Display the map
    cv::imshow("SLAM Map", map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}*/
/*
void Slam::visualizeMap() {
    // Define the map image
    int image_size = 1000;         // Adjust for resolution
    float map_scale = 20.0f;       // Pixels per meter
    cv::Mat map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f);

    // Draw the global map points
    for (const auto& point : global_map_->points) {
        int u = static_cast<int>(point.x * map_scale + map_origin.x);
        int v = static_cast<int>(-point.z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            map_image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255); // White points
        }
    }

    // Draw odometry poses and headings (red markers, green lines)
    for (const auto& pose : odometry_poses_) {
        float x = pose(0, 3);
        float z = pose(2, 3);
        float theta = std::atan2(pose(2, 0), pose(0, 0)) + M_PI_2;

        int u = static_cast<int>(x * map_scale + map_origin.x);
        int v = static_cast<int>(-z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            // Draw a circle for the odometry position
            cv::circle(map_image, cv::Point(u, v), 5, cv::Scalar(0, 0, 255), -1); // Red circle

            // Draw a line indicating heading
            int line_length = 15;
            int end_u = static_cast<int>(u + line_length * std::cos(theta));
            int end_v = static_cast<int>(v - line_length * std::sin(theta));
            cv::line(map_image, cv::Point(u, v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 0), 2); // Green line
        }
    }

    // Draw optimized poses and headings if available (blue markers, yellow lines)
    if (optimized_poses_available_) {
        for (const auto& pose : graph_poses_) {
            float x = pose(0, 3);
            float z = pose(2, 3);
            float theta = std::atan2(pose(2, 0), pose(0, 0)) + M_PI_2;

            int u = static_cast<int>(x * map_scale + map_origin.x);
            int v = static_cast<int>(-z * map_scale + map_origin.y);

            if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
                // Draw a circle for the optimized position
                cv::circle(map_image, cv::Point(u, v), 5, cv::Scalar(255, 0, 0), -1); // Blue circle

                // Draw a line indicating heading
                int line_length = 15;
                int end_u = static_cast<int>(u + line_length * std::cos(theta));
                int end_v = static_cast<int>(v - line_length * std::sin(theta));
                cv::line(map_image, cv::Point(u, v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 255), 2); // Yellow line
            }
        }
    }

    // Display the map
    cv::imshow("SLAM Map", map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}

void Slam::visualizeOptimizedMap() {
    // Define the map image
    int image_size = 1000;         // Adjust for resolution
    float map_scale = 20.0f;       // Pixels per meter
    cv::Mat optimized_map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f);

    // Draw the optimized map points
    for (const auto& point : global_map_->points) {
        int u = static_cast<int>(point.x * map_scale + map_origin.x);
        int v = static_cast<int>(-point.z * map_scale + map_origin.y);

        if (u >= 0 && u < optimized_map_image.cols && v >= 0 && v < optimized_map_image.rows) {
            optimized_map_image.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 255, 255); // Cyan points
        }
    }

    // Display the optimized map
    cv::imshow("Optimized SLAM Map", optimized_map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}

*/

#include "SlamAlgo.h"
#include <iostream>
#include <pcl/registration/gicp.h> // Include for Generalized ICP
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>

// Definitions for PI and normalization
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#include "SlamAlgo.h"
#include <iostream>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <cmath>

// Definitions for PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Slam::Slam()
    : global_map_(new PointCloudT),
      raw_map_(new PointCloudT),
      previous_map_(new PointCloudT),
      running_(true) {
    slamThread_ = std::thread(&Slam::slamThreadFunc, this);

    try {
        offlineData_ = parseDataset("C:/Users/ekorn/Documents/RoverScanData/Mags/data/ProcessedDataSofaRom.txt",0,0,0);
        std::cout << "Parsed " << offlineData_.size() << " frames.\n";
        for (size_t i = 0; i < std::min(offlineData_.size(), size_t(5)); ++i) {
            const auto& [scans, positionData] = offlineData_[i];
            std::cout << "Frame " << i + 1 << ": " << scans.size() << " lidar scans, Position: ("
                      << positionData[0] << ", " << positionData[1] << "), Heading: " << positionData[2] << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
    }
    visualizeOdometryRawMap();
}




Slam::~Slam() {
    stop();
}

void Slam::stop() {
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        running_ = false;
    }
    condition_.notify_all();
    if (slamThread_.joinable()) {
        slamThread_.join();
    }
}

// Normalize angle to [-PI, PI)
float Slam::normalizeAngle(float angle) {
    //while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    //while (angle < 0.0f) angle += 2.0f * M_PI;
    return angle;
}
void Slam::enqueueFrame(const std::pair<std::vector<LidarScan>, std::vector<float>>& data) {
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        frameQueue_.push(data);
    }
    condition_.notify_one();
}
void Slam::slamThreadFunc() {
    while (running_) {
        std::pair<std::vector<LidarScan>, std::vector<float>> frame;

        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            condition_.wait(lock, [&]() { return !frameQueue_.empty() || !running_; });

            if (!running_ && frameQueue_.empty()) {
                return;
            }

            frame = frameQueue_.front();
            frameQueue_.pop();
        }

        // Process the frame
        addFrame(frame);
    }
}


void Slam::addFrame(const std::pair<std::vector<LidarScan>, std::vector<float>>& data) {
    PointCloudT::Ptr current_scan(new PointCloudT);

    // Convert LidarScan data to PointCloudT
    for (const auto& scan : data.first) {
        PointT point;
        point.x = scan.range * std::cos(scan.angle);
        point.z = scan.range * std::sin(scan.angle);
        point.y = 0.0f; // Assume 2D LiDAR for simplicity
        if (std::isfinite(point.x) && std::isfinite(point.z)) {
            current_scan->points.push_back(point);
        }
    }

    if (current_scan->empty()) {
        std::cerr << "Warning: Empty scan received. Skipping frame." << std::endl;
        return;
    }

    // Convert odometry data (x, z, theta) to Matrix4f
    Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
    if (data.second.size() == 3) {
        float x = data.second[0];
        float z = data.second[1];
        float theta = normalizeAngle(data.second[2]);

        odometry_matrix(0, 3) = x;
        odometry_matrix(2, 3) = z;
        odometry_matrix(0, 0) = std::cos(theta);
        odometry_matrix(0, 2) = std::sin(theta);
        odometry_matrix(2, 0) = -std::sin(theta);
        odometry_matrix(2, 2) = std::cos(theta);


    } else {
        std::cerr << "Error: Odometry data must have 3 entries (x, z, theta)." << std::endl;
        return;
    }

    // Save the odometry pose
    odometry_poses_.push_back(odometry_matrix);

    // Add the raw scan to the raw map
    *raw_map_ += *current_scan;

    // Transform the point cloud using the odometry pose
    PointCloudT::Ptr transformed_scan(new PointCloudT);
    pcl::transformPointCloud(*current_scan, *transformed_scan, odometry_matrix);

    // Perform GICP refinement
    if (!previous_map_->empty()) {
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setInputSource(transformed_scan);
        gicp.setInputTarget(previous_map_); // Compare with the last processed map
        gicp.setMaxCorrespondenceDistance(1.0); // Adjust as needed
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);

        PointCloudT aligned_scan;
        gicp.align(aligned_scan);

        if (gicp.hasConverged()) {
            std::cout << "GICP converged. Fitness score: " << gicp.getFitnessScore() << std::endl;

            // Replace transformed_scan with the aligned scan
            *transformed_scan = aligned_scan;
        } else {
            std::cerr << "GICP failed to converge!" << std::endl;
        }
    }

    // Add the transformed (and optionally refined) scan to the global map
    *global_map_ += *transformed_scan;

    // Update the previous map
    *previous_map_ = *transformed_scan;

    // Visualize the transformed map
    visualizeMap();

    // Visualize the raw map
    visualizeRawMap();
}




void Slam::visualizeMap() {
    // Define the map image
    int image_size = 1000;         // Adjust for resolution
    float map_scale = 20.0f;       // Pixels per meter
    cv::Mat map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f);

    // Draw the global map points
    for (const auto& point : global_map_->points) {
        int u = static_cast<int>(-point.x * map_scale + map_origin.x);
        int v = static_cast<int>(point.z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            map_image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255); // White points
        }
    }

    // Draw odometry positions and headings
    for (const auto& pose : odometry_poses_) {
        // Get position from the odometry matrix
        float x = pose(0, 3);
        float z = -pose(2, 3);
        // Adjust the heading by adding π/2 (90 degrees)
        float theta = std::atan2(pose(2, 0), pose(0, 0));// + M_PI_2;
        int u = static_cast<int>(x * map_scale + map_origin.x);
        int v = static_cast<int>(z * map_scale + map_origin.y);

        if (u >= 0 && u < map_image.cols && v >= 0 && v < map_image.rows) {
            // Draw a circle for the odometry position
            cv::circle(map_image, cv::Point(u, v), 5, cv::Scalar(0, 0, 255), -1); // Red circle

            // Draw a line indicating heading
            int line_length = 15; // Adjust line length as needed
            int end_u = static_cast<int>(u + line_length * std::cos(theta));
            int end_v = static_cast<int>(v - line_length * std::sin(theta));
            cv::line(map_image, cv::Point(u, v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 0), 2); // Green line
        }
    }

    // Display the map
    cv::imshow("SLAM Map", map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}
void Slam::visualizeRawMap() {
    // Define the raw map image
    int image_size = 1000;         // Adjust for resolution
    float map_scale = 20.0f;       // Pixels per meter
    cv::Mat raw_map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f);

    // Draw the raw map points
    for (const auto& point : raw_map_->points) {
        int u = static_cast<int>(point.x * map_scale + map_origin.x);
        int v = static_cast<int>(-point.z * map_scale + map_origin.y);

        if (u >= 0 && u < raw_map_image.cols && v >= 0 && v < raw_map_image.rows) {
            raw_map_image.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 255, 255); // White points
        }
    }

    // Display the raw map
    cv::imshow("Raw Map (No Odometry)", raw_map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}

std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> Slam::parseDataset(const std::string& filename, float offsetX = 0, float offsetY = 0, float offsetHeading = 0) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> frames;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string segment;

        // Parse each line
        std::getline(ss, segment, ';');
        float timestamp = std::stof(segment);

        std::getline(ss, segment, ';');
        float posX = std::stof(segment);

        std::getline(ss, segment, ';');
        float posY = std::stof(segment);

        std::getline(ss, segment, ';');
        float heading = std::stof(segment) + offsetHeading;

        // Adjust positions
        //float _posX = posX * std::sin(offsetHeading) + posY * std::cos(offsetHeading) + offsetX;
        //float _posY = -(posX * std::cos(offsetHeading) - posY * std::sin(offsetHeading)) + offsetY;
        float _posX = posX;
        float _posY = posY;

        // Store the x, y, and heading
        std::vector<float> positionData = {_posX, _posY, heading};

        // Parse lidar data
        std::getline(ss, segment, ';');
        std::istringstream lidarData(segment);
        std::string point;

        std::vector<LidarScan> lidarScans;
        while (std::getline(lidarData, point, ':')) {
            auto commaPos = point.find(',');
            if (commaPos == std::string::npos) continue;

            float x = std::stof(point.substr(0, commaPos));
            float y = std::stof(point.substr(commaPos + 1));

            float _x = x * std::cos(heading) - y * std::sin(heading) + _posX;
            float _y = x * std::sin(heading) + y * std::cos(heading) + _posY;

            LidarScan scan;
            scan.angle = std::atan2(_y - _posY, _x - _posX);
            scan.range = std::sqrt(std::pow(_x - _posX, 2) + std::pow(_y - _posY, 2));
            scan.start = threepp::Vector3(_posX, 0, _posY);
            scan.end = threepp::Vector3(_x, 0, _posY);
            scan.x = x;
            scan.y = y;

            lidarScans.push_back(scan);
        }

        // Add the parsed frame to the frames vector
        frames.emplace_back(lidarScans, positionData);
    }

    file.close();
    return frames;
}
/*
void Slam::visualizeOdometryRawMap() {
    // Define the image size and scaling factor
    int image_size = 1600;         // Pixels for the visualization
    float map_scale = 40.0f;       // Pixels per meter (adjust for better resolution)
    cv::Mat map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f); // Center of the image

    // Loop through all frames in offlineDataMap_
    for (const auto& frame : offlineData_) {
        const auto& lidarScans = frame.first;  // LIDAR points
        const auto& odometry = frame.second;  // Odometry data {x, y, heading}

        // Get odometry position and heading
        float odom_x = odometry[0];
        float odom_y = odometry[1];
        float heading = odometry[2];

        // Calculate the image coordinates for the odometry position
        int odom_u = static_cast<int>(odom_x * map_scale + map_origin.x);
        int odom_v = static_cast<int>(odom_y * map_scale + map_origin.y);

        // Draw odometry position
        if (odom_u >= 0 && odom_u < map_image.cols && odom_v >= 0 && odom_v < map_image.rows) {
            // Red circle for odometry position
            cv::circle(map_image, cv::Point(odom_u, odom_v), 2, cv::Scalar(0, 0, 255), -1);

            // Draw heading line
            int line_length = 15; // Adjust as needed
            int end_u = static_cast<int>(odom_u + line_length * std::cos(heading));
            int end_v = static_cast<int>(odom_v - line_length * std::sin(heading));
            cv::line(map_image, cv::Point(odom_u, odom_v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 0), 2); // Green line
        }

        // Draw LIDAR points relative to the odometry position
        for (const auto& scan : lidarScans) {
            // Offset LIDAR points relative to the odometry position
            float relative_x = scan.x - odom_x;
            float relative_y = scan.y - odom_y;


            float newX = scan.x * cos(heading) + scan.y * sin(heading) - odom_x;
            float newY = scan.x * sin(heading) - scan.y * cos(heading) - odom_y;

            // Transform to image coordinates
            int point_u = static_cast<int>((newX) * map_scale + map_origin.x);
            int point_v = static_cast<int>((newY) * map_scale + map_origin.y);

            // Draw LIDAR points (white dots)
            if (point_u >= 0 && point_u < map_image.cols && point_v >= 0 && point_v < map_image.rows) {
                map_image.at<cv::Vec3b>(point_v, point_u) = cv::Vec3b(255, 255, 255); // White point
            }
        }
    }

    // Display the map
    cv::imshow("Odometry and Raw LIDAR Map", map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}
*/

void Slam::visualizeOdometryRawMap() {
    // Define the image size and scaling factor
    int image_size = 1600;         // Pixels for the visualization
    float map_scale = 25.0f;       // Pixels per meter (adjust for better resolution)
    cv::Mat map_image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    cv::Point2f map_origin(image_size / 2.0f, image_size / 2.0f); // Center of the image

    if (offlineData_.empty()) {
        std::cerr << "No data in offlineData_, cannot visualize map." << std::endl;
        return;
    }

    // Get the starting position and heading
    const auto& first_odometry = offlineData_.front().second; // First odometry data
    float start_x = first_odometry[0];
    float start_y = first_odometry[1];
    float start_heading = first_odometry[2];

    // Loop through all frames in offlineData_
    for (const auto& frame : offlineData_) {
        const auto& lidarScans = frame.first;  // LIDAR points
        const auto& odometry = frame.second;  // Odometry data {x, y, heading}

        // Get current odometry position and heading
        float odom_x = odometry[0];
        float odom_y = odometry[1];
        float heading = odometry[2];

        // Calculate the image coordinates for the odometry position
        int odom_u = static_cast<int>((odom_x - start_x) * map_scale + map_origin.x);
        int odom_v = static_cast<int>((odom_y - start_y) * map_scale + map_origin.y);

        // Draw odometry position
        if (odom_u >= 0 && odom_u < map_image.cols && odom_v >= 0 && odom_v < map_image.rows) {
            // Red circle for odometry position
            cv::circle(map_image, cv::Point(odom_u, odom_v), 2, cv::Scalar(0, 0, 255), -1);

            // Draw heading line
            int line_length = 15; // Adjust as needed
            int end_u = static_cast<int>(odom_u + line_length * std::cos(heading));
            int end_v = static_cast<int>(odom_v - line_length * std::sin(heading));
            cv::line(map_image, cv::Point(odom_u, odom_v), cv::Point(end_u, end_v), cv::Scalar(0, 255, 0), 2); // Green line
        }

        // Draw LIDAR points relative to the global odometry position
        for (const auto& scan : lidarScans) {
            // Transform LIDAR points to global coordinates
            float global_x = (scan.x * std::cos(heading) - scan.y * std::sin(heading)) + odom_x;
            float global_y = (scan.x * std::sin(heading) + scan.y * std::cos(heading)) + odom_y;

            // Transform global coordinates to image coordinates
            int point_u = static_cast<int>((global_x - start_x) * map_scale + map_origin.x);
            int point_v = static_cast<int>((global_y - start_y) * map_scale + map_origin.y);

            // Draw LIDAR points (white dots)
            if (point_u >= 0 && point_u < map_image.cols && point_v >= 0 && point_v < map_image.rows) {
                map_image.at<cv::Vec3b>(point_v, point_u) = cv::Vec3b(255, 255, 255); // White point
            }
        }
    }

    // Display the map
    cv::imshow("Odometry and Raw LIDAR Map", map_image);
    cv::waitKey(1); // Wait for 1 ms to allow OpenCV to process events
}

// VERSION FUCKING 4

