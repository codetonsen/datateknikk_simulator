//
// Created by Codetonsen on 11/25/2024.
//
#include "../include/Slam.h"

#include <iostream>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <cmath>

#include "threepp/input/KeyListener.hpp"


CustomSLAM::CustomSLAM(): grid(grid) {
    //  TODO: remember to initialize all variables!
    gridMap = KeyframeGrid();
    // threading
    slamThread = std::thread(&CustomSLAM::processFrames, this);
}

CustomSLAM::~CustomSLAM() {
    // threading
    exportPoseGraph("this.g2o");
    stopSLAMThread();
}

void CustomSLAM::enqueueFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame) {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        frameQueue.push(dataFrame);
    }
    frameAvailable.notify_one();
}

void CustomSLAM::processFrames() {
    while (!stopThread) {
        std::unique_lock<std::mutex> lock(queueMutex);

        // Wait for a frame to be available
        frameAvailable.wait(lock, [this] { return !frameQueue.empty() || stopThread; });

        if (stopThread && frameQueue.empty())
            break;

        // Get the next frame
        auto frame = frameQueue.front();
        frameQueue.pop();
        lock.unlock();

        // Process the frame
        addFrame(frame);
    }
}

void CustomSLAM::stopSLAMThread() {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        stopThread = true;
    }
    frameAvailable.notify_all();
    if (slamThread.joinable()) {
        slamThread.join();
    }
}

void CustomSLAM::addFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame) {

    // 0. Break if frame has no data
    // 0. Break if deltadistance is too low
    // NOW THE DATA IS KNOWN TO BE GOOD
    // 1. If the rover has been inside the grid before
    //      indexes to scan = getindex(gx,gy)
    //
    // 2. If the rover has NOT been inside the grid before




    std::cout << "Got frame!" << std::endl;
    // Checks to make sure its data in the input
    std::vector<std::pair<float, float>> lidarData;
    std::vector<float> odometryData;
    if(dataFrame.first.size() > 0 or dataFrame.second.size() > 0) {
        lidarData = dataFrame.first;
        odometryData = dataFrame.second;
        captureCounter++;
    }


    // If first frame then run through everything except loop closure
    if(lidarFrames.size() == 0 and vertexFrames.size() == 0) {
        // Set offset to whatever the odometry data is to "center data"
        x_offset = odometryData[0];
        y_offset = odometryData[1];
        heading_offset = odometryData[2];

        // Add the keyframe
        addLidar(dataFrame.first);
        // Add the pose
        addPose(dataFrame.second);
        gridMap.setValue(gridMap.getIndex(x_offset, y_offset),20);
    } else {

        // Find euclidean distance between current position and last pose and if its above threshold
        auto x1 = poseFrames[poseFrames.size()-1][0];
        auto y1 = poseFrames[poseFrames.size()-1][1];
        auto heading1 = poseFrames[poseFrames.size()-1][2];
        auto x2 = odometryData[0];
        auto y2 = odometryData[1];
        auto heading2 = odometryData[2];



        auto distance = sqrt(pow((x2 - x1),2) + pow((y2 - y1),2));
        if (distance < distanceBetweenPoses) {return;} // Break out if distance travelled is not great enough

        std::cout << "Distance is above threshold, adding frame" << distance << ">" << distanceBetweenPoses << std::endl;

        // Here we add the pose stuff
        addVertex(x2, y2, heading2);
        addEdge(x2-x1, y2-y1, heading2 - heading1);
        current_insecurity = sqrt(countSinceLastLoopClosure);

        std::cout << "current insecurity: " << current_insecurity << std::endl;





        auto i = 0;
        // This is most of the frames
        // Here I add them to processing as first, but i also process it compared to the previous frame with ICP. (Currently no ICP because lazy)

        if (gridMap.getValue(gridMap.getIndex(x2,y2)) == 20) {  // 20 = been there before
            // This means a loop closure has been found
            // run loop closure algorithm
            auto scansToCheck = gridMap.getLidarScans(gridMap.getIndex(x2,y2));
            for (auto index : scansToCheck) {std::cout<< "ScanIndex: " << index << std::endl;}

            loopClosureList.push_back({captureCounter, gridMap.getIndex(x2,y2)});
            runICP(lidarFrames[captureCounter], lidarFrames[captureCounter - 1]); // Need to find a way to get the scanIndex from the grid map


            gridMap.setValue(gridMap.getIndex(x2, y2),20);
            countSinceLastLoopClosure = 1;
        } else {
            gridMap.setValue(gridMap.getIndex(x2, y2),20);
            countSinceLastLoopClosure += 1;
        }
        addLidar(dataFrame.first);
        // Add the pose
        addPose(dataFrame.second);


        renderPreprocessedMap();
    }
}

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void CustomSLAM::runICP(std::vector<std::pair<float, float>> firstFrame, std::vector<std::pair<float, float>> secondFrame) { // Compares ICP on these frames. If good match returns error and alignment transform
    PointCloudT::Ptr firstScan(new PointCloudT);
    PointCloudT::Ptr secondScan(new PointCloudT);

    // Convert LidarScan data to PointCloudT
    for (const auto& scan : firstFrame) {
        PointT point;
        point.x = scan.first;
        point.y = 0.0f;
        point.z = scan.second;
        if (std::isfinite(point.x) && std::isfinite(point.z)) {
            firstScan->points.push_back(point);
        }
    }

    for (const auto& scan : secondFrame) {
        PointT point;
        point.x = scan.first;
        point.y = 0.0f;
        point.z = scan.second;
        if (std::isfinite(point.x) && std::isfinite(point.z)) {
            secondScan->points.push_back(point);
        }
    }

    PointCloudT::Ptr transformed_scan(new PointCloudT);


    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(firstScan);
    gicp.setInputTarget(secondScan); // Compare with the last processed map
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


bool CustomSLAM::pathIntersected() {
    return false;
}




void CustomSLAM::addLidar(std::vector<std::pair<float, float>> lidarData) {
    lidarFrames.push_back(lidarData);
}

void CustomSLAM::addPose(std::vector<float> pose) {
    poseFrames.push_back(pose);
}




// ChatGPT visualization, because im still not able to use g2o visu or pcl visu.


cv::Point worldToImage(float worldX, float worldY, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight) {
    int imgX = static_cast<int>((worldX - minX) / (maxX - minX) * imgWidth);
    int imgY = imgHeight - static_cast<int>((worldY - minY) / (maxY - minY) * imgHeight);
    return cv::Point(imgX, imgY);
}

void CustomSLAM::renderDebugGrid(cv::Mat& mapImage, const KeyframeGrid& grid, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight) {
    // Loop through each cell in the grid
    /*for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            // Map grid indices back to real-world coordinates
            float worldX = (x * grid.cellSize) - grid.xOffset;
            float worldY = (y * grid.cellSize) - grid.yOffset;

            // Map real-world coordinates to image pixel coordinates
            cv::Point topLeft = worldToImage(worldX, worldY, minX, minY, maxX, maxY, imgWidth, imgHeight);
            cv::Point bottomRight = worldToImage(worldX + grid.cellSize, worldY + grid.cellSize, minX, minY, maxX, maxY, imgWidth, imgHeight);

            // Ensure the points are within the image boundaries
            if (topLeft.x >= 0 && topLeft.x < imgWidth && topLeft.y >= 0 && topLeft.y < imgHeight &&
                bottomRight.x >= 0 && bottomRight.x < imgWidth && bottomRight.y >= 0 && bottomRight.y < imgHeight) {

                // Draw the grid cell as a rectangle
                cv::rectangle(
                    mapImage,
                    cv::Rect(topLeft, bottomRight),
                    cv::Scalar(100, 100, 100), // Gray color for the debug grid
                    1, // Line thickness
                    cv::LINE_AA
                );
                }
        }
    }

    // Display the debug grid overlay
    cv::imshow("Debug Grid", mapImage);
    cv::waitKey(1); // Adjust as needed for real-time or step-by-step debugging*/
}
void CustomSLAM::renderPreprocessedMap() {
    float minX = -10.0f, maxX = 10.0f;
    float minY = -10.0f, maxY = 10.0f;
    int imgWidth = 1000;
    int imgHeight = 1000;

    cv::Mat mapImage = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC3);

    for (size_t i = 0; i < lidarFrames.size(); ++i) {
        const auto& lidarPoints = lidarFrames[i];
        const auto& pose = poseFrames[i];
        float robotX = pose[0];
        float robotY = pose[1];
        float robotTheta = -pose[2];

        for (const auto& point : lidarPoints) {
            float x_local = point.first;
            float y_local = point.second;

            float x_global = x_local * cos(robotTheta) - y_local * sin(robotTheta) + robotX;
            float y_global = x_local * sin(robotTheta) + y_local * cos(robotTheta) + robotY;

            cv::Point imgPoint = worldToImage(x_global, y_global, minX, minY, maxX, maxY, imgWidth, imgHeight);

            if (imgPoint.x >= 0 && imgPoint.x < imgWidth && imgPoint.y >= 0 && imgPoint.y < imgHeight) {
                mapImage.at<cv::Vec3b>(imgPoint.y, imgPoint.x) = cv::Vec3b(0, 255, 0); // Green
            }
        }

        cv::Point imgRobot = worldToImage(robotX, robotY, minX, minY, maxX, maxY, imgWidth, imgHeight);
        if (imgRobot.x >= 0 && imgRobot.x < imgWidth && imgRobot.y >= 0 && imgRobot.y < imgHeight) {
            cv::circle(mapImage, imgRobot, 3, cv::Scalar(0, 0, 255), -1); // Red
        }

        renderGrid(mapImage, gridMap, minX, minY, maxX, maxY, imgWidth, imgHeight);
        //renderDebugGrid(mapImage, gridMap, minX, minY, maxX, maxY, imgWidth, imgHeight);
        cv::imshow("Real-Time Map", mapImage);
        cv::waitKey(1);
    }
}

void CustomSLAM::renderGrid(cv::Mat& mapImage, const KeyframeGrid& grid, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight) {
    /*for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            float worldX = (x * grid.cellSize) - grid.xOffset;
            float worldY = (y * grid.cellSize) - grid.yOffset;

            int value = grid.getValue(worldX, worldY);
            if (value > 0) {
                cv::Point imgPoint = worldToImage(worldX, worldY, minX, minY, maxX, maxY, imgWidth, imgHeight);

                if (imgPoint.x >= 0 && imgPoint.x < imgWidth && imgPoint.y >= 0 && imgPoint.y < imgHeight) {
                    auto color = cv::Scalar(255, 255, 255);
                    if (value == 20) { color = cv::Scalar(50, 120, 200); }
                    if (value == 60) { color = cv::Scalar(255, 255, 0); }

                    cv::rectangle(
                        mapImage,
                        cv::Rect(imgPoint.x - 2.5, imgPoint.y - 2.5, 5, 5),
                        color,
                        -1,
                        cv::LINE_AA
                    );
                }
            }
        }
    }*/
}








































/*
void CustomSLAM::renderGrid(cv::Mat& mapImage, const Grid& grid, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight) {
    // Loop through each cell in the grid
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            // Map grid indices back to real-world coordinates
            float worldX = (x * grid.cellSize) - grid.xOffset;
            float worldY = (y * grid.cellSize) - grid.yOffset;

            int value = grid.getValue(worldX, worldY); // Get the value of the cell
            if (value > 0) { // Render only cells with nonzero values
                int imgX = static_cast<int>((worldX - minX) / (maxX - minX) * imgWidth);
                int imgY = imgHeight - static_cast<int>((worldY - minY) / (maxY - minY) * imgHeight);

                if (imgX >= 0 && imgX < imgWidth && imgY >= 0 && imgY < imgHeight) {
                    // Draw a semi-transparent green rectangle
                    auto color = cv::Scalar(255, 255, 255);
                    if (value == 20) { color = cv::Scalar(50,120,200);}
                    if (value == 60) { color = cv::Scalar(255,255,0);}
                    cv::rectangle(
                        mapImage,
                        cv::Rect(imgX - 5, imgY - 5, 10, 10), // Cell size of 10x10 pixels
                        color, // Green color
                        -1, // Filled rectangle
                        cv::LINE_AA
                    );
                }
            }
        }
    }
}
*/
void CustomSLAM::overlayCoordinates(cv::Mat& mapImage, const KeyframeGrid& grid, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight) {
    // Loop through grid cells
    /*for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            // Map grid indices back to real-world coordinates
            float worldX = (x * grid.cellSize) - grid.xOffset;
            float worldY = (y * grid.cellSize) - grid.yOffset;

            // Map real-world coordinates to image pixel coordinates
            int imgX = static_cast<int>((worldX - minX) / (maxX - minX) * imgWidth);
            int imgY = imgHeight - static_cast<int>((worldY - minY) / (maxY - minY) * imgHeight);

            // Ensure the points are within the image boundaries
            if (imgX >= 0 && imgX < imgWidth && imgY >= 0 && imgY < imgHeight) {
                // Create the text for the grid coordinate
                std::string text = "(" + std::to_string(static_cast<int>(worldX)) + "," + std::to_string(static_cast<int>(worldY)) + ")";

                // Overlay coordinates as text
                cv::putText(
                    mapImage,
                    text,
                    cv::Point(imgX + 2, imgY - 2), // Offset for better visibility
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.3, // Font scale
                    cv::Scalar(255, 255, 255), // White color
                    1,
                    cv::LINE_AA
                );
            }
        }
    }*/
}

void CustomSLAM::addVertex(float gx, float gy, float gtheta) {
    vertexList.push_back({gx, gy, gtheta});
}
void CustomSLAM::addEdge(float dx, float dy, float dtheta) {
    edgeList.push_back({dx, dy, dtheta});
}


// Work more on this if it doesnt work
/*void CustomSLAM::exportPoseGraph() {
    for (auto v : vertexList) {
        auto x = v[0];
        auto y = v[1];
        auto theta = v[2];
    }
    for (auto e : edgeList) {
        auto dx = e[0];
        auto dy = e[1];
        auto dtheta = e[2];
    }
    for (auto lc : loopClosureList) {
        auto index1 = lc.first;
        auto index2 = lc.second;

    }
}*/
#include <fstream>
#include <iomanip>

void CustomSLAM::exportPoseGraph(const std::string& filename) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write vertices
    for (size_t i = 0; i < vertexList.size(); ++i) {
        const auto& v = vertexList[i];
        outFile << "VERTEX_SE2 " << i << " "
                << std::fixed << std::setprecision(6)
                << v[0] << " " << v[1] << " " << v[2] << "\n";
    }

    // Write edges
    for (size_t i = 0; i < edgeList.size(); ++i) {
        const auto& e = edgeList[i];
        int id1 = i;         // Replace with the actual vertex ID
        int id2 = i + 1;     // Replace with the actual connected vertex ID
        outFile << "EDGE_SE2 " << id1 << " " << id2 << " "
                << std::fixed << std::setprecision(6)
                << e[0] << " " << e[1] << " " << e[2] << " "
                << "1 0 0 1 0 1" << "\n"; // Simplified information matrix
    }

    // Write loop closures
    for (const auto& lc : loopClosureList) {
        int index1 = lc.first;
        int index2 = lc.second;
        outFile << "EDGE_SE2 " << index1 << " " << index2 << " "
                << "0 0 0 " // Adjust to include proper transformations if available
                << "1 0 0 1 0 1" << "\n"; // Simplified information matrix
    }

    outFile.close();
    std::cout << "Pose graph successfully exported to " << filename << std::endl;
}

void CustomSLAM::exportEverything(const std::string &filename) {


}


