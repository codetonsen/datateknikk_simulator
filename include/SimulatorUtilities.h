//
// Created by Codetonsen on 10/29/2024.
//

#ifndef SIMULATORUTILITIES_H
#define SIMULATORUTILITIES_H

#include <threepp/threepp.hpp>
#include <chrono>
#include <memory>
#include <vector>


// Struct to store lidar scan data with additional information
/*struct LidarScan {
    threepp::Vector3 start;
    threepp::Vector3 end;
    float age;  // How old this line is, in seconds
    float angle;  // Angle of the scan in degrees
    float distance;  // Distance measured
    std::chrono::steady_clock::time_point timestamp;  // Timestamp of the scan
};*/
struct LidarScan {
    float angle;  // Angle relative to the robot's forward direction
    float range;  // Distance measurement
    threepp::Vector3 start;
    threepp::Vector3 end;
    float x;
    float y;
    // Optionally include other data like intensity or timestamp
};

// Function to create a box mesh
std::shared_ptr<threepp::Mesh> createBox(const threepp::Vector3& pos, const threepp::Color& color);
std::shared_ptr<threepp::Mesh> createCubeForBelt(const std::shared_ptr<threepp::MeshBasicMaterial>& material);
std::vector<threepp::Vector3> createPathPoints(float scale, float offset);
// Class to manage an instanced mesh for lidar visualization
class InstancedMeshController {
public:
    InstancedMeshController(int instanceCount, std::shared_ptr<threepp::InstancedMesh> instancedMesh);
    void initializePositionsAndVelocities();
    void initializeInstanceColors();
    void updatePositionsFromLidar(const std::vector<LidarScan>& lidarScans);
    void updateColorsByAge(const std::vector<LidarScan>& lidarScans);

private:
    int instanceCount_;
    std::shared_ptr<threepp::InstancedMesh> instancedMesh_;

    static threepp::Color interpolateColor(const threepp::Color& color1, const threepp::Color& color2, float t);
};
#endif //SIMULATORUTILITIES_H
