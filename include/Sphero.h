//
// Created by Codetonsen on 10/29/2024.
//

#ifndef SPHERO_H
#define SPHERO_H

#include "SimulatorUtilities.h"
#include <threepp/threepp.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

#include <nlohmann/json.hpp>
#include <asio.hpp>


class InstancedMeshController;
struct LidarScan;
struct OdometryData {
    threepp::Vector3 position;
    float orientation; // Heading in radians or degrees
    float velocity;
    std::chrono::steady_clock::time_point timestamp;
};
using json = nlohmann::json;
using asio::ip::tcp;
#undef far //This is stupid because of asio, raycast far didnt work until i undefined it.



class Sphero : public threepp::Object3D {
public:
    Sphero(std::shared_ptr<threepp::Scene> theScene, const std::string& host, int port);
    ~Sphero();

    void enableSweep(bool enable);
    void adjustTiltAngle();
    const std::vector<LidarScan>& getLidarScans() const;
    void startLidar();
    void stopLidar();
    void runLidar();
    void showDebugLines();
    void setScanObjects(const std::vector<threepp::Object3D*>& objectsToScan);



    void update(float deltaTime);

    void drive(int payload, float deltaTime); // Use this for simple driving FORWARD, LEFT, RIGHT, BACK

    void drive_with_heading(float wantedHeading, float wantedSpeed, float deltaTime);


    void setLidarSleepTime(int sleepTime);                                  // Lidar runs on seperate thread, this sets sleep time between each individual scan in ms. Setting this to 1 gives max : 500-1000 scans a second so keep at 0
    void setLidarSpeed(double RPM);                                            // Sets lidar rotation speed in RPM
    void setLidarResolution(double resolution);                                // Resolution means the count of scans each 360 degree or one rotation
    void setLidarSweepAngle(float sweepAngle);                              // Sets the tilts to +- sweepangle. sweepAngle = 30 -> tilts +-30 degrees
    void setLidarSweepSpeed(float sweepSpeed);                              // The time it uses to do a full sweep in seconds.
    void setLidarSweepRadiusOffset(float sweepRadiusOffset);                // Sets the sweep offset, its not rotating around the center in the physical hardware, so its added here as well
    void setVisualization(bool visualization);                              // Turns on or off lidar scan lines. Default true (ON)

    std::vector<LidarScan> getScanFrame();                                  // Returns latest frame of lidarscans. This is a vector of size lidarResolution.
    OdometryData getOdometryData() const;                                   // Returns latest frame of odometryData. This is a vector of size lidarResolution.
    std::pair<std::vector<LidarScan>, std::vector<OdometryData>> getFullFrame(); // Returns a pair of two equally sized vectors of both lidarscans and odometrydata.
    std::pair<std::vector<std::pair<float, float>>, std::vector<float>> getSLAMframe();


    bool pov = false;
private:

    float translationDelta_;
    float rotationDelta_;
    float currentSpeed;
    int lidarSleepTime_;
    int lidarScanCount;
    int currentAngleIndex;
    bool lidarRunning = true;
    bool sweep;
    float tiltAngle;
    int tiltDirection;
    double dt;
    float time = 0.0; //For belt
    float beltPositionOffsetLeft_ = 0.0f; // Persistent offset along the belt path
    std::vector<std::shared_ptr<threepp::Mesh>> beltCubesRight_; // Second belt cubes
    float beltPositionOffsetRight_ = 0.0f;              // Offset for the second belt
    std::vector<std::shared_ptr<threepp::Mesh>> beltCubesLeft_;
    std::vector<threepp::Vector3> beltPathLeft_;
    std::vector<threepp::Vector3> beltPathRight_;


    float speed = 2;
    // RPM
    double lidarSpeed_ = 60.0;
    double lidarResolution_ = 120.0;
    float sleepToReachRPM_;

    OdometryData currentOdometry_;
    std::vector<OdometryData> odometryDataList;
    float sweepAngle_ = 30.0f;
    float sweepSpeed_ = 0.001f;
    float sweepRadiusOffset_ = 0.7f;

    bool visualization_;

    std::thread lidarThread;
    std::vector<LidarScan> lidarScans;
    std::vector<threepp::Object3D*> scanObjs;
    std::shared_ptr<threepp::Scene> theScene_;
    std::shared_ptr<InstancedMeshController> lidarController_;
    std::vector<threepp::Vector2> precomputedAngles;

    asio::io_context io_context_;
    tcp::socket socket_;

    void setupLidarInstancedMesh(int maxLines);
    void precomputeAngles(int numRays);
    void performSingleLidarScan(int angleIndex);
    float shortest_signed_angle_path(float angle1, float angle2);
    void setupSocketConnection(const std::string& host, int port);
    void closeSocketConnection();
    void sendLidarData(const LidarScan& scan, float currentTiltAngle);
};



#endif //SPHERO_H
