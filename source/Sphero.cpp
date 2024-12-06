//
// Created by Codetonsen on 10/29/2024.
//
#include "../include/Sphero.h"
#include <threepp/threepp.hpp>
#include <iostream>
#include <g2o/stuff/misc.h>
#include <random>
std::mutex lidarMutex;

using namespace threepp;








Sphero::Sphero(const std::shared_ptr<Scene> &theScene, const std::string& host, int port)
    :
lidarScanCount(360),
lidarSleepTime_(0),
currentAngleIndex(0),
tiltAngle(-30.0f),
tiltDirection(1),
socket_(io_context_),
sweep(false),
lidarSpeed_(60.0)
{
    theScene_ = theScene;
    OBJLoader loader;
    TextureLoader tl;

    auto tex = tl.load("Assets/sphero_texture.png");
    auto obj2 = loader.load("Assets/sphero.obj", true);

    obj2->traverseType<Mesh>([tex](Mesh& child) {
        auto m = MeshPhongMaterial::create();
        m->map = tex;
        child.setMaterial(m);
    });
    obj2->position.set(0, -0.1, 0);
    obj2->rotateY(-threepp::math::PI/2);
    obj2->scale.set(0.3, 0.3, 0.27);
    this->add(obj2);


    // To add belts i took this and the fiddle-example for some inspiration. I first tried using textures but did not work, so i used cubes instead: https://discourse.threejs.org/t/how-to-animate-a-conveyor-belt-along-an-axis/31331/2
    auto material = MeshBasicMaterial::create();
    material->color = Color::gray; // should be some kind of dark material






    // Create left belt path and cubes
    beltPathLeft_ = createPathPoints(0.4, -0.25);
    for (int i = 0; i < 100; i++) {
        auto cubeLeft = createCubeForBelt(material);
        beltCubesLeft_.push_back(cubeLeft);


        this->add(cubeLeft);
    }

    // Create right belt path and cubes
    beltPathRight_ = createPathPoints(0.4, 0.25); // Right belt path, same scale
    for (int i = 0; i < 100; i++) {
        auto cubeRight = createCubeForBelt(material);
        beltCubesRight_.push_back(cubeRight);

        this->add(cubeRight);
    }


    precomputeAngles(lidarScanCount);
    setupLidarInstancedMesh(lidarScanCount);
    setupSocketConnection(host, port);
    std::cout << "Sphero created" << std::endl;


}



Sphero::~Sphero() {
    stopLidar();
    closeSocketConnection();
}

void Sphero::enableSweep(bool enable) {
    sweep = enable;
}

void Sphero::adjustTiltAngle() {
    if (sweep) {
        tiltAngle += tiltDirection * sweepSpeed_;
        if (tiltAngle >= 30.0f || tiltAngle <= -30.0f) {
            tiltDirection *= -1;
        }
    } else {
        tiltAngle = 0.0f;
    }
}

const std::vector<LidarScan>& Sphero::getLidarScans() const {
    return lidarScans;
}

void Sphero::startLidar() {
    if (!lidarRunning) {
        lidarRunning = true;
        lidarThread = std::thread(&Sphero::runLidar, this);
    }
}

void Sphero::setLidarSleepTime(int sleepTime) {
    lidarSleepTime_ = sleepTime;
}

void Sphero::setLidarSpeed(double RPM) {
    lidarSpeed_ = RPM;
}

void Sphero::setLidarResolution(double resolution) {
    lidarResolution_ = resolution;
}

void Sphero::setLidarSweepAngle(float sweepAngle) {
    sweepAngle_ = sweepAngle;
}

void Sphero::setLidarSweepSpeed(float sweepSpeed) {
    sweepSpeed_ = sweepSpeed;
}

void Sphero::setLidarSweepRadiusOffset(float sweepRadiusOffset) {
    sweepRadiusOffset_ = sweepRadiusOffset;
}

void Sphero::setVisualization(bool visualization) {
    visualization_ = visualization;
}

OdometryData Sphero::getOdometryData() const {
    return currentOdometry_;
}

std::pair<std::vector<LidarScan>, std::vector<OdometryData>> Sphero::getFullFrame() {
    std::lock_guard<std::mutex> lock(lidarMutex);

    // Only return if a full set of scans and odometry data is available
    if (lidarScans.size() >= lidarResolution_) {
        return {lidarScans, odometryDataList};
    } else {
        return {};
    }
}


float addNoise(float value, float noiseLevel = 0.01f) { // For simulating bad odometry
    static std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, noiseLevel);
    return value + distribution(generator);
}


std::pair<std::vector<std::pair<float, float>>, std::vector<float>> Sphero::getSLAMframe() {
    std::lock_guard<std::mutex> lock(lidarMutex);
    if (lidarScans.size() >= lidarResolution_) {
        std::vector<float> posChange = {
            addNoise(this->position.x, noiseAmplitude), // Small noise for x
            addNoise(this->position.z, noiseAmplitude), // Small noise for z
            addNoise(this->rotation.y, noiseAmplitude*rotationNoiseWeight) // Smaller noise for rotation
        };

        //std::cout << "Rotation is currently at: " << this->rotation.y << std::endl;
        std::vector<std::pair<float, float>> convertedPoints;
        for (const auto& scan : lidarScans) {

            float angleRadians = scan.angle;

            // Calculate x and y coordinates because of new coordinate system.
            float x = scan.range * cos(angleRadians);
            float y = scan.range * sin(angleRadians);

            // push the pair into the to be returned points
            convertedPoints.emplace_back(x, y);
        }

        return {convertedPoints, posChange}; // Returns the same data as from ROVER: x,y,heading + lidarscans(x,y)
    }
    return {};
};

void Sphero::stopLidar() {
    lidarRunning = false;
    if (lidarThread.joinable()) {
        lidarThread.join();
    }
}

void Sphero::runLidar() {
    const double targetInterval = (60 * 1000) / (lidarSpeed_ * lidarResolution_); // Target interval in milliseconds. Target interval is the speed of the rotation on the lidar, this is to simulate RPM.
    auto nextTick = std::chrono::steady_clock::now();

    while (lidarRunning) {
        auto currentTime = std::chrono::steady_clock::now();

        if (currentTime >= nextTick) {
            adjustTiltAngle();
            performSingleLidarScan(currentAngleIndex);
            currentAngleIndex = (currentAngleIndex + 1) % lidarScanCount;
            showDebugLines();

            // Update the next tick time
            nextTick += std::chrono::milliseconds(static_cast<int>(targetInterval));


            //std::cout << "Target Interval: " << targetInterval << " ms" << std::endl;
        } else {
            // Calculate how much time is left and then sleep that duration
            auto sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(nextTick - currentTime).count();

            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
        }
    }
}

void Sphero::showDebugLines() {

    lidarController_->updatePositionsFromLidar(lidarScans);
    //lidarController_->updateColorsByAge(lidarScans);
}


void Sphero::setScanObjects(const std::vector<Object3D*>& objectsToScan) {
    scanObjs = objectsToScan;
}

void Sphero::drive(std::vector<bool> driveData, float deltaTime) {
    float multiplier = 1;
    float rotationDelta = 0;
    currentSpeed = 0.0;


    // DRIVE HANDLING
    if (driveData[4]) {multiplier = 2.0f;}
    if (driveData[5]) {multiplier = 0.5f;}

    if (driveData[0]) {this->translateX(speed * multiplier * deltaTime);  currentSpeed = -speed * multiplier;}
    if (driveData[1]) {this->translateX(-speed * multiplier * deltaTime); currentSpeed = speed * multiplier;}
    if (driveData[2]) {this->rotation.y += g2o::deg2rad(100.0) * deltaTime; rotationDelta = g2o::deg2rad(100.0) * deltaTime;}
    if (driveData[3]) {this->rotation.y -= g2o::deg2rad(100.0) * deltaTime; rotationDelta = -g2o::deg2rad(100.0) * deltaTime;}




    // BELT SYSTEM BASED ON SPEED
    float leftSpeed;
    float rightSpeed;
    // Calculate belt speeds
    if (rotationDelta != 0) {
        leftSpeed = currentSpeed + rotationDelta * 50.0f;
        rightSpeed = currentSpeed - rotationDelta * 50.0f;
    } else {
        leftSpeed = currentSpeed - rotationDelta * 50.0f;
        rightSpeed = currentSpeed + rotationDelta * 50.0f;
    }

    //std::cout << "rotationdelta: " << rotationDelta << std::endl;
    //std::cout << "The leftSpeed: " << leftSpeed << " rightSpeed: " << rightSpeed << std::endl;

    // Update LEFT belt position
    if (!beltPathLeft_.empty()) {
        beltPositionOffsetLeft_ += leftSpeed * deltaTime * 100;
        beltPositionOffsetLeft_ = std::fmod(beltPositionOffsetLeft_, beltPathLeft_.size()); // Makes sure the offset wraps around.
        if (beltPositionOffsetLeft_ < 0) beltPositionOffsetLeft_ += beltPathLeft_.size();
    }

    // Update RIGHT belt position
    if (!beltPathRight_.empty()) {
        beltPositionOffsetRight_ += rightSpeed * deltaTime * 100;
        beltPositionOffsetRight_ = std::fmod(beltPositionOffsetRight_, beltPathRight_.size()); // Makes sure the offset wraps around.
        if (beltPositionOffsetRight_ < 0) beltPositionOffsetRight_ += beltPathRight_.size();
    }
    // Move left belt cubes
    for (size_t i = 0; i < beltCubesLeft_.size(); ++i) {
        if (beltPathLeft_.size() > 1) {
            float t = std::fmod(i * 10.0f + beltPositionOffsetLeft_, beltPathLeft_.size() - 1); // Calculate the interpolation position for current segment. This is after we are sure there are 2 or more points to interpolate between
            int idx = static_cast<int>(t);
            float alpha = t - idx;
            auto& currentPoint = beltPathLeft_[idx];
            auto& nextPoint = beltPathLeft_[(idx + 1) % beltPathLeft_.size()];
            beltCubesLeft_[i]->position.lerpVectors(currentPoint, nextPoint, alpha);
        }
    }

    // Move right belt cubes
    for (size_t i = 0; i < beltCubesRight_.size(); ++i) {
        if (beltPathRight_.size() > 1) {
            float t = std::fmod(i * 10.0f + beltPositionOffsetRight_, beltPathRight_.size() - 1); // Calculate the interpolation position for current segment. This is after we are sure there are 2 or more points to interpolate between
            int idx = static_cast<int>(t);
            float alpha = t - idx;
            auto& currentPoint = beltPathRight_[idx];
            auto& nextPoint = beltPathRight_[(idx + 1) % beltPathRight_.size()];
            beltCubesRight_[i]->position.lerpVectors(currentPoint, nextPoint, alpha);
        }
    }

    rotationDelta = 0;



}

void Sphero::setupLidarInstancedMesh(int maxLines) {
    auto lineGeometry = BoxGeometry::create(0.02f, 0.02f, 1.0f);
    auto lineMaterial = MeshBasicMaterial::create();
    lineMaterial->color = Color(0x00ffff);

    auto lidarInstancedMesh = InstancedMesh::create(lineGeometry, lineMaterial, maxLines);
    lidarInstancedMesh->instanceMatrix()->setUsage(DrawUsage::Dynamic);

    theScene_->add(lidarInstancedMesh);
    lidarController_ = std::make_shared<InstancedMeshController>(maxLines, lidarInstancedMesh);
}

void Sphero::precomputeAngles(int numRays) { // legit made only for performance. but i dont think it helps as much as i wouldve liked it to. Basically precalculates the direction the rays are supposed to face.
    precomputedAngles.resize(numRays);
    float angleIncrement = 360.0f / numRays;
    for (int i = 0; i < numRays; ++i) {
        float angle = i * angleIncrement;
        float radians = threepp::math::degToRad(angle);
        precomputedAngles[i].set(cos(radians), sin(radians));
    }
}

void Sphero::performSingleLidarScan(int angleIndex) {
    float angleIncrement = 360.0f / lidarResolution_;
    float angleDegrees = angleIndex * angleIncrement;
    float angleRadians = math::degToRad(angleDegrees);


    Vector3 localDirection(std::cos(angleRadians), 0, std::sin(angleRadians));    //  angleIndex to find the direction of the vector


    Vector3 worldDirection = localDirection.clone().applyQuaternion(this->quaternion);  // Transform the local direction to world coordinates


    //Vector3 worldPosition = {this->position.x, this->position.y + 0.3f, this->position.z};                                           // Starting point in world coordinates at the sphero position + some height to match the lidar sensor visual
    Vector3 worldPosition = this->position;
    // Perform raycasting in world coordinates
    Raycaster raycaster(worldPosition, worldDirection);
    raycaster.far = 10;  // maximum raycast intersept


    auto intersections = raycaster.intersectObjects(scanObjs); // These were defined in main. The scene cubes as well as the room model
    //threepp::Vector3 start = {this->position.x, this->position.y + 0.3f, this->position.z};
    Vector3 start = this->position;
    Vector3 end;
    float range = 10;
    if (!intersections.empty()) {
        range = intersections[0].distance;
        end = intersections[0].point;

    }


    LidarScan scan;
    scan.angle = angleRadians;  // Angle in radians relative to the robot
    scan.range = range;
    scan.start = start;
    scan.end = end;
    {
        std::lock_guard<std::mutex> lock(lidarMutex);
        if (lidarScans.size() < lidarScanCount) {
            lidarScans.push_back(scan);
        } else {
            lidarScans[angleIndex] = scan;
        }
    }
}

float Sphero::shortest_signed_angle_path(float angle1, float angle2) { // https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles, had to use fmod instead of modulo operator (takes only int)
    float diff = std::fmod(angle2 - angle1 + 180.0f, 360.0f) - 180.0f;
    return (diff < -180.0f) ? (diff + 360.0f) : diff;
}

void Sphero::setupSocketConnection(const std::string& host, int port) {
    std::thread([this, host, port]() {
        while (true) {
            try {
                tcp::acceptor acceptor(io_context_, tcp::endpoint(tcp::v4(), port));
                std::cout << "Server listening on " << host << ":" << port << std::endl;

                while (true) {
                    std::cout << "Waiting for new client connection..." << std::endl;
                    acceptor.accept(socket_);
                    std::cout << "Client connected." << std::endl;
                }
            } catch (std::exception& e) {
                std::cerr << "Server connection error: " << e.what() << std::endl;
            }
        }
    }).detach();
}

void Sphero::closeSocketConnection() {
    if (socket_.is_open()) {
        asio::error_code ec;
        socket_.shutdown(tcp::socket::shutdown_both, ec);
        socket_.close(ec);
    }
}

void Sphero::sendLidarData(const LidarScan& scan, float currentTiltAngle) {
    if (socket_.is_open()) {
        try {
            json scanData = {
                {"angle_deg", scan.angle},
                {"distance_m", scan.range},
                {"tilt_angle_deg", currentTiltAngle}
            };

            std::string message = scanData.dump() + "\n";
            asio::write(socket_, asio::buffer(message));
        } catch (asio::system_error& e) {
            std::cerr << "Failed to send data to client: " << e.what() << std::endl;
            socket_.close();
        }
    }
}
