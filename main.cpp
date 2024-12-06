#include <iostream>
#include <threepp/threepp.hpp>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>
#include <cmath>

#include "include/Sphero.h"
#include "include/SimulatorUtilities.h"
#include "include/KeyHandler.h"
#include "include/Communication.h"
//#include "include/SlamAlgo.h"
#include "include/Slam.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <opencv2/opencv.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>


#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

/*
// Type definitions
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

const int img_size = 1200;
const int center = img_size / 2;
const float scale = 100.0f; // Scale for distances to pixels
cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC3);

// Global last frame and world map
PointCloudT::Ptr lastFrame = nullptr;
PointCloudT::Ptr worldMap = std::make_shared<PointCloudT>();

// Function to convert a vector of (x, y) points to a PCL point cloud
std::shared_ptr<PointCloudT> convertToPCL(const std::vector<std::pair<float, float>>& points) {
    auto cloud = std::make_shared<PointCloudT>();
    for (const auto& point : points) {
        cloud->points.emplace_back(point.first, 0, point.second);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

// Visualize the world map and the current frame in OpenCV
void visualizeWorldMap(const PointCloudT::Ptr& worldMap, const PointCloudT::Ptr& currentFrame) {
    img = cv::Mat::zeros(img_size, img_size, CV_8UC3);

    // Color for the world map (cyan)
    cv::Scalar worldColor = cv::Scalar(255, 255, 0);

    // Color for the current frame (red)
    cv::Scalar frameColor = cv::Scalar(0, 0, 255);

    // Draw world map points
    for (const auto& point : worldMap->points) {
        float x_img = center + point.x * scale;
        float y_img = center - point.z * scale;

        // Draw the point
        cv::circle(img, cv::Point(static_cast<int>(x_img), static_cast<int>(y_img)), 2, worldColor, -1);
    }

    // Draw current frame points
    for (const auto& point : currentFrame->points) {
        float x_img = center + point.x * scale;
        float y_img = center - point.z * scale;

        // Draw the point
        cv::circle(img, cv::Point(static_cast<int>(x_img), static_cast<int>(y_img)), 2, frameColor, -1);
    }

    // Display the image
    cv::imshow("World Map Visualization", img);
    cv::waitKey(1); // Allow OpenCV to process GUI events
}

// Main function to process lidar scans
void processLidarScans(const std::vector<LidarScan>& scans) {
    // Convert lidar scans to a point cloud
    std::vector<std::pair<float, float>> points;
    for (const auto& scan : scans) {
        float x = scan.range * std::cos(scan.angle);
        float y = scan.range * std::sin(scan.angle);
        points.emplace_back(x, y);
    }
    auto newCloud = convertToPCL(points);

    // Perform GICP alignment with the world map
    if (!worldMap->points.empty()) {
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setMaximumIterations(200);
        gicp.setInputSource(newCloud);
        gicp.setInputTarget(worldMap);

        PointCloudT alignedCloud;
        gicp.align(alignedCloud);

        if (gicp.hasConverged()) {
            std::cout << "GICP converged. Fitness score: " << gicp.getFitnessScore() << std::endl;

            // Replace the new cloud with the aligned cloud
            newCloud = std::make_shared<PointCloudT>(alignedCloud);
            *worldMap += *newCloud;

        } else {
            std::cerr << "GICP did not converge." << std::endl;
        }
    } else {
        *worldMap += *newCloud;
    }



    // Visualize the world map and the current frame
    visualizeWorldMap(worldMap, newCloud);
}
std::queue<std::vector<LidarScan>> slamQueue;
std::mutex slamMutex;
std::condition_variable slamCondition;
std::atomic<bool> running(true);

// SLAM Thread Function
void slamThreadFunction() {
    while (running) {
        std::vector<LidarScan> frame;

        // Wait for a new frame
        {
            std::unique_lock<std::mutex> lock(slamMutex);
            slamCondition.wait(lock, [] { return !slamQueue.empty() || !running; });

            if (!running && slamQueue.empty()) break;

            frame = std::move(slamQueue.front());
            slamQueue.pop();
        }

        // Convert the new frame to a point cloud
        std::vector<std::pair<float, float>> points;
        for (const auto& scan : frame) {
            float x = scan.range * std::cos(scan.angle);
            float y = scan.range * std::sin(scan.angle);
            points.emplace_back(x, y);
        }
        auto newCloud = convertToPCL(points);

        if (lastFrame) {
            // Perform GICP alignment with the last frame
            pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
            gicp.setMaximumIterations(200);
            gicp.setInputSource(newCloud);
            gicp.setInputTarget(lastFrame);

            PointCloudT alignedCloud;
            gicp.align(alignedCloud);

            if (gicp.hasConverged()) {
                std::cout << "GICP converged. Fitness score: " << gicp.getFitnessScore() << std::endl;

                // Replace the new cloud with the aligned cloud
                newCloud = std::make_shared<PointCloudT>(alignedCloud);
                *worldMap += *newCloud;

                // Update last frame
                lastFrame = newCloud;
            } else {
                std::cerr << "GICP did not converge." << std::endl;
            }
        } else {
            // Initialize the last frame with the first frame
            *worldMap += *newCloud;
            lastFrame = newCloud;
        }

        // Visualize the world map and the current frame
        visualizeWorldMap(worldMap, newCloud);
    }

    std::cout << "SLAM thread stopped." << std::endl;
}
*/
/*
const int img_size = 1200;
const int center = img_size / 2;
const float scale = 100.0f; // Scale for distances to pixels
cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC3);


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> convertToPCL(std::vector<std::pair<float, float> > points) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto point: points) {
        cloud->points.push_back({point.first, 0, point.second});
    }
    return cloud;
}

void visualizeLidar(const std::vector<LidarScan>& scans, const std::vector<float> odometry) {
    // Clear the previous visualization
    img = cv::Mat::zeros(img_size, img_size, CV_8UC3);

    std::vector<std::pair<float, float>> points;
    for (const auto& scan : scans) { // Convert the points to the actual position and add the to a list "points". This will be the point cloud well work with.
        float x_converted = scan.range * std::cos(scan.angle);
        float y_converted = scan.range * std::sin(scan.angle);
        points.push_back({x_converted, y_converted});
    }
    auto cloud = convertToPCL(points); // Converts it to a PCL dataformat. Easier to handle








    for (const auto& scan : scans) {
        // Convert angle and range to x and y coordinates in the robot's local frame
        float x_local = scan.range * std::cos(scan.angle);
        float y_local = scan.range * std::sin(scan.angle);

        // Map these coordinates to the image
        float x_img = center  + x_local * scale ;
        float y_img = center  + y_local * scale;  // Invert y-axis for image coordinates

        // Draw the point in green
        cv::circle(img, cv::Point(static_cast<int>(x_img), static_cast<int>(y_img)), 2, cv::Scalar(0, 255, 0), -1);
    }

    // Draw the robot's position at the center
    cv::circle(img, cv::Point(center, center), 5, cv::Scalar(255, 0, 0), -1);
    cv::arrowedLine(img, cv::Point(center, center), cv::Point(center, center +scans[0].end.y), cv::Scalar(0, 0, 255, 0), 5);

    // Display the image
    cv::imshow("LIDAR Visualization", img);

    cv::waitKey(1); // Allow OpenCV to process GUI events and update the display
}
*/



#include <string>
#include <utility>






using namespace threepp;


std::shared_ptr<Mesh> createBox2(const Vector3& pos, const Color& color) {
    auto geometry = BoxGeometry::create(0.90, 0.1, 0.90);
    auto material = MeshPhongMaterial::create();
    material->color = color;

    auto box = Mesh::create(geometry, material);
    box->position.copy(pos);

    return box;
}
#include <fstream> // For saving data
#include <iomanip> // For saving data
int main() {
    // Start the SLAM thread
    //std::thread slamThread(slamThreadFunction);
    // SLAM MAP CREATION


    //CustomSLAM customSLAM;



    // Rest of the scene
    Clock clock;
    Canvas canvas{ "Sphero Simulator", {{"aa", 8}} };
    GLRenderer renderer(canvas.size());




    auto scene = Scene::create();
    auto camera = PerspectiveCamera::create(75, canvas.aspect(), 0.1f, 100);
    camera->position.set(0, 3, 5);
    camera->lookAt(0, 0, 0);
    OrbitControls controls(*camera, canvas);

    // Add lights
    auto ambientLight = AmbientLight::create(Color::white, 0.5f);
    scene->add(ambientLight);
    auto directionalLight = DirectionalLight::create(Color::white, 0.5f);
    directionalLight->position.set(0, 10, 10);
    scene->add(directionalLight);

    // Add boxes for testing lidar detection
    auto cube1 = createBox({ 2, 0, 3 }, Color::blue);
    auto cube2 = createBox({ 5, 0, -5 }, Color::red);
    auto cube3 = createBox({ -3, 0, 0 }, Color::green);
    auto cube4 = createBox({ 0.3, 0, 1 }, Color::blue);
    auto cube5 = createBox({ 1, 0, 5 }, Color::red);
    auto cube6 = createBox({ 4.4, 0, 4.2 }, Color::green);

    scene->add(cube1);
    scene->add(cube2);
    scene->add(cube3);
    scene->add(cube4);
    scene->add(cube5);
    scene->add(cube6);




    // Load additional objects
    OBJLoader loader;
    TextureLoader tl;

    auto tex = tl.load("Assets/Room_texture.png");
    auto obj2 = loader.load("Assets/The Test Room.obj", true);

    obj2->traverseType<Mesh>([tex](Mesh& child) {
        auto m = MeshPhongMaterial::create();
        m->map = tex;
        //m->side = threepp::Side::Double;  // Enable backface culling
        child.setMaterial(m);
    });
    obj2->position.set(0, 0, 0);
    scene->add(obj2);

    // Create Sphero object
    Sphero sphero(scene, "127.0.0.1", 65432);  // Provide the host and port
    sphero.position.y = 0.2;
    scene->add(sphero);
    sphero.setLidarSpeed(600.0);
    sphero.enableSweep(true);
    // Set up objects to scan
    std::vector<Object3D*> objectsToScan = { cube1.get(), cube2.get(), cube3.get(), cube4.get(), cube5.get(), cube6.get() };
    obj2->traverseType<Mesh>([&objectsToScan](Mesh& child) {
        child.geometry()->computeBoundingBox();
        objectsToScan.push_back(&child);
    });
    sphero.setScanObjects(objectsToScan);

    // Attach POV camera to Sphero
    auto povCamera = PerspectiveCamera::create(75, canvas.aspect(), 0.1f, 100);
    povCamera->position.set(0.2, 0.5, 0.0);
    povCamera->lookAt(10.0,0.5,0.0);
    sphero.add(povCamera);

    // Key controls setup
    KeyController keyController(sphero);
    canvas.addKeyListener(keyController);

    // Handle window resize
    canvas.onWindowResize([&](WindowSize size) {
        camera->aspect = size.aspect();
        camera->updateProjectionMatrix();
        renderer.setSize(size);
    });

    // Prepare buffer for OpenCV
    std::vector<unsigned char> pixels(3 * 800 * 600); // RGB

    // LOGGING OF CSV.
    //std::ofstream csvFile("lidar_odometry_data.csv");
    //csvFile << "timestamp_ms,position_x,position_y,position_z,orientation,scan_angle,scan_distance\n";

    //bool loggingEnabled = false;
    //auto startTime = std::chrono::steady_clock::now();
    //auto logInterval = std::chrono::milliseconds(100);
    //auto logEndTime = startTime + std::chrono::seconds(12);
    NewSlam new_slam(scene);

    float offlineCount = 0;

    float time = 0.0f;
    const float interval = 0.01f; //SLAMINTERVAL
    bool firstTime = true;
    // Render and display loop
    canvas.animate([&]() {
        float deltaTime = clock.getDelta();
        keyController.update(deltaTime);
        sphero.update(deltaTime);
        time += deltaTime;


        // Process visualization updates
        {
            std::lock_guard<std::mutex> lock(new_slam.myGridInstance.visualizationQueueMutex);
            while (!new_slam.myGridInstance.visualizationQueue.empty()) {
                new_slam.myGridInstance.visualizationQueue.front()(); // Execute the visualization update
                new_slam.myGridInstance.visualizationQueue.pop();
            }
        }
        if (time > interval) {

            //offlineCount += 0.1;
            //myGrid.newCell(offlineCount, offlineCount,2);
            //myGrid.changeColor(sphero.position.x, sphero.position.z);
            //auto data = combinedData[offlineCount];
            //if (!data.first.empty()) {
            //    new_slam.enqueueFrame(data);
            //}
            //offlineCount++;
            auto data = sphero.getSLAMframe();
            if (!data.first.empty()) {
                new_slam.enqueueFrame(data);
            }
            //if (!data.first.empty()) {
            //    customSLAM.enqueueFrame(data);
            //}

            time = 0.0f;
        }


        /*
        for (size_t i = 0; i < sphero.beltCubes_.size(); ++i) {
            float t = std::fmod(time * 50 + i * 10.0, sphero.beltPath_.size() - 1);
            int idx = static_cast<int>(t); // current path index

            // Interpolate between the current point and the next
            float alpha = t - idx;
            auto& currentPoint = sphero.beltPath_[idx];
            auto& nextPoint = sphero.beltPath_[idx + 1];
            sphero.beltCubes_[i]->position.lerpVectors(currentPoint, nextPoint, alpha);
        }*/



        if (sphero.pov) {
            povCamera->fov = 100;
            povCamera->updateProjectionMatrix(); // to fix pov change
            renderer.render(*scene, *povCamera);
        } else {
            renderer.render(*scene, *camera);
        }


        /* LOGGING OF DATA WITH CSV
        auto currentTime = std::chrono::steady_clock::now();
        if (!loggingEnabled && currentTime - startTime >= std::chrono::seconds(2)) {
            loggingEnabled = true;
            startTime = currentTime; // Reset start time for interval tracking
        }

        if (loggingEnabled && currentTime < logEndTime) {
            if ((currentTime - startTime) >= logInterval) {
                auto frame = sphero.getFullFrame();

                // Write each scan with corresponding odometry to the CSV file
                for (size_t i = 0; i < frame.first.size(); ++i) {
                    const auto& scan = frame.first[i];
                    const auto& odometry = frame.second[i];
                    csvFile << std::chrono::duration_cast<std::chrono::milliseconds>(scan.timestamp.time_since_epoch()).count() << ","
                            << odometry.position.x << "," << odometry.position.y << "," << odometry.position.z << ","
                            << odometry.orientation << ","
                            << scan.angle << "," << scan.distance << "\n";
                }

                // Update the last log time
                startTime = currentTime;
            }
        }

        // Stop logging after 10 seconds
        if (currentTime >= logEndTime) {
            csvFile.close();
            std::cout << "Data logging completed." << std::endl;
        }
        */
        // Convert to OpenCV Mat
        /*
        cv::Mat image(600, 800, CV_8UC3, pixels.data());

        // Flip vertically and convert color format
        cv::flip(image, image, 0);
        cv::imshow("Rendered Image", image);
        cv::waitKey(1);*/
    });


    return 0;
}
