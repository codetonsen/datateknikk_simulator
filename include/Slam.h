//
// Created by Codetonsen on 11/25/2024.
//

#ifndef SLAM_H
#define SLAM_H

#include <utility>
#include <vector>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <opencv2/core/mat.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <pcl/registration/gicp.h>

#include "threepp/threepp.hpp"
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
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Sphero.h"

#include <Eigen/Geometry>




struct GridData {
  int id;
  std::vector<int> lidarIndexes;
};

class TheGrid {
public:
  TheGrid(bool isSimulator, std::shared_ptr<threepp::Scene> scene) : isSimulator(isSimulator), scene(scene) {
    std::cout << "Initializing Grid" << std::endl;
    std::cout << "Initializing Grid with scaleFactor: " << scaleFactor << std::endl;

  }

  // ChatGPT queue and mutex
  std::queue<std::function<void()>> visualizationQueue;
  std::mutex visualizationQueueMutex;

  void newCell(float gx, float gy, int frameIndex) { // makes a new cell at global input x and y
    //std::cout << "newcell called" << std::endl;

    int posX = static_cast<int>(round(gx * scaleFactor));
    int posY = static_cast<int>(round(gy * scaleFactor));
    if (theGrid[posX][posY].lidarIndexes.size() == 0) {
      theGrid[posX][posY].id = cellCount;
      addCell(posX, posY); // have a lot of problems with this, so am trying multithreading
      cellCount++;
    }
    theGrid[posX][posY].lidarIndexes.push_back(frameIndex);





    // Debug for checking if multiple indexes can be added
    std::cout << "LidarIndexes : [";
    for (auto index : theGrid[posX][posY].lidarIndexes) {
      std::cout << index;
    }
    std::cout << " ]" << std::endl;

  }

  GridData getFrameIndexes(float gx, float gy) {
    int posX = round(gx * scaleFactor); // gx0.15 -> 0.15 * 10 -> index = 2
    int posY = round(gy * scaleFactor);
    //std::cout << "posX: " << posX << " posY: " << posY << std::endl;
    return theGrid[posX][posY];
  }

  void addCell(int posX, int posY) {
    //std::cout << "Adding cellMesh to visualization" << std::endl;


    auto box = createVisuBox({static_cast<float>(posX / scaleFactor), 0.05, static_cast<float>(posY / scaleFactor)}, threepp::Color::pink);
    { // chatgpt threading stuff
      std::lock_guard<std::mutex> lock(visualizationQueueMutex);
      visualizationQueue.push([this, box]() {

          scene->add(box); // Safely add to the scene in the main thread

      });
    }//scene->add(box);
    visualizationMeshes.emplace_back(box);

  }
  void changeColor(float gx, float gy) {
    //std::cout << "Currently the size of visualizationMeshes: " << visualizationMeshes.size() << std::endl;
    //std::cout << "changing color: ";
    int posX = round(gx * scaleFactor);
    int posY = round(gy * scaleFactor);
    int index = theGrid[posX][posY].id;
    if (index < visualizationMeshes.size()) {
      auto greenMaterial = threepp::MeshBasicMaterial::create();
      greenMaterial->color = threepp::Color::green;
      visualizationMeshes[index]->setMaterial(greenMaterial);
    } else {
      std::cout << "Invalid mesh index: " << index << std::endl;
    }
  }

  std::shared_ptr<threepp::Mesh> createVisuBox(const threepp::Vector3& pos, const threepp::Color& color) {
    auto geometry = threepp::BoxGeometry::create(0.90 / scaleFactor, 0.1, 0.90 / scaleFactor);
    auto material = threepp::MeshPhongMaterial::create();
    material->color = color;
    auto box = threepp::Mesh::create(geometry, material);
    box->position.copy(pos);
    return box;
  }
  // Helpers
  void changeID(float gx, float gy) {}



  std::unordered_map<int, std::unordered_map<int, GridData>> theGrid;
  float scaleFactor = 5;
  std::vector<std::shared_ptr<threepp::Mesh>> visualizationMeshes;
  int cellCount = 0;
private:
  bool isSimulator = false;




  std::shared_ptr<threepp::Scene> scene;

};

class NewSlam {
public:
  NewSlam(std::shared_ptr<threepp::Scene> scene): myGridInstance(true, scene), scene(scene)
  {
    slamThread = std::thread(&NewSlam::processFrames, this);
    initializePoseGraph();

  };
  ~NewSlam() {
    stopSLAMThread();
    exportToG2O("myTest.g2o", optimizer.get());
  };





  void enqueueFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame) {
    {
      std::lock_guard<std::mutex> lock(queueMutex);
      frameQueue.push(dataFrame);
    }
    frameAvailable.notify_one();
  };
  void processFrames() {
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
  };
  void stopSLAMThread() {
    {
      std::lock_guard<std::mutex> lock(queueMutex);
      stopThread = true;
    }
    frameAvailable.notify_all();
    if (slamThread.joinable()) {
      slamThread.join();
    }
  };

  // ChatGPT Parser for g2o format
  void exportToG2O(const std::string& filename, g2o::SparseOptimizer* optimizer) {
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to open " << filename << " for writing." << std::endl;
      return;
    }

    // Write vertices
    for (const auto& it : optimizer->vertices()) {
      g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(it.second);
      if (v) {
        int id = v->id();
        const g2o::SE2& estimate = v->estimate();
        file << "VERTEX_SE2 " << id << " "
             << estimate.translation().x() << " "
             << estimate.translation().y() << " "
             << estimate.rotation().angle() << "\n";
      }
    }

    // Write edges
    for (const auto& it : optimizer->edges()) {
      g2o::EdgeSE2* e = dynamic_cast<g2o::EdgeSE2*>(it);
      if (e) {
        int from_id = e->vertices()[0]->id();
        int to_id = e->vertices()[1]->id();
        const g2o::SE2& measurement = e->measurement();
        const Eigen::Matrix3d& information = e->information();

        file << "EDGE_SE2 " << from_id << " " << to_id << " "
             << measurement.translation().x() << " "
             << measurement.translation().y() << " "
             << measurement.rotation().angle() << " "
             << information(0, 0) << " " << information(0, 1) << " " << information(0, 2) << " "
             << information(1, 1) << " " << information(1, 2) << " "
             << information(2, 2) << "\n";
      }
    }

    file.close();
    std::cout << "Pose graph exported to " << filename << std::endl;
  }

  void addVertex(int id, std::vector<float>& pose) {
    g2o::VertexSE2* v = new g2o::VertexSE2();
    v->setId(id);
    v->setEstimate(g2o::SE2(pose[0], pose[1], pose[2]));
    if (id == 0) {
      v->setFixed(true); // THIS BECOMES THE ANCHOR POINT
    }
    if (!optimizer->addVertex(v)) {
      std::cerr << "Failed to add vertex ID: " << id << " to optimizer." << std::endl;
    }

  };
  void addEdge(int fromID, int toID, const Eigen::Isometry2d& relativePose, const Eigen::Matrix3d informationMatrix) {
    if (!optimizer->vertex(fromID) || !optimizer->vertex(toID)) { // Check from chatGPT
      std::cerr << "One of the vertices (from: " << fromID << ", to: " << toID << ") does not exist in the optimizer." << std::endl;
      return;
    }

    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = optimizer->vertex(fromID);
    edge->vertices()[1] = optimizer->vertex(toID);
    edge->setMeasurement(g2o::SE2(relativePose.translation().x(),
      relativePose.translation().y(),
      atan2(relativePose.rotation().matrix()(1,0),
        relativePose.rotation().matrix()(0,0))));
    edge->setInformation(informationMatrix);
    if (!optimizer->addEdge(edge)) {
      std::cerr << "Failed to add edge from " << fromID << " to " << toID << std::endl;
    }
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr toPCLformat(std::vector<std::pair<float, float>> data) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto& point : data) {
      pcl::PointXYZ pclPoint;
      pclPoint.x = point.first;
      pclPoint.y = point.second;
      pclPoint.z = 0;
      cloud->emplace_back(pclPoint);
    }
    return cloud;
  }
std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> parseDataset(const std::string& filename, float offsetX = 0, float offsetY = 0, float offsetHeading = 0) {
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




  void runOptimization() {
    std::cout << "Running pose graph optimization" << std::endl;
    optimizer->initializeOptimization();
    optimizer->optimize(10); // Number of optimization iterations

    // Update poseFrames with optimized poses
    for (size_t id = 0; id < poseFrames.size(); ++id) {
      auto v = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(id));
      if (v) {
        g2o::SE2 optimized_pose = v->estimate();
        poseFrames[id][0] = optimized_pose.translation().x();
        poseFrames[id][1] = optimized_pose.translation().y();
        poseFrames[id][2] = optimized_pose.rotation().angle();
      }
    }

    std::cout << "Optimization Complete." << std::endl;

    // Export the pose graph. Hvent gotten to this yet.

  }


  bool doTheMatching(std::vector<std::pair<float, float>> sourceData, std::vector<std::pair<float, float>> targetData, Eigen::Matrix4f& transformation, double& score) {
    auto source = toPCLformat(sourceData);
    auto target = toPCLformat(targetData);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);


    // change params lateeer

    // run the alignment or the gicp
    pcl::PointCloud<pcl::PointXYZ> alignedPointCloud;
    gicp.align(alignedPointCloud);

    if (gicp.hasConverged()) {
      transformation = gicp.getFinalTransformation();
      score = gicp.getFitnessScore();
      return true;
    } else {
      return false;
    }
  }

  void initializePoseGraph() {
    optimizer = std::make_unique<g2o::SparseOptimizer>();
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto linearSolver = g2o::make_unique<LinearSolverType>();
    auto solver_ptr = g2o::make_unique<g2o::OptimizationAlgorithmLevenberg>(g2o::make_unique<BlockSolverType>(std::move(linearSolver)));
    optimizer->setAlgorithm(solver_ptr.release());
    optimizer->setVerbose(false);
  }

  void addFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame) {
    // Check for missing data
    if (dataFrame.first.size() == 0 or dataFrame.second.size() == 0) {return;};

    // Make data available and seperated
    std::vector<std::pair<float, float>> pointcloud = dataFrame.first;
    float x = dataFrame.second[0];
    float y = dataFrame.second[1];
    float heading = dataFrame.second[2];
    //std::cout << heading << " is rotation" << std::endl;

    if (lidarFrames.size() == 0) { // ADDS THE LIDAR AND POSE DATA AND VERTEX BUT NO EDGE
      std::cout << "There are no lidarframes yet, adding frame then exiting" <<std::endl;
      lidarFrames.emplace_back(pointcloud);
      poseFrames.emplace_back(dataFrame.second);
      //addVertex(dataFrame.second);
      //myGridInstance.newCell(x,y,lidarFrames.size());
      std::cout << "Lidar frame added" << std::endl;
      return;
    }

    // Calculate DeltaPosition ---------------------------------------------------------- 1
    auto lastPose = poseFrames[poseFrames.size() - 1];
    float x2 = lastPose[0];
    float y2 = lastPose[1];
    float h2 = lastPose[2];
    //auto lastPointCloud = lidarFrames[keyFrameCount - 1];
    float distanceBetweenPoses = sqrt(powf((x2 - x),2)+powf((y2 - y),2)); // sqrt((x2-x1)^2+(y2-y1)^2) (euclidian distance) lol
    float dx = x2-x;
    float dy = y2-y;
    float dtheta = lastPose[2] - heading;
    std::vector<float> deltaPose = {dx,dy,dtheta};




    if (distanceBetweenPoses < threshold) {return;} // break if not above threshold for movement



    lidarFrames.emplace_back(pointcloud);
    poseFrames.emplace_back(dataFrame.second);





    std::vector<float> relativeMovement = {x2-x, y2-y, h2-heading};

    // Create relative pose as Eigen::Isometry2d
    Eigen::Isometry2d relative_pose = Eigen::Isometry2d::Identity();
    relative_pose.translate(Eigen::Vector2d(dx, dy));
    relative_pose.rotate(Eigen::Rotation2Dd(h2-heading));
    addVertex(poseFrames.size() - 1, dataFrame.second); // dataFrame.second might not work TODO: Check this out, i think i really need delta translation

    // Define information matrix based on confidence
    double sigma_x = 0.1;
    double sigma_y = 0.1;
    double sigma_theta = 0.05;
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    information(0, 0) = 1.0 / (sigma_x * sigma_x);
    information(1, 1) = 1.0 / (sigma_y * sigma_y);
    information(2, 2) = 1.0 / (sigma_theta * sigma_theta);





    addEdge(poseFrames.size() - 2, poseFrames.size() - 1, relative_pose, information);



    // FIRST DOUBLE CHECK TRANSFORM BY RUNNING ICP
    std::cout << "Starting ICP optimization on transform" << std::endl;
    Eigen::Matrix4f transformation;
    double score;
    bool success = doTheMatching(lidarFrames[lidarFrames.size()-2], dataFrame.first, transformation, score);
    if (success) {
      std::cout << "GICP Found Match!" << std::endl;
      std::cout << transformation << std::endl;
      std::cout << "With a Score of: " << score << std::endl;
      double matchThreshold = 0.01;
      if (score > matchThreshold) {
        float delta_x = transformation(0, 3);
        float delta_y = transformation(1, 3);
        float delta_theta = std::atan2(transformation(1, 0), transformation(0, 0));




      }
    }else {
      std::cerr << "GICP alignment failed between frames " << (poseFrames.size() - 1) << " and " << poseFrames.size() << std::endl;
    }


    // add vertex and edge as well as to the main lists






    // add the pose to a cell
    myGridInstance.newCell(x,y,lidarFrames.size());

    // if keyfarme addedd
    poseInsecurity = 2;


    std::vector<int> lidarIndexes;
    int currentID = myGridInstance.cellCount;
    float searchXmax = x+poseInsecurity;
    float searchYmax = y+poseInsecurity;
    float searchXmin = x-poseInsecurity;
    float searchYmin = y-poseInsecurity;
    std::cout << "coords : " << x << y << " and searchminmax " << (searchYmax - searchYmin)/myGridInstance.scaleFactor <<std::endl;
    std::cout << "The cell id's are: ";
    for (float i = searchXmin; i <= searchXmax; i += ((1)/myGridInstance.scaleFactor)) {
      // searchXmax - searchXmin
      for (float j = searchYmin; j <= searchYmax; j += ((1)/myGridInstance.scaleFactor)) {

        if (sqrt(powf(i-x,2) + powf(j-y,2)) > poseInsecurity) {
          // saidjn
        } else {



          auto frameIndexes = myGridInstance.getFrameIndexes(i, j);
          //std::cout << "The available cells are: " << currentID-(searchYmax - searchYmin) << std::endl;
          //std::cout << "Searching at cell at : " << i << ", " << j << std::endl;


          if (!frameIndexes.lidarIndexes.empty() && frameIndexes.id < currentID-(poseInsecurity * 1.5 * myGridInstance.scaleFactor)) {

            // searchYmax - searchYmin
            // cur 5,  5-2=3 | 0 1 2


            auto index = myGridInstance.getFrameIndexes(i, j).id;
            //std::cout << index << " " << " and size of vector is: " << myGridInstance.visualizationMeshes.size() << std::endl;
            auto detectedmat = threepp::MeshBasicMaterial::create();
            detectedmat->color = threepp::Color::blue;
            if (frameIndexes.id >= 0 && frameIndexes.id < myGridInstance.visualizationMeshes.size()) {
              std::cout << "ID of Changed Color" << frameIndexes.id << std::endl;
              auto frameIndex = frameIndexes.id;
              myGridInstance.visualizationQueue.push([this, frameIndex]() {
                auto mat = threepp::MeshBasicMaterial::create();
                mat->color = threepp::Color::blue;
                myGridInstance.visualizationMeshes[frameIndex]->setMaterial(mat);
              });

            }
            //for (int i : myGridInstance.getFrameIndexes(i, j).lidarIndexes) {lidarIndexes.push_back(i);} //This takes too long
            lidarIndexes.push_back(myGridInstance.getFrameIndexes(i,j).lidarIndexes.front());
          }
        }
      }
    }


    // Define information matrix based on confidence
    double sigma_x_icp = 0.01;
    double sigma_y_icp = 0.01;
    double sigma_theta_icp = 0.05;
    Eigen::Matrix3d informationICP = Eigen::Matrix3d::Zero();
    informationICP(0, 0) = 1.0 / (sigma_x_icp * sigma_x_icp);
    informationICP(1, 1) = 1.0 / (sigma_y_icp * sigma_y_icp);
    informationICP(2, 2) = 1.0 / (sigma_theta_icp * sigma_theta_icp);

    for (int index : lidarIndexes) { // Might have to run on seperate thread or only run when there is a lock mutex on the lidarFrames data. Because right now it results in memory faults
      // Run ICP
      Eigen::Matrix4f transformation;
      double score;
      if (index > lidarFrames.size()) {break;}
      bool didGood = doTheMatching(lidarFrames[index], dataFrame.first, transformation, score);
      if (didGood) {
        Eigen::Isometry2d relative_pose = Eigen::Isometry2d::Identity();
        relative_pose.translate(Eigen::Vector2d(poseFrames[index][0]-x, poseFrames[index][1]-y));
        relative_pose.rotate(Eigen::Rotation2Dd(poseFrames[index][2]-heading));
        float delta_x = transformation(0, 3);
        float delta_y = transformation(1, 3);
        float delta_theta = std::atan2(transformation(1, 0), transformation(0, 0));
        std::vector<float> thePose;
        std::cout << "GICP Found Match!" << std::endl;
        std::cout << transformation << std::endl;
        std::cout << "With a Score of: " << score << std::endl;
        addEdge(poseFrames.size() - 1, index, relative_pose, information); // CurrentID,

      }






      /*
      frames_since_last_opt++;
      if (frames_since_last_opt >= optimize_every_n_frames) {
        runOptimization();
        frames_since_last_opt = 0;
      }*/




    };
  };

  TheGrid myGridInstance;
private:
  int frames_since_last_opt = 0;
  int optimize_every_n_frames = 10;
  std::unique_ptr<g2o::SparseOptimizer> optimizer;

  std::shared_ptr<std::unordered_map<int, std::unordered_map<int, GridData>>> myGridMap;

  std::shared_ptr<threepp::Scene> scene;
  float threshold = 0.3;
  std::vector<std::vector<std::pair<float, float>>> lidarFrames;
  std::vector<std::vector<float>> poseFrames;
  int keyFrameCount = 0;

  int countSinceLastLC = 0;
  float poseInsecurity = 0.0f;
  // G2O
  std::vector<std::vector<float>> vertexList;
  std::vector<std::pair<std::vector<float>, std::vector<float>>> edgeList;

  // Threading
  std::thread slamThread;
  std::queue<std::pair<std::vector<std::pair<float, float>>, std::vector<float>>> frameQueue;
  std::mutex queueMutex;
  std::condition_variable frameAvailable;
  bool stopThread = false;


};



/*
class CustomSLAM {
public:

  CustomSLAM();
  ~CustomSLAM();


  // Functions
  void addFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame); // Adds a frame to the slam algorithm for processing. This triggers add of pose.
  void enqueueFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame);
  void processFrames();
  void stopSLAMThread();
  void runICP(std::vector<std::pair<float, float>> firstFrame, std::vector<std::pair<float, float>> secondFrame);
  bool pathIntersected();

  //void renderDebugGrid(cv::Mat &mapImage, const KeyframeGrid &grid, float minX, float minY, float maxX, float maxY,
  //                     int imgWidth,
  //                     int imgHeight);

  void renderPreprocessedMap();
  //void renderGrid(cv::Mat &mapImage, const KeyframeGrid &grid, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight);
  //void overlayCoordinates(cv::Mat &mapImage, const KeyframeGrid &grid, float minX, float minY, float maxX, float maxY, int imgWidth, int imgHeight);

  // Variables
  int gridResolution = 0.2f;
  float distanceBetweenPoses = 0.2f; // The distance between each pose capture
  float tolerance;


private:

    // Pose Graph Optimization
    void addVertex(float gx, float gy, float gtheta);
    std::vector<std::vector<double>> vertexList;
    void addEdge(float dx, float dy, float dtheta);
    std::vector<std::vector<double>> edgeList;
    void exportPoseGraph(const std::string& filename);
    std::vector<std::pair<int, int>> loopClosureList;
    void exportEverything(const std::string& filename);

    int captureCounter = 0;

    int countSinceLastLoopClosure = 0;
    double current_insecurity = 0;


  // Functions
  void addLidar(std::vector<std::pair<float, float>> lidarData);
  void addPose(std::vector<float> pose);
  void deletePose(int index);
  bool checkLoopClosure(int targetIndex, int referenceIndex);
  void processFrame();
  float getDistanceToLastPose();

    // Threading
  std::thread slamThread;
  std::queue<std::pair<std::vector<std::pair<float, float>>, std::vector<float>>> frameQueue;
  std::mutex queueMutex;
  std::condition_variable frameAvailable;
  bool stopThread = false;


  // Variables
  float x_offset = 0.0;
  float y_offset = 0.0;
  float heading_offset = 0.0;
  std::vector<std::vector<std::pair<float, float>>>                             lidarFrames;
  std::vector<std::vector<float>>                                               poseFrames; // TEMP TODO: FIX THIS
  std::vector<std::vector<float>>                                               vertexFrames;
  std::vector<std::pair<std::vector<float>, std::vector<float>>>                edgeFrames;
  std::vector<std::vector<float>>                                               occupancyMap;
  //KeyframeGrid                                                                  gridMap;

};

*/
#endif //SLAM_H
