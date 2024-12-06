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
#include "Grid.h"
#include <Eigen/Geometry>





class NewSlam {
public:
  NewSlam(std::shared_ptr<threepp::Scene> scene);
  ~NewSlam();





  void enqueueFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame);
  void processFrames();
  void stopSLAMThread();

  // ChatGPT Parser for g2o format
  void exportToG2O(const std::string& filename, g2o::SparseOptimizer* optimizer);

  void addVertex(int id, std::vector<float>& pose);
  void addEdge(int fromID, int toID, const Eigen::Isometry2d& relativePose, const Eigen::Matrix3d informationMatrix);

  pcl::PointCloud<pcl::PointXYZ>::Ptr toPCLformat(std::vector<std::pair<float, float>> data);
  std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> parseDataset(const std::string& filename, float offsetX = 0, float offsetY = 0, float offsetHeading = 0);





  void runOptimization();
  bool doTheMatching(std::vector<std::pair<float, float>> sourceData, std::vector<std::pair<float, float>> targetData, Eigen::Matrix4f& transformation, double& score);
  void initializePoseGraph();
  void addFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame);

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



#endif //SLAM_H
