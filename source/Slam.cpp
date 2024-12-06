//
// Created by Codetonsen on 11/25/2024.
#include "../include/Slam.h"

NewSlam::NewSlam(std::shared_ptr<threepp::Scene> scene): myGridInstance(true, scene), scene(scene)
{
    slamThread = std::thread(&NewSlam::processFrames, this);
    initializePoseGraph();

};
NewSlam::~NewSlam() {
    stopSLAMThread();
    exportToG2O("myTest.g2o", optimizer.get());
};
void NewSlam::enqueueFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame) {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        frameQueue.push(dataFrame);
    }
    frameAvailable.notify_one();
};
void NewSlam::processFrames() {
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
void NewSlam::stopSLAMThread() {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        stopThread = true;
    }
    frameAvailable.notify_all();
    if (slamThread.joinable()) {
        slamThread.join();
    }
};

// Parser for REAL LidarScans
std::vector<std::pair<std::vector<LidarScan>, std::vector<float>>> NewSlam::parseDataset(const std::string& filename, float offsetX, float offsetY, float offsetHeading) {
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


// G2O SLAM METHODS and Helpers
void NewSlam::exportToG2O(const std::string& filename, g2o::SparseOptimizer* optimizer) {
    // ChatGPT Parser for g2o format
    // This method is purely chatGPT to export to g2o format. The g2o format helps us test the actual pose graph algorithm
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
void NewSlam::addVertex(int id, std::vector<float>& pose) {
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
void NewSlam::addEdge(int fromID, int toID, const Eigen::Isometry2d& relativePose, const Eigen::Matrix3d informationMatrix) {
    if (!optimizer->vertex(fromID) || !optimizer->vertex(toID)) { // tries to find the vertex with fromid and toid in the optimizer. Breaks if one is not there, this causes many problems if not here.
        std::cerr << "One of the vertices (from: " << fromID << ", to: " << toID << ") does not exist in the optimizer." << std::endl;
        return;
    }

    // Setting up the general graph optimizer edge.
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = optimizer->vertex(fromID);
    edge->vertices()[1] = optimizer->vertex(toID);
    // relativepose: x,y,delta heading
    edge->setMeasurement(g2o::SE2(relativePose.translation().x(), relativePose.translation().y(), atan2(relativePose.rotation().matrix()(1,0), relativePose.rotation().matrix()(0,0))));
    edge->setInformation(informationMatrix);
    if (!optimizer->addEdge(edge)) {
        std::cerr << "Failed to add edge from " << fromID << " to " << toID << std::endl;
    }
};
void NewSlam::runOptimization() {
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

}
void NewSlam::initializePoseGraph() {
    optimizer = std::make_unique<g2o::SparseOptimizer>();
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto linearSolver = g2o::make_unique<LinearSolverType>();
    auto solver_ptr = g2o::make_unique<g2o::OptimizationAlgorithmLevenberg>(g2o::make_unique<BlockSolverType>(std::move(linearSolver)));
    optimizer->setAlgorithm(solver_ptr.release());
    optimizer->setVerbose(false);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr NewSlam::toPCLformat(const std::vector<std::pair<float, float>>& data) {
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

// Main Methods in SLAM.
bool NewSlam::doTheMatching(const std::vector<std::pair<float, float>> &sourceData, std::vector<std::pair<float, float>> targetData, Eigen::Matrix4f& transformation, double& score) {
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
void NewSlam::addFrame(std::pair<std::vector<std::pair<float, float>>, std::vector<float>> dataFrame) {
    // Check for missing data
    if (dataFrame.first.size() == 0 or dataFrame.second.size() == 0) {return;};
    updateLidarMap(dataFrame);
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

      double matchThreshold = 0.1;

      if (score < matchThreshold) {

          std::cout << "GICP Found Match!" << std::endl;
          std::cout << transformation << std::endl;
          std::cout << "With a Score of: " << score << std::endl;
          float delta_x = transformation(0, 3);
          float delta_y = transformation(1, 3);
          float delta_theta = std::atan2(transformation(1, 0), transformation(0, 0));
          std::vector<float> newPose = {dataFrame.second[0] + delta_x, dataFrame.second[1] + delta_y, dataFrame.second[2] +delta_theta};
          //updateLidarMapPostProcess({dataFrame.first, newPose});



      }
    }else {
      std::cerr << "GICP alignment failed between frames " << (poseFrames.size() - 1) << " and " << poseFrames.size() << std::endl;
    }


    // add the pose to a cell
    myGridInstance.newCell(x,y,lidarFrames.size());


    poseInsecurity = 2;
    std::vector<int> lidarIndexes;
    int currentID = myGridInstance.cellCount;
    float searchXmax = x+poseInsecurity;
    float searchYmax = y+poseInsecurity;
    float searchXmin = x-poseInsecurity;
    float searchYmin = y-poseInsecurity;

    //std::cout << "coords : " << x << y << " and searchminmax " << (searchYmax - searchYmin)/myGridInstance.scaleFactor <<std::endl;
    //std::cout << "The cell id's are: ";
    for (float i = searchXmin; i <= searchXmax; i += ((1)/myGridInstance.scaleFactor)) {
      for (float j = searchYmin; j <= searchYmax; j += ((1)/myGridInstance.scaleFactor)) {


        if (sqrt(powf(i-x,2) + powf(j-y,2)) < poseInsecurity) {
            auto frameIndexes = myGridInstance.getFrameIndexes(i, j);
            //std::cout << "The available cells are: " << currentID-(searchYmax - searchYmin) << std::endl;
            //std::cout << "Searching at cell at : " << i << ", " << j << std::endl;
            if (!frameIndexes.lidarIndexes.empty() && frameIndexes.id < currentID-(poseInsecurity * 1.5 * myGridInstance.scaleFactor)) {

                // searchYmax - searchYmin
                // cur 5,  5-2=3 | 0 1 2

                // Adds visualization point.
                auto index = myGridInstance.getFrameIndexes(i, j).id;
                //std::cout << index << " " << " and size of vector is: " << myGridInstance.visualizationMeshes.size() << std::endl;
                auto detectedmat = threepp::MeshBasicMaterial::create();
                detectedmat->color = threepp::Color::blue;
                if (frameIndexes.id >= 0 && frameIndexes.id < myGridInstance.visualizationMeshes.size()) {
                    //std::cout << "ID of Changed Color" << frameIndexes.id << std::endl;
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
          if  (score < 0.1) {
              std::cout << "GICP Found Match!" << std::endl;
              std::cout << transformation << std::endl;
              std::cout << "With a Score of: " << score << std::endl;
              float delta_x = transformation(0, 3);
              float delta_y = transformation(1, 3);
              float delta_theta = std::atan2(transformation(1, 0), transformation(0, 0));
              std::vector<float> newPose = {dataFrame.second[0] + delta_x, dataFrame.second[1] + delta_y, dataFrame.second[2] +delta_theta};
              //updateLidarMapPostProcess({dataFrame.first, newPose});

          }
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
       *Runs the pose graph optimizer.
      frames_since_last_opt++;
      if (frames_since_last_opt >= optimize_every_n_frames) {
        runOptimization();
        frames_since_last_opt = 0;
      }*/




    };
  };

void NewSlam::updateLidarMap(const std::pair<std::vector<std::pair<float, float>>, std::vector<float>>& scanAndPose) {
    // Ensure the pose vector is of size 3
    if (scanAndPose.second.size() != 3) {
        throw std::invalid_argument("Pose vector is not of size 3");
    }
    float robotX = scanAndPose.second[0];
    float robotY = scanAndPose.second[1];
    float robotHeading = scanAndPose.second[2];
    //std::cout << "ROBOT HEADING: " << robotHeading << std::endl;

    const float scale = 50.0;
    const int mapWidth = 500.0;
    const int mapHeight = 500.0;

    for (const auto& point : scanAndPose.first) {
        float localX = point.first;
        float localY = point.second;

        // apply the rotation of the rover
        float transformedX = localX * cos(-robotHeading) - localY * sin(-robotHeading);
        float transformedY = localX * sin(-robotHeading) + localY * cos(-robotHeading);

        // convert to map scale
        int mapX = static_cast<int>(transformedX * scale + robotX * scale + mapWidth / 2);
        int mapY = static_cast<int>(transformedY * scale + robotY * scale + mapHeight / 2);

        if (mapX >= 0 && mapX < mapWidth && mapY >= 0 && mapY < mapHeight) {
            // Draw the point on the map
            cv::circle(lidarMap, cv::Point(mapX, mapY), 1, cv::Scalar(0, 255, 0), -1); // Green dots for points
        }

    }

    int robotMapX = static_cast<int>(robotX * scale + mapWidth / 2);
    int robotMapY = static_cast<int>(robotY * scale + mapHeight / 2);
    if (robotMapX >= 0 && robotMapX < mapWidth && robotMapY >= 0 && robotMapY < mapHeight) { // boundsCheck
        cv::circle(lidarMap, cv::Point(robotMapX, robotMapY), 3, cv::Scalar(0, 0, 255), -1); // Red dot for robot
    }

    // Display the updated map
    cv::imshow("Lidar Map Odometry Data Only", lidarMap);
    cv::waitKey(1);
}

void NewSlam::updateLidarMapPostProcess(const std::pair<std::vector<std::pair<float, float>>, std::vector<float>>& scanAndPose) {
    std::cout << "Got a post proccessed fframe" << std::endl;
    // Ensure the pose vector is of size 3
    if (scanAndPose.second.size() != 3) {
        throw std::invalid_argument("Pose vector is not of size 3");
    }
    float robotX = scanAndPose.second[0];
    float robotY = scanAndPose.second[1];
    float robotHeading = scanAndPose.second[2];
    //std::cout << "ROBOT HEADING: " << robotHeading << std::endl;

    const float scale = 50.0;
    const int mapWidth = 500.0;
    const int mapHeight = 500.0;

    for (const auto& point : scanAndPose.first) {
        float localX = point.first;
        float localY = point.second;

        // apply the rotation of the rover
        float transformedX = localX * cos(-robotHeading) - localY * sin(-robotHeading);
        float transformedY = localX * sin(-robotHeading) + localY * cos(-robotHeading);

        // convert to map scale
        int mapX = static_cast<int>(transformedX * scale + robotX * scale + mapWidth / 2);
        int mapY = static_cast<int>(transformedY * scale + robotY * scale + mapHeight / 2);

        if (mapX >= 0 && mapX < mapWidth && mapY >= 0 && mapY < mapHeight) {
            // Draw the point on the map
            cv::circle(lidarMapPostProccessed, cv::Point(mapX, mapY), 1, cv::Scalar(0, 255, 0), -1); // Green dots for points
        }

    }

    int robotMapX = static_cast<int>(robotX * scale + mapWidth / 2);
    int robotMapY = static_cast<int>(robotY * scale + mapHeight / 2);
    if (robotMapX >= 0 && robotMapX < mapWidth && robotMapY >= 0 && robotMapY < mapHeight) { // boundsCheck
        cv::circle(lidarMapPostProccessed, cv::Point(robotMapX, robotMapY), 3, cv::Scalar(0, 0, 255), -1); // Red dot for robot
    }

    // Display the updated map
    cv::imshow("Lidar Map Post-Proccessed", lidarMapPostProccessed);
    cv::waitKey(1);
}