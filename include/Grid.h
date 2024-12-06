//
// Created by Codetonsen on 12/6/2024.
//

#ifndef GRID_H
#define GRID_H

#include <iostream>
#include <vector>
#include <queue>
#include "Sphero.h"



struct GridData {
    int id;
    std::vector<int> lidarIndexes;
};

class TheGrid {
public:
    TheGrid(bool isSimulator, const std::shared_ptr<threepp::Scene> &scene);


    std::queue<std::function<void()>> visualizationQueue;
    std::mutex visualizationQueueMutex;

    void newCell(float gx, float gy, int frameIndex);

    GridData getFrameIndexes(float gx, float gy);

    void addCell(int posX, int posY);
    void changeColor(float gx, float gy);

    std::shared_ptr<threepp::Mesh> createVisuBox(const threepp::Vector3& pos, const threepp::Color& color);


    std::unordered_map<int, std::unordered_map<int, GridData>> theGrid;
    float scaleFactor = 5;
    std::vector<std::shared_ptr<threepp::Mesh>> visualizationMeshes;
    int cellCount = 0;
private:
    bool isSimulator = false;
    std::shared_ptr<threepp::Scene> scene;
};


#endif //GRID_H
