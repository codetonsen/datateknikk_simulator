//
// Created by Codetonsen on 12/6/2024.
//

#include "../include/Grid.h"
TheGrid::TheGrid(bool isSimulator, const std::shared_ptr<threepp::Scene> &scene)
	: isSimulator(isSimulator), scene(scene), scaleFactor(1.0f)
{
	std::cout << "Initializing Grid" << std::endl;
	std::cout << "Initializing Grid with scaleFactor: " << scaleFactor << std::endl;
}

void TheGrid::newCell(float gx, float gy, int frameIndex) { // makes a new cell at global input x and y
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


GridData TheGrid::getFrameIndexes(float gx, float gy) {
	int posX = round(gx * scaleFactor); // gx0.15 -> 0.15 * 10 -> index = 2
	int posY = round(gy * scaleFactor);
	//std::cout << "posX: " << posX << " posY: " << posY << std::endl;
	return theGrid[posX][posY];
}


void TheGrid::addCell(int posX, int posY) {
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

void TheGrid::changeColor(float gx, float gy) {
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

std::shared_ptr<threepp::Mesh> TheGrid::createVisuBox(const threepp::Vector3& pos, const threepp::Color& color) {
	auto geometry = threepp::BoxGeometry::create(0.90 / scaleFactor, 0.1, 0.90 / scaleFactor);
	auto material = threepp::MeshPhongMaterial::create();
	material->color = color;
	auto box = threepp::Mesh::create(geometry, material);
	box->position.copy(pos);
	return box;
}
