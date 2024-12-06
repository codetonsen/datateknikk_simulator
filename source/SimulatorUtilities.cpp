//
// Created by Codetonsen on 10/29/2024.
//
#include "../include/SimulatorUtilities.h"
#include <threepp/threepp.hpp>

#include "threepp/textures/DataTexture.hpp"
using namespace threepp;

std::shared_ptr<Mesh> createBox(const Vector3& pos, const Color& color) {
    auto geometry = BoxGeometry::create(0.5, 0.5, 0.5);
    auto material = MeshPhongMaterial::create();
    material->color = color;

    auto box = Mesh::create(geometry, material);
    box->position.copy(pos);

    return box;
}

#include <threepp/threepp.hpp>
#include <iostream>
#include <cmath>
#include <memory>

using namespace threepp;

std::vector<Vector3> createPathPoints(float scale = 0.3f, float offset = 0.0f) {
    std::vector<Vector3> pathPoints;

    // Define parameters for the semicircles with scaling
    float radius = 0.5f * scale;
    int segments = 100;

    // First semicircle on the left side (-0.5 * scale center)
    for (int i = 0; i <= segments; ++i) {
        float angle = math::PI * 0.5f + i * math::PI / segments;
        pathPoints.emplace_back((-0.5f * scale) + radius * std::cos(angle), radius * std::sin(angle), offset);
    }

    // Straight line connecting the left semicircle to the right semicircle
    int lineSegments = 50;  // Number of points along the straight line
    for (int i = 0; i <= lineSegments; ++i) {
        float z = (-0.5f * scale) + (scale * i / lineSegments); // Interpolate z from -0.5 * scale to 0.5 * scale
        pathPoints.emplace_back(z, -radius, offset);
    }

    // Second semicircle on the right side (0.5 * scale center)
    for (int i = 0; i <= segments; ++i) {
        float angle = math::PI * 1.5f + i * math::PI / segments;
        pathPoints.emplace_back((0.5f * scale) + radius * std::cos(angle), radius * std::sin(angle), offset);
    }

    // Straight line connecting back from right to left
    for (int i = 0; i <= lineSegments; ++i) {
        float z = (0.5f * scale) - (scale * i / lineSegments); // Interpolate z from 0.5 * scale to -0.5 * scale
        pathPoints.emplace_back(z, radius, offset);
    }

    return pathPoints;
}

std::shared_ptr<threepp::Mesh> createCubeForBelt(const std::shared_ptr<MeshBasicMaterial>& material) {
    auto scale = 0.2;
    auto geometry = BoxGeometry::create(0.05 * scale, 0.05f * scale, 1.0f * scale);
    return Mesh::create(geometry, material);
}
InstancedMeshController::InstancedMeshController(int instanceCount, std::shared_ptr<InstancedMesh> instancedMesh)
    : instanceCount_(instanceCount), instancedMesh_(std::move(instancedMesh)) {
    initializePositionsAndVelocities();
    initializeInstanceColors();
}

void InstancedMeshController::initializePositionsAndVelocities() {
    Matrix4 matrix;
    for (int i = 0; i < instanceCount_; ++i) {
        Vector3 position(40.0f, 40.0f, 40.0f);
        matrix.setPosition(position);
        instancedMesh_->setMatrixAt(i, matrix);
    }
    instancedMesh_->instanceMatrix()->needsUpdate();
}

void InstancedMeshController::initializeInstanceColors() {
    for (int i = 0; i < instanceCount_; ++i) {
        instancedMesh_->setColorAt(i, Color(0x00ffff));  // Start with teal
    }
    instancedMesh_->instanceColor()->needsUpdate();
}

void InstancedMeshController::updatePositionsFromLidar(const std::vector<LidarScan>& lidarScans) {
    int instanceIndex = 0;

    for (const auto& scan : lidarScans) {
        if (instanceIndex >= instanceCount_) break;

        Vector3 direction = scan.end - scan.start;
        float length = direction.length();
        direction.normalize();

        Matrix4 matrix;
        matrix.lookAt(scan.start, scan.end, Vector3(0, 1, 0));
        matrix.scale(Vector3(1, 1, length));
        matrix.setPosition((scan.start + scan.end) * 0.5f);

        instancedMesh_->setMatrixAt(instanceIndex, matrix);
        instanceIndex++;
    }

    instancedMesh_->setCount(instanceIndex);
    instancedMesh_->instanceMatrix()->needsUpdate();
}

void InstancedMeshController::updateColorsByAge(const std::vector<LidarScan>& lidarScans) {
    for (int i = 0; i < instanceCount_; ++i) {
        if (i >= lidarScans.size()) break;

        const auto& scan = lidarScans[i];
        float age = 1;

        Color color = interpolateColor(Color(0x00ff00), Color(0xff0000), std::min(age, 1.0f));
        instancedMesh_->setColorAt(i, color);
    }

    instancedMesh_->instanceColor()->needsUpdate();
}

Color InstancedMeshController::interpolateColor(const Color& color1, const Color& color2, float t) {
    Color result = color1;
    result.lerp(color2, t);
    return result;
}
