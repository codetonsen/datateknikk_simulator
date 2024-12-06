//
// Created by Codetonsen on 10/29/2024.
//
#include "../include/KeyHandler.h"
#include <threepp/threepp.hpp>
#include <iostream>
#include "../include/Sphero.h"
using namespace threepp;
KeyController::KeyController(Sphero& sphero) : sphero_(sphero) {}

void KeyController::update(float dt_) {
    dt = dt_;
    if (lidarScan) {
        sphero_.startLidar();
    } else {
        sphero_.stopLidar();
    }

    // Send to sphero drive function
    sphero_.drive({forward, backward, left, right, doubleSpeed, halfSpeed}, dt);
}

void KeyController::onKeyPressed(KeyEvent evt) {
    if (evt.key == Key::W) {
        forward = true;
    } else if (evt.key == Key::S) {
        backward = true;
    } else if (evt.key == Key::A) {
        left = true;
    } else if (evt.key == Key::D) {
        right = true;
    } else if (evt.key == Key::L) {
        lidarScan = !lidarScan;
    } else if (evt.key == Key::SPACE) {
        doubleSpeed = true;
    } else if (evt.key == Key::CAPS_LOCK) {
        halfSpeed = true;
    } else if (evt.key == Key::T) {
        sphero_.enableSweep(false);
    } else if (evt.key == Key::F) {
        auto frame = sphero_.getFullFrame();
        if (frame.first.empty() || frame.second.empty()) {
            std::cout << "No complete frame data available." << std::endl;
        }

    } else if (evt.key == Key::O) {
        OdometryData odometry = sphero_.getOdometryData();

        std::cout << "Odometry: " << odometry.position << std::endl;

    } else if (evt.key == Key::P) {
        if (sphero_.pov) {sphero_.pov = false;} else {sphero_.pov = true;}
    }
}

void KeyController::onKeyReleased(KeyEvent evt) {
    if (evt.key == Key::W) {
        forward = false;
    } else if(evt.key == Key::S) {
        backward = false;
    } else if (evt.key == Key::A) {
        left = false;
    } else if (evt.key == Key::D) {
        right = false;
    } else if (evt.key == Key::SPACE){
        doubleSpeed = false;
    } else if (evt.key == Key::CAPS_LOCK) {
        halfSpeed = false;
    }
}
