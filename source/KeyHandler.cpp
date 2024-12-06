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
    if (rotatingLeft) {
        keyState_.heading = std::fmod(keyState_.heading + 100.0f* dt, 360.0f) ;
    }

    if (rotatingRight) {
        keyState_.heading = std::fmod(keyState_.heading - 100.0f* dt + 360.0f, 360.0f) ;
    }
    if (lidarScan) {
        sphero_.startLidar();
    } else {
        sphero_.stopLidar();
    }
    sphero_.drive(payload, dt);
    //sphero_.drive_with_heading(keyState_.heading, keyState_.speed, dt);
}

void KeyController::onKeyPressed(KeyEvent evt) {
    if (evt.key == Key::W) {
        payload = 0x01;
        keyState_.speed = shiftPressed ? maxSpeed * 2 : maxSpeed;
    } else if (evt.key == Key::S) {
        payload = 0x04;
        keyState_.speed = shiftPressed ? -maxSpeed * 2 : -maxSpeed;
    } else if (evt.key == Key::A) {
        payload = 0x02;
        rotatingLeft = true;
    } else if (evt.key == Key::D) {
        payload = 0x03;
        rotatingRight = true;
    } else if (evt.key == Key::L) {
        lidarScan = !lidarScan;
    } else if (evt.key == Key::SPACE) {
        shiftPressed = true;
    } else if (evt.key == Key::T) {
        sphero_.enableSweep(false);
    } else if (evt.key == Key::F) {
        auto frame = sphero_.getFullFrame();
        if (frame.first.empty() || frame.second.empty()) {
            std::cout << "No complete frame data available." << std::endl;
        } else {
            std::cout << "Lidar Frame Data:" << std::endl;
            for (size_t i = 0; i < frame.first.size(); ++i) {
                const auto& scan = frame.first[i];
                const auto& odometry = frame.second[i];
                /*
                // Print Lidar Scan Data CHATGPT Print for me
                std::cout << "Scan " << i + 1 << ": "
                          << "Angle: " << scan.angle << "°, "
                          << "Distance: " << scan.distance << "m, "
                          << "Start: (" << scan.start.x << ", " << scan.start.y << ", " << scan.start.z << "), "
                          << "End: (" << scan.end.x << ", " << scan.end.y << ", " << scan.end.z << "), "
                          << "Timestamp: " << std::chrono::duration_cast<std::chrono::milliseconds>(scan.timestamp.time_since_epoch()).count()
                          << " ms" << std::endl;

                // Print Odometry Data
                std::cout << "    Odometry Data - "
                          << "Position: (" << odometry.position.x << ", " << odometry.position.y << ", " << odometry.position.z << "), "
                          << "Orientation: " << odometry.orientation << " rad, "
                          << "Velocity: " << odometry.velocity << " m/s, "
                          << "Timestamp: " << std::chrono::duration_cast<std::chrono::milliseconds>(odometry.timestamp.time_since_epoch()).count()
                          << " ms" << std::endl;*/
            }
        }

    } else if (evt.key == Key::O) {
        OdometryData odometry = sphero_.getOdometryData();

        std::cout << "Odometry: " << odometry.position << std::endl;

    } else if (evt.key == Key::P) {
        if (sphero_.pov) {sphero_.pov = false;} else {sphero_.pov = true;}
    }
}

void KeyController::onKeyReleased(KeyEvent evt) {
    if (evt.key == Key::W || evt.key == Key::S) {
        keyState_.speed = 0;
        payload = 0;
    } else if (evt.key == Key::A) {
        rotatingLeft = false;
        payload = 0;
    } else if (evt.key == Key::D) {
        rotatingRight = false;
        payload = 0;
    } else if (evt.key == Key::SPACE) {
        shiftPressed = false;
        payload = 0;
    }
}
