//
// Created by Codetonsen on 10/29/2024.
//

#ifndef KEYHANDLER_H
#define KEYHANDLER_H



#include <threepp/input/KeyListener.hpp>
#include "Sphero.h"



class KeyController : public threepp::KeyListener {
public:
    explicit KeyController(Sphero& sphero);
    void update(float dt_);
    void onKeyPressed(threepp::KeyEvent evt) override;
    void onKeyReleased(threepp::KeyEvent evt) override;

private:
    bool shiftPressed = false;
    bool lidarScan = false;
    float dt;
    Sphero& sphero_;
    bool rotatingLeft = false;
    bool rotatingRight = false;
    float maxSpeed = 2.0f;
    int payload = 0;
    struct KeyState {
        float heading = 0.0f;
        float speed = 0.0f;
    } keyState_;
};






#endif //KEYHANDLER_H
