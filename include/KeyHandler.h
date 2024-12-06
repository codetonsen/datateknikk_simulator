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
    bool lidarScan = false;
    float dt;
    Sphero& sphero_;

    bool forward = false;
    bool backward = false;
    bool left = false;
    bool right = false;
    bool doubleSpeed = false;

};






#endif //KEYHANDLER_H
