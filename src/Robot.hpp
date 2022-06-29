#pragma once
#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "Defines.hpp"
#include <Arduino.h>

class Robot {
public:
    Robot();
    ~Robot();
    void update();

    float Xbot = 0.0F;
    float Ybot = 0.0F;
    float THbot = 0.0F;

    float Xdbot = 0.0F;
    float Ydbot = 0.0F;
    float THdbot = 0.0F;

private:
    void getRobotState();


};

#endif