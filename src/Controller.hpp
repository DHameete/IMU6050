#pragma once
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "Defines.hpp"
#include "MessageHandler.hpp"

class Controller {
public:
    Controller();
    ~Controller();
    void update(float *pos, float *ang);

private:
    void getRobotState();
    void calculateError(float *pos, float *ang);
    void controlOutput();

    float R = 1.0F;

    float Xbot = R;
    float Ybot = 0.0F;
    float THbot = 0.0F;

    float Xdbot = 0.0F;
    float Ydbot = 0.0F;
    float THdbot = 0.0F;

    float u[3] = { 0.0F };
    
    // Controller parameters
    float KpR = 0.5*5.0F;//3.0F;//5.0F;
    float KpPHI = 0.5*4.0F;//2.0;//4.0F;
    float KpTH = 2.0F;

    float KdR = -1.0F;
    float KdPHI = -2.0F;
    float KdTH = -0.5F;

    float eR = 0.0F;
    float ePHI = 0.0F;
    float eTH = 0.0F;

    float PHIbot = 0.0F;
    float PHIdbot = 0.0F;

    float Rbot = 0.0F;
    float Rdbot = 0.0F;


    MessageHandler messenger = MessageHandler();
};

#endif