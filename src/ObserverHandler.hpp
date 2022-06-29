#pragma once
#ifndef OBSERVERHANDLER_HPP
#define OBSERVERHANDLER_HPP

#include "Defines.hpp"
#include <cmath>

class ObserverHandler {
public:

    ObserverHandler();
    ~ObserverHandler();
    
    void init();
    void update(unsigned long t, float* acc, float* vel, float* gyro, float* ang, float* pBot, float* dist);   
    float x[6] = { 0.0F }; // { x, xd, y, yd, th, thd }

private:

    float dt; 
    unsigned long t_prev;

    float in[3] = { 0.0F };
    float sigma[3] = { 1, 1, 0.1 };
    
    float P_k[12] = { 0.0F };
    float P_k_1[12] = { 0.0F };

    float Q_k[9] = { 0.0F };

    float y[6] = { 0.0F };

    float R[6] = { 1, 1, 1, 1, 0.1, 0.1  };
    float S[12] = { 0.0F };
    float Sinv[12] = { 0.0F };
 
    float K[12] = { 0.0F };

    void prediction(float* acc);
    void calculateInnovation(float* vel, float* gyro, float* ang, float* pBot, float* dist);
    void updateStates();
    void print();
};

#endif