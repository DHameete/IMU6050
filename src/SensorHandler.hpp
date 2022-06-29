#pragma once
#ifndef SENSORHANDLER_HPP
#define SENSORHANDLER_HPP

#include "VL53L1X.h"
#include "Defines.hpp"
#include <math.h>

class SensorHandler {
public:
    uint8_t address;

    SensorHandler();
    SensorHandler(SENSOR_SIDE side);
    ~SensorHandler();
    
    void init();
    void startContinuous();

    void calcLine();
    void movMean();

    void update();
    void read();
    void print();

    static const uint8_t num = 7;
    static const uint8_t measLen = 3;

    float distance[num] = {0};

    float avgDis = 0;
    float ang = 0;

private:

    VL53L1X distanceSensor;
    
    SENSOR_SIDE _side;
    uint8_t _XSHUT;   
    int16_t _OFFX;

    uint8_t _sizeROI[2] = {4,8};

    //1: {199}; 3: {167, 199, 231}; 4: {151, 183, 215, 247}; 5: {151, 183, 199, 215, 247}; 7: {151, 167, 183, 199, 215, 231, 247}; 13: {247,239,231,223,215,207,199,191,183,175,167,159,151};
    uint8_t _center[num] =  {151, 167, 183, 199, 215, 231, 247};
    float _distanceRaw[num] = {0};  
    float _measArray[measLen][num] = {{0}};

    uint8_t _zone = 0;
    bool _flag = false;

    uint8_t _measCount = 0;

    uint8_t _num_half = (uint8_t) num/2;

};

#endif