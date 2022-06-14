#pragma once
#ifndef DISTANCEHANDLER_HPP
#define DISTANCEHANDLER_HPP

#include "Defines.hpp"
#include "SensorHandler.hpp"

class DistanceHandler {
public:

    DistanceHandler();
    ~DistanceHandler();
    
    void init();
    void startContinuous();

    void update();   

    double ang = 0;
    int16_t avgDisX = 0;
    uint16_t avgDisY = 0;


private:
    
    SensorHandler sensorLeft = SensorHandler(LEFT);
    SensorHandler sensorRight = SensorHandler(RIGHT);
        
    void calcLine();
    void print();


};

#endif