#pragma once
#ifndef IMU_HPP
#define IMU_HPP

#include "MPU6050.h"
#include "Defines.hpp"

class IMU {
public:
    IMU();
    ~IMU();
    void init();
    void update(unsigned long t);

    float acc[3] = { 0.0F };
    float gyro[3] = { 0.0F };
    float gyro_deg[3] = { 0.0F };
    float vel[3] = { 0.0F };
    float ang[3] = { 0.0F };
    float ang_deg[3] = { 0.0F };
    float pos[3] = { 0.0F };

    static const uint8_t c_len =  10;
    static const uint8_t v_len =   3;
    static const uint8_t b_len = 255;

private:

    float N_OFF = 3000;
    float ACC_FILTER = 0.0;
    float GYRO_FILTER = 3;
    float VEL_FILTER = 0.0001; //TODO: make moving average more smooth?!!
    float OUTPUT_FILTER = 0.003;
    float alpha = 0.01;

    // Pin definitions
    int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output

    float gyroBias[3], accBias[3]; // Bias corrections for gyro and accelerometer
    float gyroOff[3], accOff[3]; // Offset corrections for gyro and accelerometer

    int16_t tempCount;               // Stores the internal chip temperature sensor output 
    float temperature;               // Scaled temperature in degrees Celsius

    float aRes, gRes; // scale resolutions per LSB for the sensors
    MPU6050lib mpu;

    int dt; 
    unsigned long t_prev = millis();

    float accRaw[3] = { 0.0F };
    float gyroRaw[3] = { 0.0F };
    float temp = 0.0;

    float accAccum[3] = { 0.0F };

    float prevAcc[3][c_len] = {{ 0.0F }};
    float prevGyro[3][c_len] = {{ 0.0F }};
    
    float prevVel[3][v_len] = {{ 0.0F }};

    int b_indx = 0;
    float biasAcc[3][b_len] = {{ 0.0F }};

    void getValues();

    void initOffset();
    void filterAccGyr();

    void expMovAvg();
    void zeroAcc();
    
    void calcVelocity();
    void calcPosition();
    void calcAngle();
    
    void convertToRad();

    void outputValues(unsigned long t);

};

#endif