#pragma once
#ifndef IMU_HPP
#define IMU_HPP

#include "MPU6050.h"
#include "MessageHandler.hpp"
#include "Defines.hpp"

class IMU {
public:
    IMU();
    ~IMU();
    void init();
    void update(unsigned long t);

private:

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

    float acc[3] = { 0.0F };
    float gyro[3] = { 0.0F };

    float accAccum[3] = { 0.0F };

    float vel[3] = { 0.0F };
    float ang[3] = { 0.0F };

    float pos[3] = { 0.0F };


    float N_OFF = 3000;
    float ACC_FILTER = 0.0;
    float GYRO_FILTER = 3;
    float VEL_FILTER = 0.0001; //TODO: make moving average more smooth?!!
    float OUTPUT_FILTER = 0.003;
    float alpha = 0.01;

    
    float prevAcc[3][c_len] = {{ 0.0F }};
    float prevGyro[3][c_len] = {{ 0.0F }};
    
    float prevVel[3][v_len] = {{ 0.0F }};

    int b_indx = 0;
    float biasAcc[3][b_len] = {{ 0.0F }};

    float R = 1.0F;

    float Xref = R;
    float Yref = 0.0F;

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

    float Xbot_ref = R;
    float Ybot_ref = 0.0F;

    float eR = 0.0F;
    float ePHI = 0.0F;
    float eTH = 0.0F;

    float PHIbot = 0.0F;
    float PHIdbot = 0.0F;

    float Rbot = 0.0F;
    float Rdbot = 0.0F;

    MessageHandler messenger = MessageHandler();
    

    void getValues();

    void initOffset();
    void filterAccGyr();

    void expMovAvg();

    void zeroAcc();

    void calculateDerivative();
    
    void calcVelocity();
    void calcPosition();
    void calcAngle();
    
    void getRobotState();
    void calculateError();
    void controlOutput(unsigned long t);
    
    void outputValues(unsigned long t);

};

#endif