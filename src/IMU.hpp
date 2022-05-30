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

    float accXraw = 0.0F;
    float accYraw = 0.0F;
    float accZraw = 0.0F;

    float accX = 0.0F;
    float accY = 0.0F;
    float accZ = 0.0F;

    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;

    float pitch = 0.0F;
    float roll  = 0.0F;
    float yaw   = 0.0F;

    float temp;

    float velX = 0.0F;
    float velY = 0.0F;
    float th = 0.0F;

    float ACC_FILTER = 0.0;//0.06;//0.03;
    float GYRO_FILTER = 3;
    float MV_FILTER = 0*0.05;
    float VEL_FILTER = 0.0001; //TODO: make moving average more smooth?!!
    float OUTPUT_FILTER = 0.003;

    int c_len = 10;
    float prevAccX[10] = { 0.0F };
    float prevAccY[10] = { 0.0F };
    float prevGyroZ[10] = { 0.0F };

    float accXaccumulator = 0.0F;
    float accYaccumulator = 0.0F;
    float accZaccumulator = 0.0F;

    float prevAccXflt[10] = { 0.0F };
    float prevAccYflt[10] = { 0.0F };
        
    int v_len = 3;
    float prevVelX[3] = { 0.0F };
    float prevVelY[3] = { 0.0F };

    int b_indx = 0;
    int b_len = 255;
    float biasAccX[255] = { 0.0F };
    float biasAccY[255] = { 0.0F };
    float biasAccZ[255] = { 0.0F };


    float outX = 0.0F;
    float outY = 0.0F;
    float outTh = 0.0F;
    
    float R = 1.0F;

    float X = R;
    float Y = 0.0F;

    float Xbot = R;
    float Ybot = 0.0F;
    float THbot = 0.0F;

    float Xdbot = 0.0F;
    float Ydbot = 0.0F;
    float THdbot = 0.0F;

    float uX = 0.0F;
    float uY = 0.0F;
    float uTH = 0.0F;
    
    // Controller parameters
    float KpR = 0.5*5.0F;//3.0F;//5.0F;
    float KpPHI = 0.5*4.0F;//2.0;//4.0F;
    float KpTH = 2.0F;

    float KdR = -1.0F;
    float KdPHI = -2.0F;
    float KdTH = -0.5F;

    float Xref = 0.0F;
    float Yref = 0.0F;

    float accumulator = 0.0F;

    void getValues();
    void calculateRotations();

    void filterAccel();
    void filterGyro();

    void zeroingAccel(float* biasAcc, float* acc, uint8_t ind);

    void calculateDerivative();
    void movingAverage(float* prevAcc, float* acc);
    void movingAverage_exponential(float* prevAcc, float* acc);

    void calcVelocity(float* prevAcc, float acc, float* vel);
    void calcAngle(float* prevStates, float vel, float* angle);


    void calcPosition(float* prevVel, float vel, float* out);
    
    void outputValues(unsigned long t);

    MessageHandler messenger = MessageHandler();


    float tpnt = 0;

};

#endif