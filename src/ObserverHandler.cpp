#include "ObserverHandler.hpp"
#include "Arduino.h"

ObserverHandler::ObserverHandler(){}

ObserverHandler::~ObserverHandler() {}

void ObserverHandler::init() {

    for (int ii = 0; ii < 3; ii++) {
        P_k[4*ii+0] = 1.0F;
        P_k[4*ii+3] = 1.0F;
    }

    t_prev = millis();

}

void ObserverHandler::prediction(float* acc) {

    in[0] = acc[0];
    in[1] = acc[1];
    in[2] = 0;

    for (int ii = 0; ii < 3; ii++) {
        x[2*ii+0] = x[2*ii+0] + x[2*ii+1] * dt + 0.5 * pow(dt,2) * in[ii];
        x[2*ii+1] = x[2*ii+1] + dt * in[ii];

        Q_k[3*ii+0] = 1/4 * pow(dt,4) * pow(sigma[ii],2);
        Q_k[3*ii+1] = 1/2 * pow(dt,3) * pow(sigma[ii],2);
        Q_k[3*ii+2] = pow(dt,2) * pow(sigma[ii],2);

        P_k_1[4*ii+0] = P_k[4*ii+0] + P_k[4*ii+1] * dt  + P_k[4*ii+2] * dt + P_k[4*ii+3] * pow(dt,2) + Q_k[3*ii];
        P_k_1[4*ii+1] = P_k[4*ii+1] + P_k[4*ii+3] * dt  + Q_k[3*ii+1];
        P_k_1[4*ii+2] = P_k[4*ii+2] + P_k[4*ii+3] * dt  + Q_k[3*ii+1];
        P_k_1[4*ii+3] = P_k[4*ii+3] + Q_k[3*ii+2];
    }

}

void ObserverHandler::calculateInnovation(float* vel, float* gyro, float* ang, float* pBot, float* dist) {

    y[0] = (pBot[0] - dist[0]) - x[0];
    y[1] = vel[0] - x[1];

    y[2] = (pBot[1] - dist[1]) - x[2];
    y[3] = vel[1] - x[3];

    y[4] = ang[2] - x[4];

    if (y[4] > M_PI)
        y[4] -= 2*M_PI;
    else if (y[4] < -M_PI)
        y[4] += 2*M_PI;

    y[5] = gyro[2] - x[5];
}

void ObserverHandler::updateStates() {

    for (int ii = 0; ii < 3; ii++) {
        // S = P_k-1 + R
        S[4*ii+0] = P_k_1[4*ii+0] + R[2*ii+0];
        S[4*ii+1] = P_k_1[4*ii+1];
        S[4*ii+2] = P_k_1[4*ii+2];
        S[4*ii+3] = P_k_1[4*ii+3] + R[2*ii+1];

        // inv S
        float det = S[4*ii+0] * S[4*ii+3] - S[4*ii+1] * S[4*ii+2];
        Sinv[4*ii+0] =  S[4*ii+3] / det;
        Sinv[4*ii+1] = -S[4*ii+1] / det;
        Sinv[4*ii+2] = -S[4*ii+2] / det;
        Sinv[4*ii+3] =  S[4*ii+0] / det;

        // K = P_k-1 * inv(S)
        K[4*ii+0] = (P_k_1[4*ii+0] * Sinv[4*ii+0] + P_k_1[4*ii+1] * Sinv[4*ii+2]);
        K[4*ii+1] = (P_k_1[4*ii+0] * Sinv[4*ii+1] + P_k_1[4*ii+1] * Sinv[4*ii+3]);
        K[4*ii+2] = (P_k_1[4*ii+2] * Sinv[4*ii+0] + P_k_1[4*ii+3] * Sinv[4*ii+2]);
        K[4*ii+3] = (P_k_1[4*ii+2] * Sinv[4*ii+1] + P_k_1[4*ii+3] * Sinv[4*ii+3]);

        // x = x_k-1 + K * y
        x[2*ii+0] = x[2*ii+0] + (K[3*ii] * y[2*ii+0] + K[3*ii+1] * y[2*ii+1]);
        x[2*ii+1] = x[2*ii+1] + (K[3*ii+1] * y[2*ii+0] + K[3*ii+2] * y[2*ii+1]);

        // P_k = (1 - K) * P_k-1
        P_k[4*ii+0] = (1 - K[4*ii+0]) * P_k_1[4*ii+0] - K[4*ii+1] * P_k_1[4*ii+2];
        P_k[4*ii+1] = (1 - K[4*ii+0]) * P_k_1[4*ii+1] - K[4*ii+1] * P_k_1[4*ii+3];
        P_k[4*ii+2] = - K[4*ii+2] * P_k_1[4*ii+0] + (1 - K[4*ii+3]) * P_k_1[4*ii+2];
        P_k[4*ii+3] = - K[4*ii+2] * P_k_1[4*ii+1] + (1 - K[4*ii+3]) * P_k_1[4*ii+3];

    }

    x[4] = fmod(2*M_PI + fmod(x[4], 2*M_PI), 2*M_PI);
}

void ObserverHandler::print() {
    Serial.print(t_prev);
    Serial.print(", ");

    for (int q = 0; q < 6; q++) {
        Serial.print(x[q]);
        Serial.print(", ");
    }
    
    Serial.println("");
}

void ObserverHandler::update(unsigned long t, float* acc, float* vel, float* gyro, float* ang, float* pBot, float* dist) {

    dt = (float)(t - t_prev) / 1000;

    prediction(acc);
    calculateInnovation(vel, gyro, ang, pBot, dist);
    updateStates();

    #ifdef DEBUG_OBS
        print();
    #endif

    t_prev = t;

}