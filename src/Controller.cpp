
#include "Controller.hpp"

Controller::Controller(){}

Controller::~Controller(){}

void Controller::update(float *posBot, float *velBot, float *pos, float *ang) {

    calculateError(posBot, velBot, pos, ang);
    controlOutput(posBot, velBot);

}

void Controller::calculateError(float *posBot, float *velBot, float *pos, float *ang) {

  float Xbot_c = posBot[0] - pos[0];
  float Ybot_c = posBot[1] - pos[1];

  Rbot = sqrt(Xbot_c*Xbot_c + Ybot_c*Ybot_c);
  PHIbot = atan2(Ybot_c, Xbot_c);
  PHIbot = fmod(2*M_PI + fmod(PHIbot, 2*M_PI), 2*M_PI);  

  Rdbot   =  velBot[0] * cos(PHIbot) + velBot[1] * sin(PHIbot);
  PHIdbot = -velBot[0] * sin(PHIbot) + velBot[1] * cos(PHIbot);

  // Calculate polar errors
  eR = R - Rbot;
  ePHI = ang[2]*DEG_TO_RAD - PHIbot;

  // Shortest angle
  if (ePHI > M_PI) {
    ePHI -= 2*M_PI;
  } else if (ePHI < -M_PI) {
    ePHI += 2*M_PI;
  }

  // Angle error
  eTH = PHIbot - posBot[2];
  
  // Shortest angle
  if (eTH > M_PI) {
    eTH -= 2*M_PI;
  } else if (eTH < -M_PI) {
    eTH += 2*M_PI;
  }

}

void Controller::controlOutput(float *posBot, float *velBot) {

  float KpPHI_ePHI = max(min(KpPHI * ePHI, M_PI/2), -M_PI/2);//5),-5);//M_PI/2), -M_PI/2);

  float deltaX = KpR * eR * cos(PHIbot) - KpPHI_ePHI * sin(PHIbot);
  float deltaY = KpR * eR * sin(PHIbot) + KpPHI_ePHI * cos(PHIbot);

  float deltaXd = KdR * Rdbot * cos(PHIbot) - KdPHI * PHIdbot * sin(PHIbot);
  float deltaYd = KdR * Rdbot * sin(PHIbot) + KdPHI * PHIdbot * cos(PHIbot);

  float uX = -(deltaY * cos(posBot[2]) - deltaX * sin(posBot[2]));
  float uY = (deltaY * sin(posBot[2]) + deltaX * cos(posBot[2]));

  float uXd = -(deltaYd * cos(posBot[2]) - deltaXd * sin(posBot[2]));
  float uYd = (deltaYd * sin(posBot[2]) + deltaXd * cos(posBot[2]));

  float uTH = KpTH * eTH;
  float uTHd = KdTH * velBot[2];

  u[0] = uX + uXd;// - R*(gyroZ*DEG_TO_RAD);
  u[0] = max(min(u[0],2),-2);
  u[1] = uY + uYd;
  u[1] = max(min(u[1],2),-2);
  u[2] = uTH + uTHd;// + gyroZ*DEG_TO_RAD;
  u[2] = max(min(u[2],8),-8);

    
  #ifndef DEBUG_OBS
    // messenger.update(0,(R*(gyroZ*DEG_TO_RAD))*32767/2);
    messenger.update(0, uX*32767/2);
    delay(1);
    // messenger.update(1,0);
    messenger.update(1, uY*32767/2);
    delay(1);
    messenger.update(2, uTH*32767/8);
    delay(1);
  #endif


}