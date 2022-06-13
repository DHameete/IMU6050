
#include "Controller.hpp"

Controller::Controller(){}

Controller::~Controller(){}

void Controller::update(float *pos, float *ang) {

    getRobotState();
    calculateError(pos, ang);
    controlOutput();

}

void Controller::getRobotState() {

  while(Serial.available()){
    int8_t xVal = (int8_t) Serial.read();
    Xbot = (float) xVal/40;
    int8_t yVal = (int8_t) Serial.read();
    Ybot = (float) yVal/40;
    int8_t thVal = (int8_t) Serial.read();
    THbot = (float) (thVal*M_PI)/127+M_PI;

    int8_t xdVal = (int8_t) Serial.read();
    Xdbot = (float) xdVal*16/1000;
    int8_t ydVal = (int8_t) Serial.read();
    Ydbot = (float) ydVal*16/1000;
    int8_t thdVal = (int8_t) Serial.read();
    THdbot = (float) (thdVal*64)/1000;
  }

}

void Controller::calculateError(float *pos, float *ang) {

  float Xbot_c = Xbot - pos[0];
  float Ybot_c = Ybot - pos[1];

  Rbot = sqrt(Xbot_c*Xbot_c + Ybot_c*Ybot_c);
  PHIbot = atan2(Ybot_c, Xbot_c);
  PHIbot = fmod(2*M_PI + fmod(PHIbot, 2*M_PI), 2*M_PI);  

  Rdbot   =  Xdbot * cos(PHIbot) + Ydbot * sin(PHIbot);
  PHIdbot = -Xdbot * sin(PHIbot) + Ydbot * cos(PHIbot);

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
  eTH = PHIbot - THbot;
  
  // Shortest angle
  if (eTH > M_PI) {
    eTH -= 2*M_PI;
  } else if (eTH < -M_PI) {
    eTH += 2*M_PI;
  }

}

void Controller::controlOutput() {

  float KpPHI_ePHI = max(min(KpPHI * ePHI, M_PI/2), -M_PI/2);//5),-5);//M_PI/2), -M_PI/2);

  float deltaX = KpR * eR * cos(PHIbot) - KpPHI_ePHI * sin(PHIbot);
  float deltaY = KpR * eR * sin(PHIbot) + KpPHI_ePHI * cos(PHIbot);

  float deltaXd = KdR * Rdbot * cos(PHIbot) - KdPHI * PHIdbot * sin(PHIbot);
  float deltaYd = KdR * Rdbot * sin(PHIbot) + KdPHI * PHIdbot * cos(PHIbot);

  float uX = -(deltaY * cos(THbot) - deltaX * sin(THbot));
  float uY = (deltaY * sin(THbot) + deltaX * cos(THbot));

  float uXd = -(deltaYd * cos(THbot) - deltaXd * sin(THbot));
  float uYd = (deltaYd * sin(THbot) + deltaXd * cos(THbot));

  float uTH = KpTH * eTH;
  float uTHd = KdTH * THdbot;

  u[0] = uX + uXd;// - R*(gyroZ*DEG_TO_RAD);
  u[0] = max(min(u[0],2),-2);
  u[1] = uY + uYd;
  u[1] = max(min(u[1],2),-2);
  u[2] = uTH + uTHd;// + gyroZ*DEG_TO_RAD;
  u[2] = max(min(u[2],8),-8);

    
  #ifndef DEBUG
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