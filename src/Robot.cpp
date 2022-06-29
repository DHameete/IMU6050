
#include "Robot.hpp"

Robot::Robot(){}

Robot::~Robot(){}

void Robot::update() {

    getRobotState();

}

void Robot::getRobotState() {

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
