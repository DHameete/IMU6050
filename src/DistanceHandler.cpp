#include "DistanceHandler.hpp"

DistanceHandler::DistanceHandler(){}

DistanceHandler::~DistanceHandler() {}

void DistanceHandler::init() {

  sensorLeft.init();
  sensorRight.init();

}

void DistanceHandler::startContinuous() {

  sensorLeft.startContinuous();
  sensorRight.startContinuous();

}

void DistanceHandler::update() {

  sensorLeft.update();
  sensorRight.update();
  
  calcLine();

  #ifdef DEBUG
    // print();
  // #else
  //   messenger.update(0, controller.valueX);
  //   delay(1);
  //   messenger.update(1, controller.valueY);
  //   delay(1);
  //   // messenger.update(2, controller.valueAng);
  //   // delay(1);
  #endif
  

}

void DistanceHandler::calcLine() {

  ang = (sensorRight.ang + sensorLeft.ang) / 2;
  avgDisX = (int16_t) ((sensorRight.avgDis - sensorLeft.avgDis) / (tan(sensorLeft.ang) - tan(sensorRight.ang)));
  avgDisY = (uint16_t) (tan(sensorLeft.ang) * avgDisX + sensorLeft.avgDis);

}

void DistanceHandler::print() {
  
  // Serial.print(millis());
  // Serial.print(", ");
  
  sensorLeft.print();
  sensorRight.print();

  // Serial.print(avgDis);
  // Serial.print((double) controller.valueX / 32767 * 127 );
  // Serial.print(", ");
  // Serial.print((double) controller.valueY / 32767 * 127);
  // Serial.print(", ");
  // Serial.print(controller.valueAng);

  Serial.print(avgDisX);
  Serial.print(", ");
  Serial.print(avgDisY);
  Serial.print(", ");
  Serial.print(ang);

  Serial.println("");

}