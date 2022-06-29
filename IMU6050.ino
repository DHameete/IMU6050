#include <Wire.h>
#include "src/IMU.hpp"
#include "src/Controller.hpp"
#include "src/DistanceHandler.hpp"
#include "src/ObserverHandler.hpp"
#include "src/Robot.hpp"

unsigned long t1;
unsigned long t2;

IMU imu;
DistanceHandler DistanceSensors;
ObserverHandler Observer;
Controller Ctrl;
Robot robot;
 
void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  imu.init();
  DistanceSensors.init();
  DistanceSensors.startContinuous();

  Observer.init();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  
  t1 = millis();

  imu.update(t1);
  DistanceSensors.update();
  robot.update();

  float pBot[3] = {robot.Xbot, robot.Ybot, robot.THbot};
  float vBot[3] = {robot.Xdbot, robot.Ydbot, robot.THdbot};
  float dist[2] = {DistanceSensors.avgDisX, DistanceSensors.avgDisY};

  t2 = millis();
  Observer.update(t2, imu.acc, imu.vel, imu.gyro, imu.ang, pBot, dist);

  Ctrl.update(pBot, vBot, imu.pos, imu.ang);
  delay(1);

}
