#include <Wire.h>
#include "src/IMU.hpp"
#include "src/Controller.hpp"
#include "src/DistanceHandler.hpp"
#include "src/ObserverHandler.hpp"

unsigned long t;
unsigned long t_prev;

IMU Imu;
DistanceHandler DistanceSensors;
ObserverHandler Observer;
Controller Ctrl;
 
void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Imu.init();
  DistanceSensors.init();
  DistanceSensors.startContinuous();

  Observer.init();

  digitalWrite(LED_BUILTIN, LOW);
  t_prev = millis() - 4;
}

void loop() {
  

  Imu.update(t);
  DistanceSensors.update();

  t = millis();
  float dt = (t - t_prev)/1000;
  float pBot[2] = {0, 0};
  float dist[2] = {DistanceSensors.avgDisX, DistanceSensors.avgDisY};

  Serial.print(Imu.ang[2]);
  Serial.print(", ");
  Serial.print(Imu.gyro[2]);
  Serial.print(" || ");

  Observer.update(dt, Imu.acc, Imu.vel, Imu.gyro, Imu.ang, pBot, dist);

  Serial.println(dt);
  t_prev = t;

  Ctrl.update(Imu.pos, Imu.ang);
  delay(1);

}
