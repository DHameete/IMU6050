#include <Wire.h>
#include "src/IMU.hpp"
#include "src/Controller.hpp"
#include "src/DistanceHandler.hpp"

unsigned long t;

IMU Imu;
DistanceHandler distanceSensors;
Controller Ctrl;
 
void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Imu.init();
  distanceSensors.init();
  distanceSensors.startContinuous();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  
  t = millis();

  Imu.update(t);
  distanceSensors.update();
  Ctrl.update(Imu.pos, Imu.ang);
  delay(1);

}
