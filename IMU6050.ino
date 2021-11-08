#include <Wire.h>
#include "src/IMU.hpp"

unsigned long t;

IMU Imu;

void setup() {
  
  Wire.begin();
  Serial.begin(115200);

  Imu.init();

}

void loop() {
  
  t = millis();

  Imu.update(t);
  delay(50);

}
