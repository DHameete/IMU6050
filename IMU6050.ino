#include <Wire.h>
#include "src/IMU.hpp"

unsigned long t;

IMU Imu;

void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Imu.init();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  
  t = millis();

  Imu.update(t);
  delay(1);

}
