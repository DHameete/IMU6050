#include <Wire.h>
#include "src/IMU.hpp"
#include "src/Controller.hpp"

unsigned long t;

IMU Imu;
Controller Ctrl;
 
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
  Ctrl.update(Imu.pos, Imu.ang);
  delay(1);

}
