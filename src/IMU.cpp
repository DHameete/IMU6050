
#include "IMU.hpp"

IMU::IMU(){}

IMU::~IMU() {}

void IMU::init() {

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050

  
  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  

    if (false) {
      float SelfTest[6];               // Gyro and accelerometer self-test sensor output
    
      mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
      Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

      if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
        Serial.println("Pass Selftest!");  
      }
    }

    mpu.calibrateMPU6050(gyroBias, accBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    mpu.initMPU6050();

  }
  else
  {
  Serial.print("Could not connect to MPU6050: 0x");
  Serial.println(c, HEX);
  while(1) ; // Loop forever if communication doesn't happen
  }

  aRes=mpu.getAres();
  gRes=mpu.getGres();

  int n = 1;
  while(n < 3000) {
      mpu.readAccelData(accelCount);  // Read the x/y/z adc values
      accOff[0] += ((float)accelCount[0]*aRes - accOff[0]) / n;
      accOff[1] += ((float)accelCount[1]*aRes - accOff[1]) / n;
      accOff[2] += ((float)accelCount[2]*aRes - accOff[2]) / n;

      mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
      gyroOff[0] += ((float)gyroCount[0]*gRes - gyroOff[0]) / n;
      gyroOff[1] += ((float)gyroCount[1]*gRes - gyroOff[1]) / n;
      gyroOff[2] += ((float)gyroCount[2]*gRes - gyroOff[2]) / n;

      n++;
  }

  t_prev = millis();

}

void IMU::getValues() {

  // If data ready bit set, all data registers have new data
  if(mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt

    mpu.readAccelData(accelCount);  // Read the x/y/z adc values
    
    // Now we'll calculate the accleration value into actual g's
    accZ = -((float)accelCount[0]*aRes - accOff[0]);  // get actual g value, this depends on scale being set
    accY = (float)accelCount[1]*aRes - accOff[1];   
    accX = (float)accelCount[2]*aRes - accOff[2];  
   
    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
 
    // Calculate the gyro value into actual degrees per second
    gyroZ = -((float)gyroCount[0]*gRes - gyroOff[0]);  // get actual gyro value, this depends on scale being set
    gyroY = (float)gyroCount[1]*gRes - gyroOff[1];  
    gyroX = (float)gyroCount[2]*gRes - gyroOff[2];   

    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

}

void IMU::calcVelocity(float* prevAcc, float acc, float* vel) {
  
  for (uint8_t i = 0; i < c_len-1; i++) {
    prevAcc[i] = prevAcc[i+1];
  }
  prevAcc[c_len-1] = acc;

  float sumdiff = 0.0F;
  for (uint8_t i = 0; i < c_len; i++) {
    sumdiff += abs(prevAcc[i] - prevAcc[i-1]);
  }
  
  if (sumdiff < 0.01) {
    *vel = 0;
  } else {
    *vel = *vel + acc * dt / 1000; // ms to s
  }
}

void IMU::calcDirection(float* prevVel, float vel, float* out) {

  for (uint8_t i = 0; i < v_len-1; i++) {
    prevVel[i] = prevVel[i+1];
  }
  prevVel[v_len-1] = vel;

  float sumVel = 0.0F;
  for (uint8_t i = 0; i < v_len; i++){
    sumVel += prevVel[i];
  }
  sumVel /= v_len;
  if (abs(sumVel) > OUTPUT_FILTER){
    *out = (OUTPUT_FILTER < sumVel) - (sumVel < OUTPUT_FILTER);
  } else {
    *out = 0;
  }
}

void IMU::calcAngle(float* prevVel, float vel, float* angle) {

  for (uint8_t i = 0; i < c_len-1; i++) {
    prevVel[i] = prevVel[i+1];
  }
  prevVel[c_len-1] = vel;

  float sumdiff = 0.0F;
  for (uint8_t i = 0; i < c_len; i++) {
    sumdiff += abs(prevVel[i] - prevVel[i-1]);
  }
  
  if (sumdiff < 0.01) {
    return;
  } else {
    *angle = *angle + vel * dt / 1000; // ms to s
  }
  *angle = fmod(360 + fmod(*angle, 360), 360);
}

void IMU::calcTurn(float rotd, float* out) {

  if (abs(rotd) > 50){
    *out = (50 < rotd) - (rotd < 50);
  } else {
    *out = 0;
  }
}

void IMU::outputValues(unsigned long t) {

  Serial.print(t); 

  Serial.print(",");
  Serial.print(X);//accX);
  Serial.print(",");
  Serial.print(velX);

  Serial.print(",");
  Serial.print(Y);//accY);
  Serial.print(",");
  Serial.print(velY);
  
  Serial.print(",");
  Serial.print(gyroZ*DEG_TO_RAD);
  Serial.print(",");
  Serial.print(th*DEG_TO_RAD);

  Serial.print(",");
  Serial.print(uX);
  Serial.print(",");
  Serial.print(uY);
  Serial.print(",");
  Serial.println(uTH);

}

void IMU::calculateRotations() {

  pitch = atan2(accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
  roll = atan2(accY, sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
  yaw = atan2(accZ, sqrt(accX*accX + accY*accY)) * RAD_TO_DEG;

}

void IMU::filterAccel() {
  // Center and filter acceleration
  
  // X
  accX -= x_off;
  accX *= (abs(accX) > ACC_FILTER);
 
  // Y
  accY -= y_off;
  accY *= (abs(accY) > ACC_FILTER);

  // Z
  accZ -= z_off;
  accZ *= (abs(accZ) > ACC_FILTER);

}

void IMU::filterGyro() {
  // Center and filter velocity

  // Pitch
  gyroX -= gX_off;
  gyroX *= (abs(gyroX) > GYRO_FILTER);
  
  // Roll
  gyroY -= gY_off;
  gyroY *= (abs(gyroY) > GYRO_FILTER);

  // Yaw
  gyroZ -= gZ_off;
  gyroZ *= (abs(gyroZ) > GYRO_FILTER);

}

void IMU::calculateDerivative() {

  calcVelocity(prevAccX, accX, &velX);
  calcDirection(prevVelX, velX, &outX);
  calcVelocity(prevAccY, accY, &velY);
  calcDirection(prevVelY, velY, &outY);

  calcAngle(prevGyroZ, gyroZ, &th);
  calcTurn(gyroZ, &outTh);
}


void IMU::update(unsigned long t) {
  
  dt = t - t_prev;

  getValues();

  filterAccel();
  filterGyro();

  // uint8_t cnt = (t/4000)%4;
  // switch (cnt) {
  //   case 0:
  //     gyroZ = 0;
  //     break;
  //   case 1:
  //     gyroZ = 1;
  //     break;
  //   case 2:
  //     gyroZ = 0;
  //     break;
  //   case 3:
  //     gyroZ = -2;
  //     break;
  // }
  // gyroZ *= 22.5;

  calculateDerivative();

  // gyroZ = 0;
  // uint8_t cnt2 = (t/14000)%3;
  // switch (cnt2) {
  //   case 0:
  //     th = 0;
  //     break;
  //   case 1:
  //     th = -90;
  //     break;
  //   case 2:
  //     th = 89;
  //     break;
  // }


  gyroZ = 0;
  uint8_t cnt2 = (t/7000)%8;
  switch (cnt2) {
    case 0:
      Xref = 0;
      Yref = 0;
      th = 0;
      break;
    case 1:
      Xref = 1;
      Yref = 0;
      th = 0;
      break;
    case 2:
      Xref = 1;
      Yref = 0;
      th = 90;
      break;
    case 3:
      Xref = 1;
      Yref = 1;
      th = 90;
      break;
    case 4:
      Xref = 1;
      Yref = 1;
      th = 180;
      break;
    case 5:
      Xref = 0;
      Yref = 1;
      th = 180;
      break;
    case 6:
      Xref = 0;
      Yref = 1;
      th = -90;
      break;
    case 7:
      Xref = 0;
      Yref = 0;
      th = -90;
      break;
  }

// gyroZ = 0;
  // uint8_t cnt3 = (t/14000)%4;
  // uint8_t cnt4 = (t/7000)%2;
  // th = cnt3 * 90;
  // Xd = 1 + cnt4 * 1;


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

  // Velocity vector based
  // X = X - R * sin(th*DEG_TO_RAD) * gyroZ*DEG_TO_RAD * dt / 1000;
  // Y = Y + R * cos(th*DEG_TO_RAD) * gyroZ*DEG_TO_RAD * dt / 1000;

  // Reference angle based
  X = R * cos(th*DEG_TO_RAD);
  Y = R * sin(th*DEG_TO_RAD);

  // Calculate cartesian errors 
  // float deltaX = X - Xbot;
  // float deltaY = Y - Ybot;
    
  // float eX = -(deltaY * cos(THbot) - deltaX * sin(THbot));
  // float eY = (deltaY * sin(THbot) + deltaX * cos(THbot));
  
  // float eXd = (eX - eXprev) * dt;
  // float eYd = (eY - eYprev) * dt;

  // Calculate polar errors
  float Rref = sqrt(X*X + Y*Y);
  
  float Xbot_c = Xbot - Xref;
  float Ybot_c = Ybot - Yref;

  float Rbot = sqrt(Xbot_c*Xbot_c + Ybot_c*Ybot_c);
  float eR = Rref - Rbot;

  float PHIref = atan2(Y,X);
  float PHIbot = atan2(Ybot_c,Xbot_c);
  float ePHI = PHIref - PHIbot;

  PHIbot = fmod(2*M_PI + fmod(PHIbot, 2*M_PI), 2*M_PI);  

  // Shortest angle
  if (ePHI > M_PI) {
    ePHI -= 2*M_PI;
  } else if (ePHI < -M_PI) {
    ePHI += 2*M_PI;
  }

  float KpPHI_ePHI = max(min(KpPHI * ePHI, 5),-5);//M_PI/2), -M_PI/2);
  // float KpPHI_ePHI = KpPHI * ePHI;

  float deltaX = KpR * eR * cos(PHIbot) - KpPHI_ePHI * sin(PHIbot);
  float deltaY = KpR * eR * sin(PHIbot) + KpPHI_ePHI * cos(PHIbot);

  float eX = -(deltaY * cos(THbot) - deltaX * sin(THbot));
  float eY = (deltaY * sin(THbot) + deltaX * cos(THbot));


  float Rdbot   =  Xdbot * cos(PHIbot) + Ydbot * sin(PHIbot);
  float PHIdbot = -Xdbot * sin(PHIbot) + Ydbot * cos(PHIbot);

  float deltaXd = KdR * Rdbot * cos(PHIbot) - KdPHI * PHIdbot * sin(PHIbot);
  float deltaYd = KdR * Rdbot * sin(PHIbot) + KdPHI * PHIdbot * cos(PHIbot);

  float eXd = -(deltaYd * cos(THbot) - deltaXd * sin(THbot));
  float eYd = (deltaYd * sin(THbot) + deltaXd * cos(THbot));

  // Angle error
  // float eTH = th*DEG_TO_RAD - THbot;
  float eTH = PHIbot - THbot;
  
  // Shortest angle
  if (eTH > M_PI) {
    eTH -= 2*M_PI;
  } else if (eTH < -M_PI) {
    eTH += 2*M_PI;
  }

  uX = eX + eXd;// - R*(gyroZ*DEG_TO_RAD);
  uX = max(min(uX,2),-2);
  uY = eY + eYd;
  uY = max(min(uY,2),-2);
  uTH = KpTH * eTH + KdTH * THdbot;// + gyroZ*DEG_TO_RAD;
  uTH = max(min(uTH,8),-8);

  #ifdef DEBUG
    outputValues(t);
  #else
    // messenger.update(0,(R*(gyroZ*DEG_TO_RAD))*32767/2);
    messenger.update(0, uX*32767/2);
    delay(1);
    // messenger.update(1,0);
    messenger.update(1, uY*32767/2);
    delay(1);
    messenger.update(2, uTH*32767/8);
    delay(1);
  #endif

  t_prev = t;

}