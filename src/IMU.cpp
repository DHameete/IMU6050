
#include "IMU.hpp"

IMU::IMU(){}

IMU::~IMU() {}

void IMU::init() {

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  

    mpu.calibrateMPU6050(gyroBias, accBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    mpu.initMPU6050();

  }
  else {
    Serial.print("Could not connect to MPU6050: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  aRes = mpu.getAres();
  gRes = mpu.getGres();

  uint16_t n = 1;
  while(n < 3000) {

    getValues();
    
    accOff[0] += (arrRaw[0] - accOff[0]) / n;
    accOff[1] += (arrRaw[1] - accOff[1]) / n;
    accOff[2] += (arrRaw[2] - accOff[2]) / n;

    gyroOff[0] += (gyro[0] - gyroOff[0]) / n;
    gyroOff[1] += (gyro[1] - gyroOff[1]) / n;
    gyroOff[2] += (gyro[2] - gyroOff[2]) / n;

    n++;
  }

  t_prev = millis();

}

void IMU::getValues() {

  // If data ready bit set, all data registers have new data
  if(mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt

    mpu.readAccelData(accelCount);  // Read the x/y/z adc values
    
    // Get actual g value, this depends on scale being set
    accRaw[0] =  9.81*((float) accelCount[2]*aRes);  
    accRaw[1] =  9.81*((float) accelCount[1]*aRes);   
    accRaw[2] = -9.81*((float) accelCount[0]*aRes);
   
    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
 
    // Get actual gyro value, this depends on scale being set
    gyroRaw[0] =   (float) gyroCount[2]*gRes;   
    gyroRaw[1] =   (float) gyroCount[1]*gRes;  
    gyroRaw[2] = -((float) gyroCount[0]*gRes);  

    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }

}

void IMU::calcVelocity(float* prevAcc, float acc, float* vel) {

  float sumdiff = 0.0F;
  for (uint8_t i = 0; i < c_len; i++) {
    sumdiff += abs(prevAcc[i] - prevAcc[i-1]);
  }
  
  // *vel = *vel + acc * dt / 1000; // ms to s
  if (sumdiff < VEL_FILTER && abs(acc) < 1000*VEL_FILTER) {
    *vel = 0;
  } else {
    *vel = *vel + acc * dt / 1000; // ms to s
  }
}


void IMU::calcPosition(float* prevVel, float vel, float* pos) {

  for (uint8_t i = 0; i < v_len-1; i++) {
    prevVel[i] = prevVel[i+1];
  }
  prevVel[v_len-1] = vel;

  float sumVel = 0.0F;
  for (uint8_t i = 0; i < v_len; i++){
    sumVel += prevVel[i];
  }
  sumVel /= v_len;
  if (abs(sumVel) < OUTPUT_FILTER){
    return;
  } else {
    *pos = *pos + vel * dt / 1000;
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

void IMU::outputValues(unsigned long t) {

  Serial.print(t); 

  // Serial.print(",");
  // Serial.print(accXraw);
  // Serial.print(",");
  // Serial.print(accYraw);
  // Serial.print(",");
  // Serial.print(accZraw);


  Serial.print(",");
  Serial.print(acc[0]);
  Serial.print(",");
  Serial.print(acc[1]);
  // Serial.print(",");
  // Serial.print(accZ);

  Serial.print(",");
  Serial.print(vel[0]);
  Serial.print(",");
  Serial.print(vel[1]);

  Serial.print(",");
  Serial.print(Xref);
  Serial.print(",");
  Serial.print(Yref);
  
  Serial.print(",");
  Serial.print(gyro[2]*DEG_TO_RAD);
  Serial.print(",");
  Serial.print(ang[2]*DEG_TO_RAD);

  // for i = 1:10:
  //   Serial.print(logger[i]);
  // logger[i] = *pointer_to_value_i

}

void IMU::filterAccel() {
  // Center and filter acceleration
  
  // X
  accX = accXraw * (abs(accXraw) > ACC_FILTER) - accOff[2];
 
  // Y
  accY = accYraw * (abs(accYraw) > ACC_FILTER) - accOff[1];

  // Z
  accZ = accZraw * (abs(accZraw) > ACC_FILTER) - accOff[0];

}


void IMU::zeroingAccel(float* biasAcc, float* acc, uint8_t ind) {

  // Remove oldest value
  // sumBiasAcc -= biasAcc[b_indx];

  // Add newest value
  biasAcc[b_indx] = *acc;
  // sumBiasAcc += biasAcc[b_indx];

  // Calculate sum of differences
  float sumdiff = 0.0F;
  float sumBiasAcc = biasAcc[0];
  for (uint8_t i = 1; i < b_len; i++) {
    sumdiff += abs(biasAcc[i] - biasAcc[i-1]);
    sumBiasAcc += biasAcc[i];
  }

  // Calculate bias
  if (sumdiff < 6){
    accOff[ind] = (sumBiasAcc / b_len);
  }

}

void IMU::movingAverage_exponential(float* accum, float* acc) {

  // Average accelerations
  *accum = (alpha * *acc) + (1 - alpha) * *accum;
  *acc = *accum;
}



void IMU::filterGyro() {
  // Center and filter velocity

  // Pitch
  gyro[0] *= (abs(gyro[0]]) > GYRO_FILTER);
  
  // Roll
  gyro[1] *= (abs(gyro[1]) > GYRO_FILTER);

  // Yaw
  gyroZ[2] *= (abs(gyro[2]) > GYRO_FILTER);

}

void IMU::calculateDerivative() {

  calcVelocity(prevAccXflt, acc[0], &velX);
  calcPosition(prevVelX, velX, &Xref);
  calcVelocity(prevAccYflt, accY, &velY);
  calcPosition(prevVelY, velY, &Yref);

  calcAngle(prevGyroZ, gyroZ, &th);

}


void IMU::update(unsigned long t) {
  
  dt = t - t_prev;

  getValues();

  filterAccel();
  filterGyro();

  movingAverage_exponential(&accAccum, acc[0]);
  movingAverage_exponential(&accYaccumulator, acc[1]);
  movingAverage_exponential(&accZaccumulator, acc[2]);

  calculateDerivative();

  zeroingAccel(biasAccX, acc[0], 2);
  zeroingAccel(biasAccY, acc[1], 1);
  zeroingAccel(biasAccZ, acc[2], 0);


  // Increase index
  b_indx++;
  b_indx = (b_indx % b_len);

  calculateReference();
  controlOutput();
}

// void IMU::calculateReference() {}

//   while(Serial.available()){
//     int8_t xVal = (int8_t) Serial.read();
//     Xbot = (float) xVal/40;
//     int8_t yVal = (int8_t) Serial.read();
//     Ybot = (float) yVal/40;
//     int8_t thVal = (int8_t) Serial.read();
//     THbot = (float) (thVal*M_PI)/127+M_PI;

//     int8_t xdVal = (int8_t) Serial.read();
//     Xdbot = (float) xdVal*16/1000;
//     int8_t ydVal = (int8_t) Serial.read();
//     Ydbot = (float) ydVal*16/1000;
//     int8_t thdVal = (int8_t) Serial.read();
//     THdbot = (float) (thdVal*64)/1000;
//   }
 
//   // Reference angle based
//   X = R * cos(th*DEG_TO_RAD);
//   Y = R * sin(th*DEG_TO_RAD);

//   float Rref = sqrt(X*X + Y*Y);
//   float PHIref = atan2(Y,X);

//   float Xbot_c = Xbot - Xref;
//   float Ybot_c = Ybot - Yref;

//   float Rbot = sqrt(Xbot_c*Xbot_c + Ybot_c*Ybot_c);
//   float PHIbot = atan2(Ybot_c,Xbot_c);

//   // Calculate polar errors
//   float eR = Rref - Rbot;
//   float ePHI = PHIref - PHIbot;

//   PHIbot = fmod(2*M_PI + fmod(PHIbot, 2*M_PI), 2*M_PI);  

//   // Shortest angle
//   if (ePHI > M_PI) {
//     ePHI -= 2*M_PI;
//   } else if (ePHI < -M_PI) {
//     ePHI += 2*M_PI;
//   }

//   float KpPHI_ePHI = max(min(KpPHI * ePHI, M_PI/2), -M_PI/2);//5),-5);//M_PI/2), -M_PI/2);
//   // float KpPHI_ePHI = KpPHI * ePHI;

//   float deltaX = KpR * eR * cos(PHIbot) - KpPHI_ePHI * sin(PHIbot);
//   float deltaY = KpR * eR * sin(PHIbot) + KpPHI_ePHI * cos(PHIbot);

//   float eX = -(deltaY * cos(THbot) - deltaX * sin(THbot));
//   float eY = (deltaY * sin(THbot) + deltaX * cos(THbot));


//   float Rdbot   =  Xdbot * cos(PHIbot) + Ydbot * sin(PHIbot);
//   float PHIdbot = -Xdbot * sin(PHIbot) + Ydbot * cos(PHIbot);

//   float deltaXd = KdR * Rdbot * cos(PHIbot) - KdPHI * PHIdbot * sin(PHIbot);
//   float deltaYd = KdR * Rdbot * sin(PHIbot) + KdPHI * PHIdbot * cos(PHIbot);

//   float eXd = -(deltaYd * cos(THbot) - deltaXd * sin(THbot));
//   float eYd = (deltaYd * sin(THbot) + deltaXd * cos(THbot));

//   // Angle error
//   // float eTH = th*DEG_TO_RAD - THbot;
//   float eTH = PHIbot - THbot;
  
//   // Shortest angle
//   if (eTH > M_PI) {
//     eTH -= 2*M_PI;
//   } else if (eTH < -M_PI) {
//     eTH += 2*M_PI;
//   }
// }

// void IMU::controlOutput() {

//   uX = eX + eXd;// - R*(gyroZ*DEG_TO_RAD);
//   uX = max(min(uX,2),-2);
//   uY = eY + eYd;
//   uY = max(min(uY,2),-2);
//   uTH = KpTH * eTH + KdTH * THdbot;// + gyroZ*DEG_TO_RAD;
//   uTH = max(min(uTH,8),-8);

//   #ifdef DEBUG
//     outputValues(t);
//   #else
//     // messenger.update(0,(R*(gyroZ*DEG_TO_RAD))*32767/2);
//     messenger.update(0, uX*32767/2);
//     delay(1);
//     // messenger.update(1,0);
//     messenger.update(1, uY*32767/2);
//     delay(1);
//     messenger.update(2, uTH*32767/8);
//     delay(1);
//   #endif

//   t_prev = t;

// }