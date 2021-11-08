
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

  // M5.MPU6886.getGyroData(&gyroX,&gyroY,&gyroZ);
  // M5.MPU6886.getAccelData(&accX,&accY,&accZ);
  // M5.MPU6886.getAhrsData(&pitch,&roll,&yaw);
  // M5.MPU6886.getTempData(&temp);

}

void IMU::calcVelocity(float* prevAcc, float acc, float* vel) {
  
  for (int i = 0; i < c_len-1; i++) {
    prevAcc[i] = prevAcc[i+1];
  }
  prevAcc[c_len-1] = acc;

  float sumdiff = 0.0F;
  for (int i = 0; i < c_len; i++) {
    sumdiff += abs(prevAcc[i] - prevAcc[i-1]);
  }
  
  if (sumdiff < 0.01) {
    *vel = 0;
  } else {
    *vel = *vel + acc * dt / 1000; // ms to s
  }
}

void IMU::calcDirection(float* prevVel, float vel, float* out) {
  for (int i = 0; i < v_len-1; i++) {
    prevVel[i] = prevVel[i+1];
  }
  prevVel[v_len-1] = vel;

  float sumVel = 0.0F;
  for (int i = 0; i < v_len; i++){
    sumVel += prevVel[i];
  }
  sumVel /= v_len;
  if (abs(sumVel) > OUTPUT_FILTER){
    *out = (OUTPUT_FILTER < sumVel) - (sumVel < OUTPUT_FILTER);
  } else {
    *out = 0;
  }
}

void IMU::calcAngle(float* prevStates, float vel, float* angle) {
  
  float sumdiff = 0;
  for (int i = 0; i < c_len; i++) {
    sumdiff += abs(prevStates[i] - prevStates[i-1]);
  }
  
  if (sumdiff < 0.01) {
    return;
  } else {
    *angle = *angle + vel * dt / 1000; // ms to s
  }
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
  Serial.print(accX);
  Serial.print(",");
  Serial.print(velX);

  Serial.print(",");
  Serial.print(accY);
  Serial.print(",");
  Serial.print(velY);
  
  Serial.print(",");
  Serial.print(gyroZ);
  Serial.print(",");
  Serial.println(th);

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

  cmpTh[q % c_len] = gyroZ;
  calcAngle(cmpTh, gyroZ, &th);
  calcTurn(gyroZ, &outTh);
}


void IMU::update(unsigned long t) {
  
  dt = t - t_prev;

  getValues();

  filterAccel();
  filterGyro();

  calculateDerivative();

  outputValues(t);

  q++;
  t_prev = t;

}