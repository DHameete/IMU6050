
#include "IMU.hpp"

IMU::IMU(){}

IMU::~IMU(){}

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

  initOffset();

}

void IMU::initOffset() {

  // Get and calculate average offset
  uint16_t n = 1;
  while(n < N_OFF) {

    getValues();
    
    // Calculate acceleration offset
    accOff[0] += (accRaw[0] - accOff[0]) / n;
    accOff[1] += (accRaw[1] - accOff[1]) / n;
    accOff[2] += (accRaw[2] - accOff[2]) / n;

    // Calculate gyro offset
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

    // Get and calculate temperature
    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }
}

void IMU::filterAccGyr() {

  // Center and filter acceleration
  for (uint8_t ii = 0; ii < 3; ii++) {
    acc[ii] = accRaw[ii] * (abs(accRaw[ii]) > ACC_FILTER) - accOff[ii];
    gyro[ii] = gyroRaw[ii] * (abs(gyroRaw[ii]) > GYRO_FILTER) - gyroOff[ii];
  }
}

void IMU::expMovAvg() {

  // Average accelerations
  for (uint8_t ii = 0; ii < 3; ii++) {
    accAccum[ii] = (alpha * acc[ii]) + (1 - alpha) * accAccum[ii];
    acc[ii] = accAccum[ii];
  }
}

void IMU::calcVelocity() {

  for (uint8_t ii = 0; ii < 3; ii++) {
    float sumdiff = 0.0F;
    for (uint8_t jj = 0; jj < c_len; jj++) {
      sumdiff += abs(prevAcc[ii][jj] - prevAcc[ii][jj-1]);
    }
    
    // *vel = *vel + acc * dt / 1000; // ms to s
    if (sumdiff < VEL_FILTER && abs(acc[ii]) < 1000*VEL_FILTER) {
      vel[ii] = 0;
    } else {
      vel[ii] = vel[ii] + acc[ii] * dt / 1000; // ms to s
    }
  }
}


void IMU::calcPosition() {

  for (uint8_t ii = 0; ii < 3; ii++) {

    for (uint8_t jj = 0; jj < v_len-1; jj++) {
      prevVel[ii][jj] = prevVel[ii][jj+1];
    }
    prevVel[ii][v_len-1] = vel[ii];

    float sumVel = 0.0F;
    for (uint8_t kk = 0; kk < v_len; kk++){
      sumVel += prevVel[ii][kk];
    }
    sumVel /= v_len;

    if (abs(sumVel) < OUTPUT_FILTER){
      return;
    } else {
      pos[ii] = pos[ii] + vel[ii] * dt / 1000;
    }
  }
}

void IMU::calcAngle() {

  for (uint8_t ii = 0; ii < 3; ii++) {

    for (uint8_t jj = 0; jj < c_len-1; jj++) {
      prevGyro[ii][jj] = prevGyro[ii][jj+1];
    }
    prevGyro[ii][c_len-1] = gyro[ii];

    float sumdiff = 0.0F;
    for (uint8_t kk = 0; kk < c_len; kk++) {
      sumdiff += abs(prevGyro[ii][kk] - prevGyro[ii][kk-1]);
    }
    
    if (sumdiff < 0.01) {
      return;
    } else {
      ang[ii] = ang[ii] + gyro[ii] * dt / 1000; // ms to s
    }
    ang[ii] = fmod(360 + fmod(ang[ii], 360), 360);
  }
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
  Serial.print(pos[0]);
  Serial.print(",");
  Serial.print(pos[1]);
  
  Serial.print(",");
  Serial.print(gyro[2]*DEG_TO_RAD);
  Serial.print(",");
  Serial.print(ang[2]*DEG_TO_RAD);

  // for i = 1:10:
  //   Serial.print(logger[i]);
  // logger[i] = *pointer_to_value_i

}

void IMU::zeroAcc() {

  for (uint8_t ii = 0; ii < 3; ii++) {
    // Add newest value
    biasAcc[ii][b_indx] = acc[ii];

    // Calculate sum of differences
    float sumdiff = 0.0F;
    float sumBiasAcc = biasAcc[ii][0];
    for (uint8_t jj = 1; jj < b_len; jj++) {
      sumdiff += abs(biasAcc[ii][jj] - biasAcc[ii][jj-1]);
      sumBiasAcc += biasAcc[ii][jj];
    }

    // Calculate bias
    if (sumdiff < 6){
      accOff[ii] = (sumBiasAcc / b_len);
    }
  }

  // Increase index
  b_indx++;
  b_indx = (b_indx % b_len);
}

void IMU::update(unsigned long t) {
  
  dt = t - t_prev;

  getValues();
  
  filterAccGyr();
  expMovAvg();
  
  calcVelocity();
  calcPosition();
  calcAngle();

  zeroAcc();

  #ifdef DEBUG
    outputValues(t);
  #endif

  t_prev = t;
}