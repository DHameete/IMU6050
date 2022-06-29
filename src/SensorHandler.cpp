
#include "SensorHandler.hpp"

SensorHandler::SensorHandler(){

  _XSHUT = XSHUTL;
  _OFFX = OFFL;
  
  _side = LEFT;

  pinMode(_XSHUT, OUTPUT);
  digitalWrite(_XSHUT, LOW);
 
}

SensorHandler::SensorHandler(SENSOR_SIDE side){

  if (side == LEFT) {
    _XSHUT = XSHUTL;
    _OFFX = OFFL;
  } else {
    _XSHUT = XSHUTR;
    _OFFX = OFFR;
  }
  
  _side = side;

  pinMode(_XSHUT, OUTPUT);
  digitalWrite(_XSHUT, LOW);
  
}

SensorHandler::~SensorHandler() {}

void SensorHandler::init() {

  digitalWrite(_XSHUT, HIGH);

  distanceSensor.setTimeout(500);
  Serial.println("Start Init sensor");
  if (!distanceSensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    // while (1);
  }

  address = 50 + _XSHUT;
  distanceSensor.setAddress(address);

  distanceSensor.setDistanceMode(VL53L1X::Short);
  distanceSensor.setMeasurementTimingBudget(12000);//20000);

  distanceSensor.setROISize(_sizeROI[0], _sizeROI[1]);
  distanceSensor.setROICenter(_center[_zone]);

}

void SensorHandler::startContinuous() {

  distanceSensor.startContinuous(20);

}

void SensorHandler::read() {
  // Check for new data
  if(distanceSensor.dataReady()) {
    // Read new distance 
    _distanceRaw[_zone] = (float)distanceSensor.read(false) / 1000;

    // Set next ROI center
    _zone++;
    _zone = _zone % num;
    distanceSensor.setROICenter(_center[_zone]);

    // New measurements ready
    if (_zone == 0) {
      _flag = true;
    }

  }
}

void SensorHandler::movMean() {

  // Set new measurement
  for (uint8_t j = 0; j < num; j++) {
    _measArray[_measCount % measLen][j] = _distanceRaw[j];
  }
  _measCount++;

  // Calculate mean values
  float sumMeas[num] = { 0.0F };
  for (uint8_t j = 0; j < num; j++) {
    for (uint8_t i = 0; i < measLen; i++) {
      sumMeas[j] += _measArray[i][j];
    }
    sumMeas[j] /= measLen;
    distance[j] = sumMeas[j];
  }

}

void SensorHandler::calcLine() {

  float x[num];
  float y[num];

  float x_avg = 0;
  float y_avg = 0;

  // Calculating x,y-values
  for(uint8_t r = 0; r < num; r++)
  {
    x[r] = distance[r] * cos((90 - (r * 4.5 - 13.5)) * M_PI / 180);
    y[r] = distance[r] * sin((90 - (r * 4.5 - 13.5)) * M_PI / 180);

    x_avg += x[r];
    y_avg += y[r];
  }
  x_avg /= num;
  x_avg += _OFFX;
  y_avg /= num;

  // Calculating distance and slope
  ang = 0;
  for (uint8_t i = 1; i < num; i++) {
    if(_side == LEFT) {
      ang += atan((y[i] - y[i-1]) / (x[i] - x[i-1]));
    }
    else {
      ang += atan((y[num-i] - y[num-i-1]) / (x[num-i] - x[num-i-1]));
    }
  }

  ang /= (num-1);
  avgDis = y_avg - tan(ang) * x_avg;
}

void SensorHandler::update() {

  read();

  if (_flag) {
    movMean();
    calcLine();
    _flag = false;
  }
}

void SensorHandler::print() {
  
  for (uint8_t i = 0; i < num; i++) {
    Serial.print(distance[i]);
    Serial.print(", ");
  }

}