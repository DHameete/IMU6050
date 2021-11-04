#include <Wire.h>
#include "src/MPU6050.h"
// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1

// Pin definitions
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gyrox, gyroy, gyroz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
float gyroOff[3], accelOff[3]; // Offset corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output 
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;
void setup()
{
  Wire.begin();
  Serial.begin(9600);
 
   // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU6050 is online...");
    
    mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
    Serial.println("Pass Selftest!");  
      
    mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

   }
   else
   {
    Serial.print("Could not connect to MPU6050: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
   }

  }

  aRes=mpu.getAres();
  gRes=mpu.getGres();


  int n = 1;
  while(n < 3000) {
      mpu.readAccelData(accelCount);  // Read the x/y/z adc values
      accelOff[0] += ((float)accelCount[0]*aRes - accelOff[0]) / n;
      accelOff[1] += ((float)accelCount[1]*aRes - accelOff[1]) / n;
      accelOff[2] += ((float)accelCount[2]*aRes - accelOff[2]) / n;

      mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
      gyroOff[0] += ((float)gyroCount[0]*gRes - gyroOff[0]) / n;
      gyroOff[1] += ((float)gyroCount[1]*gRes - gyroOff[1]) / n;
      gyroOff[2] += ((float)gyroCount[2]*gRes - gyroOff[2]) / n;

      n++;
  }

  Serial.print("accelOff: x = "); Serial.print(accelOff[0] * 1000, 1); 
  Serial.print(", accelOff: y = "); Serial.print(accelOff[1] * 1000, 1); 
  Serial.print(", accelOff: z = "); Serial.print(accelOff[2] * 1000, 1); 
  Serial.println(" mg"); 

  Serial.print("gyroOff: x = "); Serial.print(gyroOff[0], 1); 
  Serial.print(", gyroOff: y = "); Serial.print(gyroOff[1], 1); 
  Serial.print(", gyroOff: z = "); Serial.print(gyroOff[2], 1); 
  Serial.println(" degrees/sec"); 

}

void loop()
{  
  // If data ready bit set, all data registers have new data
  if(mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt

    mpu.readAccelData(accelCount);  // Read the x/y/z adc values
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelOff[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelOff[1];   
    az = (float)accelCount[2]*aRes - accelOff[2];  
   
    mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
 
    // Calculate the gyro value into actual degrees per second
    gyrox = (float)gyroCount[0]*gRes - gyroOff[0];  // get actual gyro value, this depends on scale being set
    gyroy = (float)gyroCount[1]*gRes - gyroOff[1];  
    gyroz = (float)gyroCount[2]*gRes - gyroOff[2];   

    tempCount = mpu.readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
   }
   
    uint32_t deltat = millis() - count;


    if(deltat > 500) {
 
    // Print acceleration values in milligs!
    Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg "); 
    Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg "); 
    Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg"); 
 
    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); Serial.print(gyrox, 1); Serial.print(" degrees/sec "); 
    Serial.print("Y-gyro rate: "); Serial.print(gyroy, 1); Serial.print(" degrees/sec "); 
    Serial.print("Z-gyro rate: "); Serial.print(gyroz, 1); Serial.println(" degrees/sec"); 
      
   // Print temperature in degrees Centigrade      
    Serial.print("Temperature is ");  Serial.print(temperature, 2);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    Serial.println("");
        
    count = millis();
    }

}