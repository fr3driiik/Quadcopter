#include "Kalman.h"
#include <Wire.h>

#define accAddress 0x41 // BMA180
#define gyroAddress 0x68 //ITG3050
//magnetometer HMC5883L
//barometric pressure MS5611
//GPS NEO-6Q

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

// GYRO x = 0x1D, y = 0x1F, z = 0x21

//Angles start at 180 deg
double gyroXangle = 180;
double gyroYangle = 180;
double gyroZangle = 180;

double compAngleX = 180;
double compAngleY = 180;
double compAngleZ = 180;

unsigned long timer;

uint8_t i2cBuffer[2];

long total = 0;
int antal = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  initGyro(); 

  kalmanX.setAngle(180);
  kalmanY.setAngle(180);
  kalmanZ.setAngle(180);
  timer = micros();

}

void loop() {
  double gyroXrate = (double)readGyroX() / 65.5;
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000);

  double gyroYrate = -(double)readGyroY() / 65.5;
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);

  double gyroZrate = -(double)readGyroZ() / 65.6;
  gyroZangle += gyroZrate * ((double)(micros() - timer) / 1000000);

  Serial.print("x: ");
  Serial.print(gyroXrate);
  Serial.print(" deg/s   y: ");
  Serial.print(gyroYrate);
  Serial.print(" deg/s   z: ");
  Serial.print(gyroZrate);
  Serial.print(" deg/s   x: ");
  Serial.print(gyroXangle);
  Serial.print(" deg   y: ");
  Serial.print(gyroYangle);
  Serial.print(" deg   z: ");
  Serial.print(gyroZangle);
  Serial.println(" deg");

  timer = micros();
}

void initGyro(){
  i2cWrite(gyroAddress, 0x16, 0x00); // (STANDARD) sets gyro to +/-250deg/sec and 256Hz LPF
  i2cWrite(gyroAddress, 0x15, 0x4F); //sets gyro at 100Hz sample rate
  i2cWrite(gyroAddress, 0x0C, 0x00, 0xA0); //sets offsetX
  i2cWrite(gyroAddress, 0x0E, 0xFF, 0xEF); //sets offsetY
  i2cWrite(gyroAddress, 0x10, 0x00, 0x00); //sets offsetZ
}

void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data1, uint8_t data2){
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data1);
  Wire.write(data2);
  Wire.endTransmission();
}

uint8_t* i2cRead(uint8_t address, uint8_t registerAddress, uint8_t nbytes){
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, nbytes);
  for(uint8_t i = 0; i < nbytes; i++){
    i2cBuffer[i] = Wire.read();
  }
  Wire.endTransmission();
  return i2cBuffer;
}

int readGyroX(){
  uint8_t* data = i2cRead(gyroAddress, 0x1D, 2);
  return ((data[0] << 8) | data[1]);
}
int readGyroY(){
  uint8_t* data = i2cRead(gyroAddress, 0x1f, 2);
  return ((data[0] << 8) | data[1]);
}
int readGyroZ(){
  uint8_t* data = i2cRead(gyroAddress, 0x21, 2);
  return ((data[0] << 8) | data[1]);
}

