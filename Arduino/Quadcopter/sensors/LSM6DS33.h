#ifndef LSM6DS33_H
#define LSM6DS33_H

#define LSM6DS33_ADDRESS 0b1101011b

#include <Arduino.h>
#include <Wire.h>

 namespace LSM6DS33 {
 	int16_t readGyro();
 	int16_t readAcc();
 	void setFIFOMode(uint8_t mode);
 }

#endif