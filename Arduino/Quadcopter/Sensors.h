#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Wire.h"

#define PITCH 1
#define YAW 2
#define ROLL 0
#define OUTPUT_ERRORS
//#define PRINT_UNSCALED_SENSORS
// FORWARD = x (positive) RIGHT = y (positive) UP = z (positive)

// Sensor variables [x, y, z]
extern float gyro[3];
extern float accel[3];
extern float magnetom[3];

void Accel_Init();
void Read_Accel();
void Magn_Init();
void Read_Magn();
void Gyro_Init();
void Read_Gyro();

#endif
