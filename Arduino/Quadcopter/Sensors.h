#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Wire.h"

// FORWARD = x (positive) RIGHT = y (positive) UP = z (positive)

// Sensor I2C addresses
#define ACCEL_ADDRESS 0x41
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int16_t) 0x68) // 0x68 = 0xD0 / 2
#define GYRO_GAIN 0.06103515625f

#define GYRO_X_OFFSET 9
#define GYRO_Y_OFFSET -99
#define GYRO_Z_OFFSET 9

#define MAG_X_OFFSET -153
#define MAG_Y_OFFSET -78
#define MAG_Z_OFFSET -118

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
#define OUTPUT_ERRORS

// Sensor variables [x, y, z]
extern float gyro[3] = {0, 0, 0};
extern float accel[3] = {0, 0, 0};
extern float magnetom[3] = {0, 0, 0};

extern int num_gyro_errors = 0;
extern int num_accel_errors = 0;
extern int num_magn_errors = 0;

void Accel_Init();
void Read_Accel();
void Magn_Init();
void Read_Magn();
void Gyro_Init();
void Read_Gyro();

#endif
