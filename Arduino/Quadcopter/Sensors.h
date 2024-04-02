#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Wire.h"
#include "Config.h"

#define X_AXIS	0
#define Y_AXIS	1
#define Z_AXIS	2

#define OUTPUT_ERRORS

#ifdef ALTIMU_10_V5
    // sensor chip that does not expose any interrupt pins
    #define LSM6DS33  // gyro and accelerometer
    #define LIS3MDL   // magnetometer
    #define LPS25H    // barometer
#endif
#ifdef CSGSHOP_11_DOF
    #define ITG3050   // gyro
    #define HMC5883L  // magnetometer
    #define BMA180    // accelerometer
    #define MS5611    // barometer
    #define NEO_M8N   // GPS
#endif

enum class Axis { X=0, Y=1, Z=2 };

namespace Sensors {
    extern float gyro[3];      // degrees/second
    extern float accel[3];     // g force
    extern float magnetom[3];  // gauss
    extern float temperature;  // Celsius (°C)
    extern float pressure;     // hectopascal (hPa)

    void initialize();
    void loop();
}
#endif
