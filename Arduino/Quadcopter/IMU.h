#ifndef IMU_H
#define IMU_H
#include "Arduino.h"
#include "Sensors.h"
#include "Utils.h"
#include <math.h>

//#define MAGNETOMETER

struct Orientation {
  float pitch;
  float yaw;
  float roll;
  float pitchRadians;
  float rollRadians;
};


void IMU_init();
void IMU_calculate();
Orientation IMU_getOrientation();
float IMU_getHeight();

#endif

