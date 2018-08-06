#ifndef IMU_H
#define IMU_H
#include "Arduino.h"
#include "Sensors.h"
#include "Utils.h"
#include "Config.h"
#include <math.h>

#define USE_SIMPLE_BIAS_FILTER

struct State {
  float qx, qy, qz, qw; //quaternion
  float pitch, yaw, roll, pitchDegrees, yawDegrees, rollDegrees;
  float rotationMatrix[3][3];
  float accNorth, accEast, accDown;
  float longitude, latitude;
  float height;
};

State IMU_getState();
void IMU_update(float dt);
void IMU_updateGPS(float dt); //only if gps is enabled

#endif

