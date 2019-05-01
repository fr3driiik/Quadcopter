#ifndef IMU_H
#define IMU_H
#include "Arduino.h"
#include "Sensors.h"
#include "Utils.h"
#include "Config.h"
#include <math.h>

#define USE_SIMPLE_BIAS_FILTER

namespace IMU {
  struct State {
    float qx, qy, qz, qw; //quaternion
    float pitch, yaw, roll, pitchDegrees, yawDegrees, rollDegrees;
    float rotationMatrix[3][3], rotationMatrixInv[3][3];
    float accNorth, accEast, accDown;
    float veloNorth, veloEast, veloDown;
    float longitude, latitude;
    float height;
  };
  
  State getState();
  void update(float dt);
  void updateGPS(float dt); //only if gps is enabled
}
#endif

