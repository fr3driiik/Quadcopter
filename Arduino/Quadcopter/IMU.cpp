#include "IMU.h"

#define LP_FACTOR 0.02f
#define HP_FACTOR (1.00f - LP_FACTOR)

State state;

State IMU_getState() {
  return orientation;
}

void IMU_update(float dt) {
  orientation = {gyro[PITCH] * dt + orientation.pitch, 
                 gyro[YAW] * dt + orientation.yaw, 
                 gyro[ROLL] * dt + orientation.roll};

  // compensate pitch and roll drift with accelerometer
  if (abs(accel[ROLL]) > 0.0f && abs(accel[PITCH]) > 0.0f) {
    float accPitch = -atan2(accel[ROLL], accel[YAW]) * RAD_TO_DEG;
    float accRoll = atan2(accel[PITCH], accel[YAW]) * RAD_TO_DEG;
    orientation.pitch = HP_FACTOR * orientation.pitch + LP_FACTOR * accPitch;
    orientation.roll = HP_FACTOR * orientation.roll + LP_FACTOR * accRoll;
  }

  #ifdef MAGNETOMETER
    //compensate yaw drift with magnetometer 
    //https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L_compensation_MPU6050/HMC5883L_compensation_MPU6050.ino
    if (!(orientation.roll > 45 || orientation.roll < -45 || orientation.pitch > 45 || orientation.pitch < -45)) {
      float pitchRad = orientation.pitch * DEGREES_TO_RADIANS;
      float rollRad = orientation.roll * DEGREES_TO_RADIANS;
      float cosRoll = cos(rollRad);
      float sinRoll = sin(rollRad);
      float cosPitch = cos(pitchRad);
      float sinPitch = sin(pitchRad);
      float xh = magnetom[ROLL] * cosPitch + magnetom[YAW] * sinPitch;
      float yh = magnetom[ROLL] * sinRoll * sinPitch + magnetom[PITCH] * cosRoll - magnetom[YAW] * sinRoll * cosPitch;
      orientation.yaw = atan2(yh, xh) * RADIANS_TO_DEGREES;
    }
  #endif

  //wrap yaw between -180 and 180
  if (orientation.yaw > 180.0f) 
    orientation.yaw -= 360.0f;
  else if (orientation.yaw < -180.0f)
    orientation.yaw += 360.0f;
}

void IMU_updateGPS(float dt) {
  
}

