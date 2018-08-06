#include "IMU.h"

#define LP_FACTOR 0.02f
#define HP_FACTOR (1.00f - LP_FACTOR)

State state;

State IMU_getState() {
  return state;
}

void IMU_update(float dt) {
  #ifdef USE_SIMPLE_BIAS_FILTER
    state.pitchDegrees += gyro[PITCH] * dt;
    state.yawDegrees += gyro[YAW] * dt;
    state.rollDegrees += gyro[ROLL] * dt;
  
    // compensate pitch and roll drift with accelerometer
    if (abs(accel[ROLL]) > 0.0f && abs(accel[PITCH]) > 0.0f) {
      float accPitch = -atan2(accel[ROLL], accel[YAW]) * RAD_TO_DEG;
      float accRoll = atan2(accel[PITCH], accel[YAW]) * RAD_TO_DEG;
      state.pitchDegrees = HP_FACTOR * state.pitch + LP_FACTOR * accPitch;
      state.rollDegrees = HP_FACTOR * state.roll + LP_FACTOR * accRoll;
    }
    state.pitch = state.pitchDegrees * DEGREES_TO_RADIANS;
    state.roll = state.rollDegrees * DEGREES_TO_RADIANS;
  
    #ifdef MAGNETOMETER
      //compensate yaw drift with magnetometer 
      //https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L_compensation_MPU6050/HMC5883L_compensation_MPU6050.ino
      if (!(state.roll > 45 || state.roll < -45 || state.pitch > 45 || state.pitch < -45)) { //hm is this not in radians?
        float cosRoll = cos(state.roll);
        float sinRoll = sin(state.roll);
        float cosPitch = cos(state.pitch);
        float sinPitch = sin(state.pitch);
        float xh = magnetom[ROLL] * cosPitch + magnetom[YAW] * sinPitch;
        float yh = magnetom[ROLL] * sinRoll * sinPitch + magnetom[PITCH] * cosRoll - magnetom[YAW] * sinRoll * cosPitch;
        state.yawDegrees = state.yaw * RADIANS_TO_DEGREES;
        
      }
    #endif
  #endif // USE_SIMPLE_BIAS_FILTER

  //wrap yaw between -180 and 180
  if (state.yawDegrees > 180.0f) 
    state.yawDegrees -= 360.0f;
  else if (state.yawDegrees < -180.0f)
    state.yawDegrees += 360.0f;

  state.yaw = state.yawDegrees * DEGREES_TO_RADIANS;
}

void IMU_updateGPS(float dt) {
  #ifdef GPS

  #endif
}

