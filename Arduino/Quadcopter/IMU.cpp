#include "IMU.h"

#define LP_FACTOR 0.02f
#define HP_FACTOR (1.00f - LP_FACTOR)
/*
#define sampleFreq  512.0f    // sample frequency in Hz
#define betaDef   0.1f    // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;                // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

FIX THIS 
*/

IMU::State state;

IMU::State IMU::getState() {
  return state;
}

void IMU::update(float dt) {
  #ifdef USE_SIMPLE_BIAS_FILTER
    state.pitchDegrees += gyro[PITCH] * dt;
    state.yawDegrees += gyro[YAW] * dt;
    state.rollDegrees += gyro[ROLL] * dt;
  
    // compensate pitch and roll drift with accelerometer
    if (abs(accel[ROLL]) > 0.0f && abs(accel[PITCH]) > 0.0f) {
      float accPitch = -atan2(accel[ROLL], accel[YAW]) * RAD_TO_DEG;
      float accRoll = atan2(accel[PITCH], accel[YAW]) * RAD_TO_DEG;
      state.pitchDegrees = HP_FACTOR * state.pitchDegrees + LP_FACTOR * accPitch;
      state.rollDegrees = HP_FACTOR * state.rollDegrees + LP_FACTOR * accRoll;
    }
    state.pitch = state.pitchDegrees * DEGREES_TO_RADIANS;
    state.roll = state.rollDegrees * DEGREES_TO_RADIANS;
  
    #ifdef MAGNETOMETER
      //compensate yaw drift with magnetometer 
      //https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L_compensation_MPU6050/HMC5883L_compensation_MPU6050.ino
      if (!(state.rollDegrees > 45 || state.rollDegrees < -45 || state.pitchDegrees > 45 || state.pitchDegrees < -45)) { //hm is this not in radians?
        float cosRoll = cos(state.roll);
        float sinRoll = sin(state.roll);
        float cosPitch = cos(state.pitch);
        float sinPitch = sin(state.pitch);
        float xh = magnetom[ROLL] * cosPitch + magnetom[YAW] * sinPitch;
        float yh = magnetom[ROLL] * sinRoll * sinPitch + magnetom[PITCH] * cosRoll - magnetom[YAW] * sinRoll * cosPitch;
        state.yawDegrees = state.yaw * RADIANS_TO_DEGREES;
      }
    #endif
    //update quaternion
    Utils::eulerToQuaternion(state.pitch, state.yaw, state.roll, &(state.qx), &(state.qy), &(state.qz), &(state.qw));
  #else // USE_SIMPLE_BIAS_FILTER
    #ifdef MAGNETOMETER
      //TODO use state.quaternions ..
      MadgwickAHRS_Update(gyro[PITCH], gyro[YAW], gyro[ROLL], accel[PITCH], accel[YAW], accel[ROLL], magnetom[PITCH], magnetom[YAW], magnetom[ROLL]);
      state.qx = q0; //  change to MadgwickAHRS::q0
      state.qy = q1;
      state.qz = q2;
      state.qw = q3;
    #else
      MadgwickAHRS_Update(gyro[PITCH], gyro[YAW], gyro[ROLL], accel[PITCH], accel[YAW], accel[ROLL]);
      state.qx = q0;
      state.qy = q1;
      state.qz = q2;
      state.qw = q3;
    #endif

  #endif // use madgewick kalman
  
  //wrap yaw between -180 and 180
  if (state.yawDegrees > 180.0f) 
    state.yawDegrees -= 360.0f;
  else if (state.yawDegrees < -180.0f)
    state.yawDegrees += 360.0f;

  state.yaw = state.yawDegrees * DEGREES_TO_RADIANS;

  Utils::qQuaternionToRotationMatrix(state.qx, state.qy, state.qz, state.qw, state.rotationMatrix);
  Utils::matrix3x3Inverse(state.rotationMatrix, state.rotationMatrixInv);
  Utils::rotate(state.rotationMatrixInv, accel[PITCH], accel[ROLL], accel[YAW], &(state.accNorth), &(state.accEast), &(state.accDown));

  state.accDown -= 1.00; //remove gravity

  //update velo
  state.veloNorth += state.accNorth * dt;
  state.veloEast += state.accEast * dt;
  state.veloDown += state.accDown * dt;

  state.height -= state.veloDown * dt;
}

void IMU::updateGPS(float dt) {
  #ifdef GPS

  #endif
}

