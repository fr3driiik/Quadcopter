#include "IMU.h"

IMU::State state;

IMU::State IMU::getState() {
  return state;
}

void IMU::update(float dt) {
  #ifdef MAGNETOMETER
    float magnitude = Utils::vector3Magnitude(Sensors::magnetom);
    if (abs(magnitude - EARTH_MAGNETIC_FIELD_STRENGTH) <= MAGNETIC_FIELD_MAX_DIFF) {
      // we only want to trust the magnetometer when it detects the earth magnetic field solely
      MadgwickAHRS::Update(Sensors::gyro[ROLL] * DEGREES_TO_RADIANS, Sensors::gyro[PITCH] * DEGREES_TO_RADIANS, Sensors::gyro[YAW] * DEGREES_TO_RADIANS, Sensors::accel[ROLL], Sensors::accel[PITCH], Sensors::accel[YAW], Sensors::magnetom[ROLL], Sensors::magnetom[PITCH], Sensors::magnetom[YAW]);
    } else {
      MadgwickAHRS::Update(Sensors::gyro[ROLL] * DEGREES_TO_RADIANS, Sensors::gyro[PITCH] * DEGREES_TO_RADIANS, Sensors::gyro[YAW] * DEGREES_TO_RADIANS, Sensors::accel[ROLL], Sensors::accel[PITCH], Sensors::accel[YAW]);
    }
  #else
    MadgwickAHRS::Update(Sensors::gyro[ROLL] * DEGREES_TO_RADIANS, Sensors::gyro[PITCH] * DEGREES_TO_RADIANS, Sensors::gyro[YAW] * DEGREES_TO_RADIANS, Sensors::accel[ROLL], Sensors::accel[PITCH], Sensors::accel[YAW]);
  #endif
  state.qw = MadgwickAHRS::q0;
  state.qx = MadgwickAHRS::q1;
  state.qy = MadgwickAHRS::q2;
  state.qz = MadgwickAHRS::q3;
  Utils::quaternionToEuler(state.qx, state.qy, state.qz, state.qw, &state.roll, &state.pitch, &state.yaw);
  state.pitchDegrees = state.pitch * RADIANS_TO_DEGREES;
  state.rollDegrees = state.roll * RADIANS_TO_DEGREES;
  state.yawDegrees = state.yaw * RADIANS_TO_DEGREES;

  //wrap yaw between -180 and 180
  if (state.yawDegrees > 180.0f) 
    state.yawDegrees -= 360.0f;
  else if (state.yawDegrees < -180.0f)
    state.yawDegrees += 360.0f;

  state.yaw = state.yawDegrees * DEGREES_TO_RADIANS;

//  Utils::quaternionToRotationMatrix(state.qx, state.qy, state.qz, state.qw, state.rotationMatrix);
//  Utils::matrix3x3Inverse(state.rotationMatrix, state.rotationMatrixInv);
//  Utils::rotate(state.rotationMatrixInv, Sensors::accel[PITCH], Sensors::accel[ROLL], Sensors::accel[YAW], &(state.accNorth), &(state.accEast), &(state.accDown));
//
//  state.accDown -= 1.00; //remove gravity
//
//  //update velo
//  state.veloNorth += state.accNorth * dt;
//  state.veloEast += state.accEast * dt;
//  state.veloDown += state.accDown * dt;
//
//  state.height -= state.veloDown * dt;
}

void IMU::updateGPS(float dt) {
  #ifdef GPS

  #endif
}
