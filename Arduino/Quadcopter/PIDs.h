#ifndef PIDS_H
#define PIDS_H
#include "PID.h"

float yawDegrees = 0;
float pitchDegrees = 0;
float rollDegrees = 0;

//PIDs and their variables
float pitch_stab_output = 0;
float roll_stab_output = 0;
float yaw_stab_output = 0;

float pitch_output = 0;
float roll_output = 0;
float yaw_output = 0;

float gyroDegrees[3];

float yaw_target = 0;

PID pidPitchStable(&pitchDegrees, &pitch_stab_output, &Receiver::pitchIn, 4.5, 0.0, 0.0, false); //TODO!!! This is wrong. pitchIn is in %, we need do convert to AngleDegrees
PID pidRollStable(&rollDegrees, &roll_stab_output, &Receiver::rollIn, 4.5, 0.0, 0.0, false);
PID pidYawStable(&yawDegrees, &yaw_stab_output, &yaw_target, 6, 0.0, 0.0, true); //yaw_target is wrapped 180deg before this is computed

PID pidPitchRate(&gyroDegrees[1], &pitch_output, &pitch_stab_output, 0.05, 0.1, 0.0, true);
PID pidRollRate(&gyroDegrees[0], &roll_output, &roll_stab_output, 0.05, 0.1, 0.0, false);
PID pidYawRate(&gyroDegrees[2], &yaw_output, &yaw_stab_output, 0, 0.0, 0.0, false); //off for now

void computeStabPids(float deltaTime){
  pidPitchStable.calculate(deltaTime);
  pidRollStable.calculate(deltaTime);

  //wrap yaw_target - yawDegrees so we turn the shortest way (and don't go crazy around -179deg & +179deg)
  float diffDegrees = yaw_target - yawDegrees;
  if(diffDegrees < -180){
    yaw_target += 360;
  }else if(diffDegrees > 180){
    yaw_target -= 360;
  }
  
  pidYawStable.calculate(deltaTime);
}

void computeRatePids(float deltaTime){
  pidPitchRate.calculate(deltaTime);
  pidRollRate.calculate(deltaTime);
  pidYawRate.calculate(deltaTime);
}

void resetPids(){
  pidPitchStable.reset();
  pidRollStable.reset();
  pidYawStable.reset();
  pidPitchRate.reset();
  pidRollRate.reset();
  pidYawRate.reset();
}

void setRatePidsIntegralLimits(float low, float high){
  pidPitchRate.setIntegralLimits(low, high);
  pidRollRate.setIntegralLimits(low, high);
  pidYawRate.setIntegralLimits(low, high);
}

void setStablePidsIntegralLimits(float low, float high){
  pidPitchStable.setIntegralLimits(low, high);
  pidRollStable.setIntegralLimits(low, high);
  pidYawStable.setIntegralLimits(low, high);
}

#endif
