#ifndef PIDS_H
#define PIDS_H
#include <PID_v1.h>

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

PID pidPitchStable(&pitchDegrees, &pitch_stab_output, &RCpitch, 4.5, 0.0, 0.0, DIRECT);
PID pidRollStable(&rollDegrees, &roll_stab_output, &RCroll, 4.5, 0.0, 0.0, DIRECT);
PID pidYawStable(&yawDegrees, &yaw_stab_output, &yaw_target, 6, 0.0, 0.0, REVERSE); //yaw_target is wrapped 180deg before this is computed

PID pidPitchRate(&gyroDegrees[1], &pitch_output, &pitch_stab_output, 0.05, 0.1, 0.0, REVERSE);
PID pidRollRate(&gyroDegrees[0], &roll_output, &roll_stab_output, 0.05, 0.1, 0.0, DIRECT);
PID pidYawRate(&gyroDegrees[2], &yaw_output, &yaw_stab_output, 0, 0.0, 0.0, DIRECT); //off for now

void computeStabPids(){
  pidPitchStable.Compute();
  pidRollStable.Compute();

  //wrap yaw_target - yawDegrees so we turn the shortest way (and don't go crazy around -179deg & +179deg)
  float diffDegrees = yaw_target - yawDegrees;
  if(diffDegrees < -180){
    yaw_target += 360;
  }else if(diffDegrees > 180){
    yaw_target -= 360;
  }
  
  pidYawStable.Compute();
}

void computeRatePids(){
  pidPitchRate.Compute();
  pidRollRate.Compute();
  pidYawRate.Compute();
}

void initPids(){
  pidPitchStable.SetMode(AUTOMATIC);
  pidRollStable.SetMode(AUTOMATIC);
  pidYawStable.SetMode(AUTOMATIC);
  pidPitchRate.SetMode(AUTOMATIC);
  pidRollRate.SetMode(AUTOMATIC);
  pidYawRate.SetMode(AUTOMATIC);
}

void resetPids(){
  pidPitchStable.Reset();
  pidRollStable.Reset();
  pidYawStable.Reset();
  pidPitchRate.Reset();
  pidRollRate.Reset();
  pidYawRate.Reset();
}

void setRatePidsOutputLimits(float low, float high){
  pidPitchRate.SetOutputLimits(low, high);
  pidRollRate.SetOutputLimits(low, high);
  pidYawRate.SetOutputLimits(low, high);
}

void setStablePidsOutputLimits(float low, float high){
  pidPitchStable.SetOutputLimits(low, high);
  pidRollStable.SetOutputLimits(low, high);
  pidYawStable.SetOutputLimits(low, high);
}

void setPidsSampleTime(int time){ //millis
  pidPitchStable.SetSampleTime(20);
  pidRollStable.SetSampleTime(20);
  //pidYawStable.SetSampleTime(20);
  pidPitchRate.SetSampleTime(20);
  pidRollRate.SetSampleTime(20);
  //pidYawRate.SetSampleTime(20);
}

#endif

