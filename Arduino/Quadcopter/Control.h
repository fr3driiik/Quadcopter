#ifndef CONTROL_H
#define CONTROL_H

#include "Config.h"
#include "IMU.h"
#include "PID.h"
#include "Utils.h"
#include "ESCManager.h"

namespace Control {
  enum Mode {
    STOP,
    ANGULAR_RATE_CONTROL,
    ANGLE_CONTROL,
  };
  Mode mode = STOP;

  float rate_out[3] = {0, 0, 0};
  float rate_target[3] = {0, 0, 0};
  float angle_input[3] = {0, 0, 0};
  float angle_target[3] = {0, 0, 0};
  float angle_output[3] = {0, 0, 0};
  float angle_yaw_input = 0;  // angle yaw is actually a desired rate change. Not a target yaw in degrees.
  
  PID rate_pitch(&IMU::state.pitchDegrees, &rate_target[PITCH], &rate_out[PITCH], 0.05, 0.1, 0.0, true);
  PID rate_roll(&IMU::state.rollDegrees, &rate_target[ROLL], &rate_out[ROLL], 0.05, 0.1, 0.0, false);
  PID rate_yaw(&IMU::state.yawDegrees, &rate_target[YAW], &rate_out[YAW], 0, 0.0, 0.0, false);
  PID angle_pitch(&angle_input[PITCH], &angle_target[PITCH], &angle_output[PITCH], 0.05, 0.1, 0.0, true);
  PID angle_roll(&angle_input[ROLL], &angle_target[ROLL], &angle_output[ROLL], 0.05, 0.1, 0.0, false);
  PID angle_yaw(&angle_input[YAW], &angle_target[YAW], &angle_output[YAW], 0, 0.0, 0.0, false);

  void initialize() {
    //Rate limit
    rate_pitch.setIntegralLimits(PID_RATE_INTERGRAL_LIMIT, PID_RATE_INTERGRAL_LIMIT);
    rate_roll.setIntegralLimits(PID_RATE_INTERGRAL_LIMIT, PID_RATE_INTERGRAL_LIMIT);
    rate_yaw.setIntegralLimits(PID_RATE_INTERGRAL_LIMIT, PID_RATE_INTERGRAL_LIMIT);

    //Stable limits
    angle_pitch.setIntegralLimits(PID_ANGLE_INTERGRAL_LIMIT, PID_ANGLE_INTERGRAL_LIMIT);
    angle_roll.setIntegralLimits(PID_ANGLE_INTERGRAL_LIMIT, PID_ANGLE_INTERGRAL_LIMIT);
    angle_yaw.setIntegralLimits(PID_ANGLE_INTERGRAL_LIMIT, PID_ANGLE_INTERGRAL_LIMIT);

    ESCManager::initialize();
  }

  void reset_rate_pids() {
    rate_pitch.reset();
    rate_roll.reset();
    rate_yaw.reset();
  }

  void reset_angle_pids() {
    angle_pitch.reset();
    angle_roll.reset();
    angle_yaw.reset();
  }

  void calculate_rate_pids(float deltaTime) {
    rate_pitch.calculate(deltaTime);
    rate_roll.calculate(deltaTime);
    rate_yaw.calculate(deltaTime);
  }

  void calculate_angle_pids(float deltaTime) {
    angle_pitch.calculate(deltaTime);
    angle_roll.calculate(deltaTime);
    angle_yaw.calculate(deltaTime);
  }

  void stop() {
    mode = STOP;
  }

  void set_rate_target(float pitch_rate_target, float roll_rate_target, float yaw_rate_target) {
    mode = ANGULAR_RATE_CONTROL;
    rate_target[PITCH] = pitch_rate_target;
    rate_target[ROLL] = roll_rate_target;
    rate_target[YAW] = yaw_rate_target;
  }

  void set_angle_target(float pitch_angle_target, float roll_angle_target, float yaw_rate_target) {
    mode = ANGLE_CONTROL;
    angle_target[PITCH] = pitch_angle_target;
    angle_target[ROLL] = roll_angle_target;
    angle_yaw_input = yaw_rate_target;
  }

  void update(float deltaTime) {
    switch(mode) {        
      case ANGULAR_RATE_CONTROL: {
        calculate_rate_pids(deltaTime);
        reset_angle_pids();
        ESCManager::
        break;
      }
      case ANGLE_CONTROL: {
        // wrap yaw_target - yawDegrees so we turn the shortest way (and don't go crazy around -179deg & +179deg)
        float diffDegrees = angle_target[YAW] - IMU::state.yawDegrees;
        if (diffDegrees < -180) {
          angle_target[YAW] += 360;
        } else if (diffDegrees > 180) {
          angle_target[YAW] -= 360;
        }
        calculate_angle_pids(deltaTime);
        rate_target[PITCH] = angle_output[PITCH];
        rate_target[ROLL] = angle_output[ROLL];

        if (abs(angle_yaw_input) > 0.05) {
          // yaw change desired.
          rate_target[YAW] = angle_yaw_input;
          angle_target[YAW] = IMU::state.yawDegrees;
        } else {
          // keep yaw stable        
          rate_target[YAW] = angle_output[YAW];
        }
        
        calculate_rate_pids(deltaTime);
        break;
      }
      case STOP: {}
      default: {
        ESCManager::stop();
        angle_target[YAW] = IMU::state.yawDegrees;
        reset_angle_pids();
        reset_rate_pids();
        break;
      }
    }
  }
}
#endif
