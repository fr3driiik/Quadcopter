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

  float rate_output[3] = {0, 0, 0};
  float rate_target[3] = {0, 0, 0};
  float angle_target[3] = {0, 0, 0};
  float angle_output[3] = {0, 0, 0};
  float throttle_input = 0;
  float angle_yaw_input = 0;  // angle yaw is actually a desired rate change. Not a target yaw in degrees.
  
  PID rate_pitch(&Sensors::gyro[PITCH], &rate_output[PITCH], &rate_target[PITCH], 0.5, 0.0, 0.0, DIRECT);
  PID rate_roll(&Sensors::gyro[ROLL], &rate_output[ROLL], &rate_target[ROLL], 0.5, 0.0, 0.0, DIRECT);
  PID rate_yaw(&Sensors::gyro[YAW], &rate_output[YAW], &rate_target[YAW], 0.5, 0.0, 0.0, DIRECT);
  PID angle_pitch(&IMU::state.pitchDegrees, &angle_output[PITCH], &angle_target[PITCH], 4.5, 0.0, 0.0, DIRECT);
  PID angle_roll(&IMU::state.rollDegrees, &angle_output[ROLL], &angle_target[ROLL], 4.5, 0.0, 0.0, DIRECT);
  PID angle_yaw(&IMU::state.yawDegrees, &angle_output[YAW], &angle_target[YAW], 6, 0.0, 0.0, DIRECT);

  void initialize() {
    //Rate
    rate_pitch.SetSampleTime(1000/DESIRED_CONTROL_HZ);
    rate_roll.SetSampleTime(1000/DESIRED_CONTROL_HZ);
    rate_yaw.SetSampleTime(1000/DESIRED_CONTROL_HZ);
    rate_pitch.SetIntegralLimits(-PID_RATE_INTERGRAL_LIMIT, PID_RATE_INTERGRAL_LIMIT);
    rate_roll.SetIntegralLimits(-PID_RATE_INTERGRAL_LIMIT, PID_RATE_INTERGRAL_LIMIT);
    rate_yaw.SetIntegralLimits(-PID_RATE_INTERGRAL_LIMIT, PID_RATE_INTERGRAL_LIMIT);
    rate_pitch.SetOutputLimits(-PID_RATE_OUTPUT_LIMIT, PID_RATE_OUTPUT_LIMIT);
    rate_roll.SetOutputLimits(-PID_RATE_OUTPUT_LIMIT, PID_RATE_OUTPUT_LIMIT);
    rate_yaw.SetOutputLimits(-PID_RATE_OUTPUT_LIMIT, PID_RATE_OUTPUT_LIMIT);
    rate_pitch.SetMode(AUTOMATIC);
    rate_roll.SetMode(AUTOMATIC);
    rate_yaw.SetMode(AUTOMATIC);

    //Stable
    angle_pitch.SetSampleTime(1000/DESIRED_CONTROL_HZ);
    angle_roll.SetSampleTime(1000/DESIRED_CONTROL_HZ);
    angle_yaw.SetSampleTime(1000/DESIRED_CONTROL_HZ);
    angle_pitch.SetOutputLimits(-PID_ANGLE_OUTPUT_LIMIT, PID_ANGLE_OUTPUT_LIMIT);
    angle_roll.SetOutputLimits(-PID_ANGLE_OUTPUT_LIMIT, PID_ANGLE_OUTPUT_LIMIT);
    angle_yaw.SetOutputLimits(-PID_ANGLE_OUTPUT_LIMIT, PID_ANGLE_OUTPUT_LIMIT);
    angle_pitch.SetMode(AUTOMATIC);
    angle_roll.SetMode(AUTOMATIC);
    angle_yaw.SetMode(AUTOMATIC);


    ESCManager::initialize();
  }

  void reset_rate_pids() {
    rate_pitch.Reset();
    rate_roll.Reset();
    rate_yaw.Reset();
  }

  void reset_angle_pids() {
    angle_pitch.Reset();
    angle_roll.Reset();
    angle_yaw.Reset();
  }

  void calculate_rate_pids() {
    rate_pitch.Compute();
    rate_roll.Compute();
    rate_yaw.Compute();
  }

  void calculate_angle_pids() {
    angle_pitch.Compute();
    angle_roll.Compute();
    angle_yaw.Compute();
  }

  void stop() {
    mode = STOP;
    throttle_input = 0;
  }

  void set_throttle(float throttle) {
    throttle_input = throttle;
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
        calculate_rate_pids();
        reset_angle_pids();
        ESCManager::set_input(rate_output[PITCH], rate_output[ROLL], rate_output[YAW], throttle_input);
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
        calculate_angle_pids();
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
        
        calculate_rate_pids();
        ESCManager::set_input(rate_output[PITCH], rate_output[ROLL], rate_output[YAW], throttle_input);
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
