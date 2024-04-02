#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Config.h"
#include "Receiver.h"
#include "Control.h"

namespace Navigation {
  void update(float deltaTime) {
    if (Receiver::failsafe || Receiver::get_channel_value(THROTTLE_CHANNEL) < RCRECEIVER_MIN + THROTTLE_DEAD_BAND) {
       //failsafe or too low throttle
      Control::stop();
    } else {
      float throttle = map(Receiver::get_channel_value(THROTTLE_CHANNEL), RCRECEIVER_MIN + THROTTLE_DEAD_BAND, RCRECEIVER_MAX, ESC_MIN, ESC_MAX - THROTTLE_CAP);
      Control::set_throttle(throttle);
      if (Receiver::get_channel_value(MODE_CHANNEL) < 1500) {
        //stable
        float pitch_target_angle = map(Receiver::get_channel_value(PITCH_CHANNEL), RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_ANGLE, ROLL_PITCH_MAX_ANGLE);
        float roll_target_angle = map(Receiver::get_channel_value(ROLL_CHANNEL), RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_ANGLE, ROLL_PITCH_MAX_ANGLE);
        float yaw_target_rate = map(Receiver::get_channel_value(YAW_CHANNEL), RCRECEIVER_MIN, RCRECEIVER_MAX, -YAW_MAX_DPS, YAW_MAX_DPS);
        Control::set_angle_target(pitch_target_angle, roll_target_angle, yaw_target_rate);
      } else {
        //rate
        float pitch_target_rate= map(Receiver::get_channel_value(PITCH_CHANNEL), RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_DPS, ROLL_PITCH_MAX_DPS);
        float roll_target_rate= map(Receiver::get_channel_value(ROLL_CHANNEL), RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_DPS, ROLL_PITCH_MAX_DPS);
        float yaw_target_rate = map(Receiver::get_channel_value(YAW_CHANNEL), RCRECEIVER_MIN, RCRECEIVER_MAX, -YAW_MAX_DPS, YAW_MAX_DPS);
        Control::set_rate_target(pitch_target_rate, roll_target_rate, yaw_target_rate);
      }
    }
  }
}

#endif
