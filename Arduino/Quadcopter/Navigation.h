#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Config.h"
#include "Receiver.h"
#include "Control.h"

namespace Navigation {
  void update(float deltaTime) {
    float throttle = map(Receiver::channelsRaw[THROTTLE_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, 0.0, 100.0);
    if (throttle > 5.0) {
      if (Receiver::channelsRaw[MODE_CHANNEL] < 1300) {
        //stable
        float pitch_target_angle = map(Receiver::channelsRaw[PITCH_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_ANGLE, ROLL_PITCH_MAX_ANGLE);
        float roll_target_angle = map(Receiver::channelsRaw[ROLL_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_ANGLE, ROLL_PITCH_MAX_ANGLE);
        float yaw_target_rate = map(Receiver::channelsRaw[YAW_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, -YAW_MAX_DPS, YAW_MAX_DPS);
        Control::set_angle_target(pitch_target_angle, roll_target_angle, yaw_target_rate);
      } else {
        //rate
        float pitch_target_rate= map(Receiver::channelsRaw[PITCH_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_DPS, ROLL_PITCH_MAX_DPS);
        float roll_target_rate= map(Receiver::channelsRaw[ROLL_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, -ROLL_PITCH_MAX_DPS, ROLL_PITCH_MAX_DPS);
        float yaw_target_rate = map(Receiver::channelsRaw[YAW_CHANNEL], RCRECEIVER_MIN, RCRECEIVER_MAX, -YAW_MAX_DPS, YAW_MAX_DPS);
        Control::set_rate_target(pitch_target_rate, roll_target_rate, yaw_target_rate);
      }
    } else { //too low throttle
      Control::stop();
    }
  }
}

#endif
