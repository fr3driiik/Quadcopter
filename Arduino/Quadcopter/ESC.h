#ifndef ESC_H
#define ESC_H

#include <Arduino.h>

struct ESC {
  uint8_t escPin;
  float pitchInfluence, yawInfluence, rollInfluence;
};

#endif


