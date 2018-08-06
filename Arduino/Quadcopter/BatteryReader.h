#ifndef BATTERY_READER_H
#define BATTERY_READER_H

#include "Utils.h"

float readBattery() {
  #if BOARD_VOLTAGE * VOLTAGE_DIVIDER >= VOLTAGE_MAX
    int raw = analogRead(BATTERY_PIN);
    return map(raw, 0, 1023, 0, BOARD_VOLTAGE * VOLTAGE_DIVIDER);
  #else
    return -1.0; //should be avoided..
  #endif
}

#endif
