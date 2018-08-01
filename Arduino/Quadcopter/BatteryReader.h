#ifndef BATTERY_READER_H
#define BATTERY_READER_H

#include "Utils.h"

float readBattery() {
  int raw = analogRead(BATTERY_PIN);
  return map(raw, 0, 1023, 0, 5 * VOLTAGE_DIVIDER);
}

#endif
