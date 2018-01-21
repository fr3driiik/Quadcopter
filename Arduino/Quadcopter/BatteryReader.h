#ifndef BATTERY_READER_H
#define BATTERY_READER_H

#include "Utils.h"

#define BATTERY_PIN 3
#define VOLTAGE_DIVIDER 5

float readBattery() {
  int raw = analogRead(BATTERY_PIN);
  return map(raw, 0, 1023, 0, 5);
}

#endif
