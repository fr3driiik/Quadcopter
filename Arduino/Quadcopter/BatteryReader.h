#ifndef BATTERY_READER_H
#define BATTERY_READER_H

#include "Config.h"
#include "Utils.h"

#define VOLTAGE_DIVIDER 5
#ifdef Arduino_Pro_Micro
  #define BOARD_VOLTAGE 5.0
#elif defined(Arduino_Mega_2560)
  #define BOARD_VOLTAGE 5.0
#elif defined(Teensy_3_6)
  #define BOARD_VOLTAGE 3.3
#elif defined(Teensy_4_0)
  #define BOARD_VOLTAGE 3.3
#endif

float readBattery() {
  #if BOARD_VOLTAGE * VOLTAGE_DIVIDER >= VOLTAGE_MAX
    //int raw = analogRead(BATTERY_PIN);
    return map(raw, 0, 1023, 0, BOARD_VOLTAGE * VOLTAGE_DIVIDER);
  #else
    return -1.0; //should be avoided..
  #endif
}

#endif
