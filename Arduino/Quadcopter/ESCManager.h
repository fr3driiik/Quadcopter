#ifndef ESC_MANAGER_H
#define ESC_MANAGER_H
#include "Config.h"
#include "Utils.h"
#include <Arduino.h>
/*
 * ESC pwm settings:
 * stop/arm   1060 Âµs 
 * full power 1860 Âµs
 *
 * ARDUINO
 * PWM freq for pin 3, 5, 6, 9 (timer 2, 3, 4) = 490 Hz
 * 490Hz -> pulseLength = 2040.8163265306 µs
 * 8 bit res -> 256 steps, each step = 7.97193878 µs
 * min = 132 
 * max = 232,407 analogWrite Value. Detta ger endast 100 steg, har möjlighet för 800st
 * Steps: 100 -> one step = 8 steps for esc. Losing much precision! (TODO: fix with pwm timers?)
 * 
 * TEENSY 3.6, 4.0
 * 500Hz -> pulseLength = 2000 Âµs
 * 12 bit res -> 4096 steps, each step = 0.48828125 Âµs
 * min = 2170.88 ( 2170.88 * stepLength = 1060)
 * max = 3809.28 ( 3809.28 * stepLength = 1060)
 * Steps: 1639 -> esc will only be able to detech two-steps
 */ 
#if defined(ESC_SIGNAL_PWM)
  #ifdef Arduino_Pro_Micro
    #define ESC_MIN 132
    #define ESC_MAX 232
  #elif defined(Arduino_Mega_2560)
    #define ESC_MIN 132
    #define ESC_MAX 232
  #elif defined(Teensy_3_6)
    #define ESC_MIN 2170
    #define ESC_MAX 3809
  #elif defined(Teensy_4_0)
    #define ESC_MIN 2120
    #define ESC_MAX 3809
  #endif
#elif defined(ESC_SIGNAL_DSHOT)
  #define ESC_MIN 48
  #define ESC_MAX 2047
#endif

namespace ESCManager {
  void initialize();
  void loop();  // Some commands need repeated sends. This progresses through command queue.
  void set_input(float pitch, float roll, float yaw, float throttle);
  void stop();
}

#endif
