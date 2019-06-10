#ifndef ESC_MANAGER_H
#define ESC_MANAGER_H
#include "Config.h"
#include "Utils.h"
#include "ESC.h"
#include <Arduino.h>

/*
 * ESC settings:
 * stop/arm   1060 Âµs 
 * full power 1860 Âµs
 */

/*
 * ARDUINO
 * PWM freq for pin 3, 5, 6, 9 (timer 2, 3, 4) = 490 Hz
 * 490Hz -> pulseLength = 2040.8163265306 Âµs
 * 8 bit res -> 256 steps, each step = 7.97193878 Âµs
 * min = 132 
 * max = 232,407 analogWrite Value. Detta ger endast 100 steg, har mÃ¶jlighet fÃ¥r 800st
 * Steps: 100 -> one step = 8 steps for esc. Losing much precision! (TODO: fix with pwm timers?)
 * 
 * TEENSY 3.6
 * 500Hz -> pulseLength = 2000 Âµs
 * 12 bit res -> 4096 steps, each step = 0.48828125 Âµs
 * min = 2170.88 ( 2170.88 * stepLength = 1060)
 * max = 3809.28 ( 3809.28 * stepLength = 1060)
 * Steps: 1639 -> esc will only be able to detech two-steps
 */ 

#ifdef Arduino_Pro_Micro
  #define ESC_MIN 132
  #define ESC_MAX 232
#elif defined(Arduino_Mega_2560)
  #define ESC_MIN 132
  #define ESC_MAX 232
#elif defined(Teensy_3_6)
  #define ESC_MIN 2170
  #define ESC_MAX 3809
#endif
namespace ESCManager {
  void initialize();
  void setInput(float pitch, float roll, float yaw, float throttle);
  void armAll();
  void disarmAll();
  void tooLowThrottle();
}

#endif

