#include "ESCManager.h"
#include "DShot.h"


void ESCManager::initialize() {
  #if defined(ESC_SIGNAL_PWM)
    #if defined(Teensy_3_6) || defined(Teensy_4_0)
      analogWriteResolution(12);
    #endif
    for (int i = 0; i < ESC_COUNT; i++) {
      pinMode(escs[i].escPin, OUTPUT);
      #if defined(Teensy_3_6) || defined(Teensy_4_0)
        analogWriteFrequency(escs[i].escPin, 490);
      #endif
    }
  #elif defined(ESC_SIGNAL_DSHOT)
    DSHOT_init();
  #endif
}

void ESCManager::set_input(float pitch, float roll, float yaw, float throttle) {
  uint16_t fl_speed = constrain(throttle + pitch + roll - yaw, ESC_MIN, ESC_MAX);
  uint16_t fr_speed = constrain(throttle + pitch - roll + yaw, ESC_MIN, ESC_MAX);
  uint16_t rl_speed = constrain(throttle - pitch + roll + yaw, ESC_MIN, ESC_MAX);
  uint16_t rr_speed = constrain(throttle - pitch - roll - yaw, ESC_MIN, ESC_MAX);

  #if PRINT_ESC_OUTPUT
    Serial.println((String)"ESC FL:" + fl_speed + ", FR:" + fr_speed + ", RL:" + rl_speed + ", RR:" + rr_speed);
  #endif

  #if defined(ESC_SIGNAL_PWM)
    analogWrite(ESC_FL_PIN, fl_speed);
    analogWrite(ESC_FR_PIN, fr_speed);
    analogWrite(ESC_RL_PIN, rl_speed);
    analogWrite(ESC_RR_PIN, rr_speed);
  #elif defined(ESC_SIGNAL_DSHOT)
    uint16_t speed[] = {fl_speed, fr_speed, rl_speed, rr_speed};
    uint8_t telemetry[] = { 0, 0, 0, 0 };
    DSHOT_send(speed, telemetry);
  #endif
}

void ESCManager::stop() {
  #if PRINT_ESC_OUTPUT
    Serial.println("ESC FL: 0, FR: 0, RL: 0, RR: 0");
  #endif

  #if defined(ESC_SIGNAL_PWM)
    analogWrite(ESC_FL_PIN, 0);
    analogWrite(ESC_FR_PIN, 0);
    analogWrite(ESC_RL_PIN, 0);
    analogWrite(ESC_RR_PIN, 0);
  #elif defined(ESC_SIGNAL_DSHOT)
    uint16_t speed[] = {0, 0, 0, 0};
    uint8_t telemetry[] = { 0, 0, 0, 0 };
    DSHOT_send(speed, telemetry);
  #endif
}
