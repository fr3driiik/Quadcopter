#include "ESCManager.h"

void ESCManager::initialize() {
  #ifdef Teensy_3_6
    analogWriteResolution(12);
  #endif
  for (int i = 0; i < ESC_COUNT; i++) {
    pinMode(escs[i].escPin, OUTPUT);
    #ifdef Teensy_3_6
      analogWriteFrequency(escs[i].escPin, 500);
    #endif
  }
}

void ESCManager::setInput(float pitch, float roll, float yaw, float throttle) {
  for (int i = 0; i < ESC_COUNT; i++){
    ESC esc = escs[i];
    float speed = throttle + pitch * esc.pitchInfluence + yaw * esc.yawInfluence + roll * esc.rollInfluence;
    float pwmWidth = Utils::fromDecimalPercent(speed, ESC_MIN, ESC_MAX);
    analogWrite(esc.escPin, constrain(pwmWidth, 0, ESC_MAX)); //we don't want to force arm-value
  }
}
   
void ESCManager::armAll() {
  for (int i = 0; i < ESC_COUNT; i++) {
    analogWrite(escs[i].escPin, ESC_MIN);
  }
}

void ESCManager::disarmAll() {
  for (int i = 0; i < ESC_COUNT; i++) {
    analogWrite(escs[i].escPin, 0); // This will not send any signal. SimonK ESC will disarm after 2sec.
  }
}

void ESCManager::tooLowThrottle() {
  for (int i = 0; i < ESC_COUNT; i++) {
    analogWrite(escs[i].escPin, ESC_MIN);
  }
}
