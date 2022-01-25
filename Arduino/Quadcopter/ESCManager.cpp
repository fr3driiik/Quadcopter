#include "ESCManager.h"

void set_engine_speed(int pin, float speed) {
  analogWrite(pin, speed);
  Serial.print((String)"pin" + pin + ": " + speed + ", ");
}

void ESCManager::initialize() {
  #if defined(Teensy_3_6) || defined(Teensy_4_0)
    analogWriteResolution(12);
  #endif
  for (int i = 0; i < ESC_COUNT; i++) {
    pinMode(escs[i].escPin, OUTPUT);
    #if defined(Teensy_3_6) || defined(Teensy_4_0)
      analogWriteFrequency(escs[i].escPin, 490);
    #endif
  }
}

void ESCManager::set_input(float pitch, float roll, float yaw, float throttle) {
  for (int i = 0; i < ESC_COUNT; i++){
    ESC esc = escs[i];
    float speed = throttle + pitch * esc.pitchInfluence + yaw * esc.yawInfluence + roll * esc.rollInfluence;
    speed = constrain(speed, 0, ESC_MAX);  //we don't want to force arm-value
    set_engine_speed(esc.escPin, speed);
  }
}
   
void ESCManager::armAll() {
  for (int i = 0; i < ESC_COUNT; i++) {
    set_engine_speed(escs[i].escPin, ESC_MIN);
  }
}

void ESCManager::disarmAll() {
  for (int i = 0; i < ESC_COUNT; i++) {
    set_engine_speed(escs[i].escPin, 0); // This will not send any signal. SimonK ESC will disarm after 2sec.
  }
}

void ESCManager::stop() {
  for (int i = 0; i < ESC_COUNT; i++) {
    set_engine_speed(escs[i].escPin, ESC_MIN);
  }
}
