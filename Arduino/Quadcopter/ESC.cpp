#include "ESC.h"

ESC::ESC(uint8_t escPin, unsigned int minSignal, unsigned int maxSignal) : escPin(escPin), minSignal(minSignal), maxSignal(maxSignal) {
  pinMode(escPin, OUTPUT);
}

void ESC::arm() {
  analogWrite(escPin, minSignal);
}

void ESC::disarm() {
  analogWrite(escPin, 0); // This will not send any signal. SimonK ESC will disarm after 2sec.
}

void ESC::setSpeed(float speed) {
  float pwmWidth = minSignal + (maxSignal - minSignal) * speed;
  analogWrite(escPin, constrain(pwmWidth, 0, maxSignal)); //we don't want to force arm-value
}

