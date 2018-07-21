ESC::ESC(uint8_t escPin, unsigned int min, unsigned int max) : escPin(escPin), min(min), max(max) {
  pinMode(escPin, OUTPUT);
}

void ESC::arm() {
  analogWrite(escPin, min);
}

void ESC::disarm() {
  analogWrite(escPin, 0); // This will not send any signal. SimonK ESC will disarm after 2sec.
}

void ESC::setSpeed(float speed) {
  float pwmWidth = min + (max - min) * speed
  analogWrite(escPin, constrain(pwmWidth, 0, max)); //we don't want to force arm-value
}

