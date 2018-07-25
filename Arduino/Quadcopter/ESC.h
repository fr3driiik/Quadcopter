#ifndef ESC_H
#define ESC_H

#include <Arduino.h>

class ESC {
  public:
    ESC(uint8_t escPin, unsigned int minSignal, unsigned int maxSignal);
    void arm();
    void disarm();
    void setSpeed(float speed); // 0.00-1.00

  private:
    uint8_t escPin;
    unsigned int minSignal, maxSignal;
};

struct Engine {
  //Vector3 position;
  //Vector3 rotation; //too much
  float degreesFromFront; //clockWise from forward
  ESC esc;
};

#endif

