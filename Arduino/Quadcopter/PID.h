#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  public:
    enum Direction { DIRECT, REVERSE };
    PID(float *input, float *target, float *output, float kp, float ki, float kd, bool revert);
    void reset();
    void calculate(float deltaTime);
    void setOutputLimits(float lowLimit, float highLimit);
  private:
    float *input, *target, *output;
    float kp, ki, kd, integral, previousError, lowLimit, highLimit;
    uint8_t sign;
};

#endif
