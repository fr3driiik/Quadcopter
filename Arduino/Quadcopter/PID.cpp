#include "PID.h"

PID::PID(float* input, float* target, float* output, float kp, float ki, float kd, bool revert) : input(input), target(target), output(output), kp(kp), ki(ki), kd(kd) {
  sign = revert ? -1 : 1;
  lowLimit = -3.4028235e38;
  highLimit = 3.4028234e38;
}

void PID::calculate(float deltaTime) {
  float error = *target - *input;
  integral += error * deltaTime;
  float derivative = (error - previousError) / deltaTime;
  *output = constrain((kp * error + ki * integral + kd * derivative) * sign, lowLimit, highLimit);
  previousError = error;
}

void PID::setOutputLimits(float lowLimit, float highLimit) {
  this->lowLimit = lowLimit;
  this->highLimit = highLimit;
}

void PID::reset() {
  integral = 0;
  previousError = 0;
}
