#include "PID.h"

PID::PID(float* input, float* target, float* output, float kp, float ki, float kd, bool revert) : input(input), target(target), output(output), kp(kp), ki(ki), kd(kd) {
  sign = revert ? -1 : 1;
  integralLowLimit = -3.4028235e38;
  integralHighLimit = 3.4028234e38;
}

void PID::calculate(float deltaTime) {
  float error = *target - *input;
  integral = constrain(integral + error * deltaTime, integralLowLimit, integralHighLimit);
  float derivative = (error - previousError) / deltaTime;
  *output = (kp * error + ki * integral + kd * derivative) * sign;
  previousError = error;
}

void PID::setIntegralLimits(float integralLowLimit, float integralHighLimit) {
  this->integralLowLimit = integralLowLimit;
  this->integralHighLimit = integralHighLimit;
}

void PID::reset() {
  integral = 0;
  previousError = 0;
}

