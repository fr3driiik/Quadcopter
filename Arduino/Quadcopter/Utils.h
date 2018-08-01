#ifndef UTILS_H
#define UTILS_H

#define RADIANS_TO_DEGREES 180.0f / PI;
#define DEGREES_TO_RADIANS PI / 180.0f

float inline toDecimalPercent(float x, float min, float max) {
  return (x - min) / (max - min);
}

float inline fromDecimalPercent(float x, float min, float max) {
  return min + (max - min) * x;
}

#endif
