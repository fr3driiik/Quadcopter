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

// Quaternion should be normalized before this call
void inline Utils_QuaternionToRotationMatrix(float qx, float qy, float qz, float qw, float matrix[3][3]) {
  float sqx = qx * qx;
  float sqy = qy * qy;
  float sqz = qz * qz;
  float sqw = qw * qw;

  matrix[0][0] =  sqx - sqy - sqz + sqw;
  matrix[1][1] = -sqx + sqy - sqz + sqw;
  matrix[2][2] = -sqx - sqy + sqz + sqw;

  float tmp1 = qx * qy;
  float tmp2 = qz * qw;
  matrix[0][1] = 2.0 * (tmp1 + tmp2);
  matrix[1][0] = 2.0 * (tmp1 - tmp2);

  tmp1 = qx * qz;
  tmp2 = qy * qw;
  matrix[0][2] = 2.0 * (tmp1 - tmp2);
  matrix[2][0] = 2.0 * (tmp1 + tmp2);

  tmp1 = qy * qz;
  tmp2 = qx * qw;
  matrix[1][2] = 2.0 * (tmp1 + tmp2);
  matrix[2][1] = 2.0 * (tmp1 - tmp2);
}

void inline Utils_Rotate(float matrix[3][3], float xIn, float yIn, float zIn, float *xOut, float *yOut, float *zOut) {
  *xOut = matrix[0][0] * xIn + matrix[1][0] * yIn + matrix[2][0] * zIn;
  *yOut = matrix[0][1] * xIn + matrix[1][1] * yIn + matrix[2][1] * zIn;
  *zOut = matrix[0][2] * xIn + matrix[1][2] * yIn + matrix[2][2] * zIn;
}

#endif
