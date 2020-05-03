#ifndef UTILS_H
#define UTILS_H

#define RADIANS_TO_DEGREES 180.0f / PI;
#define DEGREES_TO_RADIANS PI / 180.0f

namespace Utils {
  float inline toDecimalPercent(float x, float min, float max) {
    return (x - min) / (max - min);
  }

  float inline fromDecimalPercent(float x, float min, float max) {
    return min + (max - min) * x;
  }

  void inline eulerToQuaternion(float pitch, float yaw, float roll, float *qx, float *qy, float *qz, float *qw) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    *qx = cy * sr * cp - sy * cr * sp;
    *qy = cy * cr * sp + sy * sr * cp;
    *qz = sy * cr * cp - cy * sr * sp;
    *qw = cy * cr * cp + sy * sr * sp;
  }

  void inline quaternionToEuler(float qx, float qy, float qz, float qw, float *pitch, float *yaw, float *roll) {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1) {
        *pitch = M_PI / 2; // use 90 degrees if out of range
    } else {
        *pitch = asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp);
    Serial.println(*yaw);
  }

  // Quaternion should be normalized before this call
  void inline quaternionToRotationMatrix(float qx, float qy, float qz, float qw, float matrix[3][3]) {
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

  void inline matrix3x3Inverse(float matrix[3][3], float output[3][3]) {
    float m10x21 = matrix[1][0] * matrix[2][1];
    float m10x22 = matrix[1][0] * matrix[2][2];
    float m11x20 = matrix[1][1] * matrix[2][0];
    float m11x22 = matrix[1][1] * matrix[2][2];
    float m12x20 = matrix[1][2] * matrix[2][0];
    float m12x21 = matrix[1][2] * matrix[2][1];

    float det = matrix[0][0] * (m11x22 - m12x21)
              - matrix[0][1] * (m10x22 - m12x20)
              + matrix[0][2] * (m10x21 - m11x20);

    if (det == 0) {
      Serial.println("Matrix det division by 0 avoided.");
      return;
    }
    float invdet = 1 / det;

    output[0][0] = (m11x22 - m12x21) * invdet;
    output[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * invdet;
    output[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * invdet;
    output[1][0] = (m12x20 - m10x22) * invdet;
    output[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * invdet;
    output[1][2] = (matrix[1][0] * matrix[0][2] - matrix[0][0] * matrix[1][2]) * invdet;
    output[2][0] = (m10x21 - m11x20) * invdet;
    output[2][1] = (matrix[2][0] * matrix[0][1] - matrix[0][0] * matrix[2][1]) * invdet;
    output[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * invdet;
  }

  void inline rotate(float matrix[3][3], float xIn, float yIn, float zIn, float *xOut, float *yOut, float *zOut) {
    *xOut = matrix[0][0] * xIn + matrix[1][0] * yIn + matrix[2][0] * zIn;
    *yOut = matrix[0][1] * xIn + matrix[1][1] * yIn + matrix[2][1] * zIn;
    *zOut = matrix[0][2] * xIn + matrix[1][2] * yIn + matrix[2][2] * zIn;
  }
}
#endif

