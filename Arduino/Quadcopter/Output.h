#ifndef OUTPUT_H
#define OUTPUT_H
#include "Sensors.h"

void print_sensor_data(){
  Serial.print("SENSORS[x:y:z]");
  Serial.print("  Gyro[");
  Serial.print(gyro[0], 2);Serial.print(" : ");
  Serial.print(gyro[1], 2);Serial.print(" : ");
  Serial.print(gyro[2], 2);Serial.print("]");
  Serial.print("  Accel[");
  Serial.print(accel[0], 2);Serial.print(" : ");
  Serial.print(accel[1], 2);Serial.print(" : ");
  Serial.print(accel[2], 2);Serial.print("]");
  Serial.print("  Magneto[");
  Serial.print(magnetom[0], 2);Serial.print(" : ");
  Serial.print(magnetom[1], 2);Serial.print(" : ");
  Serial.print(magnetom[2], 2);Serial.print("]");
}

#endif
