#ifndef OUTPUT_H
#define OUTPUT_H
#include "Sensors.h"
#include "IMU.h"
#include "GPS.h"

void print_sensor_data(){
  Serial.print("SENSORS[x:y:z]");
  Serial.print("  Gyro[");
  Serial.print(Sensors::gyro[0], 2);Serial.print(" : ");
  Serial.print(Sensors::gyro[1], 2);Serial.print(" : ");
  Serial.print(Sensors::gyro[2], 2);Serial.print("]");
  Serial.print("  Accel[");
  Serial.print(Sensors::accel[0], 2);Serial.print(" : ");
  Serial.print(Sensors::accel[1], 2);Serial.print(" : ");
  Serial.print(Sensors::accel[2], 2);Serial.print("]");
  Serial.print("  Magneto[");
  Serial.print(Sensors::magnetom[0], 2);Serial.print(" : ");
  Serial.print(Sensors::magnetom[1], 2);Serial.print(" : ");
  Serial.print(Sensors::magnetom[2], 2);Serial.print("]");
  Serial.println();
}

void print_pyr() {
  IMU::State s = IMU::getState();
  Serial.print("p:");Serial.print(s.pitchDegrees);Serial.print("y:");Serial.print(s.yawDegrees);Serial.print("r:");Serial.print(s.rollDegrees);
  Serial.println();
}

void print_state() {
  IMU::State s = IMU::getState();
  String stringStart = "";
  //we dont print quaternion or rotationMatrix, they are not understandable..
  Serial.println(stringStart + "Pitch: " + s.pitchDegrees + " Yaw: " + s.yawDegrees + " Roll: " + s.rollDegrees);
  Serial.println(stringStart + "AccNorth: " + s.accNorth + " AccEast: " + s.accEast + " AccDown: " + s.accDown);
  Serial.println(stringStart + "VeloNorth: " + s.veloNorth + " VeloEast: " + s.veloEast + " VeloDown: " + s.veloDown);
  Serial.println(stringStart + "Longitude: " + s.longitude + " Latitude: " + s.latitude);
  Serial.println(stringStart + "Height: " + s.height);
}

void print_gps() {
  GPS::NAV_PVT pvt = GPS::getGPSMessage();
  Serial.print("ITOW: "); Serial.print(pvt.iTOW);
  Serial.print(" lat/long: "); Serial.print(pvt.lat/10000000.0f, 8); Serial.print(", "); Serial.print(pvt.lon/10000000.0f, 8);
  Serial.println();
}

#endif
