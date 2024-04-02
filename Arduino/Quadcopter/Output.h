#ifndef OUTPUT_H
#define OUTPUT_H
#include "Sensors.h"
#include "IMU.h"
#include "Receiver.h"
#include "GPS.h"

void print_sensor_data(){
  Serial.print(  "GyroX: ");Serial.print(Sensors::gyro[0], 2);
  Serial.print(", GyroY: ");Serial.print(Sensors::gyro[1], 2);
  Serial.print(", GyroZ: ");Serial.print(Sensors::gyro[2], 2);
  Serial.print(", AccelX: ");Serial.print(Sensors::accel[0], 2);
  Serial.print(", AccelY: ");Serial.print(Sensors::accel[1], 2);
  Serial.print(", AccelZ: ");Serial.print(Sensors::accel[2], 2);
  Serial.print(", MagnetoX: ");Serial.print(Sensors::magnetom[0], 2);
  Serial.print(", MagnetoY: ");Serial.print(Sensors::magnetom[1], 2);
  Serial.print(", MagentoZ: ");Serial.print(Sensors::magnetom[2], 2);
  Serial.print(", Temperature: ");Serial.print(Sensors::temperature, 2);
  Serial.print(", Pressure: ");Serial.print(Sensors::pressure, 2);
  Serial.println();
}

void print_pyr() {
  IMU::State s = IMU::state;
  String stringStart = "";
  Serial.println(stringStart + "roll:" + s.rollDegrees + ", pitch:" + s.pitchDegrees + ", yaw:" + s.yawDegrees);
}

void print_state() {
  IMU::State s = IMU::state;
  String stringStart = "";
  Serial.println(stringStart + "Pitch: " + s.pitchDegrees + " Yaw: " + s.yawDegrees + " Roll: " + s.rollDegrees);
  Serial.println(stringStart + "QX: " + s.qx + " QY: " + s.qy + " QZ: " + s.qz + " QW: " + s.qw);
  Serial.println(stringStart + "AccNorth: " + s.accNorth + " AccEast: " + s.accEast + " AccDown: " + s.accDown);
  Serial.println(stringStart + "VeloNorth: " + s.veloNorth + " VeloEast: " + s.veloEast + " VeloDown: " + s.veloDown);
  Serial.println(stringStart + "Longitude: " + s.longitude + " Latitude: " + s.latitude);
  Serial.println(stringStart + "Height: " + s.height);
}

void print_receiver_channels() {
  String stringStart = "";
  Serial.print(stringStart + "ch1:" + Receiver::channelsRaw[0] + ", ch2:" + Receiver::channelsRaw[1] + ", ch3:" + Receiver::channelsRaw[2] + ", ch4:" + Receiver::channelsRaw[3]);
  Serial.print(stringStart + ", ch5:" + Receiver::channelsRaw[4] + ", ch6:" + Receiver::channelsRaw[5] + ", ch7:" + Receiver::channelsRaw[6]);
  Serial.println(stringStart + ", ch8:" + Receiver::channelsRaw[7] + ", failsafe:" + (int)Receiver::failsafe);
}

void print_gps() {
  GPS::NAV_PVT pvt = GPS::getGPSMessage();
  Serial.print("ITOW: "); Serial.print(pvt.iTOW);
  Serial.print(" lat/long: "); Serial.print(pvt.lat/10000000.0f, 8); Serial.print(", "); Serial.print(pvt.lon/10000000.0f, 8);
  Serial.println();
}

#endif
