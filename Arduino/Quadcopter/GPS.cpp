#include "GPS.h"

GPS::NAV_PVT pvt;

void GPS::initialize() {
  Serial1.begin(9600);
  Serial.println("GPS serial initialized");
  delay(5);
}

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(GPS::NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

GPS::NAV_PVT GPS::getGPSMessage() {
  return pvt;
}

bool GPS::read() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(GPS::NAV_PVT);

  while ( Serial1.available() ) {
    byte c = Serial1.read();
    if ( fpos < 2 ) {
      if ( c == GPS::UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

