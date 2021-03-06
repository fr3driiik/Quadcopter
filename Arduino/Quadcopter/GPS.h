#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

namespace GPS {
  const unsigned char UBX_HEADER[] = {0xB5, 0x62};
  
  struct NAV_PVT {
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
    
    unsigned short year;         // Year (UTC) 
    unsigned char month;         // Month, range 1..12 (UTC)
    unsigned char day;           // Day of month, range 1..31 (UTC)
    unsigned char hour;          // Hour of day, range 0..23 (UTC)
    unsigned char minute;        // Minute of hour, range 0..59 (UTC)
    unsigned char second;        // Seconds of minute, range 0..60 (UTC)
    char valid;                  // Validity Flags (see graphic below)
    unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
    long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    unsigned char fixType;       // GNSSfix Type, range 0..5
    char flags;                  // Fix Status Flags
    char flags2;                 // Aditional flags
    unsigned char numSV;         // Number of satellites used in Nav Solution
    
    long lon;                    // Longitude (deg)
    long lat;                    // Latitude (deg)
    long height;                 // Height above Ellipsoid (mm)
    long hMSL;                   // Height above mean sea level (mm)
    unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
    unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
    
    long velN;                   // NED north velocity (mm/s)
    long velE;                   // NED east velocity (mm/s)
    long velD;                   // NED down velocity (mm/s)
    long gSpeed;                 // Ground Speed (2-D) (mm/s)
    long headingMotion;          // Heading of motion 2-D (deg)
    unsigned long sAcc;          // Speed Accuracy Estimate
    unsigned long headingAcc;    // Heading Accuracy Estimate
    unsigned short pDOP;         // Position dilution of precision
    unsigned char reserved1[6];    // Reserved
    long headingVehicle;         // Heading of vehicle
    short magDec;                // Magnetic declination
    unsigned short magAcc;       // Magnetic declination accuracy
  };
  
  void initialize();
  NAV_PVT getGPSMessage();
  bool read();
}

#endif

