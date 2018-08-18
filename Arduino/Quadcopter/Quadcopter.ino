/*Quadcopter!
 * 
 * Sensorsticka I2C:
 * I2C device found at address 0x1E  ! = HMC5883L magnetometer
 * I2C device found at address 0x41  ! = BMA180 accelerometer
 * I2C device found at address 0x42  ! = 0b 0100 0010, ublox m8n GPS, uses serial1 instead
 * I2C device found at address 0x68  ! = ITG3050 gyro
 * I2C device found at address 0x76  ! = MS5611 Altimeter
 * 
 * AnvÃ¤nder arduinos inbyggda watchDogTimer
 */
#include <Arduino.h>
#include <avr/wdt.h>
#include <Wire.h>

#include "Sensors.h"
#include "Reciever.h"
#include "PIDs.h"
#include "GPS.h"
#include "Utils.h"
#include "Output.h"
#include "ESCManager.h"

#define I2C_SPEED 400000 //400kHz = fast mode
#define DESIRED_HZ 200 //5ms
#define AVAIL_LOOP_TIME (1000 / DESIRED_HZ) //millis

#define OUTPUT__BAUD_RATE 115200
#define DEBUG_OUTPUT false
#define PRINT_SENSOR_DATA false
#define PRINT_PYR_DATA false
#define PRINT_STATE true
#define PRINT_GPS_DATA false
#define PRINT_LOOP_TIME_OVER false

unsigned long timer;
unsigned long timer1HZ;
unsigned long timer10HZ;

void init_sensors() {
  Gyro_Init();
  Accel_Init();
  Magn_Init();
}

void read_sensors() {
  Read_Gyro();
  Read_Accel();
  Read_Magn();
}

void setup() {
    wdt_disable();
    Serial.begin(OUTPUT__BAUD_RATE);
    Serial.println("\nInitializing..");
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    init_sensors();
    GPS_initialize();
    Reciever_initialize();
    ESCManager_initialize();
    setRatePidsIntegralLimits(-40, 40); //direct engine influence
    setStablePidsIntegralLimits(-202.5, 202.5); //max degrees per second to get right degree
    Serial.println("Ready for takeoff!");
    wdt_enable(WDTO_500MS);
    timer = millis();
}

void loop() {    
    float deltaTime = (millis() - timer) / 1000.0000f;
    timer = millis();
    Reciever_loop();
    read_sensors();

    float dt1HZ = (micros() - timer1HZ) / 1000000.0000f;
    if (dt1HZ >= 1.0f) {
      timer1HZ = micros();
      do1HZ(dt1HZ);
    }

    float dt10HZ = (micros() - timer10HZ) / 1000000.0000f;
    if (dt10HZ >= 0.1f) {
      timer10HZ = micros();
      do10HZ(dt10HZ);
    }
    
    if (GPS_read()) { //maybe move to 10hz?
      #if PRINT_GPS_DATA
        print_gps();
      #endif
    }
    IMU_update(deltaTime);
    #if PRINT_PYR_DATA
      print_pyr();
    #endif
    #if PRINT_SENSOR_DATA     
      print_sensor_data();
    #endif

    if(throttleIn > 0.05){ //calc PIDS and run engines accordingly

      //calc stab pids
      computeStabPids(deltaTime);

      //yaw change desired? overwrite stab output
      if(abs(yawIn) > 0.05){
        yaw_stab_output = fromDecimalPercent(yawIn, -YAW_MAX_DPS, YAW_MAX_DPS);
        yaw_target = yawDegrees;
      }

      //calc stab pids
      computeRatePids(deltaTime);

      ESCManager_setInput(pitch_output, roll_output, yaw_output, throttleIn); //should actually feed values in % (pyr -100 - 100, throttle 0-100)
    } else { //too low throttle
      ESCManager_tooLowThrottle();
      if (DEBUG_OUTPUT)
        Serial.print("Engines off   ");

      //reset yaw
      yaw_target = yawDegrees;

      //reset pids
      resetPids();
    }

    //stable hz, wait for desired time
    bool loopTimeExceded = true;
    #if PRINT_LOOP_TIME_OVER
      Serial.print("w8:");Serial.print(AVAIL_LOOP_TIME - (millis() - timer));Serial.println();
    #endif
    while(true){
      long timePassed = millis() - timer;
      if (timePassed >= AVAIL_LOOP_TIME){
        break;
      } else {
        loopTimeExceded = false;
      }
    }
    if (loopTimeExceded) {
      Serial.println("#:TimeExceed");
    }

    wdt_reset(); //we are still alive  
} // loop()

inline void do1HZ(float dt) {
  #if PRINT_STATE
    print_state();
  #endif
}

inline void do10HZ(float dt) {
  
}
