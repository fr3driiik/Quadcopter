#include <Arduino.h>
#include <avr/wdt.h>
#include <Wire.h>

#include "Sensors.h"
#include "IMU.h"
#include "Receiver.h"
#include "GPS.h"
#include "Utils.h"
#include "Output.h"
#include "Navigation.h"
#include "Control.h"

const uint32_t AVAIL_LOOP_TIME (1000000 / DESIRED_LOOP_HZ); //micros

unsigned long timer;
unsigned long timer1HZ;
unsigned long timer10HZ;

void setup() {
    wdt_disable();
    Serial.begin(OUTPUT_BAUD_RATE);
    Serial.println("\nInitializing..");
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    Sensors::initialize();
    #ifdef GPS
      GPS::initialize();
    #endif
    Receiver::initialize();
    Control::initialize();
    wdt_enable(WDTO_500MS);  // watchdog timer TODO not working for teensy
    Serial.println("Ready for takeoff!");
    timer = micros();
}

void loop() {    
    float deltaTime = (micros() - timer) / 1000000.0000f;
    timer = micros();
    Receiver::loop();
    Sensors::loop();

    float dt1HZ = (micros() - timer1HZ) / 1000000.0000f;
    if (dt1HZ >= 1.0f) {
      timer1HZ = micros();
      do1HZ(dt1HZ);
    }

    float dt10HZ = (micros() - timer10HZ) / 1000000.0000f;
    if (dt10HZ >= 0.01f) {
      timer10HZ = micros();
      do10HZ(dt10HZ);
    }
    
    IMU::update(deltaTime);
    Navigation::update(deltaTime);
    Control::update(deltaTime);

    //stable hz, wait for desired time
    long timeLeft = AVAIL_LOOP_TIME - (micros() - timer);
    if (timeLeft >= 0) {
      delayMicroseconds(timeLeft);
    }
    #if PRINT_LOOP_TIME
      String stringStart = "";
      Serial.println(stringStart + "loopTimeLeft:" + timeLeft);
    #endif

    wdt_reset(); //we are still alive  
} // loop()

inline void do1HZ(float dt) {
  #if PRINT_STATE
    print_state();
  #endif
}

inline void do10HZ(float dt) {
  #ifdef GPS
    if (GPS::read()) {
      #if PRINT_GPS_DATA
        print_gps();
      #endif
    }
  #endif
  #if PRINT_PYR_DATA
    print_pyr();
  #endif
  #if PRINT_SENSOR_DATA     
    print_sensor_data();
  #endif
  #if PRINT_RECEIVER_CHANNELS
    print_receiver_channels();
  #endif
}
