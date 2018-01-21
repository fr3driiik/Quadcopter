/*Quadcopter!
 * 
 * ESC settings:
 * stop/arm   1060 Âµs = 132 analogWrite Value
 * full power 1860 Âµs = 232,407 analogWrite Value. Detta ger endast 100 steg, har mÃ¶jlighet fÃ¶r 800st
 * PWM freq for pin 3, 5, 6, 9 (timer 2, 3, 4) = 490 Hz, every Pulse = 2040.8163265306 Âµs
 * analogWrite(pin, value), 0 off, 255 on
 * 
 * RC reviever 8 ch PIN: 31, 33, 35, 37, 39, 41, 43, 45
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

#include "PinChangeInt.h"
#include "Sensors.h"
#include "Reciever.h"
#include "PIDs.h"
#include "GPS.h"
#include "Utils.h"
#include "Output.h"

#define I2C_SPEED 400000 //400kHz = fast mode
#define DESIRED_HZ 200 //5ms
#define AVAIL_LOOP_TIME (1000 / DESIRED_HZ) //millis

#define OUTPUT__BAUD_RATE 115200
#define DEBUG_OUTPUT false
#define PRINT_SENSOR_DATA false
#define PRINT_PYR_DATA true
#define PRINT_GPS_DATA false
#define PRINT_LOOP_TIME_OVER false

unsigned long timer;

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
    init_gps();
    initialize_receiver();
    initPids();
    setRatePidsOutputLimits(-40, 40); //direct engine influence
    setStablePidsOutputLimits(-202.5, 202.5); //max degrees per second to get right degree
    Serial.println("Ready for takeoff!");
    IMU_init();
    wdt_enable(WDTO_500MS);
    timer = millis();
}

void loop() {       
    read_sensors();
    if (read_gps()) {
      #if PRINT_GPS_DATA
        print_gps();
      #endif
    }
    IMU_calculate();
    #if PRINT_PYR_DATA
      print_pyr();
    #endif
    #if PRINT_SENSOR_DATA
      print_sensor_data();
    #endif

    if(RCthrottle > 140){ //calc PIDS and run engines accordingly

      //calc stab pids
      computeStabPids();

      //yaw change desired? overwrite stab output
      if(abs(RCyaw) > 5){
        yaw_stab_output = RCyaw;
        yaw_target = yawDegrees;
      }

      //calc stab pids
      computeRatePids();

      //calc engine values
      long motor_FR_output = RCthrottle - roll_output - pitch_output - yaw_output;
      long motor_RL_output = RCthrottle + roll_output + pitch_output - yaw_output;
      long motor_FL_output = RCthrottle + roll_output - pitch_output + yaw_output;
      long motor_RR_output = RCthrottle - roll_output + pitch_output + yaw_output; 

      motor_FR_output = constrain(motor_FR_output, ESC_MIN, ESC_MAX);
      motor_RL_output = constrain(motor_RL_output, ESC_MIN, ESC_MAX);
      motor_FL_output = constrain(motor_FL_output, ESC_MIN, ESC_MAX);
      motor_RR_output = constrain(motor_RR_output, ESC_MIN, ESC_MAX);
  
      //send engine values
      analogWrite(MOTOR_FR_OUT_PIN, motor_FR_output);
      analogWrite(MOTOR_RL_OUT_PIN, motor_RL_output);
      analogWrite(MOTOR_FL_OUT_PIN, motor_FL_output);
      analogWrite(MOTOR_RR_OUT_PIN, motor_RR_output); 
    } else { //too low throttle
      //send engine values
      analogWrite(MOTOR_FR_OUT_PIN, ESC_MIN);
      analogWrite(MOTOR_RL_OUT_PIN, ESC_MIN);
      analogWrite(MOTOR_FL_OUT_PIN, ESC_MIN);
      analogWrite(MOTOR_RR_OUT_PIN, ESC_MIN);
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
      Serial.println("#:LOOP");
    }

    timer = millis();
    wdt_reset(); //we are still alive  
} // loop()
