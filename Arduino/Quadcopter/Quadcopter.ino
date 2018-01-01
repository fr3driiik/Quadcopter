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
 * I2C device found at address 0x42  ! = 0b 0100 0010, ublox m8n GPS
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
#include "Output.h"

#define DESIRED_HZ 50 //100 without prints
#define AVAIL_LOOP_TIME (1000 / DESIRED_HZ) //millis

#define OUTPUT__BAUD_RATE 57600
#define DEBUG_OUTPUT false
#define PRINT_SENSOR_DATA true

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
    init_sensors();
    initialize_receiver();
    initPids();
    setRatePidsOutputLimits(-40, 40); //direct engine influence
    setStablePidsOutputLimits(-202.5, 202.5); //max degrees per second to get right degree
    Serial.println("Ready for takeoff!");
    wdt_enable(WDTO_2S);
    timer = millis();
}

void loop() {       
    read_sensors();
    //FILTER THE DATA
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
    while(true){
      long timePassed = millis() - timer;
      if (timePassed >= AVAIL_LOOP_TIME){
        break;
      } else {
        loopTimeExceded = false;
      }
    }
    if (loopTimeExceded) {
      Serial.println("WARNING: Desired Hz could not be reached.");
    }

    timer = millis();
    wdt_reset(); //we are still alive  
    #if (DEBUG_OUTPUT || PRINT_PYR || PRINT_SENSOR_DATA)
      Serial.println("");
    #endif
} // loop()

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


