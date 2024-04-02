#ifndef CONFIG_H
#define CONFIG_H

//This file contains all configuration

//Select board
//#define Arduino_Pro_Micro
//#define Arduino_Mega_2560
//#define Teensy_3_6
#define Teensy_4_0

//Timings
#define I2C_SPEED 400000 //400kHz = fast mode
#define DESIRED_LOOP_HZ 500 //2ms

//Sensors
#define MAGNETOMETER
#define EARTH_MAGNETIC_FIELD_STRENGTH 466.9  // mgauss
#define MAGNETIC_FIELD_MAX_DIFF 40  // mgauss
//#define GPS
//Select sensor module
//#define CSGSHOP_11_DOF
//#define LSM9DS0
#define ALTIMU_10_V5

//Rotations
#define ROLL_PITCH_MAX_ANGLE 45
#define ROLL_PITCH_MAX_DPS 200
#define YAW_MAX_DPS 135

//ESC
//#define ESC_SIGNAL_PWM
#define ESC_SIGNAL_DSHOT
#define DSHOT_BITRATE 300000  // bit/s. dshot ESC usually supports 150k, 300k, 600k and 1200k.
#define ESC_FL_PIN 4
#define ESC_FR_PIN 8
#define ESC_RL_PIN 22
#define ESC_RR_PIN 9

//PID
#define PID_RATE_OUTPUT_LIMIT 100
#define PID_RATE_INTERGRAL_LIMIT 100
#define PID_ANGLE_OUTPUT_LIMIT 100

//RECEIVER
#define PITCH_CHANNEL 3
#define ROLL_CHANNEL 1
#define YAW_CHANNEL 4
#define THROTTLE_CHANNEL 2
#define MODE_CHANNEL 5
#define RCRECEIVER_MIN 1050
#define RCRECEIVER_MAX 1880
#define THROTTLE_DEAD_BAND 50
#define THROTTLE_CAP 2 * PID_RATE_OUTPUT_LIMIT
#define FAILSAFE_DELAY 300000 // us. after 0,3 seconds of no signal on choosen channel, in this case ch3 because ch 4 is receiveing last value..
#define FAILSAFE_CHANNEL 3
#define CHANNEL1_INPUT_PIN 0
#define CHANNEL2_INPUT_PIN 1
#define CHANNEL3_INPUT_PIN 2
#define CHANNEL4_INPUT_PIN 3
#define CHANNEL5_INPUT_PIN 5
#define CHANNEL6_INPUT_PIN 6
#define CHANNEL7_INPUT_PIN 7
#define CHANNEL8_INPUT_PIN 10

//Battery
#define VOLTAGE_MAX (4 * 4.2) //4cell LiPo
#define VOLTAGE_LOW (4 * 3.5)
#define VOLTAGE_KILL (4 * 3.2)
#define BATTERY_PIN 3

// Output
#define OUTPUT_BAUD_RATE 115200
#define PRINT_SENSOR_DATA false
#define PRINT_PYR_DATA false
#define PRINT_RECEIVER_CHANNELS false
#define PRINT_STATE false
#define PRINT_ESC_OUTPUT true
#define PRINT_GPS_DATA false
#define PRINT_LOOP_TIME false

#endif
