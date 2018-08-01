#ifndef CONFIG_H
#define CONFIG_H

#include "ESC.h"
//This file contains all configuration

//Select board
//#define Arduino_Pro_Micro
#define Arduino_Mega_2560
//#define Teensy_3_6

//Rotations
#define ROLL_PITCH_MAX_ANGLE 45
#define YAW_MAX_DPS 135

//RECIEVER
#define RCRECIEVER_MIN 1050
#define RCRECIEVER_MAX 1880
#define FAILSAFE_DELAY 300000 // after 0,3 seconds of no signal on choosen channel, in this case ch3 because ch 4 is receiveing last value..
#ifdef Arduino_Pro_Micro
  #define CHANNEL1_INPUT_PIN A0 //roll
  #define CHANNEL2_INPUT_PIN A1 //pitch
  #define CHANNEL3_INPUT_PIN A2 //throttle
  #define CHANNEL4_INPUT_PIN A3//yaw
  #define CHANNEL5_INPUT_PIN A4
  #define CHANNEL6_INPUT_PIN A5
  #define CHANNEL7_INPUT_PIN A6
  #define CHANNEL8_INPUT_PIN A7
#elif defined(Arduino_Mega_2560)
  #define CHANNEL1_INPUT_PIN A15 //roll
  #define CHANNEL2_INPUT_PIN A14 //pitch
  #define CHANNEL3_INPUT_PIN A13 //throttle
  #define CHANNEL4_INPUT_PIN A12 //yaw
  #define CHANNEL5_INPUT_PIN A11
  #define CHANNEL6_INPUT_PIN A10
  #define CHANNEL7_INPUT_PIN A9
  #define CHANNEL8_INPUT_PIN A8
#elif defined(Teensy_3_6)
  #define CHANNEL1_INPUT_PIN A15 //roll
  #define CHANNEL2_INPUT_PIN A14 //pitch
  #define CHANNEL3_INPUT_PIN A13 //throttle
  #define CHANNEL4_INPUT_PIN A12 //yaw
  #define CHANNEL5_INPUT_PIN A11
  #define CHANNEL6_INPUT_PIN A10
  #define CHANNEL7_INPUT_PIN A9
  #define CHANNEL8_INPUT_PIN A8
#endif

//ESC
#define ESC_COUNT 4
#ifdef Arduino_Pro_Micro
  const ESC escs[] = {
    {3, -1.0, -1.0, -1.0}, //FR
    {5, 1.0, 1.0, -1.0},   //RR
    {6, 1.0, -1.0, 1.0},   //RL
    {9, -1.0, 1.0, 1.0}    //FL
  };
#elif defined(Arduino_Mega_2560)
  const ESC escs[] = {
    {3, -1.0, -1.0, -1.0}, //FR
    {5, 1.0, 1.0, -1.0},   //RR
    {6, 1.0, -1.0, 1.0},   //RL
    {9, -1.0, 1.0, 1.0}    //FL
  };
#elif defined(Teensy_3_6)
  const ESC escs[] = {
    {3, -1.0, -1.0, -1.0}, //FR
    {5, 1.0, 1.0, -1.0},   //RR
    {6, 1.0, -1.0, 1.0},   //RL
    {9, -1.0, 1.0, 1.0}    //FL
  };
#endif

//Battery
#define BATTERY_PIN 3
#define VOLTAGE_DIVIDER 5

#endif
