#ifndef RECEIVER_H
#define RECEIVER_H

#include "Utils.h"
#include "Config.h"
#if defined(Arduino_Mega_2560) || defined(Arduino_Pro_Micro)
  #include <EnableInterrupt.h>
#endif

/* 
 * RC reviever 8 ch PIN: 31, 33, 35, 37, 39, 41, 43, 45
 */

//flags for channels to indicate new signal
#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define CHANNEL3_FLAG 4
#define CHANNEL4_FLAG 8
#define CHANNEL5_FLAG 16
#define CHANNEL6_FLAG 32
#define CHANNEL7_FLAG 64
#define CHANNEL8_FLAG 128

namespace Receiver {
  
  boolean failsafe;
  
  //holds time of last received transmission
  uint32_t timeOfLastTransmission;
  
  //keeper of the flags above, max 8 channels because 8bits
  volatile uint8_t channelFlagsShared;
  
  //RCinput signal length in %
  float rollIn = 0;
  float pitchIn = 0;
  float throttleIn = 0;
  float yawIn = 0;
  
  //RCinput signal length in micros
  uint16_t channel1Raw;
  uint16_t channel2Raw;
  uint16_t channel3Raw;
  uint16_t channel4Raw;
  uint16_t channel5Raw;
  uint16_t channel6Raw;
  uint16_t channel7Raw;
  uint16_t channel8Raw;
  
  //shared variables are updated by the ISR and ONLY read by the loop. Length of signal in micros.
  //Add for all channels!!!
  volatile uint16_t channel1Shared;
  volatile uint16_t channel2Shared;
  volatile uint16_t channel3Shared;
  volatile uint16_t channel4Shared;
  volatile uint16_t channel5Shared;
  volatile uint16_t channel6Shared;
  volatile uint16_t channel7Shared;
  volatile uint16_t channel8Shared;
  
  //holds time for the different channels signals
  uint32_t channel1Start;
  uint32_t channel2Start;
  uint32_t channel3Start;
  uint32_t channel4Start;
  uint32_t channel5Start;
  uint32_t channel6Start;
  uint32_t channel7Start;
  uint32_t channel8Start;
  
  //---------------------------------------------------
  //--------------INTERUPT HANDLERS--------------------
  //---------------------------------------------------
  void calcChannel1(){
      if(digitalReadFast(CHANNEL1_INPUT_PIN) == HIGH){ //start of signal
          channel1Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel1Shared = (uint16_t)(micros() - channel1Start);
          channelFlagsShared |= CHANNEL1_FLAG;
      }
  }
  void calcChannel2(){
      if(digitalReadFast(CHANNEL2_INPUT_PIN) == HIGH){ //start of signal
          channel2Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel2Shared = (uint16_t)(micros() - channel2Start);
          channelFlagsShared |= CHANNEL2_FLAG;
      }
  }
  void calcChannel3(){
      if(digitalReadFast(CHANNEL3_INPUT_PIN) == HIGH){ //start of signal
          channel3Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel3Shared = (uint16_t)(micros() - channel3Start);
          channelFlagsShared |= CHANNEL3_FLAG;
      }
  }
  void calcChannel4(){
      if(digitalReadFast(CHANNEL4_INPUT_PIN) == HIGH){ //start of signal
          channel4Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel4Shared = (uint16_t)(micros() - channel4Start);
          channelFlagsShared |= CHANNEL4_FLAG;
      }
  }
  void calcChannel5(){
      if(digitalReadFast(CHANNEL5_INPUT_PIN) == HIGH){ //start of signal
          channel5Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel5Shared = (uint16_t)(micros() - channel5Start);
          channelFlagsShared |= CHANNEL5_FLAG;
      }
  }
  void calcChannel6(){
      if(digitalReadFast(CHANNEL6_INPUT_PIN) == HIGH){ //start of signal
          channel6Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel6Shared = (uint16_t)(micros() - channel6Start);
          channelFlagsShared |= CHANNEL6_FLAG;
      }
  }
  void calcChannel7(){
      if(digitalReadFast(CHANNEL7_INPUT_PIN) == HIGH){ //start of signal
          channel7Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel7Shared = (uint16_t)(micros() - channel7Start);
          channelFlagsShared |= CHANNEL7_FLAG;
      }
  }
  void calcChannel8(){
      if(digitalReadFast(CHANNEL8_INPUT_PIN) == HIGH){ //start of signal
          channel8Start = micros();
      }else{ //signal ended. Save signal and set flag
          channel8Shared = (uint16_t)(micros() - channel8Start);
          channelFlagsShared |= CHANNEL8_FLAG;
      }
  }

  void initialize() {
    pinMode(CHANNEL1_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL2_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL3_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL4_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL5_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL6_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL7_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL8_INPUT_PIN, INPUT_PULLUP);
    #if defined(Teensy_3_6) || defined(Teensy_4_0)
      attachInterrupt(CHANNEL1_INPUT_PIN, calcChannel1, CHANGE);
      attachInterrupt(CHANNEL2_INPUT_PIN, calcChannel2, CHANGE);
      attachInterrupt(CHANNEL3_INPUT_PIN, calcChannel3, CHANGE);
      attachInterrupt(CHANNEL4_INPUT_PIN, calcChannel4, CHANGE);
      attachInterrupt(CHANNEL5_INPUT_PIN, calcChannel5, CHANGE);
      attachInterrupt(CHANNEL6_INPUT_PIN, calcChannel6, CHANGE);
      attachInterrupt(CHANNEL7_INPUT_PIN, calcChannel7, CHANGE);
      attachInterrupt(CHANNEL8_INPUT_PIN, calcChannel8, CHANGE);
    #else
      enableInterrupt(CHANNEL1_INPUT_PIN, calcChannel1, CHANGE);
      enableInterrupt(CHANNEL2_INPUT_PIN, calcChannel2, CHANGE);
      enableInterrupt(CHANNEL3_INPUT_PIN, calcChannel3, CHANGE);
      enableInterrupt(CHANNEL4_INPUT_PIN, calcChannel4, CHANGE);
    #endif
  }
  
  void loop() {
      static uint8_t channelFlags;
  
      if(timeOfLastTransmission == 0 || (micros() - timeOfLastTransmission) > FAILSAFE_DELAY){ //transmitter not connected
          failsafe = true;
  
          //make stable
          rollIn = 0; 
          pitchIn = 0;
  
          //set throttle
          throttleIn = 0; //engine shutoff, maybe set to 20-30%?
      }else{ 
        failsafe = false;
      }
      failsafe=false;
      
      if(channelFlagsShared && !failsafe){ //handle interupted pins
        noInterrupts();

        //copy changed shared variables
        channelFlags = channelFlagsShared;

        if(channelFlags & CHANNEL1_FLAG){
            channel1Raw = channel1Shared;
        }
        if(channelFlags & CHANNEL2_FLAG){
            channel2Raw = channel2Shared;
        }
        if(channelFlags & CHANNEL3_FLAG){
            channel3Raw = channel3Shared;
        }
        if(channelFlags & CHANNEL4_FLAG){
            channel4Raw = channel4Shared;
        }
        if(channelFlags & CHANNEL5_FLAG){
            channel5Raw = channel5Shared;
        }
        if(channelFlags & CHANNEL6_FLAG){
            channel6Raw = channel6Shared;
        }
        if(channelFlags & CHANNEL7_FLAG){
            channel7Raw = channel7Shared;
        }
        if(channelFlags & CHANNEL8_FLAG){
            channel8Raw = channel8Shared;
        }

        channelFlagsShared = 0;
        interrupts();
      }
      rollIn = map(channel1Raw, RCRECEIVER_MIN, RCRECEIVER_MAX, -45, 45);
      pitchIn = -map(channel2Raw, RCRECEIVER_MIN, RCRECEIVER_MAX, -45, 45);
      throttleIn = Utils::toDecimalPercent(channel4Raw, RCRECEIVER_MIN, RCRECEIVER_MAX);
      yawIn = -map(channel3Raw, RCRECEIVER_MIN, RCRECEIVER_MAX, -135, 135); 
  }
}
#endif
