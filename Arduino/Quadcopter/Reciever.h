#ifndef RECIEVER_H
#define RECIEVER_H

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

//FAILSAFE var
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
uint16_t pitchInRaw;
uint16_t yawInRaw;
uint16_t rollInRaw;
uint16_t throttleInRaw;

//shared variables are updated by the ISR and ONLY read by the loop. Length of signal in micros.
//Add for all channels!!!
volatile uint16_t pitchInShared;
volatile uint16_t yawInShared;
volatile uint16_t rollInShared;
volatile uint16_t throttleInShared;

//holds time for the different channels signals
uint32_t pitchStart;
uint32_t yawStart;
uint32_t rollStart;
uint32_t throttleStart;

//---------------------------------------------------
//--------------INTERUPT HANDLERS--------------------
//---------------------------------------------------
void calcChannel1(){
    if(digitalRead(CHANNEL1_INPUT_PIN) == HIGH){ //start of signal
        rollStart = micros();
    }else{ //signal ended. Save signal and set flag
        rollInShared = (uint16_t)(micros() - rollStart);
        channelFlagsShared |= CHANNEL1_FLAG;
    }
}
void calcChannel2(){
    if(digitalRead(CHANNEL2_INPUT_PIN) == HIGH){ //start of signal
        pitchStart = micros();
    }else{ //signal ended. Save signal and set flag
        pitchInShared = (uint16_t)(micros() - pitchStart);
        channelFlagsShared |= CHANNEL2_FLAG;
    }
}
void calcChannel3(){
    if(digitalRead(CHANNEL3_INPUT_PIN) == HIGH){ //start of signal
        throttleStart = micros();
    }else{ //signal ended. Save signal and set flag
        throttleInShared = (uint16_t)(micros() - throttleStart);
        channelFlagsShared |= CHANNEL3_FLAG;
    }
    timeOfLastTransmission = micros(); // This channel keeps track of last received transmission
}
void calcChannel4(){
    if(digitalRead(CHANNEL4_INPUT_PIN) == HIGH){ //start of signal
        yawStart = micros();
    }else{ //signal ended. Save signal and set flag
        yawInShared = (uint16_t)(micros() - yawStart);
        channelFlagsShared |= CHANNEL4_FLAG;
    }
}

//INITIALIZE
void Reciever_initialize() {
  #ifdef Teensy_3_6
    attachInterrupt(CHANNEL1_INPUT_PIN, calcChannel1, CHANGE);
    attachInterrupt(CHANNEL2_INPUT_PIN, calcChannel2, CHANGE);
    attachInterrupt(CHANNEL3_INPUT_PIN, calcChannel3, CHANGE);
    attachInterrupt(CHANNEL4_INPUT_PIN, calcChannel4, CHANGE);
  #else
    pinMode(CHANNEL1_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL2_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL3_INPUT_PIN, INPUT_PULLUP);
    pinMode(CHANNEL4_INPUT_PIN, INPUT_PULLUP);
    enableInterrupt(CHANNEL1_INPUT_PIN, calcChannel1, CHANGE);
    enableInterrupt(CHANNEL2_INPUT_PIN, calcChannel2, CHANGE);
    enableInterrupt(CHANNEL3_INPUT_PIN, calcChannel3, CHANGE);
    enableInterrupt(CHANNEL4_INPUT_PIN, calcChannel4, CHANGE);
  #endif
}

void Reciever_loop() {
    static uint8_t channelFlags;

    if(timeOfLastTransmission == 0 || (micros() - timeOfLastTransmission) > FAILSAFE_DELAY){ //transmitter not connected
        failsafe = true; //transmitter not connected

        //make stable
        rollIn = 0; 
        pitchIn = 0;

        //set throttle
        throttleIn = 0; //engine shutoff, maybe set to 20-30%?
    }else{ 
      failsafe = false;
    }
    
    if(channelFlagsShared && !failsafe){ //handle interupted pins
        noInterrupts();

        //copy changed shared variables
        channelFlags = channelFlagsShared;

        if(channelFlags & CHANNEL1_FLAG){
            rollInRaw = rollInShared;
        }
        if(channelFlags & CHANNEL2_FLAG){
            pitchInRaw = pitchInShared;
        }
        if(channelFlags & CHANNEL3_FLAG){
            throttleInRaw = throttleInShared;
        }
        if(channelFlags & CHANNEL4_FLAG){
            yawInRaw = yawInShared;
        }
        channelFlagsShared = 0;

        interrupts();
    }

    rollIn = map(rollInRaw, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
    pitchIn = -map(pitchInRaw, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
    throttleIn = toDecimalPercent(throttleInRaw, RCRECIEVER_MIN, RCRECIEVER_MAX);
    yawIn = -map(yawInRaw, RCRECIEVER_MIN, RCRECIEVER_MAX, -135, 135); 
}

#endif

