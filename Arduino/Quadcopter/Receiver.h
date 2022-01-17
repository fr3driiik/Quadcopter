#ifndef RECEIVER_H
#define RECEIVER_H

#include "Utils.h"
#include "Config.h"
#if defined(Arduino_Mega_2560) || defined(Arduino_Pro_Micro)
  #include <EnableInterrupt.h>
#endif

//flags for channels to indicate new signal
#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define CHANNEL3_FLAG 4
#define CHANNEL4_FLAG 8
#define CHANNEL5_FLAG 16
#define CHANNEL6_FLAG 32
#define CHANNEL7_FLAG 64
#define CHANNEL8_FLAG 128

#define PWM_INTERRUPT_HANDLER(channel, pin, flag) \
void handle_channel_interrupt_##channel(void){ \
  if(digitalReadFast(pin) == HIGH){ \
      channelsStart[channel-1] = micros(); \
  }else{ \
      channelsShared[channel-1] = (uint16_t)(micros() - channelsStart[channel-1]); \
      channelFlagsShared |= flag; \
  } \
}


namespace Receiver {
  
  boolean failsafe;
  
  //keeper of the flags above, max 8 channels because 8bits
  volatile uint8_t channelFlagsShared;
  
  //RCinput signal length in %
  float rollIn = 0;
  float pitchIn = 0;
  float throttleIn = 0;
  float yawIn = 0;
  
  //RCinput signal length in micros
  uint16_t channelsRaw[8];
  
  //shared variables are updated by the ISR and ONLY read by the loop. Length of signal in micros.
  //Add for all channels!!!
  volatile uint16_t channelsShared[8];
  
  //holds time for the different channels signals
  uint32_t channelsStart[8];
  
  PWM_INTERRUPT_HANDLER(1, CHANNEL1_INPUT_PIN, CHANNEL1_FLAG);
  PWM_INTERRUPT_HANDLER(2, CHANNEL2_INPUT_PIN, CHANNEL2_FLAG);
  PWM_INTERRUPT_HANDLER(3, CHANNEL3_INPUT_PIN, CHANNEL3_FLAG);
  PWM_INTERRUPT_HANDLER(4, CHANNEL4_INPUT_PIN, CHANNEL4_FLAG);
  PWM_INTERRUPT_HANDLER(5, CHANNEL5_INPUT_PIN, CHANNEL5_FLAG);
  PWM_INTERRUPT_HANDLER(6, CHANNEL6_INPUT_PIN, CHANNEL6_FLAG);
  PWM_INTERRUPT_HANDLER(7, CHANNEL7_INPUT_PIN, CHANNEL7_FLAG);
  PWM_INTERRUPT_HANDLER(8, CHANNEL8_INPUT_PIN, CHANNEL8_FLAG);

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
      attachInterrupt(CHANNEL1_INPUT_PIN, handle_channel_interrupt_1, CHANGE);
      attachInterrupt(CHANNEL2_INPUT_PIN, handle_channel_interrupt_2, CHANGE);
      attachInterrupt(CHANNEL3_INPUT_PIN, handle_channel_interrupt_3, CHANGE);
      attachInterrupt(CHANNEL4_INPUT_PIN, handle_channel_interrupt_4, CHANGE);
      attachInterrupt(CHANNEL5_INPUT_PIN, handle_channel_interrupt_5, CHANGE);
      attachInterrupt(CHANNEL6_INPUT_PIN, handle_channel_interrupt_6, CHANGE);
      attachInterrupt(CHANNEL7_INPUT_PIN, handle_channel_interrupt_7, CHANGE);
      attachInterrupt(CHANNEL8_INPUT_PIN, handle_channel_interrupt_8, CHANGE);
    #else
      enableInterrupt(CHANNEL1_INPUT_PIN, handle_channel_interrupt_1, CHANGE);
      enableInterrupt(CHANNEL2_INPUT_PIN, handle_channel_interrupt_2, CHANGE);
      enableInterrupt(CHANNEL3_INPUT_PIN, handle_channel_interrupt_3, CHANGE);
      enableInterrupt(CHANNEL4_INPUT_PIN, handle_channel_interrupt_4, CHANGE);
    #endif
  }
  
  void loop() {
      static uint8_t channelFlags;
      uint32_t timeOfLastTransmission = channelsStart[FAILSAFE_CHANNEL-1];
      if(timeOfLastTransmission == 0 || (micros() - timeOfLastTransmission) > FAILSAFE_DELAY){ //transmitter not connected
          failsafe = true;
  
          //make stable
          rollIn = 0; 
          pitchIn = 0;
          throttleIn = 0; //engine shutoff, maybe set to 20-30%?
      }else{ 
        failsafe = false;
      }
      
      if(channelFlagsShared && !failsafe){ //handle interupted pins
        noInterrupts();

        //copy changed shared variables
        channelFlags = channelFlagsShared;
        for (uint8_t i = 0; i < 8; i++) {
          if (channelFlags & (0x1 << i)) {
            channelsRaw[i] = channelsShared[i];
          }
        }

        channelFlagsShared = 0;
        interrupts();
      }
      rollIn = map(channelsRaw[0], RCRECEIVER_MIN, RCRECEIVER_MAX, -45, 45);
      pitchIn = -map(channelsRaw[1], RCRECEIVER_MIN, RCRECEIVER_MAX, -45, 45);
      throttleIn = Utils::toDecimalPercent(channelsRaw[3], RCRECEIVER_MIN, RCRECEIVER_MAX);
      yawIn = -map(channelsRaw[2], RCRECEIVER_MIN, RCRECEIVER_MAX, -135, 135); 
  }
}
#endif
