#ifndef RECIEVER_H
#define RECIEVER_H

//inputsPins
#define CHANNEL1_IN_PIN A15 // ROLL
#define CHANNEL2_IN_PIN A14 // PITCH
#define CHANNEL3_IN_PIN A13 // THROTTLE
#define CHANNEL4_IN_PIN A12 // YAW
#define CHANNEL5_IN_PIN A11
#define CHANNEL6_IN_PIN A10
#define CHANNEL7_IN_PIN A9
#define CHANNEL8_IN_PIN A8

//flags for channels to indicate new signal
#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define CHANNEL3_FLAG 4
#define CHANNEL4_FLAG 8
#define CHANNEL5_FLAG 16
#define CHANNEL6_FLAG 32
#define CHANNEL7_FLAG 64
#define CHANNEL8_FLAG 128

//outputPins
#define MOTOR_FR_OUT_PIN 3
#define MOTOR_RL_OUT_PIN 5
#define MOTOR_FL_OUT_PIN 6
#define MOTOR_RR_OUT_PIN 9

//values
#define ESC_MIN 132
#define ESC_MAX 232
#define ESC_MID (ESC_MAX - ESC_MIN)

#define RCRECIEVER_MIN 1050
#define RCRECIEVER_MAX 1880
#define FAILSAFE_DELAY 300000 // after 0,3 seconds of no signal on choosen channel, in this case ch3 because ch 4 is receiveing last value..

//FAILSAFE var
boolean failsafe;

//holds time of last received transmission
uint32_t timeOfLastTransmission;

//keeper of the flags above, max 8 channels because 8bits
volatile uint8_t channelFlagsShared;

//RCinput
float RCroll = 0;
float RCpitch = 0;
float RCthrottle = 0;
float RCyaw = 0;

uint16_t pitchIn;
uint16_t yawIn;
uint16_t rollIn;
uint16_t throttleIn;

//shared variables are updated by the ISR and ONLY read by the loop
//¤add for all channels!!!¤
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
    if(digitalRead(CHANNEL1_IN_PIN) == HIGH){ //start of signal
        rollStart = micros();
    }else{ //signal ended. Save signal and set flag
        rollInShared = (uint16_t)(micros() - rollStart);
        channelFlagsShared |= CHANNEL1_FLAG;
    }
}
void calcChannel2(){
    if(digitalRead(CHANNEL2_IN_PIN) == HIGH){ //start of signal
        pitchStart = micros();
    }else{ //signal ended. Save signal and set flag
        pitchInShared = (uint16_t)(micros() - pitchStart);
        channelFlagsShared |= CHANNEL2_FLAG;
    }
}
void calcChannel3(){
    if(digitalRead(CHANNEL3_IN_PIN) == HIGH){ //start of signal
        throttleStart = micros();
    }else{ //signal ended. Save signal and set flag
        throttleInShared = (uint16_t)(micros() - throttleStart);
        channelFlagsShared |= CHANNEL3_FLAG;
    }
    timeOfLastTransmission = micros(); // This channel keeps track of last received transmission
}
void calcChannel4(){
    if(digitalRead(CHANNEL4_IN_PIN) == HIGH){ //start of signal
        yawStart = micros();
    }else{ //signal ended. Save signal and set flag
        yawInShared = (uint16_t)(micros() - yawStart);
        channelFlagsShared |= CHANNEL4_FLAG;
    }
}

//INITIALIZE
void initialize_receiver() {
    pinMode(CHANNEL1_IN_PIN, INPUT);
    pinMode(CHANNEL2_IN_PIN, INPUT);
    pinMode(CHANNEL3_IN_PIN, INPUT);
    pinMode(CHANNEL4_IN_PIN, INPUT);
    PCintPort::attachInterrupt(A15, calcChannel1, CHANGE);
    PCintPort::attachInterrupt(A14, calcChannel2, CHANGE);
    PCintPort::attachInterrupt(A13, calcChannel3, CHANGE);
    PCintPort::attachInterrupt(A12, calcChannel4, CHANGE);
}

void loopReceiver() {
    static uint8_t channelFlags;

    if(timeOfLastTransmission == 0 || (micros() - timeOfLastTransmission) > FAILSAFE_DELAY){ //transmitter not connected
        failsafe = true; //transmitter not connected

        //make stable
        RCroll = 0; 
        RCpitch = 0;

        //set throttle
        RCthrottle = ESC_MIN; //engine shutoff
    }else{ 
      failsafe = false;
    }
    
    if(channelFlagsShared && !failsafe){ //handle interupted pins
        noInterrupts();

        //copy changed shared variables
        channelFlags = channelFlagsShared;

        if(channelFlags & CHANNEL1_FLAG){
            rollIn = rollInShared;
        }
        if(channelFlags & CHANNEL2_FLAG){
            pitchIn = pitchInShared;
        }
        if(channelFlags & CHANNEL3_FLAG){
            throttleIn = throttleInShared;
        }
        if(channelFlags & CHANNEL4_FLAG){
            yawIn = yawInShared;
        }
        channelFlagsShared = 0;

        interrupts();
    }

    RCroll = map(rollIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
    RCpitch = -map(pitchIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
    RCthrottle = map(throttleIn, RCRECIEVER_MIN, RCRECIEVER_MAX, ESC_MIN, ESC_MAX);
    RCyaw = -map(yawIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -135, 135); 
}

#endif
