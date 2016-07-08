/*Quadcopter!
 * 
 * ESC settings:
 * stop/arm   1060 µs = 132 analogWrite Value
 * full power 1860 µs = 232,407 analogWrite Value. Detta ger endast 100 steg, har möjlighet för 800st
 * PWM freq for pin 3, 5, 6, 9 (timer 2, 3, 4) = 490 Hz, every Pulse = 2040.8163265306 µs
 * analogWrite(pin, value), 0 off, 255 on
 * 
 * RC reviever 8 ch PIN: 31, 33, 35, 37, 39, 41, 43, 45
 * 
 * Sensorsticka I2C:
 * I2C device found at address 0x1E  ! = 0b 0001 1110
 * I2C device found at address 0x41  ! = BMA180 accelerometer
 * I2C device found at address 0x42  ! = 0b 0100 0010
 * I2C device found at address 0x68  ! = ITG3050 gyro
 * I2C device found at address 0x76  ! = 0b 0111 0110
 * 
 */

#include <PinChangeInt.h>
#include <PID_v1.h>

//inputsPins
#define CHANNEL1_IN_PIN A15 // ROLL
#define CHANNEL2_IN_PIN A14 // PITCH
#define CHANNEL3_IN_PIN A13 // THROTTLE
#define CHANNEL4_IN_PIN A12 // YAW
#define CHANNEL5_IN_PIN A11
#define CHANNEL6_IN_PIN A10
#define CHANNEL7_IN_PIN A9
#define CHANNEL8_IN_PIN A8

//outputPins
#define MOTOR_FR_OUT_PIN 3
#define MOTOR_RL_OUT_PIN 5
#define MOTOR_FL_OUT_PIN 6
#define MOTOR_RR_OUT_PIN 9

//flags for channels to indicate new signal
#define CHANNEL1_FLAG 1
#define CHANNEL2_FLAG 2
#define CHANNEL3_FLAG 4
#define CHANNEL4_FLAG 8
#define CHANNEL5_FLAG 16
#define CHANNEL6_FLAG 32
#define CHANNEL7_FLAG 64
#define CHANNEL8_FLAG 128

//values
#define RCRECIEVER_MIN 1050
#define RCRECIEVER_MAX 1880
#define ESC_MIN 132
#define ESC_MAX 232
#define ESC_MID (ESC_MAX - ESC_MIN)
#define FAILSAFE_DELAY 300000 // after 0,3 seconds of no signal on choosen channel, in this case ch3 because ch 4 is receiveing last value..

//holds time of last received transmission
uint32_t timeOfLastTransmission;

//keeper of the flags above, max 8 channels because 8bits
volatile uint8_t channelFlagsShared;

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

void setup() {
    Serial.begin(9600);
    pinMode(CHANNEL1_IN_PIN, INPUT);
    pinMode(CHANNEL2_IN_PIN, INPUT);
    pinMode(CHANNEL3_IN_PIN, INPUT);
    pinMode(CHANNEL4_IN_PIN, INPUT);
    PCintPort::attachInterrupt(A15, calcChannel1, CHANGE);
    PCintPort::attachInterrupt(A14, calcChannel2, CHANGE);
    PCintPort::attachInterrupt(A13, calcChannel3, CHANGE);
    PCintPort::attachInterrupt(A12, calcChannel4, CHANGE);
}

void loop() {
    static uint8_t channelFlags;
    static uint16_t pitchIn;
    static uint16_t yawIn;
    static uint16_t rollIn;
    static uint16_t throttleIn;

    if(timeOfLastTransmission == 0 || (micros() - timeOfLastTransmission) > FAILSAFE_DELAY){
        Serial.println("FAILSAFE");
        //run engines at proper level
        return;
    }
    
    if(channelFlagsShared){
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

        long roll = map(rollIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
        long pitch = map(pitchIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
        long throttle = map(throttleIn, RCRECIEVER_MIN, RCRECIEVER_MAX, ESC_MIN, ESC_MAX);
        long yaw = map(yawIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -135, 135);

        long motor_FR_output = throttle - roll - pitch - yaw;
        long motor_RL_output = throttle + roll + pitch - yaw;
        long motor_FL_output = throttle + roll - pitch + yaw;
        long motor_RR_output = throttle - roll + pitch + yaw;

        if(throttle < 140){
          motor_FR_output = 132;
          motor_RL_output = 132;
          motor_FL_output = 132;
          motor_RR_output = 132;
          Serial.print("Throttle to low to run engines.       ");
        }

        

        // PRINT FOR TESTINT******************************************************
        Serial.print("thr: ");
        Serial.print(throttle);
        Serial.print("     ");
        Serial.print("pitch: ");
        Serial.print(pitch);
        Serial.print("     ");
        Serial.print("yaw: ");
        Serial.print(yaw);
        Serial.print("     ");
        Serial.print("roll: ");
        Serial.print(roll);
        Serial.print("     ");
       
        Serial.print("motorFR: ");
        Serial.print(motor_FR_output);
        Serial.print("     ");
        Serial.print("motorRL: ");
        Serial.print(motor_RL_output);
        Serial.print("     ");
        Serial.print("motorFL: ");
        Serial.print(motor_FL_output);
        Serial.print("     ");
        Serial.print("motorRR: ");
        Serial.println(motor_RR_output);
        // PRINT FOR TESTINT******************************************************
        
        analogWrite(MOTOR_FR_OUT_PIN, motor_FR_output);
        analogWrite(MOTOR_RL_OUT_PIN, motor_RL_output);
        analogWrite(MOTOR_FL_OUT_PIN, motor_FL_output);
        analogWrite(MOTOR_RR_OUT_PIN, motor_RR_output);
    }
}

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
long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

