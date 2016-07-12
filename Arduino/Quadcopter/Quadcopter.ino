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
 * I2C device found at address 0x1E  ! = HMC5883L magnetometer
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

//RCinput
float RCroll = 0;
float RCpitch = 0;
float RCthrottle = 0;
float RCyaw = 0;

//------AA-------HH----HH---RRRR---------SSSS---
//-----AAAA------HH----HH---RR--RR-----SS------
//----AA--AA-----HHHHHHHH---RRRR---------SS----
//---AAAAAAAA----HH----HH---RR--RR---------SS--
//--AA------AA---HH----HH---RR----RR---SSSS----

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

// Euler angles (radians)
float yaw;
float pitch;
float roll;

float yawDegrees;
float pitchDegrees;
float rollDegrees;

//---------------------------------------------
//---------------------------------------------
//---------------------------------------------

//PIDs and their variables
float pitch_stab_output = 0;
float roll_stab_output = 0;
float yaw_stab_output = 0;

float pitch_output = 0;
float roll_output = 0;
float yaw_output = 0;

float yaw_target = 0;

PID pidPitchStable(&pitchDegrees, &pitch_stab_output, &RCpitch, 4.5, 0, 0, REVERSE);
PID pidRollStable(&rollDegrees, &roll_stab_output, &RCroll, 4.5, 0, 0, REVERSE);
//PID pidYawStable(&yawDegrees, &yaw_stab_output, &RCyaw, 6, 0, 0, REVERSE);

PID pidPitchRate(&gyro[1], &pitch_output, &pitch_stab_output, 0.01, 0, 0, REVERSE);
PID pidRollRate(&gyro[0], &roll_output, &roll_stab_output, 0.01, 0, 0, DIRECT);
//PID pidYawRate(&gyro[2], &yaw_output, &yaw_stab_output, 0.01, 0, 0, REVERSE);


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
    setupAHRS();
    initPids();
    setRatePidsOutputLimits(-40, 40); //direct engine influence
    setStablePidsOutputLimits(-202.5, 202.5); //max degrees per second to get right degree
}

void loop() {
    static uint8_t channelFlags;
    static uint16_t pitchIn;
    static uint16_t yawIn;
    static uint16_t rollIn;
    static uint16_t throttleIn;

    if(timeOfLastTransmission == 0 || (micros() - timeOfLastTransmission) > FAILSAFE_DELAY){ //transmitter not connected
        Serial.println("FAILSAFE");
        //run engines at proper level
        return;
    } //transmitter not connected
    
    if(channelFlagsShared){ //handle interupted pins
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

        RCroll = map(rollIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
        RCpitch = -map(pitchIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -45, 45);
        RCthrottle = map(throttleIn, RCRECIEVER_MIN, RCRECIEVER_MAX, ESC_MIN, ESC_MAX);
        RCyaw = map(yawIn, RCRECIEVER_MIN, RCRECIEVER_MAX, -135, 135);   
        
    } //handle interupted pins

    loopAHRS();  //ahrs algorithm

    if(RCthrottle > 140){ //calc PIDS and run engines accordingly

      //calc pids
      computePids();

      Serial.print("pid: ");
      Serial.print(gyro[0]);
      Serial.print(" | ");
      Serial.print(pitchDegrees);
      Serial.print(" | ");
      Serial.print(RCpitch);


      //calc engine values
      long motor_FR_output = RCthrottle - roll_output - pitch_output - yaw_output;
      long motor_RL_output = RCthrottle + roll_output + pitch_output - yaw_output;
      long motor_FL_output = RCthrottle + roll_output - pitch_output + yaw_output;
      long motor_RR_output = RCthrottle - roll_output + pitch_output + yaw_output; 
      
      // PRINT FOR TESTING******************************************************
      //Serial.print("RC in (#typr): "); Serial.print(RCthrottle); Serial.print(", "); Serial.print(RCyaw); Serial.print(", "); Serial.print(RCpitch); Serial.print(", "); Serial.print(RCroll); Serial.print("   "); 
      Serial.print("EngOut: fr: "); Serial.print(motor_FR_output); Serial.print(" fl:"); Serial.print(motor_FL_output); Serial.print(" rr:"); Serial.print(motor_RR_output); Serial.print(" rl:"); Serial.print(motor_RL_output); Serial.print("   ");
      // PRINT FOR TESTING******************************************************

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
      Serial.print("Throttle too low to run engines.   ");

      //reset pids
      resetPids();
    }
    

    
} // loop()

void computePids(){
  pidPitchStable.Compute();
  pidRollStable.Compute();
  //pidYawStable.Compute();
  pidPitchRate.Compute();
  pidRollRate.Compute();
  //pidYawRate.Compute();
}

void initPids(){
  pidPitchStable.SetMode(AUTOMATIC);
  pidRollStable.SetMode(AUTOMATIC);
  //pidYawStable.SetMode(AUTOMATIC);
  pidPitchRate.SetMode(AUTOMATIC);
  pidRollRate.SetMode(AUTOMATIC);
  //pidYawRate.SetMode(AUTOMATIC);
}

void resetPids(){
  pidPitchStable.Reset();
  pidRollStable.Reset();
  //pidYawStable.Reset();
  pidPitchRate.Reset();
  pidRollRate.Reset();
  //pidYawRate.Reset();
}

void setRatePidsOutputLimits(float low, float high){
  pidPitchRate.SetOutputLimits(low, high);
  pidRollRate.SetOutputLimits(low, high);
  //pidYawRate.SetOutputLimits(low, high);
}

void setStablePidsOutputLimits(float low, float high){
  pidPitchStable.SetOutputLimits(low, high);
  pidRollStable.SetOutputLimits(low, high);
  //pidYawStable.SetOutputLimits(low, high);
}

void setPidsSampleTime(int time){ //millis
  pidPitchStable.SetSampleTime(50);
  pidRollStable.SetSampleTime(50);
  //pidYawStable.SetSampleTime(50);
  pidPitchRate.SetSampleTime(50);
  pidRollRate.SetSampleTime(50);
  //pidYawRate.SetSampleTime(50);
}

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
long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

