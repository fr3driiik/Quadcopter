#include "Sensors.h"

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

// Sensor I2C addresses
#define ACCEL_ADDRESS 0x41
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int16_t) 0x68) // 0x68 = 0xD0 / 2

#define GYRO_GAIN 0.06103515625f
#define GYRO_X_OFFSET 3
#define GYRO_Y_OFFSET -96
#define GYRO_Z_OFFSET 8

#define ACC_X_OFFSET 0
#define ACC_Y_OFFSET 0
#define ACC_Z_OFFSET 0

#define MAG_X_OFFSET -153
#define MAG_Y_OFFSET -78
#define MAG_Z_OFFSET -118

float gyro[3] = {0, 0, 0};
float accel[3] = {0, 0, 0};
float magnetom[3] = {0, 0, 0};

int num_gyro_errors = 0;
int num_accel_errors = 0;
int num_magn_errors = 0;

void Accel_Init(){
 byte temp[1]{0};
 byte temp1;
  //
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x10);
  WIRE_SEND(0xB6);
  Wire.endTransmission();
  
  //wake up mode
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x0D);
  WIRE_SEND(0x10);
  Wire.endTransmission();
  
  // low pass filter,
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 1);

  int i = 0;
  while(Wire.available()){
    temp[i] = WIRE_RECEIVE();
    i++;
  }
  Wire.endTransmission();
  
  temp1=temp[0]&0x0F; 

  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x20);
  WIRE_SEND(temp1);
  Wire.endTransmission();
  
  // range +/- 2g 
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x35);
  Wire.endTransmission();

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 1);

  i = 0;
  while(Wire.available()){
    temp[i] = WIRE_RECEIVE();
    i++;
  }
  Wire.endTransmission();
  
  temp1=(temp[0]&0xF1) | 0x04; 

  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x35);
  WIRE_SEND(temp1);
  Wire.endTransmission();
}

void Read_Accel()
{
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x02);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    accel[0] = (int16_t)(( buff[2] | buff[3]<<8)>>6);  // X axis (internal sensor y axis)
    accel[1] = (int16_t)(( buff[0] | buff[1]<<8)>>6);  // Y axis (internal sensor x axis)
    accel[2] = (int16_t)(( buff[4] | buff[5]<<8)>>6);  // Z axis (internal sensor z axis)
  }
  else
  {
    num_accel_errors++;
    #ifdef OUTPUT_ERRORS
      Serial.println("!ERR: reading accelerometer");
    #endif
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  uint8_t buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y ??
    magnetom[0] =  -1 * ((int16_t)(((((uint16_t) buff[0]) << 8) | buff[1])) - MAG_X_OFFSET);  // X axis (internal sensor x axis)
    magnetom[1] = 1 * ((int16_t)(((((uint16_t) buff[4]) << 8) | buff[5])) - MAG_Z_OFFSET);  // Y axis (internal sensor -y axis)
    magnetom[2] = 1 * ((int16_t)(((((uint16_t) buff[2]) << 8) | buff[3])) - MAG_Y_OFFSET);  // Z axis (internal sensor -z axis)
  }
  else
  {
    num_magn_errors++;
    #ifdef OUTPUT_ERRORS
      Serial.println("!ERR: reading magnetometer");
    #endif
  }
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}

void Read_Gyro()
{
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    gyro[0] = -GYRO_GAIN * ((int16_t)(((((uint16_t) buff[2]) << 8) | buff[3])) - GYRO_X_OFFSET);    // X axis (internal sensor -y axis)
    gyro[1] = -GYRO_GAIN * ((int16_t)(((((uint16_t) buff[0]) << 8) | buff[1])) - GYRO_Y_OFFSET);    // Y axis (internal sensor -x axis)
    gyro[2] = -GYRO_GAIN * ((int16_t)(((((uint16_t) buff[4]) << 8) | buff[5])) - GYRO_Z_OFFSET);    // Z axis (internal sensor -z axis)
    #ifdef PRINT_UNSCALED_SENSORS
      float x = ((int16_t)(((((uint16_t) buff[2]) << 8) | buff[3])) - GYRO_X_OFFSET); //internal -y
      float y = ((int16_t)(((((uint16_t) buff[0]) << 8) | buff[1])) - GYRO_Y_OFFSET); //internal -x
      float z = ((int16_t)(((((uint16_t) buff[4]) << 8) | buff[5])) - GYRO_Z_OFFSET); //internal -z
      Serial.print("gyroRawX: ");Serial.print(x);Serial.print(" gyroRawY: ");Serial.print(y);Serial.print(" gyroRawZ: ");Serial.println(z);
    #endif
  }
  else
  {
    num_gyro_errors++;
    #ifdef OUTPUT_ERRORS
      Serial.println("!ERR: reading gyroscope");
    #endif

  }
}
