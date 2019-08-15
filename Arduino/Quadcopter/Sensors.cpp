#include "Sensors.h"

void writeI2CByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readI2CBytes(uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count) {
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false);
	uint8_t i = 0;
	Wire.requestFrom(address, count);
	while (Wire.available()) {
		dest[i++] = Wire.read();
	}
}

#ifdef LSM9DS0
	#define LSM9DS0_XM      0x1D
	#define LSM9DS0_G	      0x6B

  //GYRO PART
  #define GYRO_RES        2000.0 / 32768.0
	#define OUT_X_L_G	      0x28
  #define CTRL_REG1_G     0x20
  #define CTRL_REG2_G     0x21
  #define CTRL_REG3_G     0x22
  #define CTRL_REG4_G     0x23
  #define CTRL_REG5_G     0x24

  //ACC MAG PART
  #define ACC_RES         2.0 / 32768.0
  #define MAG_RES         2.0 / 32768.0
	#define OUT_X_L_M	      0x08
	#define OUT_X_L_A	      0x28
  #define CTRL_REG0_XM    0x1F
  #define CTRL_REG1_XM    0x20
  #define CTRL_REG2_XM    0x21
  #define CTRL_REG3_XM    0x22
  #define CTRL_REG4_XM    0x23
  #define CTRL_REG5_XM    0x24
  #define CTRL_REG6_XM    0x25
  #define CTRL_REG7_XM    0x26

	void Sensors::initGyro() {
    //Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
    writeI2CByte(LSM9DS0_G, CTRL_REG1_G, 0b00001111);

    //Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
    writeI2CByte(LSM9DS0_G, CTRL_REG2_G, 0x00);

    //Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
    writeI2CByte(LSM9DS0_G, CTRL_REG4_G, 0b00100000);
	}

	void Sensors::initAcc() {
    //Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
    writeI2CByte(LSM9DS0_XM, CTRL_REG0_XM, 0b00000000);
    
    //Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
    writeI2CByte(LSM9DS0_XM, CTRL_REG1_XM, 0b01100111);

    //Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
    writeI2CByte(LSM9DS0_XM, CTRL_REG2_XM, 0b00000000);
	}

	void Sensors::initMag() {
    //Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
    writeI2CByte(LSM9DS0_XM, CTRL_REG5_XM, 0b00010100);

    //Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
    writeI2CByte(LSM9DS0_XM, CTRL_REG6_XM, 0b00000000);

    //Bits (7-0): AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
    writeI2CByte(LSM9DS0_XM, CTRL_REG7_XM, 0b00000000);
	}

	void Sensors::readGyro() {
		uint8_t temp[6];
		readI2CBytes(LSM9DS0_G, OUT_X_L_G | 0x80, temp, 6);
		Sensors::gyro[X_AXIS] = GYRO_RES * (int16_t)((temp[1] << 8) | temp[0]);
		Sensors::gyro[Y_AXIS] = GYRO_RES * (int16_t)((temp[3] << 8) | temp[2]);
		Sensors::gyro[Z_AXIS] = GYRO_RES * (int16_t)((temp[5] << 8) | temp[4]);
	}

	void Sensors::readAcc() {
		uint8_t temp[6];
		readI2CBytes(LSM9DS0_XM, OUT_X_L_A | 0x80, temp, 6);
		Sensors::accel[X_AXIS] = ACC_RES * (int16_t)((temp[1] << 8) | temp[0]);
		Sensors::accel[Y_AXIS] = ACC_RES * (int16_t)((temp[3] << 8) | temp[2]);
		Sensors::accel[Z_AXIS] = ACC_RES * (int16_t)((temp[5] << 8) | temp[4]);
	}

	void Sensors::readMag() {
		uint8_t temp[6];
		readI2CBytes(LSM9DS0_XM, OUT_X_L_M | 0x80, temp, 6);
		Sensors::magnetom[X_AXIS] = MAG_RES * (int16_t)((temp[1] << 8) | temp[0]);
		Sensors::magnetom[Y_AXIS] = MAG_RES * (int16_t)((temp[3] << 8) | temp[2]);
		Sensors::magnetom[Z_AXIS] = MAG_RES * (int16_t)((temp[5] << 8) | temp[4]);
	}
#endif // LSM9DS0



// Sensor I2C addresses
#define ACCEL_ADDRESS 0x41
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int16_t) 0x68) // 0x68 = 0xD0 / 2

#define GYRO_GAIN 0.06103515625f
#define GYRO_X_OFFSET 3
#define GYRO_Y_OFFSET -96
#define GYRO_Z_OFFSET 8

#define ACC_GAIN 0.000244140625
#define ACC_X_OFFSET 0
#define ACC_Y_OFFSET 90
#define ACC_Z_OFFSET 0

#define MAG_X_OFFSET -153
#define MAG_Y_OFFSET -78
#define MAG_Z_OFFSET -118

float Sensors::gyro[3] = {0, 0, 0};
float Sensors::accel[3] = {0, 0, 0};
float Sensors::magnetom[3] = {0, 0, 0};

int num_gyro_errors = 0;
int num_accel_errors = 0;
int num_magn_errors = 0;
/*
void Accel_Init(){
 byte temp[1]{0};
 byte temp1;
  //
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x10);
  Wire.write(0xB6);
  Wire.endTransmission();
  
  //wake up mode
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x0D);
  Wire.write(0x10);
  Wire.endTransmission();
  
  // low pass filter,
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 1);

  int i = 0;
  while(Wire.available()){
    temp[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  
  temp1=temp[0]&0x0F; 

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x20);
  Wire.write(temp1);
  Wire.endTransmission();
  
  // range +/- 2g 
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x35);
  Wire.endTransmission();

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 1);

  i = 0;
  while(Wire.available()){
    temp[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  
  temp1=(temp[0]&0xF1) | 0x04; 

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x35);
  Wire.write(temp1);
  Wire.endTransmission();
}

void Read_Accel()
{
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  Wire.write(0x02);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())
  { 
    buff[i] = Wire.read();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    accel[0] = (float) (ACC_GAIN * ((( buff[2] | buff[3]<<8)>>2) + ACC_X_OFFSET));  // X axis (internal sensor y axis)
    accel[1] = (float) (ACC_GAIN * ((( buff[0] | buff[1]<<8)>>2) + ACC_Y_OFFSET));  // Y axis (internal sensor x axis)
    accel[2] = (float) (ACC_GAIN * ((( buff[4] | buff[5]<<8)>>2) + ACC_Z_OFFSET));  // Z axis (internal sensor z axis)
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
  Wire.write(0x02); 
  Wire.write(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.write(0x00);
  Wire.write(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int i = 0;
  uint8_t buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.write(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // Read one byte
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
  Wire.write(0x3E);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x16);
  Wire.write(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x15);
  Wire.write(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3E);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5);
}

void Read_Gyro()
{
  int i = 0;
  uint8_t buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  Wire.write(0x1D);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // Read one byte
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
*/
