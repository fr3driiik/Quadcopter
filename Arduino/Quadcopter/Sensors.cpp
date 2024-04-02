#include "Sensors.h"


float Sensors::gyro[3] = {0, 0, 0};
float Sensors::accel[3] = {0, 0, 0};
float Sensors::magnetom[3] = {0, 0, 0};
float Sensors::temperature = 0;
float Sensors::pressure = 0;

int num_gyro_errors = 0;
int num_accel_errors = 0;
int num_magn_errors = 0;

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
  #define GYRO_RES        2000.0 / 32768.0  // dps
	#define OUT_X_L_G	      0x28
  #define CTRL_REG1_G     0x20
  #define CTRL_REG2_G     0x21
  #define CTRL_REG3_G     0x22
  #define CTRL_REG4_G     0x23
  #define CTRL_REG5_G     0x24

  //ACC MAG TEMP PART
  #define ACC_RES             2.0 / 32768.0 // g
  #define MAG_RES             2000.0 / 32768.0  // mgauss
  #define TEMPERATURE_OFFSET  21.0  // celsius. Not documented, so just a guess
  #define TEMPERATURE_RES     1.0 / 8.0
  #define TEMP_OUT_L_XM       0x05
	#define OUT_X_L_M	          0x08
	#define OUT_X_L_A	          0x28
  #define CTRL_REG0_XM        0x1F
  #define CTRL_REG1_XM        0x20
  #define CTRL_REG2_XM        0x21
  #define CTRL_REG3_XM        0x22
  #define CTRL_REG4_XM        0x23
  #define CTRL_REG5_XM        0x24
  #define CTRL_REG6_XM        0x25
  #define CTRL_REG7_XM        0x26

  const int16_t gyro_calibration[3] = {-25, 15, 25};  // raw reading offset
  const int16_t accel_calibration[3] = {1530, 565, 650};  // raw reading offset
  // magnetometer calibration values calculated using adafruit sensor calibration and MotionCal
  const int16_t magnetom_offset[3] = {53.8/((float)MAG_RES), -58.4/((float)MAG_RES), 32.7/((float)MAG_RES)};  // raw reading offset. (mgauss to raw)
  const float magnetom_soft_iron[3][3] = {
    {0.927, -0.064, 0.007},
    {-0.064, 1.064, 0.004},
    {0.007, 0.004, 1.018},
  };

  void Sensors::initialize() {
    Sensors::initGyro();
    Sensors::initAcc();
    Sensors::initMag();
  }

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
    writeI2CByte(LSM9DS0_XM, CTRL_REG5_XM, 0b10010100);

    //Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
    writeI2CByte(LSM9DS0_XM, CTRL_REG6_XM, 0b00000000);

    //Bits (7-0): AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
    writeI2CByte(LSM9DS0_XM, CTRL_REG7_XM, 0b00000000);
	}

  void Sensors::loop() {
    Sensors::readGyro();
    Sensors::readAcc();
    Sensors::readMag();
    Sensors::readTemp();
  }

	void Sensors::readGyro() {
		uint8_t temp[6];
		readI2CBytes(LSM9DS0_G, OUT_X_L_G | 0x80, temp, 6);
		Sensors::gyro[X_AXIS] = GYRO_RES * ((int16_t)((temp[1] << 8) | temp[0]) + gyro_calibration[X_AXIS]);
		Sensors::gyro[Y_AXIS] = GYRO_RES * ((int16_t)((temp[3] << 8) | temp[2]) + gyro_calibration[Y_AXIS]);
		Sensors::gyro[Z_AXIS] = GYRO_RES * ((int16_t)((temp[5] << 8) | temp[4]) + gyro_calibration[Z_AXIS]);
	}

	void Sensors::readAcc() {
		uint8_t temp[6];
		readI2CBytes(LSM9DS0_XM, OUT_X_L_A | 0x80, temp, 6);
		Sensors::accel[X_AXIS] = ACC_RES * ((int16_t)((temp[1] << 8) | temp[0]) + accel_calibration[X_AXIS]);
		Sensors::accel[Y_AXIS] = ACC_RES * ((int16_t)((temp[3] << 8) | temp[2]) + accel_calibration[Y_AXIS]);
		Sensors::accel[Z_AXIS] = ACC_RES * ((int16_t)((temp[5] << 8) | temp[4]) + accel_calibration[Z_AXIS]);
	}

	void Sensors::readMag() {
    uint8_t temp[6];
    readI2CBytes(LSM9DS0_XM, OUT_X_L_M | 0x80, temp, 6);
    int16_t x = MAG_RES * ((int16_t)((temp[1] << 8) | temp[0]) + magnetom_offset[X_AXIS]);
    int16_t y = MAG_RES * ((int16_t)((temp[3] << 8) | temp[2]) + magnetom_offset[Y_AXIS]);
    int16_t z = MAG_RES * ((int16_t)((temp[5] << 8) | temp[4]) + magnetom_offset[Z_AXIS]);

    //soft iron calibration
    Sensors::magnetom[X_AXIS] = x * magnetom_soft_iron[0][0] + y * magnetom_soft_iron[0][1] + z * magnetom_soft_iron[0][2];
    Sensors::magnetom[Y_AXIS] = x * magnetom_soft_iron[1][0] + y * magnetom_soft_iron[1][1] + z * magnetom_soft_iron[1][2];
    Sensors::magnetom[Z_AXIS] = x * magnetom_soft_iron[2][0] + y * magnetom_soft_iron[2][1] + z * magnetom_soft_iron[2][2];
	}

  void Sensors::readTemp() {
    uint8_t temp[2];
    readI2CBytes(LSM9DS0_XM, TEMP_OUT_L_XM | 0x80, temp, 2);
    Sensors::temperature = TEMPERATURE_OFFSET + TEMPERATURE_RES * (int16_t)((temp[1] << 8) | temp[0]);
  }
#endif // LSM9DS0

#ifdef CGSHOP_11_DOF
  /* Sensorsticka I2C:
  * I2C device found at address 0x1E  ! = HMC5883L magnetometer
  * I2C device found at address 0x41  ! = BMA180 accelerometer
  * I2C device found at address 0x42  ! = ublox m8n GPS, uses serial1 instead
  * I2C device found at address 0x68  ! = ITG3050 gyro
  * I2C device found at address 0x76  ! = MS5611 Altimeter
  */
  #define ACCEL_ADDRESS 0x41
  #define MAGN_ADDRESS  0x1E
  #define GYRO_ADDRESS  0x68

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
#endif // CGSHOP_11_DEF

#ifdef ALTIMU_10_V5
  #define LSM6DS33_ADDRESS           0x6B
  #define LSM6DS33_WHO_ID            0x69
  #define LSM6DS33_FIFO_CTRL1        0x06
  #define LSM6DS33_FIFO_CTRL2        0x07
  #define LSM6DS33_FIFO_CTRL3        0x08
  #define LSM6DS33_FIFO_CTRL4        0x09
  #define LSM6DS33_FIFO_CTRL5        0x0A
  #define LSM6DS33_ORIENT_CFG_G      0x0B
  #define LSM6DS33_INT1_CTRL         0x0D
  #define LSM6DS33_INT2_CTRL         0x0E
  #define LSM6DS33_WHO_AM_I          0x0F
  #define LSM6DS33_CTRL1_XL          0x10
  #define LSM6DS33_CTRL2_G           0x11
  #define LSM6DS33_CTRL3_C           0x12
  #define LSM6DS33_CTRL4_C           0x13
  #define LSM6DS33_CTRL5_C           0x14
  #define LSM6DS33_CTRL6_C           0x15
  #define LSM6DS33_CTRL7_G           0x16
  #define LSM6DS33_CTRL8_XL          0x17
  #define LSM6DS33_CTRL9_XL          0x18
  #define LSM6DS33_CTRL10_C          0x19
  #define LSM6DS33_WAKE_UP_SRC       0x1B
  #define LSM6DS33_TAP_SRC           0x1C
  #define LSM6DS33_D6D_SRC           0x1D
  #define LSM6DS33_STATUS_REG        0x1E
  #define LSM6DS33_OUT_TEMP_L        0x20
  #define LSM6DS33_OUT_TEMP_H        0x21
  #define LSM6DS33_OUTX_L_G          0x22
  #define LSM6DS33_OUTX_H_G          0x23
  #define LSM6DS33_OUTY_L_G          0x24
  #define LSM6DS33_OUTY_H_G          0x25
  #define LSM6DS33_OUTZ_L_G          0x26
  #define LSM6DS33_OUTZ_H_G          0x27
  #define LSM6DS33_OUTX_L_XL         0x28
  #define LSM6DS33_OUTX_H_XL         0x29
  #define LSM6DS33_OUTY_L_XL         0x2A
  #define LSM6DS33_OUTY_H_XL         0x2B
  #define LSM6DS33_OUTZ_L_XL         0x2C
  #define LSM6DS33_OUTZ_H_XL         0x2D
  #define LSM6DS33_FIFO_STATUS1      0x3A
  #define LSM6DS33_FIFO_STATUS2      0x3B
  #define LSM6DS33_FIFO_STATUS3      0x3C
  #define LSM6DS33_FIFO_STATUS4      0x3D
  #define LSM6DS33_FIFO_DATA_OUT_L   0x3E
  #define LSM6DS33_FIFO_DATA_OUT_H   0x3F
  #define LSM6DS33_TIMESTAMP0_REG    0x40
  #define LSM6DS33_TIMESTAMP1_REG    0x41
  #define LSM6DS33_TIMESTAMP2_REG    0x42
  #define LSM6DS33_STEP_TIMESTAMP_L  0x49
  #define LSM6DS33_STEP_TIMESTAMP_H  0x4A
  #define LSM6DS33_STEP_COUNTER_L    0x4B
  #define LSM6DS33_STEP_COUNTER_H    0x4C
  #define LSM6DS33_FUNC_SRC          0x53
  #define LSM6DS33_TAP_CFG           0x58
  #define LSM6DS33_TAP_THS_6D        0x59
  #define LSM6DS33_INT_DUR2          0x5A
  #define LSM6DS33_WAKE_UP_THS       0x5B
  #define LSM6DS33_WAKE_UP_DUR       0x5C
  #define LSM6DS33_FREE_FALL         0x5D
  #define LSM6DS33_MD1_CFG           0x5E
  #define LSM6DS33_MD2_CFG           0x5F

  const float GYRO_RESOLUTION = 1000.0 / 32768.0;  // dps
  const float ACCELEROMETER_RESOLUTION = 23.0;

  const int16_t gyro_calibration[3] = {-50, 150, 127};  // raw reading offset
  const int16_t accel_calibration[3] = {0, 0, 0};  // raw reading offset

  void initGyro() {
    //Bits[7:0]: 0 0 Z_EN Y_EN X_EN FUNC_EN PEDO_RST_STEP SIGN_MOTION_EN
    writeI2CByte(LSM6DS33_ADDRESS, LSM6DS33_CTRL10_C, 0b00111000);  // enable all axes

    //Bits[7:0]: DR3 DR2 DR1 DR0 FS1 FS0 FS_125 0
    writeI2CByte(LSM6DS33_ADDRESS, LSM6DS33_CTRL2_G, 0b10001000);  // DR = 1000 (1.66kHz high performance) FS = 10 (+/-1000 dps)
	
    //Bits[7:0]: DR3 DR2 DR1 DR0 FS1 FS0 FS_125 0  // TODO
    writeI2CByte(LSM6DS33_ADDRESS, LSM6DS33_CTRL3_C, 0b00000100);  // DR = 1000 (1.66kHz high performance) FS = 10 (+/-1000 dps)
  }

	void initAcc() {
    //Bits (7-0): 0 0 Z_EN Y_EN X_EN 0 0 0
    writeI2CByte(LSM6DS33_ADDRESS, LSM6DS33_CTRL9_XL, 0b00111000);  // enable all axes

    //Bits (7-0): DR3 DR2 DR1 DR0 FS1 FS0 BW1 BW0
    writeI2CByte(LSM6DS33_ADDRESS, LSM6DS33_CTRL1_XL, 0b01100100);  // DR = 1000 (1.66kHz high performance) FS = 01 (+/-4 g)
	}

  void readGyro() {
		uint8_t temp[6];
		readI2CBytes(LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_G, temp, 6);    
		Sensors::gyro[X_AXIS] = GYRO_RESOLUTION * ((int16_t)((temp[1] << 8) | temp[0]) + gyro_calibration[X_AXIS]);
		Sensors::gyro[Y_AXIS] = GYRO_RESOLUTION * ((int16_t)((temp[3] << 8) | temp[2]) + gyro_calibration[Y_AXIS]);
		Sensors::gyro[Z_AXIS] = GYRO_RESOLUTION * ((int16_t)((temp[5] << 8) | temp[4]) + gyro_calibration[Z_AXIS]);
	}

	void readAcc() {
		uint8_t temp[6];
		readI2CBytes(LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_XL, temp, 6);
		Sensors::accel[X_AXIS] = ACCELEROMETER_RESOLUTION * ((int16_t)((temp[1] << 8) | temp[0]) + accel_calibration[X_AXIS]);
		Sensors::accel[Y_AXIS] = ACCELEROMETER_RESOLUTION * ((int16_t)((temp[3] << 8) | temp[2]) + accel_calibration[Y_AXIS]);
		Sensors::accel[Z_AXIS] = ACCELEROMETER_RESOLUTION * ((int16_t)((temp[5] << 8) | temp[4]) + accel_calibration[Z_AXIS]);
	}

  #define LIS3MDL_ADDRESS            0x1E
  #define LIS3MDL_OFFSET_X_REG_L_M   0x05
  #define LIS3MDL_OFFSET_X_REG_H_M   0x06
  #define LIS3MDL_OFFSET_Y_REG_L_M   0x07
  #define LIS3MDL_OFFSET_Y_REG_H_M   0x08
  #define LIS3MDL_OFFSET_Z_REG_L_M   0x09
  #define LIS3MDL_OFFSET_Z_REG_H_M   0x0A
  #define LIS3MDL_WHO_AM_I           0x0F
  #define LIS3MDL_CTRL_REG1          0x20
  #define LIS3MDL_CTRL_REG2          0x21
  #define LIS3MDL_CTRL_REG3          0x22
  #define LIS3MDL_CTRL_REG4          0x23
  #define LIS3MDL_CTRL_REG5          0x24
  #define LIS3MDL_STATUS_REG         0x27
  #define LIS3MDL_OUT_X_L            0x28
  #define LIS3MDL_OUT_X_H            0x29
  #define LIS3MDL_OUT_Y_L            0x2A
  #define LIS3MDL_OUT_Y_H            0x2B
  #define LIS3MDL_OUT_Z_L            0x2C
  #define LIS3MDL_OUT_Z_H            0x2D
  #define LIS3MDL_TEMP_OUT_L         0x2E
  #define LIS3MDL_TEMP_OUT_H         0x2F
  #define LIS3MDL_INT_CFG            0x30
  #define LIS3MDL_INT_SRC            0x31
  #define LIS3MDL_INT_THS_L          0x32
  #define LIS3MDL_INT_THS_H          0x33

  const float MAGNETOMETER_RESOLUTION = 8.0 / 32768.0;
    // magnetometer calibration values calculated using adafruit sensor calibration and MotionCal
  const float magnetom_offset[3] = {0.0/MAGNETOMETER_RESOLUTION, 0.0/MAGNETOMETER_RESOLUTION, 0.0/MAGNETOMETER_RESOLUTION};  // raw reading offset. (mgauss to raw)
  const float magnetom_soft_iron[3][3] = {
    {1.000, 0.000, 0.000},
    {0.000, 1.000, 0.000},
    {0.000, 0.000, 1.000},
  };
  const float PRESSURE_RESOLUTION = 1/4096.0;
  const float TEMPERATURE_RESOLUTION = 1/480.0;//1/480 for lp25;  // 1/88 for lis3mdl
  const float TEMPERATURE_OFFSET = 42.5;//42.5 for lp25;  // 25 for lis3mdl

  void initMag() {
    // ultra high performance for each axis, temp sensor enabled and 155 ODR (FAST_ODR=1)
    //Bits (7-0): TEMP_EN OM1 OM0 DO2 DO1 DO0 FAST_ODR ST
    writeI2CByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0b11100010);

    //Bits (7-0): 0 FS1 FS0 0 REBOOT SOFT_RST 0 0
    writeI2CByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0b00000000);

    //Bits (7-0): 0 0 LP 0 0 SIM MD1 MD0
    writeI2CByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0b00000000);

    //Bits (7-0): 0 0 0 0 OMZ1 OMZ0 BLE 0
    writeI2CByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG4, 0b00001100);

    //Bits (7-0): FAST_READ BDU 0 0 0 0 0 0
    writeI2CByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG5, 0b01000000);
	}

	void readMag() {
    uint8_t temp[6];
    readI2CBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L | 0x80, temp, 6);
    float x = MAGNETOMETER_RESOLUTION * ((int16_t)((temp[1] << 8) | temp[0]) + magnetom_offset[X_AXIS]);
    float y = MAGNETOMETER_RESOLUTION * ((int16_t)((temp[3] << 8) | temp[2]) + magnetom_offset[Y_AXIS]);
    float z = MAGNETOMETER_RESOLUTION * ((int16_t)((temp[5] << 8) | temp[4]) + magnetom_offset[Z_AXIS]);

    //soft iron calibration
    Sensors::magnetom[X_AXIS] = x * magnetom_soft_iron[0][0] + y * magnetom_soft_iron[0][1] + z * magnetom_soft_iron[0][2];
    Sensors::magnetom[Y_AXIS] = x * magnetom_soft_iron[1][0] + y * magnetom_soft_iron[1][1] + z * magnetom_soft_iron[1][2];
    Sensors::magnetom[Z_AXIS] = x * magnetom_soft_iron[2][0] + y * magnetom_soft_iron[2][1] + z * magnetom_soft_iron[2][2];
	}

  void readTemp() {
    uint8_t temp[2];
    readI2CBytes(LIS3MDL_ADDRESS, LIS3MDL_TEMP_OUT_L | 0x80, temp, 2);
    Sensors::temperature = TEMPERATURE_OFFSET + TEMPERATURE_RESOLUTION * (int16_t)((temp[1] << 8) | temp[0]);
  }

  #define LPS25H_ADDRESS             0x5D
  #define LPS25H_REF_P_XL            0x08
  #define LPS25H_REF_P_L             0x09
  #define LPS25H_REF_P_H             0x0A
  #define LPS25H_WHO_AM_I            0x0F
  #define LPS25H_RES_CONF            0x10
  #define LPS25H_CTRL_REG1           0x20
  #define LPS25H_CTRL_REG2           0x21
  #define LPS25H_CTRL_REG3           0x22
  #define LPS25H_CTRL_REG4           0x23
  #define LPS25H_INT_CFG             0x24
  #define LPS25H_INT_SOURCE          0x25
  #define LPS25H_STATUS_REG          0x27
  #define LPS25H_PRESS_POUT_XL       0x28
  #define LPS25H_PRESS_OUT_L         0x29
  #define LPS25H_PRESS_OUT_H         0x2A
  #define LPS25H_TEMP_OUT_L          0x2B
  #define LPS25H_TEMP_OUT_H          0x2C
  #define LPS25H_FIFO_CTRL           0x2E
  #define LPS25H_FIFO_STATUS         0x2F
  #define LPS25H_THS_P_L             0x30
  #define LPS25H_THS_P_H             0x31
  #define LPS25H_RPDS_L              0x39
  #define LPS25H_RPDS_H              0x3A

  void initBarometer() {
    //Bits (7-0): PD ODR2 ODR1 ODR0 DIFF_EN BDU RESET_AZ SIM
    writeI2CByte(LPS25H_ADDRESS, LPS25H_CTRL_REG1, 0b11000000);  // PD = 1 (active), ODR = 100 (25Hz)
  }

  void readBarometer() {
    uint8_t temp[3];
    readI2CBytes(LPS25H_ADDRESS, LPS25H_PRESS_POUT_XL | 0x80, temp, 3);
    Sensors::pressure = PRESSURE_RESOLUTION * (int32_t)((temp[2] << 16) | (temp[1] << 8) | temp[0]);
  }

  void readTemperature() {
    uint8_t temp[2];
    readI2CBytes(LPS25H_ADDRESS, LPS25H_TEMP_OUT_L | 0x80, temp, 2);
    Sensors::temperature = TEMPERATURE_OFFSET + TEMPERATURE_RESOLUTION * (int16_t)((temp[1] << 8) | temp[0]);
  }

#endif // ALTIMU_10_V5

void Sensors::initialize() {
    initGyro();
    initAcc();
    initMag();
    initBarometer();
}

 void Sensors::loop() {
    readGyro();
    readAcc();
    readMag();
    readTemp();
    readTemperature();
    readBarometer();
 }
