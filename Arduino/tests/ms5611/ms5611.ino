#include <Wire.h>

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

#define ADDRESS 0x76

#define SEA_PRESSURE 1013.25

//**COMMANDS**
#define CMD_RESET 0x1E
#define CMD_ADC_READ 0x00
#define CMD_CONV_D1 0x40
#define CMD_CONV_D2 0x50
#define CMD_READ_PROM 0xA0

//resolutions
#define RES_ULTRA_HIGH 0x08

uint16_t prom[7]; // first byte contains unused data, but this maps better to datasheet (C1-C6 = 1-6)
uint8_t waitTimeForOversampling = 10; //ultra high res needs this time
int32_t TEMP2;
int64_t OFF2, SENS2;
int32_t TEMP, P; // centiCelsius and centiMilliBar, division by 100 to get C and mbar

float altitudeOrigin;

void setup(){
  Serial.begin(250000);
  Serial.println("Initialing sensor");
  Wire.begin();
  reset();
  readProm();
  delay(10);
  readSensor();
  altitudeOrigin = getAltitude();
}

void loop(){
  readSensor();
  float alt = getAltitude();
  
  Serial.print("   T: ");
  Serial.print(TEMP / 100.00);
  Serial.print(" C   P: ");
  Serial.print(P / 100.00);
  Serial.print(" mbar   A: ");
  Serial.print(alt / 1000);
  Serial.print(" m   R A: ");
  Serial.print((alt - altitudeOrigin) / 1000);
  Serial.print(" m");

  Serial.println();
}

void reset(){
  Wire.beginTransmission(ADDRESS);
  WIRE_SEND(CMD_RESET);
  Wire.endTransmission();
  delay(5);
}

void readProm(){
  for(uint8_t i = 0; i < 7; i++){
    prom[i] = readRegister16(CMD_READ_PROM + (i * 2));
  }
}

uint32_t readRawTemperature(){
  Wire.beginTransmission(ADDRESS);
  WIRE_SEND(CMD_CONV_D2 + RES_ULTRA_HIGH);
  Wire.endTransmission();

  delay(waitTimeForOversampling);

  return readRegister24(CMD_ADC_READ);
}

uint32_t readRawPressure(){
  Wire.beginTransmission(ADDRESS);
  WIRE_SEND(CMD_CONV_D1 + RES_ULTRA_HIGH);
  Wire.endTransmission();

  delay(waitTimeForOversampling);

  return readRegister24(CMD_ADC_READ);
}

void readSensor(){
  uint32_t D1 = readRawPressure();
  uint32_t D2 = readRawTemperature();

  int32_t dT = D2 - ((int32_t)prom[5] << 8); // = * 2^8
  TEMP = 2000 + (((int64_t)dT * prom[6]) >> 23);

  int64_t OFF = ((int64_t)prom[2] << 16) + (((int64_t)dT * prom[4]) >> 7);
  int64_t SENS = ((int64_t)prom[1] << 15) + (((int64_t)dT * prom[3]) >> 8);

  if(TEMP < 2000){
    int32_t T2 = (((int64_t)dT * dT) >> 31);
    int64_t OFF2 = ((int64_t)5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
    int64_t SENS2 = (OFF2 >> 1);
  }
  P = ((D1 * (SENS >> 21) - OFF) >> 15);
}

float getAltitude(){
  return ((pow((SEA_PRESSURE / P), 1/5.257) - 1.0) * (TEMP + 273.15)) / 0.0065;
}

uint16_t readRegister16(uint8_t reg){
  uint16_t value;

  int i = 0;
  uint8_t buff[2];
  
  Wire.beginTransmission(ADDRESS);
  WIRE_SEND(reg);
  Wire.endTransmission();

  Wire.beginTransmission(ADDRESS);
  Wire.requestFrom(ADDRESS, 2);
  while(Wire.available()){
    buff[i] = WIRE_RECEIVE();
    i++;
  };
  Wire.endTransmission();
  value = ((uint32_t)buff[0] << 8) | buff[1];
  return value;
}

uint32_t readRegister24(uint8_t reg){
  uint32_t value;

  int i = 0;
  uint8_t buff[3];
  
  Wire.beginTransmission(ADDRESS);
  WIRE_SEND(reg);
  Wire.endTransmission();

  Wire.beginTransmission(ADDRESS);
  Wire.requestFrom(ADDRESS, 3);
  while(Wire.available()){
    buff[i] = WIRE_RECEIVE();
    i++;
  };
  Wire.endTransmission();
  value = ((uint32_t)buff[0] << 16) | ((uint32_t)buff[1] << 8) | buff[2];
  return value;
}

