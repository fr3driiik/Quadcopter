#define M8N_ADDRESS 0x42
#define M8N_DATA_SIZE_LOW 0xFD
#define M8N_DATA_SIZE_HIGH 0xFE
#define M8N_DATA_BUFFER 0xFF
#define CHUNK_SIZE 32 //bytes at a time

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  byte buffer1[2];
  readFrom(M8N_ADDRESS, M8N_DATA_SIZE_LOW, 2, buffer1);
  uint16_t small = (uint16_t) buffer1[0] | (buffer1[1] << 8);
  byte buffer2[2];
  readFrom(M8N_ADDRESS, M8N_DATA_SIZE_HIGH, 2, buffer2);
  uint32_t availables = (uint32_t) ((buffer2[0] << 24) | (buffer2[1] << 16) | (buffer1[0] << 8) | buffer1[1]);
  Serial.print("makes: "); Serial.println(availables);

  byte data[3];
  readFrom(M8N_ADDRESS, M8N_DATA_BUFFER, 3, data);
  
}

void readFrom(int DEVICE, byte address , int num ,byte buff[]) {
   Wire.beginTransmission(DEVICE); //start transmission to ACC
   Wire.write(address);            //send reguster address
   Wire.endTransmission();        //end transmission
  
   Wire.beginTransmission(DEVICE); //start transmission to ACC
   Wire.requestFrom(DEVICE,num);  //request 6 bits from ACC
  
   int i=0;
   while(Wire.available())        //ACC may abnormal
   {
     byte temp = Wire.read();
     if (i < num) {
       buff[i] = temp;        //receive a byte
     }
     i++;
   }
   Wire.endTransmission();         //end transmission
 }
