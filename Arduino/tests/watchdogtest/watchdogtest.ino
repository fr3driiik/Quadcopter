#include <Arduino.h>
#include <avr/wdt.h>

void setup(){
  wdt_disable(); //so we dont get stuck in reboot loop because of slow setup
  /*do all setup  
  blablabl
  ablabl
  balba
  */
  Serial.begin(57600);
  Serial.println("Initializing..");
  wdt_enable(WDTO_2S);
  Serial.println("Lift off!");
}

void loop(){
  if(millis() < 10000){
    wdt_reset();
    Serial.println("waiting for the end to come.. wishing i had strength to last..");
    delay(500);
  }
}

