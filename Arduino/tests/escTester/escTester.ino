int speed = 132;

void setup(){
  Serial.begin(9600);
}

void loop() {
  Serial.println(speed);
  
  analogWrite(3, speed);
  analogWrite(5, speed);
  analogWrite(6, speed);
  analogWrite(9, speed);
 
  if(speed == 132){
    delay(5000);
  }
  if(speed < 145){
    speed++;
    delay(1000);
  }
}
