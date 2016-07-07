#define BATT_CELL1 A0
#define BATT_CELL2 A1
#define BATT_CELL3 A2
#define BATT_CELL4 A3

#define BATT_V_MAX 3,7
#define BATT_V_WARN 3,5
#define BATT_V_MIN 3,3

long cell1_v;
long cell2_v;
long cell3_v;
long cell4_v;

void setup() {
  
}

void loop() {
    cell1_v = map(analogRead(BATT_CELL1), 0, 1023, 0, 5);
    cell2_v = map(analogRead(BATT_CELL2), 0, 1023, 0, 5);
    cell3_v = map(analogRead(BATT_CELL3), 0, 1023, 0, 5);
    cell4_v = map(analogRead(BATT_CELL4), 0, 1023, 0, 5);
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
