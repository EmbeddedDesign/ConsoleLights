// Shift register pin definitions
#define SHIFT_LATCH_PIN 4
#define SHIFT_CLOCK_PIN 5
#define SHIFT_DATA_PIN  2
//#define SHIFT_OE        3
//#define SHIFT_CLEAR     6

void setup(){
  //Configure shift register pins
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);  
  pinMode(SHIFT_DATA_PIN, OUTPUT);
//  pinMode(SHIFT_OE, OUTPUT);
//  pinMode(SHIFT_CLEAR, OUTPUT);
//  digitalWrite(SHIFT_CLEAR, HIGH);
//  digitalWrite(SHIFT_OE, LOW);
}

void loop() {
  uint16_t test1 = B01111111;
  updateLEDS(test1);
  delay(500);
  uint16_t test2 = B00000000;
  updateLEDS(test2);
  delay(500);
}

void updateLEDS(uint8_t bits) {
  // Update shift register
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, LSBFIRST, bits);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, LSBFIRST, bits);
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
}

