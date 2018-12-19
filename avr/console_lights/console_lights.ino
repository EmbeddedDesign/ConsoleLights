// Shift register pin definitions
#define SHIFT_DATA_PIN  11
#define SHIFT_LATCH_PIN 12
#define SHIFT_CLOCK_PIN 13

// Mode pin
#define MODE 19

// Switch & LED state 
struct switch_struct {
  uint8_t switchNumber;
  boolean isOn;
  uint8_t bitPosition;
};
switch_struct switches[10] = {
  {15, false,  5}, // SWITCH 0
  {14, false,  4}, // SWITCH 1
  {6,  false,  3}, // SWITCH 2
  {4,  false,  2}, // SWITCH 3
  {2,  false,  1}, // SWITCH 4
  {3,  false, 13}, // SWITCH 5
  {5,  false, 12}, // SWITCH 6
  {7,  false, 11}, // SWITCH 7
  {9,  false, 10}, // SWITCH 8
  {16, false,  9}  // SWITCH 9
};

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
uint8_t patternNumber = 0;

void setup(){
//  // Configure 5V passthrough pin
//  pinMode(10, INPUT);
  // Configure shift register pins
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);  
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  // Configure switch pins
  for(uint8_t i=0; i<sizeof(switches); i++) {
    pinMode(switches[i].switchNumber, INPUT_PULLUP);
  }

  // Seed random
  randomSeed(analogRead(17));
}

typedef void (*PatternList[])();
PatternList patterns = {scramble, off};

void loop() {
  getMode();
  getSwitchStates();
  patterns[patternNumber]();
}

void getMode() {
  #define debounce 200
  static long time = 0;
  static uint8_t previousState = HIGH;
  uint8_t currentState = digitalRead(MODE);
  if((currentState == LOW) && (previousState == HIGH) && (millis() - time > debounce)) {
    patternNumber = (patternNumber + 1) % ARRAY_SIZE(patterns);
    time = millis();
  }
  previousState = currentState;
}

void getSwitchStates() {
  for(uint8_t i=0; i<sizeof(switches); i++) {
    (digitalRead(switches[i].switchNumber)) ? (switches[i].isOn = false) : (switches[i].isOn = true);
  }
}

void scramble() {
  updateShiftRegister(updateBits());
}

void off() {
  updateShiftRegister(0);
}

uint16_t updateBits() {
  // Set shift regsiter bit values
  uint16_t bits = 0;
  for(uint8_t i=0; i<sizeof(switches); i++) {
    if(switches[i].isOn) {
      ((bits) |= (1UL << (switches[i].bitPosition))); // Equivilent to bitSet()
    }
  }
  return bits;
}

void updateShiftRegister(uint16_t bits) {
  // Update shift register
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, LSBFIRST, (bits>>8)); // Shift out high byte
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, LSBFIRST, bits);      // Shift out low byte
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
}

