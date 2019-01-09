// Configuration definitions
#define BLINK_N_TIMES 5

/*PROBABLY DON'T MODIFY ANYTHING BELOW THIS LINE*/

// Include headers
#include <avr/power.h>
#include "util/delay.h"

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
  {15, false, 12}, // SWITCH 0
  {14, false, 11}, // SWITCH 1
  {6,  false, 10}, // SWITCH 2
  {4,  false,  9}, // SWITCH 3
  {2,  false,  8}, // SWITCH 4
  {3,  false,  4}, // SWITCH 5
  {5,  false,  3}, // SWITCH 6
  {7,  false,  2}, // SWITCH 7
  {9,  false,  1}, // SWITCH 8
  {16, false,  0}  // SWITCH 9
};

// Global state for any switch change event
boolean switchStateChanged = false;

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
uint8_t patternNumber = 0;

void setup(){
//  // Configure 5V passthrough pin
//  pinMode(10, INPUT);
  // Configure shift register pins
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);  
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  // Configure MODE pin
  pinMode(MODE, INPUT_PULLUP);
  // Configure switch pins
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    pinMode(switches[i].switchNumber, INPUT_PULLUP);
  }

  // Seed random
  randomSeed(analogRead(17));

  // Wait a moment for hardware to finish initializing
  _delay_ms(200);
}

typedef void (*PatternList[])();
PatternList patterns = {onOff, scramble, off};

void loop() {
  getMode();
  getSwitchStates();
  patterns[patternNumber]();
}

void getMode() {
  #define debounceDelay 200
  static unsigned long debounceTime = 0;
  static uint8_t previousState = HIGH;
  static uint8_t currentState;
  
  uint8_t reading = digitalRead(MODE);
  
  if(reading != previousState) {
    debounceTime = millis();
  }
  
  if((millis() - debounceTime > debounceDelay) && (reading != currentState)) {
    currentState = reading;
    if(currentState == LOW){
      patternNumber = (patternNumber + 1) % ARRAY_SIZE(patterns);
    }
  }
  previousState = reading;
}

void getSwitchStates() {
  switchStateChanged = false;
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
//    (digitalRead(switches[i].switchNumber)) ? (switches[i].isOn = true) : (switches[i].isOn = false);
    boolean state = digitalRead(switches[i].switchNumber);
    if(state != switches[i].isOn) {
      switchStateChanged = true;
    }
    switches[i].isOn = state;
  }
}

void onOff() {
  updateShiftRegister(updateBits());
}

void scramble() {
  uint16_t randomBits;
  // Determine if any button switch have changed
  if(switchStateChanged) {
    // Blink random lights BLINK_N_TIMES
    for(uint8_t i=0; i<BLINK_N_TIMES; i++){
      randomBits = random(65536L) & 7967; // 0001111100011111 binary mask
      updateShiftRegister(randomBits);
      _delay_ms(350);
    }
  }

  // Show actual switch states
  getSwitchStates();
  updateShiftRegister(updateBits());
}

void off() {
  updateShiftRegister(0);
}

uint16_t updateBits() {
  // Set shift regsiter bit values
  uint16_t bits = 0;
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
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

