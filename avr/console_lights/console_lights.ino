// Configuration definitions
#define BLINK_N_TIMES 5   // How many times the random lights blink when a switch is toggled
#define BLINK_DELAY   350 // The delay in milliseconds between random blinks

/*PROBABLY DON'T MODIFY ANYTHING BELOW THIS LINE*/

// Include headers
#include <avr/power.h>
#include "util/delay.h"

// Switch & LED structure
struct switch_struct {
  uint8_t switchPin;
  boolean isOn;
  uint8_t bitPosition;
};

// Array of switch_struct elements
switch_struct switches[10] = {
  {15, false, 12}, // SWITCH 0 PC1
  {14, false, 11}, // SWITCH 1 PC0
  {6,  false, 10}, // SWITCH 2 PD6
  {4,  false,  9}, // SWITCH 3 PD4
  {2,  false,  8}, // SWITCH 4 PD2
  {3,  false,  4}, // SWITCH 5 PD3
  {5,  false,  3}, // SWITCH 6 PD5
  {7,  false,  2}, // SWITCH 7 PD7
  {9,  false,  1}, // SWITCH 8 PB1
  {16, false,  0}  // SWITCH 9 PC2
};

// Global state for any switch change event
boolean switchStateChanged = false;

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0])) // Size function for arbitrary array types
uint8_t patternNumber = 0; // Default pattern number

// Configure hardware
void setup(){
  // Configure shift register pins
  DDRB |= 0x38; // DATA_PIN (11), LATCH_PIN (12), CLOCK_PIN (13) to OUTPUT
  // Configure switch and mode pins
  PORTB |= 0x02; // SWITCH 8 to INPUT_PULLUP
  PORTC |= 0x27; // SWITCHS 0, 1, 9, and MODE to INPUT_PULLUP
  PORTD |= 0xFC; // SWITCHS 2, 3, 4, 5, 6, 7 to INPUT_PULLUP

  srandom(analogRead(17)); // Seed random from floating pin

  _delay_ms(200); // Wait a moment for hardware to finish initializing
}

// List of available pattern functions
typedef void (*PatternList[])();
PatternList patterns = {scramble, onOff, off};

// Run forever
void loop() {
  getMode();
  getSwitchStates();
  patterns[patternNumber]();
}

// Set the mode based upon MODE switch presses
// Increments to the next pattern on each press, wrap around to the first pattern at the end
// Includes button debounce
void getMode() {
  #define debounceDelay 150
  static unsigned long debounceTime = 0;
  static uint8_t previousState = HIGH;
  static uint8_t currentState;
  
  uint8_t reading = PINC & _BV(PC5); // Read MODE (19) pin;
  
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

// Read the current state of all the switches
void getSwitchStates() {
  switchStateChanged = false; // Reset global switch state changed variable
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    boolean state = digitalRead(switches[i].switchPin);
    if(state != switches[i].isOn) {
      switchStateChanged = true;
    }
    switches[i].isOn = state;
  }
}

// LED is on if switch is depressed, off otherwise
void onOff() {
  updateShiftRegister(updateBits());
}

// Toggling any switch blinks random LEDs 'BLINK_N_TIMES' times then returns to onOff states
void scramble() {
  uint16_t randomBits;
  // Determine if any button switch have changed
  if(switchStateChanged) {
    // Blink random lights BLINK_N_TIMES
    for(uint8_t i=0; i<BLINK_N_TIMES; i++){
      randomBits = random(65536L) & 7967; // 0001111100011111 binary mask
      updateShiftRegister(randomBits);
      _delay_ms(BLINK_DELAY);
    }
  }

  // Show actual switch states
  getSwitchStates();
  updateShiftRegister(updateBits());
}

// Turn off all LEDs, regardless of switch states
void off() {
  updateShiftRegister(0);
}

// Update corresponding shift-register bits in a 16-bit number to reflect current switch states
uint16_t updateBits() {
  uint16_t bits = 0;
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    if(switches[i].isOn) {
      ((bits) |= (1UL << (switches[i].bitPosition))); // Equivalent to bitSet()
    }
  }
  return bits;
}

// Update shift register
void updateShiftRegister(uint16_t bits) {
  PORTB &= ~_BV(PB4); // LATCH_PIN (12) LOW
  shiftBits(bits>>8); // Shift out high byte
  shiftBits(bits);    // Shift out low byte
  PORTB |= _BV(PB4);  // LATCH_PIN (12) HIGH
}

// Shift out 1 byte
void shiftBits(uint8_t value) {
   for (uint8_t i=0; i<8; i++) { // Shift out byte
    (!!(value & (1 << i))) ? (PORTB |= _BV(PB3)) : (PORTB &= ~_BV(PB3)); // DATA_PIN (11) to bit value
    // Toggle clock
    PORTB |= _BV(PB5);  // CLOCK_PIN (13) HIGH
    PORTB &= ~_BV(PB5); // CLOCK_PIN (13) LOW
  }
}

