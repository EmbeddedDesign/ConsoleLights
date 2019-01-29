// Configuration definitions
#define BLINK_N_TIMES 5   // How many times the random lights blink when a switch is toggled
#define BLINK_DELAY   350 // The delay in milliseconds between random blinks

/*PROBABLY DON'T MODIFY ANYTHING BELOW THIS LINE*/

// Include headers
#include "util/delay.h"
#include "PinChangeInterrupt.h"

// Switch & LED structure
struct switch_struct {
  boolean isOn;
  uint8_t switchPin;
  uint8_t bitPosition;
  uint8_t PCINT;
};

// Array of switch_struct elements
// Format: {isOn, switchPin, bitPosition, PCINT}
switch_struct switches[10] = {
  {false, 15, 12,  9}, // SWITCH 0 PC1
  {false, 14, 11,  8}, // SWITCH 1 PC0
  {false, 6,  10, 22}, // SWITCH 2 PD6
  {false, 4,  9,  20}, // SWITCH 3 PD4
  {false, 2,  8,  18}, // SWITCH 4 PD2
  {false, 3,  4,  19}, // SWITCH 5 PD3
  {false, 5,  3,  21}, // SWITCH 6 PD5
  {false, 7,  2,  23}, // SWITCH 7 PD7
  {false, 9,  1,   1}, // SWITCH 8 PB1
  {false, 16, 0,  10}  // SWITCH 9 PC2
};

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0])) // Size function for arbitrary array types
uint8_t patternNumber = 0; // Default pattern number

// Pattern function prototypes
// Declared here so we can call them in setup before properly defining them later on
void scramble(void);
void onOff(void);
void off(void);

// List of available pattern functions
typedef void (*PatternList[])();
PatternList patterns = {scramble, onOff, off};

// Configure hardware
void setup(){
  // Configure shift register pins
  DDRB |= 0x38; // DATA_PIN (11), LATCH_PIN (12), CLOCK_PIN (13) to OUTPUT
  // Configure switch and mode pins
  PORTB |= 0x02; // SWITCH 8 to INPUT_PULLUP
  PORTC |= 0x27; // SWITCHS 0, 1, 9, and MODE (19) to INPUT_PULLUP
  PORTD |= 0xFC; // SWITCHS 2, 3, 4, 5, 6, 7 to INPUT_PULLUP

  // Attach FALLING interrupt to MODE (19/PCINT13) pin
  attachPCINT(13, getMode, FALLING);

  // Attach CHANGE interrupts to switch pins
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    attachPCINT(switches[i].PCINT, pinChangedINT, CHANGE);
  }

  srandom(analogRead(17)); // Seed random from floating pin

  _delay_ms(200); // Wait a moment for hardware to finish initializing

  getSwitchStates(); // Get initial switch states
  patterns[patternNumber](); // Run the default pattern
}

// Run forever
void loop() {
  // Empty loop, everything is interrupt based
}

// Set the mode based on MODE (19) switch presses
// Increments to the next pattern on each press, wrap around to the first pattern at the end
void getMode() {
  patternNumber = (patternNumber + 1) % ARRAY_SIZE(patterns);
  patterns[patternNumber]();
}

void pinChangedINT() {
  // Disable interrupts to switch pins
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    disablePCINT(switches[i].PCINT);
  }

  // Run the current pattern
  patterns[patternNumber]();
  
  // Enable interrupts to switch pins
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    enablePCINT(switches[i].PCINT);
  }
}

// Called whenever a switch changes state
void getSwitchStates() {
  for(uint8_t i=0; i<ARRAY_SIZE(switches); i++) {
    uint8_t pin = switches[i].switchPin;
    // Faster, lighter digitalRead()
    switches[i].isOn = *portInputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin);
  }
}

// LED is on if switch is depressed, off otherwise
void onOff() {
  _delay_ms(50); // Debounce delay
  getSwitchStates();
  updateShiftRegister(updateBits());
}

// Toggling any switch blinks random LEDs 'BLINK_N_TIMES' times then returns to onOff states
void scramble() {
  uint16_t randomBits;
  // Blink random lights BLINK_N_TIMES
  for(uint8_t i=0; i<BLINK_N_TIMES; i++){
    randomBits = random(65536L) & 7967; // 0001111100011111 binary mask for shift register bits
    updateShiftRegister(randomBits);
    _delay_ms(BLINK_DELAY);
  }

  // Show actual switch states
  getSwitchStates(); // Check states again in case anything changed during routine
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
    // DATA_PIN (11) to bit value
    (!!(value & (1 << i))) ? (PORTB |= _BV(PB3)) : (PORTB &= ~_BV(PB3));
    // Toggle clock
    PORTB |= _BV(PB5);  // CLOCK_PIN (13) HIGH
    PORTB &= ~_BV(PB5); // CLOCK_PIN (13) LOW
  }
}

