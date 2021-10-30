// ===================================================================================
// Project:   TinyRemoteXL - IR Remote Control based on ATtiny13A
// Version:   v1.0
// Year:      2021
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// IR remote control using an ATtiny 13A. Timer0 generates a carrier
// frequency with a duty cycle of 25% on the output pin to the
// IR LED. The signal is modulated by toggling the pin to input/output.
//
// The code utilizes the sleep mode power down function. The device will
// work several months on a CR2032 battery.
//
// Wiring:
// -------
//                          +-\/-+
//       --- RST ADC0 PB5  1|°   |8  Vcc
// BINT2 ------- ADC3 PB3  2|    |7  PB2 ADC1 -------- BADC1
// BADC2 ------- ADC2 PB4  3|    |6  PB1 AIN1 OC0B --- IR LED
//                    GND  4|    |5  PB0 AIN0 OC0A --- BINT1
//                          +----+
//
// Compilation Settings:
// ---------------------
// Controller: ATtiny13A
// Core:       MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed: 1.2 MHz internal
// BOD:        BOD disabled
// Timing:     Micros disabled
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// Fuse settings: -U lfuse:w:0x2a:m -U hfuse:w:0xff:m


// ===================================================================================
// Libraries and Definitions
// ===================================================================================

// Oscillator calibration value (uncomment and set if necessary)
//#define OSCCAL_VAL  0x48

// Libraries
#include <avr/io.h>           // for GPIO
#include <avr/sleep.h>        // for sleep mode
#include <avr/pgmspace.h>     // to store data in programm memory
#include <avr/interrupt.h>    // for interrupts
#include <util/delay.h>       // for delays

// Pin assignments
#define IR_PIN      PB1       // IR LED pin
#define BINT1_PIN   PB0       // interrupt pin for buttons 1..6
#define BINT2_PIN   PB3       // interrupt pin for buttons 7..12
#define BADC1_AP    1         // ADC port for buttons 1..6
#define BADC2_AP    2         // ADC port for buttons 7..12

// Assign IR commands to the keys. Different protocols and device addresses
// can be used. Several codes can also be assigned to a single key, separated
// by semicolons.
#define KEY1  NEC_sendCode(0x04,0x08)     // LG TV Power: addr 0x04, cmd 0x08
#define KEY2  RC5_sendCode(0x00,0x0B)     // Philips TV Power: addr 0x00, cmd 0x0B
#define KEY3  SON_sendCode(0x01,0x15,12)  // Sony TV Power: addr 0x01, cmd 0x15, 12-bit version
#define KEY4  SAM_sendCode(0x07,0x02)     // Samsung TV Power: addr: 07, cmd: 02
#define KEY5  NEC_sendCode(0xAB04,0x08);SON_sendCode(0xE401,0x15,20)
#define KEY6  NEC_sendCode(0x04,0x01)
#define KEY7  NEC_sendCode(0x04,0x02)
#define KEY8  NEC_sendCode(0x04,0x03)
#define KEY9  NEC_sendCode(0x04,0x04)
#define KEY10 NEC_sendCode(0x04,0x05)
#define KEY11 NEC_sendCode(0x04,0x06)
#define KEY12 NEC_sendCode(0x04,0x07)

// Macros to switch on/off IR LED
#define IR_on()   DDRB |=  (1<<IR_PIN)    // PB1 as output = IR at OC0B
#define IR_off()  DDRB &= ~(1<<IR_PIN)    // PB1 as input  = LED off

// Port mask for button interrupt pins
#define BT_MASK   ((1<<BINT1_PIN)|(1<<BINT2_PIN))

// ===================================================================================
// NEC Protocol Implementation
// ===================================================================================
//
// The NEC protocol uses pulse distance modulation.
//
//       +--------------+          +-------+       +-------+          +-- ON
//       |              |          |       |       |       |          |
//       |    9000us    |  4500us  |562.5us|562.5us|562.5us| 1687.5us |   ...
//       |              |          |       |       |       |          |
// ------+              +----------+       +-------+       +----------+   OFF
//
//       |<----- Start Frame ----->|<--- Bit=0 --->|<----- Bit=1 ---->| 
//
// IR telegram starts with a 9ms leading burst followed by a 4.5ms pause.
// Afterwards 4 data bytes are transmitted, least significant bit first.
// A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is
// a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst
// signifies the end of the transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// The Extended NEC protocol uses 16-bit addresses. Instead of sending an
// 8-bit address and its logically inverse, first the low byte and then the
// high byte of the address is transmitted.
//
// If the key on the remote controller is kept depressed, a repeat code
// will be issued consisting of a 9ms leading burst, a 2.25ms pause and
// a 562.5us burst to mark the end. The repeat code will continue to be
// sent out at 108ms intervals, until the key is finally released.

// Define values for 38kHz PWM frequency and 25% duty cycle
#define NEC_TOP   31          // 1200kHz / 38kHz - 1 = 31
#define NEC_DUTY  7           // 1200kHz / 38kHz / 4 - 1 = 7

// Macros to modulate the signals according to NEC protocol with compensated timings
#define NEC_startPulse()    {IR_on(); _delay_us(9000); IR_off(); _delay_us(4500);}
#define NEC_repeatPulse()   {IR_on(); _delay_us(9000); IR_off(); _delay_us(2250);}
#define NEC_normalPulse()   {IR_on(); _delay_us( 562); IR_off(); _delay_us( 557);}
#define NEC_bit1Pause()     _delay_us(1120) // 1687.5us - 562.5us = 1125us
#define NEC_repeatCode()    {_delay_ms(40); NEC_repeatPulse(); NEC_normalPulse(); _delay_ms(56);}

// Send a single byte via IR
void NEC_sendByte(uint8_t value) {
  for(uint8_t i=8; i; i--, value>>=1) {   // send 8 bits, LSB first
    NEC_normalPulse();                    // 562us burst, 562us pause
    if(value & 1) NEC_bit1Pause();        // extend pause if bit is 1
  }
}

// Send complete telegram (start frame + address + command) via IR
void NEC_sendCode(uint16_t addr, uint8_t cmd) {
  // Prepare carrier wave
  OCR0A = NEC_TOP;            // set PWM frequency
  OCR0B = NEC_DUTY;           // set duty cycle

  // Send telegram
  NEC_startPulse();           // 9ms burst + 4.5ms pause to signify start of transmission
  if(addr > 0xFF) {           // if extended NEC protocol (16-bit address):
    NEC_sendByte(addr);       // send address low byte
    NEC_sendByte(addr >> 8);  // send address high byte
  } else {                    // if standard NEC protocol (8-bit address):
    NEC_sendByte(addr);       // send address byte
    NEC_sendByte(~addr);      // send inverse of address byte
  }
  NEC_sendByte(cmd);          // send command byte
  NEC_sendByte(~cmd);         // send inverse of command byte
  NEC_normalPulse();          // 562us burst to signify end of transmission
  while(~PINB & BT_MASK) NEC_repeatCode();  // send repeat command until button is released
}

// ===================================================================================
// SAMSUNG Protocol Implementation
// ===================================================================================
//
// The SAMSUNG protocol corresponds to the NEC protocol, except that the start pulse is
// 4.5ms long and the address byte is sent twice. The telegram is repeated every 108ms
// as long as the button is pressed.

#define SAM_startPulse()    {IR_on(); _delay_us(4500); IR_off(); _delay_us(4500);}
#define SAM_repeatPause()   _delay_ms(44)

// Send complete telegram (start frame + address + command) via IR
void SAM_sendCode(uint8_t addr, uint8_t cmd) {
  // Prepare carrier wave
  OCR0A  = NEC_TOP;           // set PWM frequency
  OCR0B  = NEC_DUTY;          // set duty cycle

  // Send telegram
  do {
    SAM_startPulse();         // 9ms burst + 4.5ms pause to signify start of transmission
    NEC_sendByte(addr);       // send address byte
    NEC_sendByte(addr);       // send address byte again
    NEC_sendByte(cmd);        // send command byte
    NEC_sendByte(~cmd);       // send inverse of command byte
    NEC_normalPulse();        // 562us burst to signify end of transmission
    SAM_repeatPause();        // wait for next repeat
  } while(~PINB & BT_MASK);   // repeat sending until button is released
}

// ===================================================================================
// RC-5 Protocol Implementation
// ===================================================================================
//
// The RC-5 protocol uses bi-phase modulation (Manchester coding).
//
//   +-------+                     +-------+    ON
//           |                     |
//     889us | 889us         889us | 889us
//           |                     |
//           +-------+     +-------+            OFF
//
//   |<-- Bit "0" -->|     |<-- Bit "1" -->|
//
// IR telegram starts with two start bits. The first bit is always "1",
// the second bit is "1" in the original protocol and inverted 7th bit
// of the command in the extended RC-5 protocol. The third bit toggles
// after each button release. The next five bits represent the device
// address, MSB first and the last six bits represent the command, MSB
// first.
//
// As long as a key remains down the telegram will be repeated every
// 114ms without changing the toggle bit.

// Define values for 36kHz PWM frequency and 25% duty cycle
#define RC5_TOP   32          // 1200kHz / 36kHz - 1 = 32
#define RC5_DUTY  7           // 1200kHz / 36kHz / 4 - 1 = 7

// Macros to modulate the signals according to RC-5 protocol with compensated timings
#define RC5_bit0Pulse()     {IR_on();  _delay_us(889); IR_off(); _delay_us(884);}
#define RC5_bit1Pulse()     {IR_off(); _delay_us(889); IR_on();  _delay_us(884);}
#define RC5_repeatPause()   _delay_ms(89) // 114ms - 14 * 2 * 889us

// Bitmasks
#define RC5_startBit  0b0010000000000000
#define RC5_cmdBit7   0b0001000000000000
#define RC5_toggleBit 0b0000100000000000

// Toggle variable
uint8_t RC5_toggle = 0;

// Send complete telegram (startbits + togglebit + address + command) via IR
void RC5_sendCode(uint8_t addr, uint8_t cmd) {
  // Prepare carrier wave
  OCR0A  = RC5_TOP;                           // set PWM frequency
  OCR0B  = RC5_DUTY;                          // set duty cycle

  // Prepare the message
  uint16_t message = addr << 6;               // shift address to the right position
  message |= (cmd & 0x3f);                    // add the low 6 bits of the command
  if(~cmd & 0x40) message |= RC5_cmdBit7;     // add inverse of 7th command bit
  message |= RC5_startBit;                    // add start bit
  if(RC5_toggle) message |= RC5_toggleBit;    // add toggle bit

  // Send the message
  do {
    uint16_t bitmask = RC5_startBit;          // set the bitmask to first bit to send
    for(uint8_t i=14; i; i--, bitmask>>=1) {  // 14 bits, MSB first
      (message & bitmask) ? (RC5_bit1Pulse()) : (RC5_bit0Pulse());  // send the bit
    }
    IR_off();                                 // switch off IR LED
    RC5_repeatPause();                        // wait for next repeat
  } while(~PINB & BT_MASK);                   // repeat sending until button is released
  RC5_toggle ^= 1;                            // toggle the toggle bit
}

// ===================================================================================
// SONY SIRC Protocol Implementation
// ===================================================================================
//
// The SONY SIRC protocol uses pulse length modulation.
//
//       +--------------------+     +-----+     +----------+     +-- ON
//       |                    |     |     |     |          |     |
//       |       2400us       |600us|600us|600us|  1200us  |600us|   ...
//       |                    |     |     |     |          |     |
// ------+                    +-----+     +-----+          +-----+   OFF
//
//       |<------ Start Frame ----->|<- Bit=0 ->|<--- Bit=1 ---->| 
//
// A "0" bit is a 600us burst followed by a 600us space, a "1" bit is a
// 1200us burst followed by a 600us space. An IR telegram starts with a
// 2400us leading burst followed by a 600us space. The command and
// address bits are then transmitted, LSB first. Depending on the
// protocol version, these are in detail:
// - 12-bit version: 7 command bits, 5 address bits
// - 15-bit version: 7 command bits, 8 address bits
// - 20-bit version: 7 command bits, 5 address bits, 8 extended bits
//
// As long as a key remains down the message will be repeated every 45ms.

// Define values for 40kHz PWM frequency and 25% duty cycle
#define SON_TOP   29                      // 1200kHz / 40kHz - 1 = 29
#define SON_DUTY  7                       // 1200kHz / 40kHz / 4 - 1 = 7

// Macros to modulate the signals according to SONY protocol with compensated timings
#define SON_startPulse()    {IR_on(); _delay_us(2400); IR_off(); _delay_us( 595);}
#define SON_bit0Pulse()     {IR_on(); _delay_us( 600); IR_off(); _delay_us( 595);}
#define SON_bit1Pulse()     {IR_on(); _delay_us(1200); IR_off(); _delay_us( 595);}
#define SON_repeatPause()   _delay_ms(27)

// Send "number" of bits of "value" via IR
void SON_sendByte(uint8_t value, uint8_t number) {
  do {                                        // send number of bits, LSB first
    (value & 1) ? (SON_bit1Pulse()) : (SON_bit0Pulse());  // send bit
    value>>=1;                                // next bit
  } while(--number);
}

// Send complete telegram (start frame + command + address) via IR
void SON_sendCode(uint16_t addr, uint8_t cmd, uint8_t bits) {
  // Prepare carrier wave
  OCR0A  = SON_TOP;                           // set PWM frequency
  OCR0B  = SON_DUTY;                          // set duty cycle

  // Send telegram
  do {
    SON_startPulse();                         // signify start of transmission
    SON_sendByte(cmd, 7);                     // send 7 command bits
    switch (bits) {
      case 12: SON_sendByte(addr, 5); break;  // 12-bit version: send 5 address bits
      case 15: SON_sendByte(addr, 8); break;  // 15-bit version: send 8 address bits
      case 20: SON_sendByte(addr, 8); SON_sendByte(addr>>8, 5); break; // 20-bit: 13 bits
      default: break;
    }
    SON_repeatPause();                        // wait until next repeat
  } while(~PINB & BT_MASK);                   // repeat sending until button is released
}

// ===================================================================================
// ADC Implementation for Buttons
// ===================================================================================

// Button ADC thresholds
const uint8_t THRESHOLDS[] PROGMEM = {217, 173, 158, 136, 103, 41, 0};

// ADC read button row and return button number
uint8_t readButtonRow(uint8_t port) {
  PRR     = 0;                            // power on ADC
  ADMUX   = (1<<ADLAR) | port;            // set port, Vcc as reference, 8-bit sample
  ADCSRA |= (1<<ADEN) | (1<<ADSC);        // enable ADC and start sampling
  while(ADCSRA & (1<<ADSC));              // wait until sampling complete
  uint8_t raw = ADCH;                     // read sampling result (8 bits)
  ADCSRA &= ~(1<<ADEN);                   // disable ADC
  PRR     = 1<<PRADC;                     // shut down ADC
  uint8_t  button = 0;                    // figure out button number
  while(raw < pgm_read_byte(&THRESHOLDS[button])) button++;
  return button;                          // return button number
}

// Read and return button number (0 = no button)
uint8_t readButton(void) {
  uint8_t  button = 0;                    // start with number 0
  if(~PINB & (1<<BINT1_PIN)) button = readButtonRow(BADC1_AP);
  else if(~PINB & (1<<BINT2_PIN)) {
    button = readButtonRow(BADC2_AP);
    if(button) button += 6;
  }
  return button;                          // return button number
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                // set the value if defined above
  #endif

  // Setup pins
  PORTB = BT_MASK;                      // pull-up for button pins
  
  // Set timer0 to toggle IR pin at carrier wave frequency
  TCCR0A = (1<<COM0B1)|(1<<WGM01)|(1<<WGM00); // PWM on OC0B (PB1)
  TCCR0B = (1<<WGM02)|(1<<CS00);              // start timer, no prescaler
  
  // Setup ADC to read button values
  ADCSRA = (1<<ADPS1)|(1<<ADPS0);       // set ADC prescaler 8

  // Disable unused peripherals and prepare sleep mode to save power
  ACSR  =  1<<ACD;                      // disable analog comperator
  DIDR0 = ~BT_MASK & 0x1F;              // disable digital intput buffer except button INT
  PRR   =  1<<PRADC;                    // shut down ADC
  GIMSK =  1<<PCIE;                     // turn on pin change interrupts
  PCMSK =  BT_MASK;                     // turn on interrupt on button pins
  sei();                                // enable global interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down

  // Main loop
  while(1) {
    sleep_mode();                       // sleep until button is pressed
    _delay_ms(1);                       // debounce
    uint8_t button = readButton();      // read button number
    switch(button) {                    // send corresponding IR code
      case  1:  KEY1;  break;
      case  2:  KEY2;  break;
      case  3:  KEY3;  break;
      case  4:  KEY4;  break;
      case  5:  KEY5;  break;
      case  6:  KEY6;  break;
      case  7:  KEY7;  break;
      case  8:  KEY8;  break;
      case  9:  KEY9;  break;
      case 10:  KEY10; break;
      case 11:  KEY11; break;
      case 12:  KEY12; break;
      default: break;
    }
  }
}

// Pin change interrupt service routine
EMPTY_INTERRUPT(PCINT0_vect);           // nothing to be done here, just wake up from sleep
