#include <Arduino.h>
#include <util/delay.h>
#define NOP __asm__ __volatile__("nop")

void delay800ns() {
    NOP; // 16 MHz clock 1 NOP = 62.5 ns 
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
}

void setup() {
  Serial.begin(9600);
  cli(); //deactivate interrupts
  // PORTD |= _BV(PB1);   // PB1 HIGH (pin 9 arduino nano)
  // _delay_us(3.875);
  // PORTD &= ~_BV(PB1);  // PB1 LOW
  // sei(); // activate interrupts

  Serial.println("SSSSSSSSSSSTCCCC");
  int8_t telemetry_bit = 0;  
  int16_t throttle = 1046;
  int16_t value = 0;
  int8_t crc = 0;
  int8_t i;

  throttle = 1046;
  value = (throttle << 1) | telemetry_bit;  // 13 bits: throttle (12 bits) + telemetry bit

  // CRC calculation 
  crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

  for (i = 0; i < 12; i++) {
    // Serial.print((value >> (11 - i)) & 1);
    if ((value >> (11 - i)) & 1) {
      PORTD |= _BV(PB1);   // PB1 HIGH (pin 9 arduino nano)
      _delay_us(2.5);
      PORTD &= ~_BV(PB1);  // PB1 LOW
      _delay_us(0.8);
    } else {
      PORTD |= _BV(PB1);
      _delay_us(1.25);
      PORTD &= ~_BV(PB1);
      _delay_us(2.05);
    }
  }

  for (i = 0; i < 4; i++) {
    Serial.print((crc >> (3 - i)) & 1);
  }
    
  sei(); // activate interrupts
}

void loop() {
  // put your main code here, to run repeatedly:
}
