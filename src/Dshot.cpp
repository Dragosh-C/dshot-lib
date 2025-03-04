#include "Dshot.h"

#define NOP __asm__ __volatile__("nop")

volatile uint16_t value;
volatile uint8_t crc;

// Transmit a bit
#define TRANSMIT_BIT(bit, port, pin)  \
  if (bit) {                         \
    SET_PIN_HIGH(port, pin);         \
    _delay_us(2.10);                 \
    SET_PIN_LOW(port, pin);          \
    _delay_us(0.45);                \
  } else {                           \
    SET_PIN_HIGH(port, pin);         \
    _delay_us(0.778);                \
    SET_PIN_LOW(port, pin);          \
    _delay_us(1.55);                \
  }

void dshot_run(int16_t throttle, int8_t pin) {
  cli();
  value = (throttle << 1) | 0;  // 12 bits: throttle (12 bits) + telemetry bit
  crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;  // CRC calculation

  volatile uint8_t *port;
  if (pin < 8) {
      port = &PORTD;  // Pin 0-7 -> PORTD
  } else if (pin < 14) {
      port = &PORTB;  // Pin 8-13 -> PORTB
  } else {
      port = &PORTC;  // Pin 14-19 -> PORTC
  }

  pin = pin % 8;

  // For 'value' (12 bits)
  TRANSMIT_BIT((value >> 11) & 1, *port, pin);
  TRANSMIT_BIT((value >> 10) & 1, *port, pin);
  TRANSMIT_BIT((value >> 9) & 1, *port, pin);
  TRANSMIT_BIT((value >> 8) & 1, *port, pin);
  TRANSMIT_BIT((value >> 7) & 1, *port, pin);
  TRANSMIT_BIT((value >> 6) & 1, *port, pin);
  TRANSMIT_BIT((value >> 5) & 1, *port, pin);
  TRANSMIT_BIT((value >> 4) & 1, *port, pin);
  TRANSMIT_BIT((value >> 3) & 1, *port, pin);
  TRANSMIT_BIT((value >> 2) & 1, *port, pin);
  TRANSMIT_BIT((value >> 1) & 1, *port, pin);
  TRANSMIT_BIT(value & 1, *port, pin);

  // For 'crc' (4 bits)
  TRANSMIT_BIT((crc >> 3) & 1, *port, pin);
  TRANSMIT_BIT((crc >> 2) & 1, *port, pin);
  TRANSMIT_BIT((crc >> 1) & 1, *port, pin);
  TRANSMIT_BIT(crc & 1, *port, pin);
  sei();
}

void dshot_init(int8_t pin) {
  pinMode(pin, OUTPUT);
  _delay_us(100);
  int time = millis();
  while (millis() - time < 500) {
    dshot_run(0, pin);
    _delay_us(192.5);
  }
}
