// #include <Arduino.h>
// #include <util/delay.h>
// #define NOP __asm__ __volatile__("nop")



//   /*-----------------------------

//     PORTB &= ~_BV(PB7); // two cycles (16 MHz) = 125 ns delay

//     Setting the registers and if condition take aprox 180 ns each
//     HIGH: 2.5 us
//     LOW: 1.25 us
//     Period: 3.3 us

//     For each iteration of the loop, the total cycle count is approximately:

//     1 cycle for comparison (CP).
//     5 cycles for the i++ operation (2 cycles to load, 1 cycle to increment, 2 cycles to store).
//     2 cycles for the branch (BRNE).


//   -------------------------------*/


// #define SET_PIN_HIGH(port, pin)   (port |= _BV(pin))
// #define SET_PIN_LOW(port, pin)    (port &= ~_BV(pin))

// // // transmit a bit
// // #define TRANSMIT_BIT(bit, port, pin)  \
// //   if (bit) {                         \
// //     SET_PIN_HIGH(port, pin);         \
// //     _delay_us(2.20);                 \
// //     SET_PIN_LOW(port, pin);          \
// //     _delay_us(0.45);                \
// //   } else {                           \
// //     SET_PIN_HIGH(port, pin);         \
// //     _delay_us(0.878);                \
// //     SET_PIN_LOW(port, pin);          \
// //     _delay_us(1.55);                \
// //   }



// // transmit a bit
// #define TRANSMIT_BIT(bit, port, pin)  \
//   if (bit) {                         \
//     SET_PIN_HIGH(port, pin);         \
//     _delay_us(2.10);                 \
//     SET_PIN_LOW(port, pin);          \
//     _delay_us(0.45);                \
//   } else {                           \
//     SET_PIN_HIGH(port, pin);         \
//     _delay_us(0.778);                \
//     SET_PIN_LOW(port, pin);          \
//     _delay_us(1.55);                \
//   }


//   volatile uint16_t value;
//   volatile uint8_t crc;

// void dshot_run(int16_t throttle, int8_t pin) {
//   cli();
//   value = (throttle << 1) | 0;  // 13 bits: throttle (12 bits) + telemetry bit
//   crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;  // CRC calculation

//   volatile uint8_t *port;
//   if (pin < 8) {
//       port = &PORTD;  // Pin 0-7 -> PORTD
//   } else if (pin < 14) {
//       port = &PORTB;  // Pin 8-13 -> PORTB
//   } else {
//       port = &PORTC;  // Pin 14-19 -> PORTC
//   }

//   pin = pin % 8;

//   // For 'value' (13 bits)
//   TRANSMIT_BIT((value >> 11) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 10) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 9) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 8) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 7) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 6) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 5) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 4) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 3) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 2) & 1, *port, pin);
//   TRANSMIT_BIT((value >> 1) & 1, *port, pin);
//   TRANSMIT_BIT(value & 1, *port, pin);

//   // For 'crc' (4 bits)
//   TRANSMIT_BIT((crc >> 3) & 1, *port, pin);
//   TRANSMIT_BIT((crc >> 2) & 1, *port, pin);
//   TRANSMIT_BIT((crc >> 1) & 1, *port, pin);
//   TRANSMIT_BIT(crc & 1, *port, pin);
//   sei();
// }



// void dshot_init(int8_t pin) {
//   pinMode(pin, OUTPUT);
//   _delay_us(100);
//   int time = millis();
//   while (millis() - time < 500) {
//     dshot_run(0, pin);
//     _delay_us(192.5);
//   }
//   // _delay_ms(500);
// }



// void setup() {

//   dshot_init(9);
//   int time = millis();
//   // int i = 50;
//   int16_t pot_value = 0;

//   pinMode(A0, INPUT);

//   Serial.begin(115200);

//   while(1){

//     pot_value = analogRead(A0);
//     pot_value = map(pot_value, 0, 1023, 0, 2047);

//     if (pot_value < 47) {
//       pot_value = 0;
//     }

//     if (pot_value > 1000) {
//       pot_value = 1000;
//     }
//     Serial.println(pot_value);

//     dshot_run(pot_value, 9);
//     // delay(1);
//     _delay_us(192.5);
//     // dshot_run(0, 9);
//     // delay(1);

    
//   }

// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }



#include <Arduino.h>
#include "Dshot.h"


void setup() {
  dshot_init(9);  // Initialize DShot on pin 9

  int16_t pot_value = 0;

  pinMode(A0, INPUT);

  while (1) {
    pot_value = analogRead(A0);
    pot_value = map(pot_value, 0, 1023, 0, 2047);

    if (pot_value < 48) {
      pot_value = 0;
    }

    if (pot_value > 1000) { // Not to spin the motor too fast (full range is 48 - 2047)
      pot_value = 1000;
    }

    dshot_run(pot_value, 9);
    _delay_us(192.5);
  }
}

void loop() {

}
