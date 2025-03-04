#ifndef DSHOT_H
#define DSHOT_H

#include <Arduino.h>
#include <util/delay.h>

#define SET_PIN_HIGH(port, pin)   (port |= _BV(pin))
#define SET_PIN_LOW(port, pin)    (port &= ~_BV(pin))

// Function prototypes
void dshot_init(int8_t pin);
void dshot_run(int16_t throttle, int8_t pin);

#endif
