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
