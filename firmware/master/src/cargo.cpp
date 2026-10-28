#include "cargo.h"

#include <Arduino.h>

namespace cargo {

namespace {

int16_t grams = 0;
uint8_t flags = 0;
bool fresh = false;

}

void on_frame(const uint8_t* data, uint8_t dlc) {
  if (dlc < 3) return;
  grams = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
  flags = data[2];
  fresh = true;
}

void update() {
  if (!fresh) return;
  fresh = false;
  Serial.print(F("C "));
  Serial.print(grams);
  for (uint8_t bit = 0; bit < 4; bit++) {
    Serial.print(' ');
    Serial.print((flags >> bit) & 1);
  }
  Serial.println();
}

}
