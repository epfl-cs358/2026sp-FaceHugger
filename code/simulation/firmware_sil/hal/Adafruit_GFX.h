// firmware_sil/hal/Adafruit_GFX.h
//
// Host stub for <Adafruit_GFX.h>. The SIL never drives a screen; this exists only
// so the firmware's face.h (OLED eyes) parses on the desktop. Adafruit_SSD1306
// derives from this base in the real library, so we provide an empty base here.
#pragma once

#include <cstdint>

#include <Arduino.h>  // real Adafruit_GFX pulls this in too — gives face.cpp /
                      // eye_sprites.h their PROGMEM, F(), Serial, millis on the host.

class Adafruit_GFX {};
