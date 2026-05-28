// firmware_sil/hal/Adafruit_SSD1306.h
//
// Host stub for <Adafruit_SSD1306.h>. A no-op OLED so the firmware's Face (eye
// animations) compiles and runs in the SIL with no I2C panel attached: every draw
// call is discarded and begin() reports success so Face marks itself ready. The
// SIL drives PyBullet, not a screen — the eyes are irrelevant to servo output.
#pragma once

#include <cstdint>

#include <Adafruit_GFX.h>
#include <Wire.h>

#define SSD1306_BLACK 0
#define SSD1306_WHITE 1
#define SSD1306_INVERSE 2
#define SSD1306_SWITCHCAPVCC 0x02
#define SSD1306_EXTERNALVCC 0x01

class Adafruit_SSD1306 : public Adafruit_GFX {
   public:
    Adafruit_SSD1306(int = 128, int = 64, TwoWire* = nullptr, int = -1) {}
    bool begin(uint8_t = 0, uint8_t = 0, bool = true, bool = true) { return true; }
    void clearDisplay() {}
    void display() {}
    void drawBitmap(int, int, const uint8_t*, int, int, uint16_t) {}
};
