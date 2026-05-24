#ifndef FACE_H
#define FACE_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../display/assets/eye_sprites.h"

// Standard dimensions for SSD1306 OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

enum EyeState {
    EYES_FRONT,
    EYES_LEFT,
    EYES_RIGHT,
    EYES_CONFUSED
};

class Face {
private:
    Adafruit_SSD1306 display;
    EyeState currentState;
    bool ready;

    unsigned long lastFrameMs;
    bool confusedToggle;
    bool needsRedraw;

public:
    Face();
    bool begin();
    void setState(EyeState newState);
    void update();
};

#endif