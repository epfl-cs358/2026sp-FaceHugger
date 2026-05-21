#include "face.h"
#include <Wire.h>

// Initialize the display object connected to standard I2C pins
Face::Face() :
    display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),
    currentState(EYES_FRONT),
    ready(false),
    lastFrameMs(0),
    confusedToggle(false),
    needsRedraw(true)
{}

bool Face::begin() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }
    ready = true;

    display.clearDisplay();
    display.drawBitmap(0, 0, eyes_front, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    display.display();
    return true;
}

void Face::setState(EyeState newState) {
    if (currentState != newState) {
        currentState = newState;
        needsRedraw = true; // Flag that the screen needs to be updated
    }
}

void Face::update() {
    if (!ready) return;

    if (currentState == EYES_CONFUSED) {
        if (millis() - lastFrameMs > 300) { // Swap frames every 300ms
            confusedToggle = !confusedToggle;
            needsRedraw = true;
            lastFrameMs = millis();
        }
    }

    // Only talk to the screen over I2C if something actually changed
    if (needsRedraw) {
        display.clearDisplay();
        
        switch (currentState) {
            case EYES_FRONT:
                display.drawBitmap(0, 0, eyes_front, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
                break;
            case EYES_LEFT:
                display.drawBitmap(0, 0, eyes_left, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
                break;
            case EYES_RIGHT:
                display.drawBitmap(0, 0, eyes_right, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
                break;
            case EYES_CONFUSED:
                if (confusedToggle) {
                    display.drawBitmap(0, 0, eyes_confused_1, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
                } else {
                    display.drawBitmap(0, 0, eyes_confused_2, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
                }
                break;
        }
        
        display.display(); // Push the buffer to the hardware
        needsRedraw = false;
    }
}