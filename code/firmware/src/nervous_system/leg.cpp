#include "leg.h"
#include <Arduino.h>

Leg::Leg(int legID) {
    id = legID;
}

void Leg::setPose(float x, float y, float z) {
    Serial.print("Leg ");
    Serial.print(id);
    Serial.printf(" moving to target: %.1f, %.1f, %.1f\n", x, y, z);
}