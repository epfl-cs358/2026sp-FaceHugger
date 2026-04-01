#include "sensors.h"
#include <Arduino.h>

void initSensors() {
    Serial.println("Sensors: Initializing (Simulation Mode)...");
}

int getDistance() {
    // Mock data: returns a random distance between 50 and 500mm
    return random(50, 500); 
}