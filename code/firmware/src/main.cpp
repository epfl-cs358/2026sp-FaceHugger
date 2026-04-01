#include <Arduino.h>
#include "shared/config.h"
#include "brain/sensors.h"
#include "nervous_system/movements.h"

void setup() {
    Serial.begin(115200);
    initSensors();
    Serial.println("FaceHugger Boot Complete.");
}

void loop() {
    int dist = getDistance();
    Serial.print("Current Distance: ");
    Serial.println(dist);

    if (dist < 150) {
        Serial.println("Obstacle detected!");
        executeWalk(); 
    }

    delay(1000);
}