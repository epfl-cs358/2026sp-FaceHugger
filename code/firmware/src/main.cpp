#include <Arduino.h>
#include "brain/network.h"
#include "brain/diagnostics.h"
#include "shared/data.h"
#include "nervous_system/spinal_cord.h"

SpinalCord spinalCord(0);

void setup() {
    Serial.begin(115200);
    delay(2000);
    spinalCord.begin();
    initNetwork();
    diagnostics::begin(spinalCord);

    Serial.println("FaceHugger OS Online.");
}

void loop() {
    updateNetwork();
    spinalCord.update();
    diagnostics::tick();
    diagnostics::handleHttp();

    // Heartbeat every 5 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.println("Robot Pulse: OK");
    }
}