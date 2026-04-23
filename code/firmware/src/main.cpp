#include <Arduino.h>
#include "brain/network.h"
#include "shared/data.h"
#include "nervous_system/spinal_cord.h"

SpinalCord spinalCord(0);

void setup() {
    Serial.begin(115200);
    delay(2000);
    spinalCord.begin();
    // Initialize the network stack
    initNetwork();
    
    Serial.println("FaceHugger OS Online.");
}

void loop() {
    // Keep the WebSocket server alive and listening
    updateNetwork();
    //spinalCord.update();
    // Heartbeat every 5 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.println("Robot Pulse: OK");
    }
}