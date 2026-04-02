#include <Arduino.h>
#include "brain/network.h"
#include "shared/data.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize the network stack
    initNetwork();
    
    Serial.println("FaceHugger OS Online.");
}

void loop() {
    // Keep the WebSocket server alive and listening
    updateNetwork();
}