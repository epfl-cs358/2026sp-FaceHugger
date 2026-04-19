#include <Arduino.h>
#include "brain/network.h"
#include "shared/data.h"
#include "nervous_system/spinal_cord.h"

SpinalCord spinalCord(0);

void setup() {
    Serial.begin(115200);
    
    spinalCord.begin();
    // Initialize the network stack
    initNetwork();
    
    Serial.println("FaceHugger OS Online.");
}

void loop() {
    // Keep the WebSocket server alive and listening
    updateNetwork();
    spinalCord.update();
}