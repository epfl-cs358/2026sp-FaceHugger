#include <Arduino.h>
#include "brain/network.h"
#include "brain/diagnostics.h"
#include "brain/sensors.h"
#include "shared/data.h"
#include "nervous_system/spinal_cord.h"

SpinalCord spinalCord(0);

void setup() {
    Serial.begin(115200);
    delay(2000);
    spinalCord.begin();
    initNetwork();
    // Boot-time orientation: initSensors() reads ONE accel sample and uses a
    // hardcoded UPRIGHT reference to decide whether to start in upright or
    // inverted mode. Returns a result struct; main.cpp acts on it to keep the
    // brain layer decoupled from nervous_system/spinal_cord.h.
    {
        SensorBootResult boot = initSensors();
        if (boot.inverted) spinalCord.setInverted(true);
    }
    diagnostics::begin(spinalCord);

    Serial.println("FaceHugger OS Online.");
}

void loop() {
    updateNetwork();
    tickImu();

    // Auto-flip gate. The edge-trigger latch lives on SpinalCord so the SIL
    // (bindings.cpp::tick) gets identical behavior without duplicating the
    // logic. The 150°/30° IMU hysteresis already prevents chatter near 90°.
    spinalCord.tickAutoInvert(imuIsInverted());

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
