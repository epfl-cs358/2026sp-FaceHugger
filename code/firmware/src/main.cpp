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
    // inverted mode. Needs SpinalCord so it can call setInverted(true) before
    // the first servo writes if the robot was powered on upside-down.
    initSensors(spinalCord);
    diagnostics::begin(spinalCord);

    Serial.println("FaceHugger OS Online.");
}

void loop() {
    updateNetwork();
    tickImu();

    // Auto-flip gate: forward the IMU latch to SpinalCord whenever auto-invert
    // is enabled. The STATE_IDLE gate was dropped — applyServos / applyInvert
    // is safe to call mid-motion (that's the entire point of the live-flip
    // pose mirror), and the 150°/30° hysteresis on the IMU side already
    // prevents chatter near 90°. Disabling auto-invert via T:6 freezes
    // isInverted at its current value until re-enabled or a T:9 overrides it.
    static bool s_prevInverted = false;
    bool        nowInverted    = imuIsInverted();
    if (nowInverted != s_prevInverted) {
        if (spinalCord.isAutoInvertEnabled()) {
            spinalCord.setInverted(nowInverted);
            s_prevInverted = nowInverted;
        }
        // else: hold s_prevInverted so we re-evaluate next loop once re-enabled.
    }

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
