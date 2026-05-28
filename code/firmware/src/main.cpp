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
    initSensors();
    diagnostics::begin(spinalCord);

    Serial.println("FaceHugger OS Online.");
}

void loop() {
    updateNetwork();
    tickImu();

    // Auto-flip gate: only forward the IMU latch to SpinalCord when the FSM is
    // idle. This guarantees an ongoing gait or clip is never mid-motion-
    // interrupted by an accidental tilt — the robot has to be parked first.
    static bool s_prevInverted = false;
    bool        nowInverted    = imuIsInverted();
    if (nowInverted != s_prevInverted) {
        if (spinalCord.getRobotState() == STATE_IDLE) {
            spinalCord.setInverted(nowInverted);
            s_prevInverted = nowInverted;
        }
        // else: hold s_prevInverted so we re-evaluate next loop once idle.
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
