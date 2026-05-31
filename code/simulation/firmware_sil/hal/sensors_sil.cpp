// firmware_sil/hal/sensors_sil.cpp
//
// SIL replacement for code/firmware/src/brain/sensors.cpp. The production
// sensors.cpp talks to an MPU6050 over Wire; on the host there is no I2C, so
// we substitute this file in firmware_sil/CMakeLists.txt and source the IMU
// state from the fh_sim namespace instead.
//
// The bridge (sil_bridge.py) reads the PyBullet body orientation each tick,
// applies the same hysteresis as the firmware (flip>150°, clear<30°), and
// writes fh_sim::imu_upside_down / pitch_deg / roll_deg. The firmware's
// sensors.h API (declared in code/firmware/src/brain/sensors.h) is satisfied
// here so the rest of the firmware code (network.cpp, main.cpp would, if
// compiled) sees the IMU as ready and gets the same values it would on real
// hardware.
//
// initSensors() and tickImu() are no-ops in SIL: the bridge owns when state
// updates and the values are written directly via Python bindings; the firmware
// just reads them through the accessors.
#include "brain/sensors.h"
#include <Arduino.h>  // brings in fh_sim::imu_upside_down etc.

SensorBootResult initSensors() {
    // No hardware to bring up — leave the fh_sim::* defaults (false / 0) and
    // let the bridge start writing on the first tick. The bridge owns the IMU
    // latch path in SIL, so we report not-inverted and imu_ready=true.
    return {false, true};
}

void tickImu() {
    // The bridge writes fh_sim::imu_upside_down directly each tick; no
    // per-tick work needed on this side.
}

bool  imuIsInverted() { return fh_sim::imu_upside_down; }
float imuTiltDeg()    {
    // Approximation: when the bridge says "inverted", report 180° tilt;
    // otherwise 0°. The firmware code that calls tiltDeg() does so for
    // logging only (the latch is what gates motion), so the binary
    // approximation is sufficient for the SIL.
    return fh_sim::imu_upside_down ? 180.0f : 0.0f;
}
float imuPitchDeg()   { return fh_sim::pitch_deg; }
float imuRollDeg()    { return fh_sim::roll_deg; }
bool  imuReady()      { return true; }  // always ready in the sim

int getDistance() {
    // Stub: no ToF in the SIL. The production version returns a random sample;
    // returning 0 here keeps T:10 telemetry parseable without surfacing fake
    // distance jitter into tests.
    return 0;
}
