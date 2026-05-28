#ifndef SENSORS_H
#define SENSORS_H

#include "../nervous_system/spinal_cord.h"

// MPU6050-based IMU + (stub) ToF accessor surface.
//
//   initSensors(sc)  starts Wire and the MPU6050 on the shared I2C bus, then
//                    takes ONE accel sample (after a 50 ms settle) and compares
//                    it to a hardcoded UPRIGHT reference. If the robot booted
//                    upside-down (dot < -0.7), sc.setInverted(true) is called
//                    so the very first servo writes use the inverted NEUTRAL.
//                    Replaces the old "average 50 samples = upright reference"
//                    scheme, which assumed the robot was upright at power-on.
//   tickImu()        reads the accel once per loop, normalises it, computes the
//                    tilt angle vs. the upright reference (deg) and the
//                    pitch/roll (deg). Stores them for the accessors below.
//   imuIsInverted()  latched upside-down flag with hysteresis (flip > 150,
//                    clear < 30). See imu_hysteresis.h.
//   imuTiltDeg()     last tilt angle vs. reference (0..180 deg).
//   imuPitchDeg()    signed pitch (atan2(ay, sqrt(ax^2 + az^2)) * 180/PI).
//   imuRollDeg()     signed roll  (atan2(-ax, az) * 180/PI).
//   imuReady()       false until initSensors() found an MPU; lets the rest of
//                    the firmware degrade gracefully (no MPU on the bench).
//
// The ToF stub stays for backwards compatibility with the existing telemetry
// payload — the real ToF wiring isn't part of this change.
void initSensors(SpinalCord& sc);
void tickImu();
bool imuIsInverted();
float imuTiltDeg();
float imuPitchDeg();
float imuRollDeg();
bool imuReady();
int getDistance();
#endif
