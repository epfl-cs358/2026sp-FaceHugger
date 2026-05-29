#include "sensors.h"
#include "imu_hysteresis.h"
#include "boot_orientation.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Single MPU6050 on the shared I2C bus (default address 0x68; PCA9685 sits at
// 0x40, OLED at 0x3C, VL53L0X at 0x29 — no clash). We never touch begin() on
// Wire if the network/PCA already started it; Adafruit_MPU6050::begin() handles
// the late-init case fine.
static Adafruit_MPU6050 s_mpu;
static bool   s_ready          = false;
static bool   s_inverted       = false;
// Hardcoded "world-up" reference in the IMU's body frame. See boot_orientation.h
// for the assumption (chip mounted Z-up; gravity pulls along -Z when upright).
static const float s_ref[3]    = {UPRIGHT_REF_X, UPRIGHT_REF_Y, UPRIGHT_REF_Z};
static float  s_tilt_deg       = 0.0f;
static float  s_pitch_deg      = 0.0f;
static float  s_roll_deg       = 0.0f;

static bool readAccelUnit(float out[3]) {
    sensors_event_t a, g, temp;
    if (!s_mpu.getEvent(&a, &g, &temp)) return false;
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float n  = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-3f) return false;
    out[0] = ax / n;
    out[1] = ay / n;
    out[2] = az / n;
    return true;
}

void initSensors(SpinalCord& sc) {
    Serial.println("Sensors: Initializing MPU6050 on shared I2C bus...");
    Wire.begin();
    if (!s_mpu.begin(0x68)) {
        Serial.println("[WARN] MPU6050 not found at 0x68 — IMU disabled.");
        s_ready = false;
        return;
    }
    // Defaults are fine for an upside-down latch: ±2g accel range, ±250 deg/s
    // gyro, 21 Hz bandwidth — we only need accel for the gravity vector.
    s_mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    s_mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    s_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Boot-orientation: read ONE accel sample (after a brief settle), classify
    // it against the hardcoded UPRIGHT_REF using the pure host-tested helper,
    // and arm setInverted(true) if the robot booted upside-down — before the
    // first servo writes leave the rest pose. No more averaging-50-samples-
    // as-the-reference; that scheme silently assumed the robot was upright.
    delay(50);
    float u[3];
    if (!readAccelUnit(u)) {
        Serial.println("[WARN] MPU6050 present but no sample — IMU disabled.");
        s_ready = false;
        return;
    }
    float dot = u[0]*s_ref[0] + u[1]*s_ref[1] + u[2]*s_ref[2];
    BootOrientation boot = classifyBootOrientation(dot);
    if (boot == BOOT_INVERTED) {
        Serial.printf("Sensors: boot orientation = INVERTED (dot=%.2f). Arming setInverted(true).\n", dot);
        sc.setInverted(true);
    } else {
        Serial.printf("Sensors: boot orientation = UPRIGHT (dot=%.2f).\n", dot);
    }
    s_ready  = true;
}

void tickImu() {
    if (!s_ready) return;
    float u[3];
    if (!readAccelUnit(u)) return;

    // Tilt vs. upright reference: angle between current gravity and stored ref.
    float dot = u[0]*s_ref[0] + u[1]*s_ref[1] + u[2]*s_ref[2];
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;
    s_tilt_deg = acosf(dot) * 180.0f / (float)M_PI;

    // Pitch / roll: standard accel-only formulas in deg. Sign convention:
    //   pitch = nose up positive
    //   roll  = right-side-down positive
    s_pitch_deg = atan2f(u[1], sqrtf(u[0]*u[0] + u[2]*u[2])) * 180.0f / (float)M_PI;
    s_roll_deg  = atan2f(-u[0], u[2]) * 180.0f / (float)M_PI;

    s_inverted  = imuInvertedHysteresis(s_tilt_deg, s_inverted);
}

bool  imuIsInverted() { return s_inverted; }
float imuTiltDeg()    { return s_tilt_deg; }
float imuPitchDeg()   { return s_pitch_deg; }
float imuRollDeg()    { return s_roll_deg; }
bool  imuReady()      { return s_ready; }

int getDistance() {
    // Mock data: returns a random distance between 50 and 500mm
    return random(50, 500);
}
