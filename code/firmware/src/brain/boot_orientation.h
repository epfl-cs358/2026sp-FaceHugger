#ifndef BOOT_ORIENTATION_H
#define BOOT_ORIENTATION_H

// Pure helpers for the boot-time IMU orientation decision (sensors.cpp
// initSensors()). Kept separate from sensors.{h,cpp} so they can be unit-
// tested on the host without dragging in Wire / MPU stubs.
//
// Replaces the old "average 50 accel samples => use as upright reference"
// scheme, which implicitly assumed the robot was upright at power-on. The
// new design uses a hardcoded reference vector (UPRIGHT_REF_*) and a single
// dot-product classification:
//
//   dot >  +BOOT_DOT_THRESHOLD ( 0.7) → upright   → caller leaves isInverted=false
//   dot <  -BOOT_DOT_THRESHOLD (-0.7) → inverted  → caller calls setInverted(true)
//   otherwise (robot on its side at boot, unusual) → assume upright (safe default)
//
// 0.7 ≈ cos(45°). Anything tilted more than 45° from upright is treated as
// "we don't really know" and we fall back to the upright assumption.
//
// UPRIGHT_REF is the unit gravity vector the MPU6050 should read when the
// chassis is upright. The chip is mounted Z-down in the chassis, so the
// -Z body axis points up when the robot is upright; gravity then pulls in
// the +Z direction in the IMU's frame.
constexpr float UPRIGHT_REF_X = 0.0f;
constexpr float UPRIGHT_REF_Y = 0.0f;
constexpr float UPRIGHT_REF_Z = 1.0f;

constexpr float BOOT_DOT_THRESHOLD = 0.7f;

enum BootOrientation {
    BOOT_UPRIGHT  = 0,
    BOOT_INVERTED = 1,
};

// Classify a precomputed dot product (unit_accel · UPRIGHT_REF).
inline BootOrientation classifyBootOrientation(float dot) {
    if (dot >  BOOT_DOT_THRESHOLD) return BOOT_UPRIGHT;
    if (dot < -BOOT_DOT_THRESHOLD) return BOOT_INVERTED;
    // Dead zone (robot on its side at boot — unusual). Default upright is the
    // safe fallback: the IMU latch will catch a real inversion on the first
    // tickImu() once the robot settles into a recognisable pose.
    return BOOT_UPRIGHT;
}

// Compute (normalized accel) · UPRIGHT_REF from a raw accel triple. Returns
// 0 if the accel magnitude is degenerate (caller should treat that as "no
// signal" and default to upright too).
inline float dotUpright(float ax, float ay, float az) {
    float n2 = ax * ax + ay * ay + az * az;
    if (n2 < 1e-6f) return 0.0f;
    // sqrt via Newton's avoided: just use the standard.
    float n = (float)__builtin_sqrtf(n2);
    float ux = ax / n;
    float uy = ay / n;
    float uz = az / n;
    return ux * UPRIGHT_REF_X + uy * UPRIGHT_REF_Y + uz * UPRIGHT_REF_Z;
}

#endif
