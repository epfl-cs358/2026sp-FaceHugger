#ifndef CONFIG_H
#define CONFIG_H

#include "calib.h"

// I2C Addresses
#define ADDR_SERVO_DRIVER 0x40
#define ADDR_TOF_FRONT    0x29  // Default VL53L0X

// Pins
#define PIN_SDA 21
#define PIN_SCL 22

// servos work with pulses not angles
#define MIN_PULSE 150
#define MAX_PULSE 600

// T:2 optional `dur_ms` ease window for the REST/STAND pose buttons. A wire
// value above this is clamped down so a malicious / buggy app can't park the
// robot in an arbitrarily long ease (during which the user can't take control
// back with another T:2). 5 s is well above the 1 s default the app sends.
#define POSE_EASE_MS_MAX 5000

// Clip-playback servo-clamp half-widths, derived from the URDF joint limits.
// clampClipServos (motion_math.cpp) uses these to build a per-leg envelope:
//   hip   = [90 - HIP_CLAMP_FROM_NINETY, 90 + HIP_CLAMP_FROM_NINETY]
//           Uniform across legs. The URDF shoulder window is asymmetric per leg
//           (one side 90°, the tight side 52°); 52 is safe on every leg.
//   thigh = [CALIB_THIGH_BY_LEG[leg] ± THIGH_CLAMP_FROM_CALIB]
//           URDF thigh range is symmetric ±60° on every leg.
//   knee  = [CALIB_KNEE_BY_LEG[leg]  ± KNEE_CLAMP_FROM_CALIB]
//           URDF knee range is symmetric ±90° on every leg.
// Per-leg + CALIB-relative so post-calibration NEUTRALs (e.g. FL thigh at 27,
// which is CALIB_FL_THIGH - 60) sit exactly on the lower edge instead of
// being clipped by a uniform [30, 150] window.
#define HIP_CLAMP_FROM_NINETY  52
#define THIGH_CLAMP_FROM_CALIB 60
#define KNEE_CLAMP_FROM_CALIB  90

// define PCA addresses for servos as well as default angles (IDLE_STAND pose)

// Front right leg (Leg 0 in spinal_cord.cpp)
#define FRONT_RIGHT_LEG_HIP_PCA_CHANNEL 8
#define FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL 9
#define FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL 10

// Front left leg (Leg 1 in spinal_cord.cpp)
#define FRONT_LEFT_LEG_HIP_PCA_CHANNEL 12
#define FRONT_LEFT_LEG_THIGH_PCA_CHANNEL 13
#define FRONT_LEFT_LEG_KNEE_PCA_CHANNEL 14

// Bottom right leg (Leg 2 in spinal_cord.cpp)
#define BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL 4
#define BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL 5
#define BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL 6

// Bottom left leg (Leg 3 in spinal_cord.cpp)
#define BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL 0
#define BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL 1
#define BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL 2

// Servo ID constants for use as indices
#define SERVO_HIP   0
#define SERVO_THIGH 1
#define SERVO_KNEE  2

// LEG_SERVO_CHANNEL[leg_id][servo_id] -> PCA channel
// leg_id:   0=Front Right, 1=Front Left, 2=Bottom Right, 3=Bottom Left
// servo_id: 0=Hip, 1=Thigh, 2=Knee
#ifdef __cplusplus
#include <stdint.h>
constexpr uint8_t LEG_SERVO_CHANNEL[4][3] = {
    {FRONT_RIGHT_LEG_HIP_PCA_CHANNEL,  FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL,  FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL},
    {FRONT_LEFT_LEG_HIP_PCA_CHANNEL,   FRONT_LEFT_LEG_THIGH_PCA_CHANNEL,   FRONT_LEFT_LEG_KNEE_PCA_CHANNEL},
    {BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL, BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL, BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL},
    {BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL,  BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL,  BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL},
};

// Single source of truth for indexing LEG_SERVO_CHANNEL from untrusted input
// (e.g. T:4 / CMD_CALIBRATE packets). Bounds match the [4][3] dimensions above.
constexpr bool isValidServoIndex(int leg_id, int servo_id) {
    return leg_id >= 0 && leg_id < 4 && servo_id >= 0 && servo_id < 3;
}
#endif

// Minimum servo angle change (degrees) to trigger a PWM update.
// Filters floating-point noise without perceptible motion loss.
// Mechanical joint slop typically exceeds 1°, so 0.5° is safe.
static constexpr float SERVO_DEADBAND_DEG = 0.5f;

#endif
