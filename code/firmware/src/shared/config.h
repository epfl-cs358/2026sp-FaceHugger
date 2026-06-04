#ifndef CONFIG_H
#define CONFIG_H

#ifdef FIRMWARE_SIM
#include "calib_sim.h"
#else
#include "calib.h"
#endif

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

// ---------------------------------------------------------------------------
// Clip-playback servo clamp — clampClipServos() in motion_math.cpp.
// Defensive symmetric envelope applied AFTER translateToServo() on the clip
// path only. The authoritative per-side asymmetric limits live below
// (SHOULDER_THETA_*) and are enforced by enforceShoulderLimits() before
// translateToServo() on every tick (gait + clip + T:12 stream).
//
//   hip   = [90 - HIP_CLAMP_FROM_NINETY, 90 + HIP_CLAMP_FROM_NINETY]
//           Uniform [38°, 142°]. The URDF shoulder is asymmetric per leg
//           (tight side 52°, wide side 90°); 52° is safe on every leg.
//   thigh = [CALIB_THIGH_BY_LEG[leg] ± THIGH_CLAMP_FROM_CALIB]
//           CALIB-relative, symmetric ±75° per the URDF on every leg.
//   knee  = [CALIB_KNEE_BY_LEG[leg]  ± KNEE_CLAMP_FROM_CALIB]
//           CALIB-relative, symmetric ±90° per the URDF on every leg.
#define HIP_CLAMP_FROM_NINETY  52
#define THIGH_CLAMP_FROM_CALIB 75
#define KNEE_CLAMP_FROM_CALIB  90

// ---------------------------------------------------------------------------
// Math-space shoulder limits — enforceShoulderLimits() in motion_math.cpp.
//
// Convention: math-space +θ = CCW yaw (looking from +Z down). Neutral
// shoulders point outward from the body: FR=+45°, FL=+135°, BR=-45°, BL=-135°.
// All four link1 URDF axes are (0,0,1) uniform +Z.
//
// The URDF shoulder window is asymmetric: the tight (inward) side stops at
// 52° from rest; the wide (outward) side goes to 90°. Left/right mirror
// each other through the body X-axis.
//
//   LEFT  (FL, BL):  θ ∈ [-NARROW, +WIDE]  →  displacement from rest: [-52°, +90°]
//   RIGHT (FR, BR):  θ ∈ [-WIDE,   +NARROW] →  displacement from rest: [-90°, +52°]
//
// Absolute math-space range (rest + displacement):
//   FL: 135° + [-52°, +90°] = [  83°,  225°]  →  wraps to [-97°, +45°] in [-180,180]
//   BL: -135° + [-52°, +90°] = [-187°,  -45°]  →  wraps to [+173°, -45°] in [-180,180]
//   FR:  45° + [-90°, +52°] = [ -45°,  +97°]
//   BR: -45° + [-90°, +52°] = [-135°,   +7°]
//
// Inter-leg buffer: back leg stays 5° away from the front leg on the same
// side (front-leads-back-follows). Hard limits dominate at extremes — when
// the front leg is at its hard max, the buffer collapses to 0°.
#define SHOULDER_THETA_NARROW_DEG 52
#define SHOULDER_THETA_WIDE_DEG   90
#define INTER_LEG_BUFFER_DEG       5

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
