#ifndef CALIB_H
#define CALIB_H

// Per-joint calibration — servo PWM angle (degrees) that corresponds to the
// math-space zero position (thigh/knee flat). Measured on hardware 2026-05-28.
// Hip servos are not calibrated (always 90).
// The SIL build uses calib_sim.h (selected via -DFIRMWARE_SIM) instead of
// this file; the two files share the same structure so tests can cross-check.

#define CALIB_FR_THIGH  84
#define CALIB_FR_KNEE   95
#define CALIB_FL_THIGH  87
#define CALIB_FL_KNEE   82
#define CALIB_BR_THIGH 103
#define CALIB_BR_KNEE   80
#define CALIB_BL_THIGH  84
#define CALIB_BL_KNEE   87

#ifdef __cplusplus
constexpr int CALIB_THIGH_BY_LEG[4] = { 84, 87, 103, 84 };
constexpr int CALIB_KNEE_BY_LEG[4]  = { 95, 82,  80, 87 };
#endif

// DEFAULT_ANGLE = translateToServo(NEUTRAL[leg]) with the CALIB values above.
// Recompute if NEUTRAL[] in neutral_pose.h changes or CALIB values change.
// Formula per leg (mirrors calib_sim.h, direction signs from translateToServo):
//   FR: thigh = CALIB - th,  knee = CALIB + kn
//   FL: thigh = CALIB + th,  knee = CALIB - kn
//   BR: thigh = CALIB + th,  knee = CALIB - kn
//   BL: thigh = CALIB - th,  knee = CALIB + kn
#define FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE    90
#define FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE 144  // 84 - NEUTRAL[FR].th = 84-(-60)
#define FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE   58  // 95 + NEUTRAL[FR].kn = 95+(-37)

#define FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE     90
#define FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE   27  // 87 + NEUTRAL[FL].th = 87+(-60)
#define FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE   122  // 82 - NEUTRAL[FL].kn = 82-(-40)

#define BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE   90
#define BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE 53  // 103 + NEUTRAL[BR].th = 103+(-50)
#define BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE 130  // 80 - NEUTRAL[BR].kn = 80-(-50)

#define BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE    90
#define BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE 144  // 84 - NEUTRAL[BL].th = 84-(-60)
#define BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE   52  // 87 + NEUTRAL[BL].kn = 87+(-35)

#endif
