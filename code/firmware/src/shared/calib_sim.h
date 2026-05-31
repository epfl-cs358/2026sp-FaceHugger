#ifndef CALIB_SIM_H
#define CALIB_SIM_H

// Simulation-only calibration file — all CALIB values are 90 (servo centre =
// mechanical zero) so the sim robot's joint positions match the math-space
// angles exactly. The real robot uses calib.h (non-trivial offsets measured on
// hardware 2026-05-28). To test the effect of specific miscalibration in
// simulation without touching the real robot config, edit this file.
//
// Selected via -DFIRMWARE_SIM in firmware_sil/CMakeLists.txt; never compiled
// into the real ESP32 firmware.

#define CALIB_FR_THIGH  90
#define CALIB_FR_KNEE   90
#define CALIB_FL_THIGH  90
#define CALIB_FL_KNEE   90
#define CALIB_BR_THIGH  90
#define CALIB_BR_KNEE   90
#define CALIB_BL_THIGH  90
#define CALIB_BL_KNEE   90

#ifdef __cplusplus
constexpr int CALIB_THIGH_BY_LEG[4] = { 90, 90, 90, 90 };
constexpr int CALIB_KNEE_BY_LEG[4]  = { 90, 90, 90, 90 };
#endif

// DEFAULT_ANGLE = translateToServo(NEUTRAL[leg]) with CALIB=90 (no offset).
// Recompute if NEUTRAL[] in neutral_pose.h changes.
// Formula per leg (mirrors calib.h, direction signs from translateToServo):
//   FR: thigh = CALIB - th,  knee = CALIB + kn
//   FL: thigh = CALIB + th,  knee = CALIB - kn
//   BR: thigh = CALIB + th,  knee = CALIB - kn
//   BL: thigh = CALIB - th,  knee = CALIB + kn
#define FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE    90
#define FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE 150  // 90 - NEUTRAL[FR].th = 90-(-60)
#define FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE   53  // 90 + NEUTRAL[FR].kn = 90+(-37)

#define FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE     90
#define FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE   30  // 90 + NEUTRAL[FL].th = 90+(-60)
#define FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE   130  // 90 - NEUTRAL[FL].kn = 90-(-40)

#define BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE   90
#define BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE 40  // 90 + NEUTRAL[BR].th = 90+(-50)
#define BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE 140  // 90 - NEUTRAL[BR].kn = 90-(-50)

#define BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE    90
#define BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE 150  // 90 - NEUTRAL[BL].th = 90-(-60)
#define BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE   55  // 90 + NEUTRAL[BL].kn = 90+(-35)

#endif
