#ifndef CALIB_H
#define CALIB_H

// Last calibrated: 2026-05-28
// Method: mechanical flat (robot on flat surface, all legs horizontal),
//         read servo angle where each joint sits level (spirit level).
// Set all values to 90 to revert to uncalibrated behavior.

// Per-servo zero-point calibration: servo angle when the joint is at mechanical
// zero (thigh horizontal, knee horizontal — measured 2026-05-28 on real hardware
// by setting each joint to visually flat with the app's individual servo control).
// translateToServo uses these instead of the default 90. Set all to 90 to revert
// to uncalibrated behaviour. DEFAULT_ANGLE values below are derived from these.
#define CALIB_FR_THIGH  84
#define CALIB_FR_KNEE   95
#define CALIB_FL_THIGH  87
#define CALIB_FL_KNEE   82
#define CALIB_BR_THIGH 103
#define CALIB_BR_KNEE   80
#define CALIB_BL_THIGH  84
#define CALIB_BL_KNEE   87

// Indexed-by-LegId views for runtime lookup (e.g. applyInvert mirrors about CALIB).
// Order matches LegId enum: 0=FR, 1=FL, 2=BR/RR, 3=BL/RL.
#ifdef __cplusplus
constexpr int CALIB_THIGH_BY_LEG[4] = { CALIB_FR_THIGH, CALIB_FL_THIGH, CALIB_BR_THIGH, CALIB_BL_THIGH };
constexpr int CALIB_KNEE_BY_LEG[4]  = { CALIB_FR_KNEE,  CALIB_FL_KNEE,  CALIB_BR_KNEE,  CALIB_BL_KNEE };
#endif

// DEFAULT_ANGLE = translateToServo(NEUTRAL[leg]) with the calibration above applied.
// Recompute whenever CALIB_* or NEUTRAL[] changes (guarded by test_neutral_consistency).
#define FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE 90
#define FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE 144  // CALIB_FR_THIGH - NEUTRAL[FR].th = 84-(-60)
#define FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE 58    // CALIB_FR_KNEE  + NEUTRAL[FR].kn = 95+(-37)

#define FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE 90  // Change B: was 75; FL now stands at servo 90 like FR/BR/BL
#define FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE 27   // CALIB_FL_THIGH + NEUTRAL[FL].th = 87+(-60)
#define FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE 122   // CALIB_FL_KNEE  - NEUTRAL[FL].kn = 82-(-40)

#define BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE 90
#define BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE 53  // CALIB_BR_THIGH + NEUTRAL[BR].th = 103+(-50)
#define BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE 130  // CALIB_BR_KNEE  - NEUTRAL[BR].kn = 80-(-50)

#define BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE 90
#define BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE 144  // CALIB_BL_THIGH - NEUTRAL[BL].th = 84-(-60)
#define BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE 52    // CALIB_BL_KNEE  + NEUTRAL[BL].kn = 87+(-35)

#endif
