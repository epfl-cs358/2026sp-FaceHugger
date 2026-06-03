#ifndef CALIB_H
#define CALIB_H

// Default calibration — all values are 90 (servo centre = mechanical zero).
// On the real robot, replace with measured per-joint offsets.
// The SIL build uses calib_sim.h (selected via -DFIRMWARE_SIM) instead of
// this file; the two files share the same structure so tests can cross-check.

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

#endif
