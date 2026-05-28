#ifndef CONFIG_H
#define CONFIG_H

// I2C Addresses
#define ADDR_SERVO_DRIVER 0x40
#define ADDR_TOF_FRONT    0x29  // Default VL53L0X

// Pins
#define PIN_SDA 21
#define PIN_SCL 22

// servos work with pulses not angles
#define MIN_PULSE 150
#define MAX_PULSE 600

// define PCA addresses for servos as well as default angles (IDLE_STAND pose)

// Front right leg (Leg 0 in spinal_cord.cpp)
#define FRONT_RIGHT_LEG_HIP_PCA_CHANNEL 8
#define FRONT_RIGHT_LEG_THIGH_PCA_CHANNEL 9
#define FRONT_RIGHT_LEG_KNEE_PCA_CHANNEL 10

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

// Front left leg (Leg 1 in spinal_cord.cpp)
#define FRONT_LEFT_LEG_HIP_PCA_CHANNEL 12
#define FRONT_LEFT_LEG_THIGH_PCA_CHANNEL 13
#define FRONT_LEFT_LEG_KNEE_PCA_CHANNEL 14

#define FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE 90  // Change B: was 75; FL now stands at servo 90 like FR/BR/BL
#define FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE 27   // CALIB_FL_THIGH + NEUTRAL[FL].th = 87+(-60)
#define FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE 122   // CALIB_FL_KNEE  - NEUTRAL[FL].kn = 82-(-40)

// Bottom right leg (Leg 2 in spinal_cord.cpp)
#define BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL 4
#define BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL 5
#define BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL 6

#define BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE 90
#define BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE 53  // CALIB_BR_THIGH + NEUTRAL[BR].th = 103+(-50)
#define BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE 130  // CALIB_BR_KNEE  - NEUTRAL[BR].kn = 80-(-50)

// Bottom left leg (Leg 3 in spinal_cord.cpp)
#define BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL 0
#define BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL 1
#define BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL 2

#define BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE 90
#define BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE 144  // CALIB_BL_THIGH - NEUTRAL[BL].th = 84-(-60)
#define BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE 52    // CALIB_BL_KNEE  + NEUTRAL[BL].kn = 87+(-35)

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

#endif
