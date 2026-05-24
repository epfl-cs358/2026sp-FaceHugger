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

#define FRONT_RIGHT_LEG_HIP_DEFAULT_ANGLE 90
#define FRONT_RIGHT_LEG_THIGH_DEFAULT_ANGLE 150
#define FRONT_RIGHT_LEG_KNEE_DEFAULT_ANGLE 53

// Front left leg (Leg 1 in spinal_cord.cpp)
#define FRONT_LEFT_LEG_HIP_PCA_CHANNEL 12
#define FRONT_LEFT_LEG_THIGH_PCA_CHANNEL 13
#define FRONT_LEFT_LEG_KNEE_PCA_CHANNEL 14

#define FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE 90  // Change B: was 75; FL now stands at servo 90 like FR/BR/BL
#define FRONT_LEFT_LEG_THIGH_DEFAULT_ANGLE 30
#define FRONT_LEFT_LEG_KNEE_DEFAULT_ANGLE 130

// Bottom right leg (Leg 2 in spinal_cord.cpp)
#define BOTTOM_RIGHT_LEG_HIP_PCA_CHANNEL 4
#define BOTTOM_RIGHT_LEG_THIGH_PCA_CHANNEL 5
#define BOTTOM_RIGHT_LEG_KNEE_PCA_CHANNEL 6

#define BOTTOM_RIGHT_LEG_HIP_DEFAULT_ANGLE 90
#define BOTTOM_RIGHT_LEG_THIGH_DEFAULT_ANGLE 40
#define BOTTOM_RIGHT_LEG_KNEE_DEFAULT_ANGLE 140  // = translateToServo(NEUTRAL[BR].kn=-50); was 130, caused rear-knee twitch on gait stop

// Bottom left leg (Leg 3 in spinal_cord.cpp)
#define BOTTOM_LEFT_LEG_HIP_PCA_CHANNEL 0
#define BOTTOM_LEFT_LEG_THIGH_PCA_CHANNEL 1
#define BOTTOM_LEFT_LEG_KNEE_PCA_CHANNEL 2

#define BOTTOM_LEFT_LEG_HIP_DEFAULT_ANGLE 90
#define BOTTOM_LEFT_LEG_THIGH_DEFAULT_ANGLE 150
#define BOTTOM_LEFT_LEG_KNEE_DEFAULT_ANGLE 55  // = translateToServo(NEUTRAL[BL].kn=-35); was 50, caused rear-knee twitch on gait stop

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
