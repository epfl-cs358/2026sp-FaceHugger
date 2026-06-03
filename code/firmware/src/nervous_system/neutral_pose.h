#ifndef NEUTRAL_POSE_H
#define NEUTRAL_POSE_H

#include "movements.h"  // LEG_COUNT, LegId order (FR,FL,RR,RL)

// Math-space neutral (standing) pose per leg: [shoulder, thigh, knee] in degrees,
// indexed by LegId. Single source of truth — the gait engine eases to this pose,
// and config.h's per-servo *_DEFAULT_ANGLE (the boot / idle / graceful-stop / clip-
// return pose written by returnToDefaultAngles) MUST equal translateToServo(NEUTRAL[leg])
// for every joint, or the robot snaps between the two whenever a gait stops.
// Guarded by test_neutral_consistency.
struct NeutralPose { float sh, th, kn; };
constexpr NeutralPose NEUTRAL[LEG_COUNT] = {
    {  45.0f, -60.0f, -37.0f },  // LEG_FR (0)
    { 135.0f, -60.0f, -40.0f },  // LEG_FL (1)
    { -45.0f, -50.0f, -50.0f },  // LEG_RR / BR (2)
    {-135.0f, -60.0f, -35.0f },  // LEG_RL / BL (3)
};

#endif  // NEUTRAL_POSE_H