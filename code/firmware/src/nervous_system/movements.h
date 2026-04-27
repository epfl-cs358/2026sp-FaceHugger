#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>

// Gait identifiers, sent from the dashboard as the `g` field of CMD_MOVE
// (see code/API_SPEC.md v1.1). Numeric values are part of the wire contract.
enum GaitType : uint8_t {
    GAIT_NONE = 0,
    GAIT_WALK = 1,
    GAIT_TROT = 2,
    GAIT_CRAB = 3,
};

// Leg index into GaitParams::offsets and SpinalCord's leg array.
// Order matches SpinalCord: leg1=FR, leg2=FL, leg3=RR (BR), leg4=RL (BL).
enum LegId : uint8_t {
    LEG_FR    = 0,
    LEG_FL    = 1,
    LEG_RR    = 2,
    LEG_RL    = 3,
    LEG_COUNT = 4,
};

struct GaitParams {
    float       period_s;
    float       step_length_m;
    float       step_height_m;
    float       duty;
    float       offsets[LEG_COUNT];   // indexed by LegId
    char        axis;                 // 'y' forward, 'x' sideways
    const char* label;
};

#endif
