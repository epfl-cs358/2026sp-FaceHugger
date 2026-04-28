#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <stdint.h>

// Numeric values are the `g` wire contract from CMD_MOVE (API_SPEC v1.1).
enum GaitType : uint8_t {
    GAIT_NONE = 0,
    GAIT_WALK = 1,
    GAIT_TROT = 2,
    GAIT_CRAB = 3,
};

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
    float       offsets[LEG_COUNT];
    char        axis;
    const char* label;
};

#endif
