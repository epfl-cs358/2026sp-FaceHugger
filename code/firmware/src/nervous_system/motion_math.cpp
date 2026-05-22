#include "motion_math.h"

ServoTriple translateToServo(uint8_t legId, double sh, double th, double kn) {
    ServoTriple out = {90.0, 90.0, 90.0};
    switch (legId) {
        case 0:  // LEG_FR
            out.hip   = 90.0 + (sh - 45.0);
            out.thigh = 90.0 - th;
            out.knee  = 90.0 + kn;
            break;
        case 1:  // LEG_FL
            out.hip   = sh;
            out.thigh = 90.0 + th;
            out.knee  = 90.0 - kn;
            break;
        case 2:  // LEG_RR / BR
            out.hip   = 90.0 - (sh + 45.0);
            out.thigh = 90.0 + th;
            out.knee  = 90.0 - kn;
            break;
        case 3:  // LEG_RL / BL
            out.hip   = 90.0 + (sh + 135.0);
            out.thigh = 90.0 - th;
            out.knee  = 90.0 + kn;
            break;
        default:
            break;
    }
    return out;
}
