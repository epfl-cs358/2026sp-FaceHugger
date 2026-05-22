#include "motion_math.h"
#include "clips_all.h"

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

void clipPoseAt(const FhClipFrame* frames, uint16_t frame_count,
                uint32_t elapsed_ms, uint16_t* cursor, float out[12]) {
    if (frame_count == 0) {
        for (int j = 0; j < 12; ++j) out[j] = 90.0f;  // defensive
        return;
    }
    // Before first frame -> first frame.
    if (elapsed_ms <= frames[0].t_ms || frame_count == 1) {
        for (int j = 0; j < 12; ++j) out[j] = frames[0].a[j];
        return;
    }
    // At/after last frame -> last frame.
    const uint16_t last = frame_count - 1;
    if (elapsed_ms >= frames[last].t_ms) {
        for (int j = 0; j < 12; ++j) out[j] = frames[last].a[j];
        return;
    }
    // Advance the monotonic cursor to the frame whose t_ms is the lower
    // bracket. cursor never exceeds last-1.
    uint16_t i = (cursor && *cursor < last) ? *cursor : 0;
    while (i + 1 < frame_count && frames[i + 1].t_ms <= elapsed_ms) ++i;
    if (cursor) *cursor = i;
    const FhClipFrame& lo = frames[i];
    const FhClipFrame& hi = frames[i + 1];
    const uint16_t span = hi.t_ms - lo.t_ms;
    const float f = span == 0 ? 0.0f
                              : (float)(elapsed_ms - lo.t_ms) / (float)span;
    for (int j = 0; j < 12; ++j) out[j] = lo.a[j] + (hi.a[j] - lo.a[j]) * f;
}
