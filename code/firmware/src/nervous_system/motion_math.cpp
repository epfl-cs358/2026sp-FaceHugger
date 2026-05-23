#include "motion_math.h"
#include "clips_all.h"

double easeFraction(uint32_t elapsed_ms, uint32_t dur_ms) {
    if (dur_ms == 0 || elapsed_ms >= dur_ms) return 1.0;
    double t = (double)elapsed_ms / (double)dur_ms;
    return t * t * (3.0 - 2.0 * t);  // smoothstep
}

ServoTriple translateToServo(uint8_t legId, double sh, double th, double kn) {
    ServoTriple out = {90.0, 90.0, 90.0};
    switch (legId) {
        case 0:  // LEG_FR
            out.hip   = 90.0 + (sh - 45.0);
            out.thigh = 90.0 - th;
            out.knee  = 90.0 + kn;
            break;
        case 1:  // LEG_FL
            out.hip   = 90.0 + (sh - 135.0);  // Change B: regularized so servo 90 = outward (+135),
                                              // matching FR/BR/BL. Requires FL horn remount on hardware.
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

ClipStep clipPlayerStep(ClipState* st, uint32_t now,
                        uint32_t duration_ms, uint32_t return_ms) {
    ClipStep r = { CLIP_ACT_NONE, 0 };
    switch (st->phase) {
        case CLIP_PLAYING: {
            uint32_t elapsed = now - st->clipStartMs;
            if (elapsed >= duration_ms) {
                // Apply the final pose, then transition into the ease.
                r.action = CLIP_ACT_BEGIN_RETURN;
                r.elapsed_ms = duration_ms;     // sample the last frame exactly
                st->phase = CLIP_RETURNING;
                st->returnStartMs = now;
            } else {
                r.action = CLIP_ACT_APPLY_POSE;
                r.elapsed_ms = elapsed;
            }
            break;
        }
        case CLIP_RETURNING:
            if (now - st->returnStartMs >= return_ms) {
                r.action = CLIP_ACT_FINISH;
                st->phase = CLIP_DONE;
            } else {
                r.action = CLIP_ACT_EASE;
            }
            break;
        case CLIP_DONE:
        default:
            r.action = CLIP_ACT_NONE;
            break;
    }
    return r;
}
