#include "motion_math.h"
#include <math.h>
#include "../shared/config.h"  // CALIB_*, SHOULDER_THETA_*, INTER_LEG_BUFFER_DEG
#include "neutral_pose.h"      // NEUTRAL[] math-space rest
#include "clips_all.h"

// Per-side asymmetric URDF-θ envelope. LEFT/RIGHT mirror each other through
// the body-X axis: the URDF tight (narrow) bound is on the same physical
// "inward" side for both, but its signed value flips with side.
//   LEFT  (FL, BL):  θ ∈ [-NARROW, +WIDE]   (URDF + sweeps outward)
//   RIGHT (FR, BR):  θ ∈ [-WIDE,   +NARROW] (mirror)
static inline void shoulder_hard_clamp_for_side(double& theta, bool is_left) {
    const double lo = is_left ? -(double)SHOULDER_THETA_NARROW_DEG
                              : -(double)SHOULDER_THETA_WIDE_DEG;
    const double hi = is_left ? +(double)SHOULDER_THETA_WIDE_DEG
                              : +(double)SHOULDER_THETA_NARROW_DEG;
    if (theta < lo) theta = lo;
    if (theta > hi) theta = hi;
}

// Signed CCW arc from a to b, normalised into (-180, +180]. Positive means b
// is CCW of a; magnitude is the shorter-arc distance.
static inline double signed_shorter_arc(double a, double b) {
    double d = fmod(b - a, 360.0);
    if (d <= -180.0) d += 360.0;
    if (d >   180.0) d -= 360.0;
    return d;
}

// Push `back` away from `front` along the shorter-arc direction so the gap is
// exactly INTER_LEG_BUFFER_DEG. Direction = sign of the current signed arc
// (preserves which side back currently sits on). Returns the adjusted back
// math angle, NOT yet hard-clamped.
static inline double widen_gap_to_buffer(double front_math, double back_math) {
    const double buf = (double)INTER_LEG_BUFFER_DEG;
    double signed_d = signed_shorter_arc(front_math, back_math);
    double sign = (signed_d >= 0.0) ? 1.0 : -1.0;
    return front_math + sign * buf;
}

void enforceShoulderLimits(double sh[4]) {
    // 1. Hard clamp each leg to its per-side URDF-θ envelope.
    double theta[4];
    for (int i = 0; i < 4; ++i) theta[i] = sh[i] - (double)NEUTRAL[i].sh;
    shoulder_hard_clamp_for_side(theta[0], /*is_left=*/false);  // FR
    shoulder_hard_clamp_for_side(theta[1], /*is_left=*/true);   // FL
    shoulder_hard_clamp_for_side(theta[2], /*is_left=*/false);  // BR (RR)
    shoulder_hard_clamp_for_side(theta[3], /*is_left=*/true);   // BL (RL)
    double m[4];
    for (int i = 0; i < 4; ++i) m[i] = (double)NEUTRAL[i].sh + theta[i];

    // 2. Same-side gap: front leads, back follows. If the math-space shorter-
    //    arc gap is below the buffer, push the back leg along the gap-widening
    //    direction. Re-apply the back leg's hard clamp in case the push
    //    crossed its envelope.
    const double buf = (double)INTER_LEG_BUFFER_DEG;

    // RIGHT: FR=0, BR=2
    if (fabs(signed_shorter_arc(m[0], m[2])) < buf) {
        double pushed = widen_gap_to_buffer(m[0], m[2]);
        double pushed_theta = pushed - (double)NEUTRAL[2].sh;
        shoulder_hard_clamp_for_side(pushed_theta, /*is_left=*/false);
        m[2] = (double)NEUTRAL[2].sh + pushed_theta;
    }
    // LEFT: FL=1, BL=3
    if (fabs(signed_shorter_arc(m[1], m[3])) < buf) {
        double pushed = widen_gap_to_buffer(m[1], m[3]);
        double pushed_theta = pushed - (double)NEUTRAL[3].sh;
        shoulder_hard_clamp_for_side(pushed_theta, /*is_left=*/true);
        m[3] = (double)NEUTRAL[3].sh + pushed_theta;
    }

    for (int i = 0; i < 4; ++i) sh[i] = m[i];
}

uint32_t clampPoseEaseMs(uint32_t dur_ms) {
    // 0 ("snap") is preserved; otherwise cap at POSE_EASE_MS_MAX. uint32_t so a
    // wire negative read as int becomes a huge unsigned and clamps to MAX, not 0.
    return dur_ms > (uint32_t)POSE_EASE_MS_MAX ? (uint32_t)POSE_EASE_MS_MAX : dur_ms;
}

double easeFraction(uint32_t elapsed_ms, uint32_t dur_ms) {
    if (dur_ms == 0 || elapsed_ms >= dur_ms) return 1.0;
    double t = (double)elapsed_ms / (double)dur_ms;
    return t * t * (3.0 - 2.0 * t);  // smoothstep
}

ServoTriple clampClipServos(uint8_t legId, ServoTriple s) {
    auto cl = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };
    // Hip: uniform across legs. The tight side of every URDF shoulder is 52°,
    // so 90 ± 52 is inside every leg's reach regardless of left/right.
    const double hip_lo = 90.0 - (double)HIP_CLAMP_FROM_NINETY;
    const double hip_hi = 90.0 + (double)HIP_CLAMP_FROM_NINETY;
    // Thigh + knee: CALIB-relative per leg. Bounds-guard legId so a junk caller
    // can't read past the [4] arrays; out-of-range legs fall back to the old
    // uniform 90-centered envelope.
    const double thigh_center = (legId < 4) ? (double)CALIB_THIGH_BY_LEG[legId] : 90.0;
    const double knee_center  = (legId < 4) ? (double)CALIB_KNEE_BY_LEG[legId]  : 90.0;
    s.hip   = cl(s.hip,   hip_lo, hip_hi);
    s.thigh = cl(s.thigh, thigh_center - (double)THIGH_CLAMP_FROM_CALIB,
                          thigh_center + (double)THIGH_CLAMP_FROM_CALIB);
    s.knee  = cl(s.knee,  knee_center  - (double)KNEE_CLAMP_FROM_CALIB,
                          knee_center  + (double)KNEE_CLAMP_FROM_CALIB);
    return s;
}

float emaStep(float prev, float target, float alpha) {
    return alpha * prev + (1.0f - alpha) * target;
}

ServoTriple applyInvert(uint8_t legId, ServoTriple s, bool inverted) {
    if (inverted) {
        // Mirror about CALIB (= flat in servo space) per joint, so mirror equals
        // math-space negation. Pre-calibration this was (180 - s) because flat was
        // assumed to be servo 90 uniformly. With per-joint CALIB the mirror axis
        // is 2*CALIB - s; reduces to 180 - s exactly when CALIB == 90.
        s.thigh = 2.0 * CALIB_THIGH_BY_LEG[legId] - s.thigh;
        s.knee  = 2.0 * CALIB_KNEE_BY_LEG[legId]  - s.knee;
    }
    return s;
}

ServoTriple translateToServo(uint8_t legId, double sh, double th, double kn) {
    ServoTriple out = {90.0, 90.0, 90.0};
    switch (legId) {
        case 0:  // LEG_FR
            out.hip   = 90.0 + (sh - 45.0);
            out.thigh = CALIB_FR_THIGH - th;
            out.knee  = CALIB_FR_KNEE  + kn;
            break;
        case 1:  // LEG_FL
            out.hip   = 90.0 + (sh - 135.0);  // Change B: regularized so servo 90 = outward (+135),
                                              // matching FR/BR/BL. Requires FL horn remount on hardware.
            out.thigh = CALIB_FL_THIGH + th;
            out.knee  = CALIB_FL_KNEE  - kn;
            break;
        case 2:  // LEG_RR / BR
            // BR shoulder un-mirrored (2026-05-25): identical motor, yaw shaft
            // on the same vertical axis as the others, so +sh = CCW = +servo for
            // every leg. tickYawRotation YAW_COEF[BR] and tickGait fwdDir[BR] are
            // flipped in tandem so gait servo output is unchanged. See
            // docs/.work/convention-docs/DRAFT-delta-conventions.md §3.
            out.hip   = 90.0 + (sh + 45.0);  // was 90.0 - (sh + 45.0)
            out.thigh = CALIB_BR_THIGH + th;
            out.knee  = CALIB_BR_KNEE  - kn;
            break;
        case 3:  // LEG_RL / BL
            out.hip   = 90.0 + (sh + 135.0);
            out.thigh = CALIB_BL_THIGH - th;
            out.knee  = CALIB_BL_KNEE  + kn;
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
