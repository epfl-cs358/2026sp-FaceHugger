#ifndef FH_MOTION_MATH_H
#define FH_MOTION_MATH_H
#include <stdint.h>

/* Pure, Arduino-free motion math shared by the gait engine and the clip
 * player. Compiled in the `native` env so it is unit-tested on the host. */

typedef struct { double hip; double thigh; double knee; } ServoTriple;

/* Math-space (sh,th,kn) -> servo-space (0..180) for one leg. The exact
 * per-leg mounting transform lifted verbatim from tickGait's switch.
 * NOT a clamp and NOT IK. legId is the firmware LegId (0=FR,1=FL,2=RR,3=RL). */
ServoTriple translateToServo(uint8_t legId, double sh, double th, double kn);

/* Math-space shoulder safety net — applied before translateToServo on every
 * gait / clip / T:12 frame. Two layers (see config.h for the constants):
 *   1. Per-side asymmetric hard clamp in URDF θ (signed displacement from rest):
 *        LEFT  (FL, BL):  θ ∈ [-NARROW, +WIDE]
 *        RIGHT (FR, BR):  θ ∈ [-WIDE,   +NARROW]
 *   2. Back-leg inter-leg coupling — back stays ≥ INTER_LEG_BUFFER_DEG from
 *      the front leg on the same side:
 *        LEFT:  BL_θ ≥ FL_θ + INTER_LEG_BUFFER_DEG
 *        RIGHT: BR_θ ≤ FR_θ - INTER_LEG_BUFFER_DEG
 * Hard limits dominate the buffer at extremes (collapses to 0° gap when the
 * front leg is at its hard wide bound). Mutates sh[] in place; indices are
 * the firmware LegId (0=FR, 1=FL, 2=BR/RR, 3=BL/RL). Pure, host-tested. */
void enforceShoulderLimits(double sh[4]);

/* Per-leg, CALIB-aware servo clamp for the CLIP PLAYBACK path only (see
 * tickClip). The envelope is derived from the URDF joint limits and centered
 * on each joint's per-leg CALIB (post-calibration servo zero):
 *   hip   uniform 90 ± HIP_CLAMP_FROM_NINETY  (tight side of every URDF
 *         shoulder; safe on every leg regardless of asymmetry).
 *   thigh CALIB_THIGH_BY_LEG[legId] ± THIGH_CLAMP_FROM_CALIB  (URDF ±75°).
 *   knee  CALIB_KNEE_BY_LEG[legId]  ± KNEE_CLAMP_FROM_CALIB   (URDF ±90°).
 * Per-leg + CALIB-relative so post-CALIB NEUTRALs (e.g. FL thigh = 27 = CALIB_FL_THIGH - 60)
 * sit on the window edge instead of being clipped by a uniform [30, 150].
 * NOT applied to gaits or T:4 — gait output stays bit-for-bit. */
ServoTriple clampClipServos(uint8_t legId, ServoTriple s);

/* Clamp a wire-supplied T:2 `dur_ms` (pose-button ease window) into the safe
 * range [0, POSE_EASE_MS_MAX]. 0 is "snap" (handler keeps existing instant
 * path); any value above the ceiling is capped. Pulled out as a pure helper
 * so it is unit-tested on the host (network.cpp is not native-buildable). */
uint32_t clampPoseEaseMs(uint32_t dur_ms);

/* Smoothstep ease-in-out fraction for a timed servo move (Servo::tickEase).
 * smoothstep(t) = t*t*(3-2t): slow at both ends, fast through the middle, and
 * reaches exactly 1.0 at elapsed_ms == dur_ms (so the move lands on target).
 * dur_ms == 0 -> 1.0 (instantaneous). Pure, host-tested. */
double easeFraction(uint32_t elapsed_ms, uint32_t dur_ms);

/* One per-channel exponential-moving-average step for clip-playback smoothing:
 * smoothed = alpha*prev + (1-alpha)*target. alpha in [0,1) — higher is smoother
 * and laggier; alpha==0 is pass-through. Used ONLY inside tickClip(); never on
 * the gait or calibration path. Pure, host-tested. */
float emaStep(float prev, float target, float alpha);

/* Robot-invert mirror, applied once at the servo write point (Change D). When
 * inverted, mirror the pitch joints (thigh, knee) about each joint's CALIB
 * (per-leg zero-flat in servo space) — shoulder untouched. The mirror equals
 * math-space negation regardless of horn-mounting tolerance, so routing gait
 * output through it stays bit-for-bit identical to th=-th / kn=-kn. legId is
 * the firmware LegId (0=FR, 1=FL, 2=RR/BR, 3=RL/BL). Pure, host-tested. */
ServoTriple applyInvert(uint8_t legId, ServoTriple s, bool inverted);

/* FhClipFrame is defined in clips_all.h (generated header / test fixture),
 * which uses an anonymous-struct typedef and so cannot be tag-forward-
 * declared. Pull in its definition here. The guard FH_CLIPS_ALL_H is shared
 * across the generated header, the production placeholder, and the test
 * fixture, so this include is idempotent with whichever copy the test
 * includes. */
#include "clips_all.h"

/* Fill out[12] with the clip pose at `elapsed_ms`, linearly interpolating
 * between the two frames bracketing it. Before the first frame -> first
 * frame; at/after the last frame -> last frame (caller decides what to do
 * past duration). `cursor` is an in/out monotonic hint to avoid an O(n)
 * scan each tick (pass a value that only increases across a playback;
 * reset to 0 on (re)start). frames must have frame_count >= 1 and
 * strictly increasing t_ms. */
void clipPoseAt(const FhClipFrame* frames, uint16_t frame_count,
                uint32_t elapsed_ms, uint16_t* cursor, float out[12]);

/* Clip lifecycle state machine — pure, so all transitions + timing are
 * unit-tested on the host (the orchestration in tickClip() is a thin
 * dispatcher over this). One ClipState per SpinalCord. */
typedef enum {
    CLIP_PLAYING = 0,   /* interpolating clip frames */
    CLIP_RETURNING,     /* easing back to NEUTRAL stand */
    CLIP_DONE           /* finished; tickClip is a no-op */
} ClipPhase;

typedef enum {
    CLIP_ACT_NONE = 0,    /* do nothing this tick */
    CLIP_ACT_APPLY_POSE,  /* clipPoseAt(elapsed_ms) -> translateToServo -> setJointAngles */
    CLIP_ACT_BEGIN_RETURN,/* apply final pose at elapsed_ms, then start the 500ms ease */
    CLIP_ACT_EASE,        /* advance each servo's ease (tickEase) */
    CLIP_ACT_FINISH       /* ease done -> caller sets STATE_IDLE */
} ClipAction;

typedef struct {
    ClipPhase phase;
    uint32_t  clipStartMs;
    uint32_t  returnStartMs;
    uint16_t  cursor;        /* monotonic hint for clipPoseAt */
    uint8_t   clipId;
} ClipState;

typedef struct { ClipAction action; uint32_t elapsed_ms; } ClipStep;

/* Advance the lifecycle by one tick. Pure: `now` and the durations are
 * passed in (no millis()). Mutates *st (phase/returnStartMs). Returns the
 * action the caller must execute + the clip-relative elapsed_ms to sample.
 * duration_ms == clip.duration_ms; return_ms == the return-to-NEUTRAL ease
 * (500). On entry to RETURNING, returnStartMs is set to `now`. */
ClipStep clipPlayerStep(ClipState* st, uint32_t now,
                        uint32_t duration_ms, uint32_t return_ms);

#endif /* FH_MOTION_MATH_H */
