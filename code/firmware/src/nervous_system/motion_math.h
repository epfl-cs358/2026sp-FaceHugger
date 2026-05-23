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
