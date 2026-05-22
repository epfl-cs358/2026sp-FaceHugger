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

#endif /* FH_MOTION_MATH_H */
