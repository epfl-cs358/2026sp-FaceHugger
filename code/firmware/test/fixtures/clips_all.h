#ifndef FH_CLIPS_TEST_FIXTURE_H
#define FH_CLIPS_TEST_FIXTURE_H

/* Test-only clip frame data for the pure clipPoseAt tests.
 *
 * Distinct include guard (FH_CLIPS_TEST_FIXTURE_H) from the production
 * clips_all.h (FH_CLIPS_ALL_H) so this fixture is never shadowed when a test
 * also pulls in the production header via motion_math.h. The FhClipFrame type
 * comes from that production header (included below); this fixture does NOT
 * redefine FhClipFrame/FhClip/FH_CLIPS, and its array symbols (fh_clip_a /
 * fh_clip_single) are distinct from any production clip — so test_clip_interp
 * keeps compiling regardless of what the production registry holds
 * (placeholder today, real bundle after INT1). */

#include "../../src/nervous_system/motion_math.h"

static const FhClipFrame fh_clip_a[] = {
    {   0, { 0,0,0, 0,0,0, 0,0,0, 0,0,0 }},
    { 100, { 10,20,30, 0,0,0, 0,0,0, 0,0,0 }},
    { 200, { 30,40,50, 0,0,0, 0,0,0, 0,0,0 }},
};
static const FhClipFrame fh_clip_single[] = {
    {   0, { 5,5,5, 5,5,5, 5,5,5, 5,5,5 }},
};

#endif /* FH_CLIPS_TEST_FIXTURE_H */
