#ifndef FH_CLIPS_ALL_H
#define FH_CLIPS_ALL_H
#include <stdint.h>

/* PLACEHOLDER clip registry — replaced by the Blender-generated bundle at
 * integration. Generate with:
 *   blender --background --factory-startup animation/fh_rigged_latest.blend \
 *     --python animation/scripts/export_all_clips.py
 * then copy animation/exported_clips/clips_all.h over this file.
 *
 * a[12] = PRE-SCALED math-space joint degrees (firmware applies only
 * translateToServo at runtime), firmware LegId order FR,FL,RR,RL ×
 * (shoulder,thigh,knee). One trivial clip so FH_CLIPS[]/playClip compile;
 * its symbol name is intentionally distinct from the test fixture's. */

typedef struct { uint16_t t_ms; float a[12]; } FhClipFrame;
typedef struct {
    const char*        name;
    const FhClipFrame* frames;
    uint16_t           frame_count;
    uint16_t           duration_ms;
} FhClip;

#define FH_CLIP_COUNT 1
static const FhClipFrame fh_clip_placeholder[] = {
    {   0, { 45.0f,-60.0f,-37.0f, 75.0f,-60.0f,-40.0f, -45.0f,-50.0f,-50.0f, -135.0f,-60.0f,-35.0f }},
};
static const FhClip FH_CLIPS[FH_CLIP_COUNT] = {
    { "placeholder", fh_clip_placeholder, 1, 0 },
};
#endif /* FH_CLIPS_ALL_H */
