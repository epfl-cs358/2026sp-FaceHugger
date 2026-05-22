#ifndef FH_CLIPS_ALL_H
#define FH_CLIPS_ALL_H
#include <stdint.h>
typedef struct { uint16_t t_ms; float a[12]; } FhClipFrame;
typedef struct {
    const char*        name;
    const FhClipFrame* frames;
    uint16_t           frame_count;
    uint16_t           duration_ms;
} FhClip;
#define FH_CLIP_COUNT 2
static const FhClipFrame fh_clip_a[] = {
    {   0, { 0,0,0, 0,0,0, 0,0,0, 0,0,0 }},
    { 100, { 10,20,30, 0,0,0, 0,0,0, 0,0,0 }},
    { 200, { 30,40,50, 0,0,0, 0,0,0, 0,0,0 }},
};
static const FhClipFrame fh_clip_single[] = {
    {   0, { 5,5,5, 5,5,5, 5,5,5, 5,5,5 }},
};
static const FhClip FH_CLIPS[FH_CLIP_COUNT] = {
    { "a",      fh_clip_a,      3, 200 },
    { "single", fh_clip_single, 1,   0 },
};
#endif /* FH_CLIPS_ALL_H */
