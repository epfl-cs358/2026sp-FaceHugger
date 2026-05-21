# On-board one-shot animation clip player + bundled export format

**Status:** design — awaiting user review before implementation planning
**Date:** 2026-05-19
**Author:** Marcus + Claude (brainstorming)

## 1. Goal & scope

Author arbitrary animations in Blender (e.g. "wiggle"), export them in a
single bundled header, and play any of them on the robot on demand. The
robot plays the clip once on its exported timeline, then **freezes and holds
the final pose**. Selection is by clip id over the existing WebSocket API,
modelled on how gaits are selected today.

**In scope**

- A bundled, pre-scaled, registry-style export header (`clips_all.h`).
- A firmware one-shot clip player on the existing empty `STATE_ACTION` slot.
- One API command to select/play a clip by id (mirrors `CMD_GAIT_MODE`).
- Phase 1 path that needs zero firmware (reuse existing `to_js`).

**Non-goals (YAGNI)**

- No looping clips, no blending between clips, no clip queue.
- No steering/direction coupling (clips are canned, unlike gaits).
- No auto-return to neutral after a clip (a separate existing command does
  that — e.g. set gait none / rest).
- No dynamic upload of clips at runtime; clips are compiled in (like
  `GAITS[]`).
- No change to the existing gait engine behaviour.

## 2. Context (as of 2026-05-19)

- **Firmware** `feat/hardware-integration-milestone-2` @ origin `3899ebc`
  (clean). Angle-space gait engine: `NEUTRAL[]` (raw math-space deg),
  `GAITS[]` presets pre-scaled to 2/3 of JS values, `tickGait()` on
  `STATE_WALK` applies an inline per-leg `translateToServo` switch →
  `setJointAngles()`. `STATE_ACTION` dispatch is empty. `CMD_POSE=3` is
  defined but unhandled.
- **Animation** `feat/animation-pipeline` — local tip `b7e85cb`, **24
  commits ahead of origin (unpushed)**. Two-layer exporter
  (`_sample_clip` → `to_csv`/`to_c_header`/`to_js`) wired via
  `FH_OT_export_clip` to `animation/exported_gaits/<clip>/`.
  `animation/convention.json` (`neutral_joint_deg`, `scale` 0.6667,
  `channels`) is the single source of truth and is byte-identical to the
  firmware `NEUTRAL[]`/channels in `3899ebc`.
- **PR #72** (`feat/animation-pipeline` → `main`) is open but **stale**
  (last update 2026-05-12); it predates all the export work. Not load-bearing
  for this design; see §11.

## 3. Locked decisions

1. **One-shot, hold-at-end.** `playClip(id)` runs the clip once on its
   `t_ms` timeline; past the last frame it holds the final pose (keeps
   writing the last frame's servo angles) and stays in `STATE_ACTION`. No
   loop, no auto-return.
2. **Pre-scaled math-space header.** The 2/3 scale-from-NEUTRAL is applied
   at **bake time** (exporter), exactly as `GAITS[]` presets are
   "pre-scaled to 2/3". The header carries math-space joint degrees. The
   firmware applies **only** `translateToServo` at runtime — identical
   convention to `tickGait`.
3. **Clip player is the gait engine's structural twin.** Same shapes:
   `CLIPS[]` registry ↔ `GAITS[]`; `playClip(id)` ↔ `setGait(g)`;
   `tickClip()` ↔ `tickGait()`; shared `translateToServo()` helper used by
   both.
4. **`convention.json` stays the single source of truth** for
   neutral/scale/channels/leg-order; the header is generated from it; docs
   updated to state the pre-scale rule explicitly.
5. **Single motion owner (mutual-exclusion invariant).** At most one of
   {a gait, a clip} drives the servos at any instant. This is *structural*,
   not a runtime check: the firmware has one `robotState`, and `update()`
   ticks exactly one handler per loop — `STATE_WALK→tickGait`,
   `STATE_ACTION→tickClip`. Transitions are explicit and pre-emptive:
   - `playClip(id)` → set `STATE_ACTION` (any running gait stops being
     ticked immediately; it is not "paused", it simply ceases).
   - any move/gait command (`CMD_MOVE`/`CMD_GAIT_MODE`) → set `STATE_WALK`
     (a running clip ceases).
   - `playClip` while already in `STATE_ACTION` → restart that clip
     (newest command wins; no blending, no queue).
   There is deliberately no concurrency to manage because two motion
   sources never coexist.

## 4. Architecture

Two branches + the app:

- **`feat/animation-pipeline`** (exporter): add a bundled converter
  `to_clips_header()` + an "export all clips" path. Existing `to_js`
  unchanged (Phase 1).
- **New branch off origin/`3899ebc`** (firmware): extract a shared
  `translateToServo()` helper from `tickGait`; add `CLIPS[]`, `tickClip()`,
  `playClip(id)`, one command case.
- **App / webpage**: a row of buttons. Phase 1 sends the `to_js`-generated
  stream; Phase 2 sends `playClip(id)`.

```
feat/animation-pipeline                      branch off origin 3899ebc (firmware)
───────────────────────                      ───────────────────────────────────
Blender rig (fh_clip_panel)
 └ _sample_clip  (raw joint deg/frame)
    └ NEW to_clips_header()  ── 1 file ──►  clips_all.h
       · scale-from-N (2/3) applied             FhClip FH_CLIPS[]= {name,frames,n,dur}
       · reordered to firmware LegId             FhClipFrame {t_ms, a[12]}
       · math-space deg                                 │
 └ to_js() ─ browser ─ WS:81 ─┐ (Phase 1)               ▼
                              │            ┌───────────────────────────────────┐
                              └─ servo ───►│ SpinalCord                         │
                                           │  CMD_PLAY_CLIP → playClip(id)      │
                                           │  STATE_ACTION  → tickClip()        │
                                           │     interp frame → translateToServo│◄─ shared
                                           │     → setJointAngles()             │   helper
                                           │  STATE_WALK → tickGait() unchanged │◄─ (refactor)
                                           └───────────────────────────────────┘
                                                           │ Leg→Servo→PCA9685
App buttons: Phase 1 = run to_js stream | Phase 2 = send playClip(id)
```

## 5. Export format — `clips_all.h` (the contract to hone)

One generated header, all clips bundled, indexed by id = registry index.

```c
#ifndef FH_CLIPS_ALL_H
#define FH_CLIPS_ALL_H
#include <stdint.h>

/* Auto-generated by FH Clip Panel — do not edit.
 * a[12] = PRE-SCALED math-space joint degrees (2/3 scale-from-NEUTRAL
 * applied at bake). Order is firmware LegId: FR,FL,RR,RL × (sh,th,kn).
 * Firmware applies ONLY translateToServo() at runtime (see tickGait). */

typedef struct { uint16_t t_ms; float a[12]; } FhClipFrame;
typedef struct {
    const char*        name;
    const FhClipFrame* frames;
    uint16_t           frame_count;
    uint16_t           duration_ms;   /* == frames[frame_count-1].t_ms */
} FhClip;

#define FH_CLIP_COUNT 2
static const FhClipFrame fh_clip_wiggle[] = {
    {   0, { /*FR sh,th,kn*/ ..., /*FL*/ ..., /*RR*/ ..., /*RL*/ ... }},
    {  33, { ... }},
    /* ... */
};
static const FhClipFrame fh_clip_bow[] = { /* ... */ };

static const FhClip FH_CLIPS[FH_CLIP_COUNT] = {
    { "wiggle", fh_clip_wiggle, 42, 1386 },
    { "bow",    fh_clip_bow,    18,  594 },
};
#endif
```

Format rules (all enforced by the exporter, documented in
`convention.json` neighbours):

- **Angle space:** math-space joint degrees, 2/3 scale-from-`NEUTRAL`
  already applied. Firmware never scales.
- **Leg/axis order:** `a[12]` is firmware `LegId` order (FR=0, FL=1,
  RR/BR=2, RL/BL=3), each leg `(shoulder, hip/thigh, knee)`. The exporter
  remaps from Blender `JOINT_BONES` order (`fl,fr,bl,br`) using
  `convention.json`. This `fl/fr/bl/br` ↔ `FR/FL/RR/RL` mapping is the
  known footgun (see CLAUDE.md leg-naming note) and is asserted in tests.
- **Timing:** `t_ms` from clip start, taken from the existing
  `_sample_clip` frame/fps math. `duration_ms` = last frame's `t_ms`.
- **Id:** index into `FH_CLIPS[]`. Stable within a build; the app learns
  names→ids from the same generated artifact (also emit a tiny
  `clips_manifest.json` for the app).
- Frame rate is whatever Blender authored (no resampling); firmware
  linearly interpolates between bracketing `t_ms` (see §6).

## 6. Firmware design (branch off origin `3899ebc`)

1. **Refactor (non-behavioural):** lift `tickGait`'s per-leg
   `translateToServo` switch into
   `ServoTriple translateToServo(uint8_t legId, float sh, float th, float kn)`.
   `tickGait` calls it; gait output is bit-identical (regression-tested).
2. **`CLIPS[]`** = `#include "clips_all.h"`; the registry is the generated
   `FH_CLIPS[]`.
3. **`playClip(uint8_t id)`**: validate `id < FH_CLIP_COUNT`; set
   `currentClip_`, `clipStartMs_ = millis()`, enter `STATE_ACTION`.
   Invalid id → ignored (logged), like the gait bound-check.
4. **`tickClip()`** on `STATE_ACTION`:
   - `elapsed = millis() - clipStartMs_`.
   - If `elapsed >= duration_ms` → **hold**: use the last frame's `a[12]`,
     `translateToServo` per leg, `setJointAngles`, return (stay in
     `STATE_ACTION`; do not advance, do not loop).
   - Else find the two frames bracketing `elapsed` by `t_ms`, linearly
     interpolate the 12 angles, `translateToServo` per leg,
     `setJointAngles`.
   - Cursor cached (monotonic) to avoid O(n) scan each tick.
5. **API:** add `CMD_PLAY_CLIP` (next free `T`, e.g. `T:6`) with `{ "c":
   <id> }`, mirroring `CMD_GAIT_MODE`'s validation shape; dispatch →
   `playClip(c)`. Update `code/API_SPEC.md`. (Alternative considered:
   reuse `CMD_POSE=3` — rejected: `POSE` has different documented
   semantics; a clean new command is clearer and lower-risk.)

## 7. App / webpage

Minimal: a button per clip, labels/ids from `clips_manifest.json`.
- **Phase 1:** button runs that clip's `to_js` stream over the existing
  socket (already functional today).
- **Phase 2:** button sends `{ "T":6, "c": <id> }`.

Existing `code/remote-control-app/MyApp` (socket + GaitControl screen) is
the natural home; a throwaway single HTML page is acceptable for first
validation.

## 8. Phasing

- **Phase 1 — now, zero firmware.** Use existing `to_js`; validate clips,
  convention, timing, and the button UX from the browser. No new code
  required to start.
- **Phase 2 — additive firmware.** Refactor + `to_clips_header()` +
  `tickClip()` + `CMD_PLAY_CLIP`. Robot becomes self-contained; same
  buttons switch to `playClip(id)`.

Phase 2 does not begin until Phase 1 has validated at least one real clip
on hardware.

## 9. Data flow

Blender pose → `_sample_clip` (raw math-space deg/frame) → `to_clips_header`
(scale-from-N 2/3, reorder to LegId, emit `clips_all.h` +
`clips_manifest.json`) → compiled into firmware → `playClip(id)` →
`tickClip()` (interpolate → `translateToServo` → `setJointAngles`) →
`Leg` → `Servo` → PCA9685.

## 10. Error handling & edge cases

- Unknown/out-of-range clip id → ignore + serial log (matches gait
  bound-check behaviour). Robot stays in current state.
- Empty clip (0 frames) → exporter refuses to emit it; firmware treats
  `frame_count==0` as no-op.
- Single-frame clip → immediately holds that pose.
- `elapsed` between frames → linear interpolation; before first frame →
  first frame; after last → hold last.
- `millis()` wrap (~49 days) → out of scope (robot uptime ≪ that);
  documented assumption.
- Command arriving mid-clip → newest command wins (re-`playClip` restarts;
  `setGait`/move pre-empts back to gait engine).
- Clip angle outside servo range → exporter clamps to [0,180] after
  `translateToServo` parity check; flagged at bake (reuse heatmap warning).

## 11. Branch / PR coordination

- Exporter changes land on `feat/animation-pipeline` (already 24 ahead;
  push + refresh **PR #72**, or open a focused PR — decide at plan time).
- Firmware changes land on a **new branch off origin/`3899ebc`** (not off
  our discarded local work), PR'd into `feat/hardware-integration-milestone-2`.
- The two are independent until integration; the only shared contract is
  `clips_all.h`'s format + `convention.json`. No cross-branch merge needed.
- Implementation therefore splits naturally into **two independent plans**:
  (A) exporter `to_clips_header` + manifest on `feat/animation-pipeline`;
  (B) firmware refactor + `tickClip` + `CMD_PLAY_CLIP` on the new firmware
  branch. (A) can ship and be validated via Phase 1 before (B) starts.

## 12. Convention & documentation updates

- Add the pre-scale rule to `animation/convention.json`'s doc neighbour
  and `code/simulation/docs/API_ANIMATION_SPEC.md`: *"clip headers are
  pre-scaled math-space; firmware applies only translateToServo."*
- Note the `fl/fr/bl/br` ↔ `FR/FL/RR/RL` reorder in the exporter and in
  `MERGE_AND_CONVENTION.md` cross-reference.
- `clips_all.h` carries the rule in its banner comment.

## 13. Testing strategy

- **Exporter:** unit test `to_clips_header` on a fixture clip — frame
  count, `t_ms` monotonic, `duration_ms`, leg reorder correctness, scale
  applied once. Determinism (byte-stable output).
- **Firmware (native Unity):** `translateToServo` helper parity vs the
  pre-refactor inline switch (gait regression: identical servo output for
  a sampled set). `tickClip` interpolation math (bracketing, lerp, hold,
  single-frame). Round-trip: a known clip frame through
  `to_clips_header`-equivalent values → `translateToServo` matches
  `_frame_to_servo` from `fh_clip_panel.py` within ≤1°.
- **On hardware:** Phase 1 `to_js` of one clip; then Phase 2 same clip via
  `playClip` — visually identical motion + holds final pose.

## 14. Open questions / risks

- **Command number:** `T:6` proposed; confirm no collision in
  `API_SPEC.md` evolution.
- **Header size:** dense per-frame × clips could grow flash; mitigation:
  author at modest fps, `float`→`int16` deciDegrees if needed (deferred —
  measure first).
- **PR #72 disposition:** refresh vs new focused PR — decided at plan
  time, not blocking design.
