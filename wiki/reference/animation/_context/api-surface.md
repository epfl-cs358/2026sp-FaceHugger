<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/animation-pipeline/api-surface.md
Original kind:  design
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/animation/fhc-format.md
-->

> **Reference material.** Verbatim copy of `doc/animation-pipeline/api-surface.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/animation-pipeline/api-surface.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [todo]  Design-only, no implementation. No gait_engine.h / gait_engine_* / GaitFsmState / clips_all in code/firmware/ (grep finds nothing). This is the T7 contract for the unstarted .fhc on-board engine (CLAUDE.md: "implementation has not started").
- [drift] WebSocket command mapping table (T:1/T:2/T:5/T:10) is roughly consistent with code/API_SPEC.md, but the engine maps clip-name strings ("walk_forward", "trot"); the actually-implemented path is angle-space clip ids (playClip(id), CMD_PLAY_CLIP) per doc/animation-pipeline/onboard-clip-player-design.md, not this clip-name FreeRTOS-queue API.
- [drift] T:4 calibration: API_SPEC wire shape is {T:4, id:<leg 0-3>, servo_id:<0-2>, a}; firmware maps via LEG_SERVO_CHANNEL[id][servo_id] (config.h), not a flat servo_id 0..11.
- [todo]  Open contract questions §4 (IMU correction R/dh_mm semantics) and §3 (default_pole_sign) remain unresolved — no T7/T10 code exists to pin them.
- [ok]    Dual-core split (WiFi core 0 / motion core 1, FreeRTOS queue) matches the recommendation in firmware-research.md §1; consistent with that doc's architecture.
-->
# Runtime Gait Engine — API Surface

How the runtime gait engine (T7 in
[`animation-pipeline-roadmap.md`](animation-pipeline-roadmap.md))
exposes itself to the rest of the firmware, and how the existing
WebSocket protocol from
[`code/API_SPEC.md`](../../code/API_SPEC.md) maps onto it.

This document is the **contract** between the WebSocket task (core 0)
and the gait engine (core 1). Lock the signatures here before T7 and
T8 start, so they can be developed in parallel.

---

## TL;DR

- **One C struct + 6 functions** is the entire API surface.
- **Cross-core communication via FreeRTOS queue** — never call
  gait_engine functions directly from core 0; they enqueue commands
  the engine pops on its tick.
- **The engine owns the timing** — it ticks at 100 Hz on core 1; the
  WebSocket task only sends commands and reads telemetry.
- **The IMU correction layer plugs in via one function**
  (`gait_engine_set_pose_correction`); identity arguments mean
  pass-through.

---

## API

```c
// gait_engine.h

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------- //
// Lifecycle
// ---------------------------------------------------------------- //

// Called once at boot from main(). Loads every /clips/*.fhc into
// pre-allocated SRAM, starts the 100 Hz tick task pinned to core 1,
// transitions to IDLE state.
void gait_engine_init(void);

// ---------------------------------------------------------------- //
// Command interface (called from core 0, enqueues onto core 1)
// ---------------------------------------------------------------- //

typedef enum {
    FSM_IDLE      = 0,
    FSM_LOADING   = 1,
    FSM_PLAYING   = 2,
    FSM_BLENDING  = 3,
    FSM_FAILSAFE  = 4,
} GaitFsmState;

// Switch to a clip. Triggers a 100-200 ms joint-space blend from
// current pose to the new clip's start. clip_name is null-terminated
// e.g. "trot", "walk_forward", "stand", "wave_fl".
// Returns true if the clip is loaded and the transition was queued;
// false if clip_name is unknown.
bool gait_engine_load_clip(const char* clip_name);

// 0.5 = half speed, 1.0 = nominal, 2.0 = double. Multiplies the
// clip's period_ms at runtime — same clip, faster/slower playback.
// Persists across clip changes.
void gait_engine_set_speed_scale(float scale);

// Soft-stop the gait engine: lerps current pose to neutral over
// 500 ms, transitions to IDLE. Servos remain energized.
void gait_engine_idle(void);

// Hard-stop: transitions to FAILSAFE immediately, holds last pose.
// Used for T:2 → STATE_FAILSAFE and timeout-after-2s-no-T:1 logic
// from API_SPEC.md.
void gait_engine_emergency_stop(void);

// ---------------------------------------------------------------- //
// IMU correction hook (called from core 0, owned by IMU teammate)
// ---------------------------------------------------------------- //

typedef struct { float m[9]; } Mat3;   // row-major 3x3

// Provide the chassis-correction matrix and a height offset to apply
// before IK. The engine multiplies foot_clip by this matrix on every
// tick. Pass identity + 0 to disable correction (pass-through).
//
// Convention: R rotates body-frame foot positions to where they
// "should be" after the chassis tilts; dh_mm shifts the body up/down.
//
//   foot_corrected = R * foot_clip + (0, 0, dh_mm)
//
// Called at e.g. 50 Hz from the IMU task. Engine uses the most
// recent value on its tick; no per-tick handshake needed.
void gait_engine_set_pose_correction(const Mat3* R, float dh_mm);

// ---------------------------------------------------------------- //
// Telemetry (read from core 0 to fill T:10 status messages)
// ---------------------------------------------------------------- //

typedef struct {
    GaitFsmState state;
    char         current_clip[32];
    float        clip_progress_pct;   // 0.0-1.0; for T:10's "pc"
    uint16_t     ticks_per_sec;       // observed; for diagnostics
    uint8_t      iteration_count;     // # of loop completions
    bool         imu_correction_active;
    char         last_error[64];      // empty if no error
} GaitEngineStatus;

void gait_engine_get_status(GaitEngineStatus* out);
```

That's it. ~6 commands + 1 telemetry reader.

---

## Cross-core communication pattern

The functions above all run on **core 0**. They don't directly mutate
engine state — they push a command onto a FreeRTOS queue:

```c
// Internal — illustrative, not part of public API
typedef enum {
    CMD_LOAD_CLIP,
    CMD_SET_SPEED,
    CMD_IDLE,
    CMD_E_STOP,
    CMD_SET_POSE_CORRECTION,
} CmdType;

typedef struct {
    CmdType type;
    union {
        char  clip_name[32];
        float speed_scale;
        struct { Mat3 R; float dh_mm; } pose;
    } u;
} GaitCmd;

extern QueueHandle_t g_gait_cmd_queue;   // sized for ~16 commands
```

**Core 1's tick loop**:

```c
void gait_engine_task(void* arg) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);   // 100 Hz

    for (;;) {
        // 1. Drain the command queue (non-blocking).
        GaitCmd cmd;
        while (xQueueReceive(g_gait_cmd_queue, &cmd, 0) == pdTRUE) {
            apply_command(&cmd);
        }

        // 2. Tick the engine:
        //    a. Bezier eval  → foot_clip[4] in body frame
        //    b. Mirror       → negate Y for mirror_mask legs (SHARED layout only)
        //    c. IMU correct  → foot_corrected[i] = R * foot_clip[i] + (0,0,dh)
        //    d. IK           → JointAngles legs[4]  (yaw, hip, knee per leg)
        //    e. Servo write  → servo_write_all(legs) → PCA9685 I2C
        engine_tick(10.0f);

        // 3. Sleep until next period boundary.
        vTaskDelayUntil(&last, period);
    }
}
```

This is the standard pattern from `firmware-research.md` §1's
dual-core split. Core 0 (WiFi + AsyncWebServer) is free to run
blocking-ish code without disturbing core 1's deterministic loop.

---

## How `code/API_SPEC.md` WebSocket commands map to engine calls

| WebSocket command | Engine call | Notes |
|---|---|---|
| `T:1, d:0` (FW) | `gait_engine_load_clip("walk_forward")` | One clip per direction; FW/BW/FR/FL/BR/BL = 6 clips. |
| `T:1, d:6` (STOP) | `gait_engine_idle()` | Soft-stop, transitions to neutral pose. |
| `T:2, s:0` (IDLE) | `gait_engine_idle()` | Same as T:1 STOP. |
| `T:2, s:1` (WALK) | `gait_engine_load_clip(<current_gait>)` | Re-uses the gait set by T:5. |
| `T:2, s:2` (ACTION) | `gait_engine_load_clip("flip")` (or whatever the action clip is) | The clip's `clip_class = FLIP` triggers IMU-correction-off behavior. |
| `T:2, s:3` (FAILSAFE) | `gait_engine_emergency_stop()` | Immediate. |
| `T:3, h, p, r` (Body Pose) | Convert (p, r) → 3x3 matrix; `gait_engine_set_pose_correction(R, h)` | Body-pose command sets a desired *target* correction; the IMU layer can override. Spec needs to clarify priority — recommend body-pose command takes precedence for 2 s, then IMU resumes. |
| `T:4, id, servo_id, a` (Calib) | Direct PCA9685 write — bypasses engine | Calibration mode disables gait engine via `gait_engine_idle()` first. |
| `T:5, g:0` (TROT) | Sets active locomotion clip name to `"trot"` | T:1 then plays this clip on next direction command. |
| `T:5, g:1` (CRAB) | Active clip = `"crab"` | |
| `T:5, g:2` (CRAWL) | Active clip = `"crawl"` | |
| `T:10` (telemetry) | `gait_engine_get_status(&out)`; populate JSON | `pc` ← `clip_progress_pct`, `g` ← gait mode (read from active clip name), `e` ← `last_error`. |

The `walk_forward.fhc`, `walk_back.fhc`, etc. naming is a convention
T8 owns. The runtime engine doesn't care about the names — it just
loads whatever `gait_engine_load_clip("...")` is called with.

---

## What the engine does NOT expose

Deliberately, to keep the API small and hard to misuse:

- **No per-frame foot position injection** (e.g., "set FL foot to
  this XYZ right now"). If you want this, author a clip. The runtime
  is for *playback*, not realtime trajectory streaming.
- **No direct joint angle commands**. Use `T:4` for calibration (which
  bypasses the engine entirely).
- **No "pause" / "resume" / "scrub to time t"**. Could add later if
  needed; not needed for v1.
- **No clip composition** (run two clips simultaneously, e.g., walk
  + wave one leg). Theoretically possible (one clip per leg), but
  v2.
- **No direct foot-position injection per tick from core 0.** The
  correction layer (`gait_engine_set_pose_correction`) is the only
  runtime modification of foot targets. It is applied inside the
  tick, not externally per frame.
- **No per-leg speed scaling.** `gait_engine_set_speed_scale` applies
  to the entire clip. Per-leg tempo variation is a clip-authoring
  concern (use `phase_offset_ms` in a SHARED clip, or author a
  PER_LEG clip).

If you find yourself wanting to add one of these, ask whether
authoring a clip would solve the problem instead. Usually it does.

---

## Memory budget for the engine

Per [`firmware-research.md`](firmware-research.md) §2, you have ~200
KB free heap with WiFi up. The engine consumes:

| Item | Bytes |
|---|---|
| Loaded clip library (~12 clips × ~1 KB avg) | ~12 KB |
| Per-leg runtime state (4 legs × LegRuntime) | ~512 B |
| Cmd queue (16 entries × 60 B) | ~1 KB |
| FSM state | ~256 B |
| Last-pose buffer for blend lerp | ~128 B |
| Stack for engine task | 4 KB |
| **Total** | **~18 KB** |

Plus the IK working set (a few hundred bytes on the engine's stack).
**~9 % of available heap**, well within budget.

---

## Open contract questions

1. **`T:3` priority vs IMU correction.** If the dashboard sends
   `T:3` (body pose) and the IMU layer is also writing
   pose_correction, who wins? Recommend: dashboard command
   takes precedence for 2 s, then IMU resumes. Document explicitly.
2. **Clip transition during a `T:5` change.** If we're mid-trot and
   `T:5, g:1` arrives changing to crab, do we (a) finish the
   current trot cycle then transition, (b) blend immediately, (c)
   wait for next swing apex before blending? Recommend (c) for
   visual smoothness — but (b) is simpler firmware. Decide before
   T7 ships.
3. **Per-clip `default_pole_sign`.** Header field defined; engine
   reads it. But: should the runtime API expose a "force pole
   sign" override for debug? Probably not — calibration is enough.
4. **TODO — IMU correction convention: needs explicit agreement
   with IMU teammate.**

   The API signature is defined
   (`gait_engine_set_pose_correction(const Mat3* R, float dh_mm)`),
   but the exact semantics of `R` have not been formally agreed on
   between the gait engine owner (T7) and the IMU layer owner (T10).
   Before T10 integration begins, the two owners must explicitly
   agree on:

   - **Frame of `R`:** does it rotate body-frame foot positions, or
     world-frame positions? These are transposes of each other. The
     gait engine currently assumes body-frame:
     `foot_corrected = R · foot_clip + (0, 0, dh)`.
   - **What `R` represents physically:** a gravity-alignment
     correction (keep feet level despite chassis tilt) vs. a
     body-pose command (actively tilt the body).
   - **Sign convention of `dh_mm`:** positive = body moves up (feet
     lower in body frame) or positive = body moves down?

   This must be documented in a short agreed note before any T10
   code is written. Neither T7 nor T10 should assume the other's
   convention. The current API surface doc is intentionally silent
   on this until that conversation happens.

---

## Cross-references

- [`leg-coordinates.md`](leg-coordinates.md) — file format,
  IK math, what each engine call does internally.
- [`animation-pipeline-roadmap.md`](animation-pipeline-roadmap.md) —
  T7 task description, ownership.
- [`firmware-research.md`](firmware-research.md) — ESP32 dual-core
  pattern, FreeRTOS queue idioms.
- [`code/API_SPEC.md`](../../code/API_SPEC.md) — WebSocket
  protocol the engine plugs in behind.
