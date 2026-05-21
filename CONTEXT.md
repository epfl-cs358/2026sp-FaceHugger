# FaceHugger — Animation & Motion Context

Shared language for the Blender→firmware animation pipeline and the on-board motion
engine. Captures terms that are easy to confuse across the CAD/sim/firmware/Blender
domains. Code is the source of truth; this file names the concepts.

## Language

### Motion sources

**Gait**:
A *looping* locomotion pattern (trot/crab/crawl) driven by `tickGait()` on `STATE_WALK`,
parameterised and direction-coupled. Selected by `T:5`.
_Avoid_: "animation" for gaits.

**Clip**:
A *canned, one-shot* authored gesture (e.g. "wiggle", "bow") played by `tickClip()` on
`STATE_ACTION`. Runs once on its own timeline; **no loop**. Selected by `CMD_PLAY_CLIP` (`T:7`,
payload `{"c":<id>}`).
_Avoid_: "action" (overloaded — see below), "gait".

**Clip end behaviour** *(resolved 2026-05-21)*:
**Return-to-stand.** At the clip's last frame the player linear-interpolates the final clip pose
→ `NEUTRAL` over 500 ms (via `setServoAngleTimed`), then transitions to `STATE_IDLE`. This
supersedes the earlier "hold-at-end / freeze and wait" proposal. No loop, no manual
auto-return command needed — it is the clip player's end state.

**setServoAngleTimed(angle, ms)**:
Non-blocking servo move that eases to `angle` over `ms` (default ~200 ms), sharing the same
0–180 clamp and pulse `map()` as `setServoAngle()`. Used by the clip player and return-to-stand;
the gait engine keeps using the instantaneous `setServoAngle()`.

### Angle spaces (the core footgun)

**Math-space angle**:
Abstract joint angle centred on `NEUTRAL[]`, where left/right legs are *symmetric*. Gaits
and clips both compute/store motion in math-space.

**Servo-space angle**:
Physical 0–180° angle written to a servo via the PCA9685. Per-leg signs/offsets differ
because of mirrored mounting.

**translateToServo**:
The per-leg transform **math-space → servo-space** (the `switch(legId)` in `tickGait`). It is
*not* a clamp and *not* IK — purely the mounting-convention remap. To be extracted into one
shared helper so `tickGait` and `tickClip` use the identical transform.
_Avoid_: conflating with the `constrain(0,180)` electrical clamp (a separate, lower layer).

**SCALE**:
The 2/3 (`0.6667`) shrink-from-`NEUTRAL` applied to gait/clip amplitudes. For clips it is
applied **at bake time** in the exporter; firmware never re-scales clip data.

### Safety layers

**Angle clamp**:
`constrain(angle, 0, 180)` inside `Servo::setServoAngle()` — the electrical backstop on every
path (IK, pose, calibrate, clip). Logs `[WARN] servo <ch> clamped: <raw> -> <clamped>`.

**Frame-delta warning**:
Authoring-time guard in the exporter: if any joint moves more than `FRAME_DELTA_WARN_DEG`
(~20°) between consecutive frames, warn in the Blender console. Catches accidental fast
keyframes (torque/shock risk) *before* they reach the robot. The clamp and this warning are
the two safety layers; there is deliberately **no firmware rate limiter** (YAGNI until
hardware data exists).

### Leg naming

**`fl/fr/bl/br`** (Blender/URDF/Python) ↔ **`FR/FL/RR/RL`** = `LegId` (firmware). The exporter
remaps order when emitting the clip header. `rr`/`rl` = rear = `br`/`bl`. Never propagate
firmware leg names into Blender/Python code.

## Relationships

- A **Clip** is authored in Blender, baked (with **SCALE** applied) into the bundled clip
  header, and played one-shot by `tickClip()`; a **Gait** is a built-in looping pattern.
- At most one motion source — one **Gait** *or* one **Clip** — drives the servos at any
  instant (single `robotState`; one tick handler per loop).
- Both **Gait** and **Clip** output **math-space** angles → **translateToServo** →
  **servo-space** → **angle clamp** → PCA9685.

## Flagged ambiguities

- "Action" is overloaded: the FSM `STATE_ACTION`, the `CMD_ACTION_SELECTION` (`T:6`,
  invert-robot), and informal use for a **Clip**. The clip player *runs on* `STATE_ACTION`
  but is a distinct command (`T:7`). Prefer "**Clip**" for authored gestures.
- "Animation" is informal; resolve to **Gait** (looping) or **Clip** (one-shot) per context.
- `T:6` is `CMD_ACTION_SELECTION` in `data.h`; an older design doc reuses `T:6` for the clip
  player — that is stale. The live free slot is `T:7`.
