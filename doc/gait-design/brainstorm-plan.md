# Brainstorm Plan: Blender → ESP32 Gait Pipeline

> Captured from Claude Code design session, 2026-04-01.
> This is the full design conversation distilled into a reference.
> For the detailed technical pipeline, see `gait-animation-pipeline.md`.
> For per-topic specs, see `specs/`.

---

## The Big Picture

The FaceHugger quadruped has 4 legs × 3 servos = 12 DSS-M15S servos (270°, 15kg·cm), controlled by an ESP32. We need a way to visually design gait cycles and get them running on hardware.

### The pipeline

```
Fusion 360 (CAD, mechanical design)
    ↓ export as STL (one per rigid body part)
Blender (visual animation)
    ↓ import STL → rig armature at servo shafts → animate with IK
    ↓ bake IK → per-bone FK keyframes
    ↓ export_gait.py (Blender Python script)
.gait JSON (intermediate, human-readable)
    ↓ gait_to_c.py (PC-side converter)
C header arrays (compiled into firmware)
    ↓ ESP32 playback engine at 50 Hz
servo PWM signals → physical robot movement
```

### What a gait cycle is

A gait cycle is a **loopable animation clip** that defines how all 12 servos move over a short duration (typically 0.5–2 seconds). Key constraints:

- **Start pose = end pose** (the neutral stance) — enables seamless looping
- All 4 legs can have independent trajectories (no forced symmetry)
- The ESP32 plays cycles in a loop; on command change, it finishes the current cycle then switches

---

## Key Design Decisions Made

### 1. Sparse keyframes, not dense sampling

The `.gait` file stores only inflection points (where motion changes direction), not a value at every 20ms tick. The ESP32 interpolates between keyframes at runtime. This keeps gait files tiny (~480 bytes per gait for 12 servos × ~10 keyframes each).

### 2. All gaits share a neutral start/end pose

Every gait cycle begins and ends at the same neutral stance: every Blender
bone at `rotation_euler = 0` → URDF θ=0 → servo PWM at `offset_deg`
(typically 135° = middle of 270° range). These are three views of the
same physical state. This means:
- No transition interpolation needed between gaits
- The state machine just plays cycle → cycle with no blending
- A "stop" command finishes the cycle and lands at neutral

The "splayed at -45°/+45°/-135°/+135° per leg" appearance of the rest
pose is encoded in the URDF joint origins (Convention A, see
[`MERGE_AND_CONVENTION.md` §0/§3](../../code/simulation/docs/MERGE_AND_CONVENTION.md)),
not in non-zero Blender rotation values.

We considered alternatives:
- **Runtime IK interpolation** between arbitrary poses — rejected (unnecessary ESP32 complexity for v1)
- **Explicit transition animations** (N×N gait pairs) — rejected (combinatorial explosion)
- **Return to neutral between gaits** with a visible pause — acceptable fallback if neutral pose constraint is too limiting

### 3. Inverted mode via runtime transform

The robot can flip upside down (legs have enough range). Instead of creating duplicate inverted gaits, apply `angle = 270 - angle` at runtime for pitch joints. One boolean flag, no extra data.

### 4. Speed multiplier as a minor adjustment

`cycle_time = elapsed * speed_multiplier`. Useful for ±20% tweaks. For significantly different speeds, create a separate gait (the timing/dynamics change too much for simple time-scaling to look right).

### 5. Gait files compiled into firmware

No SD card, no runtime file loading. The `.gait` JSON → `gait_to_c.py` → C headers → compiled into ESP32 flash. 20 gaits ≈ 10KB, ESP32 has 4MB flash. Not a concern.

### 6. Command interface via REST API

The ESP32 already runs a REST API for control. Gait commands are:
- `POST /gait {"name": "walk_forward"}` — queue a gait
- `POST /gait {"name": "stop"}` — queue stop (finish cycle → idle)
- `GET /gait/status` — current gait, progress, queue depth

Commands go into a circular buffer (capacity 4–8). Checked at cycle boundaries.

### 7. Assembly calibration: set zero in code, assemble in position

Standard servo calibration approach:
1. Power up the ESP32, command all servos to 0° (or 135° if using center-of-range as zero)
2. Physically assemble the robot while servos hold that position
3. The mechanical assembly now defines what 0° means physically
4. The Blender rig's rest pose must match this same stance

---

## Leg Architecture

Each leg has 3 joints (per URDF):
- **link1 (shoulder, yaw):** rotates the leg laterally. URDF axis = +Z.
- **link2 (hip, pitch):** raises/lowers the upper leg. URDF axis = ±Y (per-side sign).
- **link3 (knee, pitch):** extends/folds the lower leg. URDF axis = ±Y (per-side sign).

Naming convention (matches URDF link names, lowercase):
```
fl_link1, fl_link2, fl_link3    (front-left)
fr_link1, fr_link2, fr_link3    (front-right)
bl_link1, bl_link2, bl_link3    (back-left)
br_link1, br_link2, br_link3    (back-right)
```

Plus `base_link` for the chassis bone. After `EditBone.align_roll(joint.axis)`
at rig-build time, every joint rotates uniformly on bone-local Z
(`rotation_euler[2]`) — including the per-side ±Y sign flip on hip/knee,
which is absorbed into the bone roll.

Link lengths are read from the URDF (current values documented in
[`MERGE_AND_CONVENTION.md` §6](../../code/simulation/docs/MERGE_AND_CONVENTION.md)).
Don't hardcode them in spec or code — they drift whenever the CAD changes.

---

## Gait State Machine

```
States:  IDLE, PLAYING

Events:
  PLAY(gait)  — start or queue a gait cycle
  STOP        — finish current cycle, return to IDLE at neutral pose

IDLE:
  → PLAY(gait): set current_gait, start cycle → PLAYING

PLAYING:
  - Interpolation engine runs at 50 Hz
  - cycle_time = (millis() - cycle_start) % duration
  - At cycle boundary (time wraps past duration):
    - Check command queue
    - If queue has next gait: switch to it, reset cycle_start
    - If queue has STOP: go to IDLE
    - If queue empty: loop current gait
  - Mid-cycle commands are queued, never applied immediately
```

The "finish current cycle before switching" rule prevents jerky mid-motion transitions and guarantees the robot is always at the neutral pose when switching gaits.

---

## What the ESP32 Actually Does Each Tick

```
every 20ms (50 Hz):
  1. Compute cycle_time_ms
     - Apply speed multiplier: elapsed * speed_x100 / 100
     - Wrap: cycle_time = scaled_elapsed % gait.duration_ms

  2. For each of 12 servo tracks:
     a. Binary search for surrounding keyframes (kf_a, kf_b)
     b. Compute normalized time: t = (cycle_time - kf_a.t) / (kf_b.t - kf_a.t)
     c. Interpolate angle based on type:
        - LINEAR:   a + (b - a) * t
        - CUBIC:    a + (b - a) * smoothstep(t)    [smoothstep = 3t² - 2t³]
        - CONSTANT: a                               [step function]
     d. Apply inversion if active: angle = 270 - angle
     e. Write PWM to servo

  3. If cycle_time wrapped (new cycle started):
     - Check command queue, switch gait if needed
```

---

## Open Questions (To Resolve During Implementation)

1. ~~**Bone naming** — are the Blender bones already named FL_shoulder etc.?~~
   **Resolved 2026-05-04**: bones are named after URDF link names (lowercase,
   `fl_link1` / `fl_link2` / `fl_link3` per leg, plus `base_link`). See
   [`MERGE_AND_CONVENTION.md` §0](../../code/simulation/docs/MERGE_AND_CONVENTION.md)
   and [`blender-rig-and-export.md` §2](specs/blender-rig-and-export.md).
2. ~~**Axis alignment** — does each bone's local rotation axis match the physical servo axis?~~
   **Resolved 2026-05-04**: bone roll set via `EditBone.align_roll(joint.axis)`
   at rig-build time, so bone-local Z is the joint axis for all 12 joints
   uniformly. Export reads `rotation_euler[2]` for every servo.
3. ~~**Rest pose alignment** — does the Blender rest pose match the physical neutral stance?~~
   **Resolved 2026-05-04**: yes by construction — Blender bones at
   `rotation_euler = 0` ↔ URDF θ=0 ↔ servos at `offset_deg`. The splayed
   appearance is in URDF joint origins (Convention A), not in Blender
   rotation values.
4. **PWM refresh rate** — 50 Hz playback loop assumed; does the ESP32 servo library support this rate?
5. **Calibration tooling** — do we need a "calibration mode" firmware that sweeps servos one at a time for offset measurement?
6. **Gait versioning** — if the rig changes, how do we track which `.gait` files need re-export?

---

## Implementation Order

1. Blender rig (import STL, create armature, joint limits, neutral pose)
2. Export script (one servo first, then all 12)
3. Servo mapping config (placeholder IDs until wiring)
4. Test export with a simple "lift one leg" animation
5. C converter (`gait_to_c.py`)
6. ESP32 playback engine (linear interpolation first)
7. State machine + command queue
8. REST API endpoints
9. Full walk cycle in Blender → export → test on hardware
10. Additional gaits (turn, backward, stand, inverted mode)
