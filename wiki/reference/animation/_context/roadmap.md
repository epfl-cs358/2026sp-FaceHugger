<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/animation-pipeline/animation-pipeline-roadmap.md
Original kind:  research
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference-only
-->

> **Reference material.** Verbatim copy of `doc/animation-pipeline/animation-pipeline-roadmap.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/animation-pipeline/animation-pipeline-roadmap.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [todo]  T1-T10 all unstarted. None of the named firmware outputs exist: no leg_ik.{h,cpp}, fhc_format.h/fhc_loader.cpp, bezier_eval.{h,cpp}, gait_engine.{h,cpp}, servo_writer.{h,cpp}, leg_geom.h in code/firmware/src/nervous_system/; no export_fhc.py / generate_leg_geom_h.py. Matches CLAUDE.md "implementation has not started".
- [ok]    T1 input link kinematics.py:135 ik_v2 is exact — `def ik_v2(cfg, foot_body, leg_id)` is at generate line 135 (kinematics.py:135).
- [ok]    "Status of existing pieces" table accurate: urdf_to_blender_rigged.py exists/working; kinematics.py ik_v2 canonical (used at kinematics.py:313 leg_ik=ik_v2); firmware kinematics.cpp present but legacy; .gait legacy. Servo writer is in servo.cpp (firmware), as the ⚠ row states.
- [stale] Sibling relative links (leg-coordinates.md, firmware-research.md, api-surface.md) resolve only from original doc/animation-pipeline/ — all siblings still present there, but will break in the published wiki location.
- [drift] T1 signature shows `JointAngles leg_ik(int leg_id, FootPos foot_body, int8_t pole_sign)` — design-only, no implementation to check against; left for Phase B authors.
- [drift] T8 lists T:1/T:3/T:5/T:2 dispatch; firmware now also has CMD_ACTION_SELECTION=6 (post-dates this roadmap) — fold into the command map when T8 is planned.
-->
# Animation Pipeline — Roadmap and Work Segmentation

Companion to [`leg-coordinates.md`](leg-coordinates.md) and
[`firmware-research.md`](firmware-research.md). This document breaks
the new animation pipeline into discrete tasks, identifies which can
run in parallel, and suggests ownership.

The design itself is in `leg-coordinates.md`; this document is about
**how to build it as a team**.

---

## Status of existing pieces

| Piece | State | Notes |
|---|---|---|
| Blender rig (`urdf_to_blender_rigged.py`) | ✅ exists, working | armature + IK + foot empties |
| Sim IK (`code/simulation/kinematics.py`) | ✅ canonical | `ik_v2` is the authority for on-board IK |
| Firmware IK (`code/firmware/.../kinematics.cpp`) | ❌ legacy, wrong for this rig | scheduled for removal |
| Firmware servo writer | ⚠️ partial in `code/firmware/.../servo.cpp` | needs PCA9685 + per-servo calibration table |
| `.gait` baked-angle format | ❌ legacy | replaced by `.fhc` (foot-XYZ Bezier) |
| WebSocket API ([`code/API_SPEC.md`](../../code/API_SPEC.md)) | ✅ design exists | runtime engine plugs in behind it |
| IMU sensor integration | 🟡 teammate-owned | runtime engine leaves a hook |

---

## Task graph

```
   ┌─ T1. C IK port ─────────────────────────────────────┐
   │                                                     │
   ├─ T2. .fhc binary structs + loader ─────────────┐    │
   │                                                │    │
   ├─ T3. Bezier evaluator ─────────────────────────┼────┼─→ T7. Runtime gait engine ─→ T9. Bench test
   │                                                │    │           ↑
   ├─ T4. Blender exporter (export_fhc.py) ─────┐   │    │           │
   │                                            │   │    │           │
   ├─ T5. Servo writer + calibration ───────────┼───┼────┼───────────┤
   │                                            │   │    │           │
   └─ T6. URDF → LegGeom[] header generator ────┘   │    │           │
                                                    │    │           │
                          T8. WebSocket ↔ engine glue ───┴───────────┤
                                                                     │
                          T10. IMU correction layer ──────────────────┘
                                  (teammate-owned, separate track)
```

T1 through T6 can run **fully in parallel** after the design is
locked. T7 needs T1, T2, T3, T5 done. T8 plugs into T7. T9 is the
end-to-end test. T10 runs on its own track and lands when T7 is
ready to receive its hook.

---

## Tasks in detail

### T1. Port `ik_v2` to C

**Owner**: 1 person, comfortable with C and the math.

**Inputs**:
- [`code/simulation/kinematics.py:135 ik_v2`](../../code/simulation/kinematics.py#L135)
- `LegGeom` struct definition from §3 of `leg-coordinates.md`

**Outputs**:
- `code/firmware/src/nervous_system/leg_ik.h`
- `code/firmware/src/nervous_system/leg_ik.cpp`
- A host-side test (PyBullet round-trip or pure Python comparison)
  confirming the C IK matches `ik_v2` to within 1e-4 rad on a grid
  of foot positions.

**Concrete deliverable**:
```c
// leg_ik.h
typedef struct { float yaw, hip, knee; } JointAngles;
typedef struct { float x, y, z; } FootPos;
JointAngles leg_ik(int leg_id, FootPos foot_body, int8_t pole_sign);
```

The function signature takes `foot_body` in body frame. Subtraction
of the per-leg hip offset (`LegGeom.hip_x/y/z`) must happen **inside**
`leg_ik` before cylindrical decomposition, not at the call site. Call
sites always pass body-frame positions.

**Estimated effort**: 1–2 days. The math is mechanical; the host-side
test is the time sink.

**Risks**: getting the per-leg axis signs and `chain_x_sign` right.
Mitigate by porting `kinematics.py` line-for-line first, optimizing
later.

---

### T2. `.fhc` binary structs and loader

**Owner**: 1 person.

**Inputs**: §8 of `leg-coordinates.md` (file format spec).

**Outputs**:
- `code/firmware/src/nervous_system/fhc_format.h` — packed structs,
  enum definitions.
- `code/firmware/src/nervous_system/fhc_loader.cpp` — `bool
  fhc_load_from_lfs(const char* path, FhcClip* out)`.
- A boot-time enumerator that loads every file in `/clips/*.fhc`
  into pre-allocated SRAM (per [`firmware-research.md`](firmware-research.md)
  §2: never read flash from the IK loop).
- Unit test: hand-write a known-bytes `.fhc`, load it, assert struct
  fields match.

**Estimated effort**: 1 day for structs + loader + test.

**Risks**: endianness if anyone runs this on a different MCU later.
Mitigate by writing little-endian everywhere and documenting it.

---

### T3. Bezier evaluator

**Owner**: 1 person. Smallest task; can be combined with T2.

**Inputs**: §5 of `leg-coordinates.md`.

**Outputs**:
- `code/firmware/src/nervous_system/bezier_eval.h`
- `code/firmware/src/nervous_system/bezier_eval.cpp`
- Unit test: feed a 4-keyframe track, sample at known `t`, compare
  to a reference Python implementation (e.g. NumPy + `scipy.special.bernstein_poly`).

**Concrete deliverable**:
```c
// bezier_eval.h
float fhc_sample_axis(const FhcKeyframe* track, int n_keyframes, uint32_t t_ms);
Vec3  fhc_sample_foot(const FhcLegTracks* tracks, uint32_t t_ms);
int8_t fhc_sample_pole(const FhcPoleKeyframe* track, int n, uint32_t t_ms,
                       int8_t default_sign);
```

**Estimated effort**: 0.5 day.

---

### T4. Blender exporter `export_fhc.py`

**Owner**: 1 person comfortable with `bpy`. Probably you (Marcus) —
you wrote `urdf_to_blender_rigged.py`.

**Inputs**: §2 + §4 + §5 + §6 + §8 of `leg-coordinates.md`.

**Outputs**:
- `animation/scripts/export_fhc.py`.
- Reads custom Action properties (`track_layout`, `clip_class`,
  `phase_offset_ms`, `mirror_mask`).
- Bakes IK to FCurves (`bpy.ops.nla.bake` with `visual_keying=True`).
- Reads X/Y/Z FCurves on each foot empty + `pole_sign` custom
  property FCurves.
- Per-frame: world→body via `body_ctrl.matrix_world.inverted()`.
- This transform is applied regardless of whether `body_ctrl` has
  keyframes. Body ctrl movement is an intentional authoring affordance:
  it bakes into the stored foot trajectories and is not represented
  separately in the file. The clip stores only body-relative foot
  positions — the ESP32 has no concept of where the body was in world
  space during authoring.
- Validates:
  - Time-aligned keyframes per leg (X/Y/Z keyed at same frames).
  - Auto Clamped (or Auto/Vector) handles only.
  - First and last keyframe at neutral pose (≤1 mm tolerance).
  - For SHARED layout: FR/BL/BR are FL phase-shifted + mirrored.
  - For pole flips: each occurs near full extension.
  - Every keyframe is reachable through `kinematics.py:ik_v2` without
    hitting joint limits.
  - `body_ctrl` has no keyframes (is static) for v1. Reject with a
    clear message if any property of `body_ctrl` is animated within
    the clip's frame range.
- Writes `.fhc` binary.

**Concrete deliverable**:
```bash
uv run python animation/scripts/export_fhc.py \
    --blend animation/blend-iterations/<file>.blend \
    --action <action_name> \
    --out  code/firmware/data/clips/<clip_name>.fhc
```

**Estimated effort**: 2 days (the validation is half the work).

**Risks**: getting the world→body transform right when `body_ctrl`
moves. Convention for v1: **`body_ctrl` must be static within a
clip**. Validated at export. Animated `body_ctrl` deferred to v2.

---

### T5. Servo writer + per-servo calibration

**Owner**: 1 person. Hardware-adjacent; pairs well with bench
testing.

**Inputs**:
- [`code/firmware/src/shared/config.h`](../../code/firmware/src/shared/config.h)
- [`animation/SERVO_ID_CONVENTION.md`](../../animation/SERVO_ID_CONVENTION.md)

**Outputs**:
- `code/firmware/src/nervous_system/servo_writer.{h,cpp}` —
  `void servo_write_all(const JointAngles legs[4])`.
- A `servo_calibration.yaml` (or compiled-in C struct) per servo:
  `offset_deg`, `direction`, `min_deg`, `max_deg`.
- PCA9685 driver wrapper at I2C 400 kHz.
- Conversion: `servo_deg = clamp(offset_deg + direction ·
  degrees(joint_rad), min_deg, max_deg)`.
- Pulse: `MIN_PULSE + (servo_deg / 180) · (MAX_PULSE − MIN_PULSE)`.
- A bench-side calibration tool: `T:4` API command (per
  [`code/API_SPEC.md` §4](../../code/API_SPEC.md#L51)) drives a
  single servo, animator notes its physical position, calibration
  YAML updated.

**Estimated effort**: 1–2 days, plus calibration time per assembled
robot.

**Pre-requisite resolution**: drop the 0–270° / DSS-M15S references
from [`API_ANIMATION_SPEC.md` §6](../../code/simulation/docs/API_ANIMATION_SPEC.md#L194).
180° everywhere. Fold this into T5.

---

### T6. URDF → LegGeom header generator

**Owner**: 1 person. Could be the same as T1.

**Inputs**: `code/simulation/generated/facehugger.urdf`.

**Outputs**:
- A Python script `code/simulation/generate_leg_geom_h.py` that
  reads the URDF and emits
  `code/firmware/src/nervous_system/leg_geom.h` with `const
  LegGeom LEG_GEOM[4] = { ... };` populated from the URDF.
- Make this a regenerate-on-build step so the firmware never
  drifts from the URDF.
- The generated `leg_geom.h` must include per-leg:
  `hip_x, hip_y, hip_z` (hip joint position in body frame, mm),
  `l2, l3` (link lengths, mm), `w_y` (lateral offset of link2 from
  yaw axis, mm), `chain_x_sign` (+1 or -1). All values read directly
  from the URDF; **none are hardcoded in firmware**.

**Estimated effort**: 0.5 day.

---

### T7. Runtime gait engine

**Owner**: 1 person. Most architectural piece.

**Depends on**: T1, T2, T3, T5.

**Inputs**:
- [`firmware-research.md`](firmware-research.md) §3 (FSM + per-leg
  phase clocks).
- §6 + §7 of `leg-coordinates.md` (runtime pipeline ordering).
- [`api-surface.md`](api-surface.md) for the engine's outward API.

**Outputs**:
- `code/firmware/src/nervous_system/gait_engine.{h,cpp}`.
- A 100 Hz tick pinned to core 1 via `xTaskCreatePinnedToCore`.
- FSM: `IDLE | LOADING | PLAYING | TRANSITIONING | FAILSAFE`.
- Per-leg phase clocks; clip swap with joint-space lerp blend
  (100–200 ms).
- Calls into T1 (IK), T3 (Bezier), T5 (servo write), T10 (IMU
  correction stub) every tick.

**Concrete deliverable**: see [`api-surface.md`](api-surface.md) for
the function signatures.

**Estimated effort**: 2–3 days.

**Risks**: clip transition smoothness, FSM edge cases. Mitigate by
shipping with one clip first (T9 step 1: stand still), then layering
in transitions only after that works.

---

### T8. WebSocket ↔ engine glue

**Owner**: 1 person. Pairs well with the existing
[`code/API_SPEC.md`](../../code/API_SPEC.md) author.

**Depends on**: T7.

**Inputs**: [`code/API_SPEC.md`](../../code/API_SPEC.md),
[`api-surface.md`](api-surface.md).

**Outputs**:
- `code/firmware/src/brain/network.{h,cpp}` extensions to dispatch
  `T:1`, `T:3`, `T:5`, `T:2` to gait_engine API calls.
- Telemetry: `T:10` reads gait_engine state every 500 ms.

**Estimated effort**: 1 day after T7 lands.

---

### T9. Bench test sequence

**Owner**: Marcus + whoever else is around.

**Depends on**: T1–T8.

**Test sequence** (each step gates the next):

1. **`stand.fhc`**: 2 keyframes, identical neutral pose, period
   1000 ms, PER_LEG. Robot holds neutral pose. **Validates**: file
   format, loader, Bezier eval (degenerate), IK, servo writer,
   calibration.
2. **`one_leg_lift.fhc`**: FL foot lifts and lowers in Z over 1 s,
   others neutral. PER_LEG. **Validates**: Bezier eval (real
   handles), per-leg track independence.
3. **`trot.fhc`**: SHARED, period 600 ms,
   `phase_offset_ms = [0, 300, 300, 0]`, mirror_mask for L↔R.
   Robot suspended in air or on a stand. **Validates**: SHARED
   layout, phase offsets, mirror, FSM looping.
4. **Walk forward on the ground**: same trot but on a flat surface.
   **Validates**: end-to-end physical behavior. Reveals any
   coordinate-frame errors that didn't show in air.
5. **Gait transitions**: command trot → stand → walk via `T:5`.
   **Validates**: T7's FSM blend logic.
6. **IMU layer in**: T10 lands, body-pose correction active.
   **Validates**: T10 + T7 integration.
7. **First expressive clip**: e.g., `wave_fl.fhc`. Stunt territory.

---

### T10. IMU correction layer (teammate-owned)

**Owner**: separate teammate per the user's note.

**Hook from T7**: `gait_engine_set_pose_correction(const Mat3* R,
float dh_mm)`. Identity matrix + 0 height = pass-through (no
correction).

**The teammate's responsibilities**:
- Read MPU6050 at e.g. 200 Hz on core 0 (or a dedicated task).
- Compute chassis-correction matrix from the gravity vector.
- Call `gait_engine_set_pose_correction` at e.g. 50 Hz.

**T7's responsibility**: define the API, accept the matrix, apply it
in the foot-target → IK pipeline as
`foot_corrected = R_correction · foot_clip + (0, 0, dh)`.

This decouples the two completely — T7 ships with an identity-matrix
stub; T10 lands later without changing T7.

---

## Suggested ownership (small team)

If you have 3 people including yourself:

| Person | Tasks |
|---|---|
| **Marcus** | Design (this doc), T4 (exporter), T6 (URDF→header), T9 (testing) |
| **Teammate A** | T1 (IK port), T2 (file format), T3 (Bezier eval), T7 (engine) — the firmware-meaty path |
| **Teammate B** | T5 (servo writer + calibration), T8 (WebSocket glue) — the hardware-adjacent path |
| **Teammate C** (separate) | T10 (IMU layer) |

If you have 5+ people, split T1+T2+T3 from T7, and the engine-builder
becomes a fourth role.

---

## Sequencing (calendar view)

Week 1 (parallel start):
- T1, T2, T3, T6 (firmware track)
- T4 (Blender track)
- T5 (hardware track)

Week 2:
- T7 starts as soon as T1+T2+T3+T5 land
- Continue T4 testing
- T9 step 1 (stand pose) end-to-end as soon as T7's first version exists

Week 3+:
- T8, T9 steps 2–4
- T10 integration
- First expressive clips authored in Blender

This is aspirational; adjust to actual capacity. The point is that
**T1 through T6 are all independent and can be picked up by anyone
once the design is locked** (which, after this synthesis, it is).

---

## What this roadmap deliberately does not cover

- **Procedural locomotion authoring** (parametric Bezier control
  points authored in Python rather than Blender). Future tool that
  emits `.fhc` files, same format. Out of scope for v1.
- **Terrain adaptation** (per-foot Z offset based on contact
  sensors). Hooks exist in T7's pipeline (after IMU correction);
  implementation deferred.
- **Wall-flip stunt clip**. The format supports it (`clip_class =
  FLIP`); the actual stunt is its own design exercise.
- **OTA updates** to clip files. Separate concern; the boot-time
  loader doesn't care how the files got there.

---

## Cross-references

- [`leg-coordinates.md`](leg-coordinates.md) — design canon.
- [`firmware-research.md`](firmware-research.md) — ESP32 platform
  constraints (memory, dual-core, flash).
- [`api-surface.md`](api-surface.md) — runtime engine API spec.
- [`code/API_SPEC.md`](../../code/API_SPEC.md) — WebSocket protocol.
- [`code/simulation/kinematics.py`](../../code/simulation/kinematics.py)
  — canonical IK implementation to port.
- [`animation/scripts/urdf_to_blender_rigged.py`](../../animation/scripts/urdf_to_blender_rigged.py)
  — existing Blender rig (Action authoring happens here).
