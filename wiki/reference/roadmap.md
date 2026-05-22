# Roadmap & future work

**Last updated:** 2026-05-22

---

## Project state

### What works today

- **Baked clips (`clips_all.h`):** Five joint-angle clips (lie-down/stand-up, one-leg lift, tiny wiggle, wave, wiggle) compiled into firmware. See `animation/exported_clips/clips_manifest.json`.
- **Clip player (T:7):** `tickClip()` plays a clip one-shot, then auto-returns to neutral over 500 ms and transitions to `STATE_IDLE`.
- **Gait engine:** Procedural WALK/TROT/CRAB gaits in `spinal_cord.cpp`, parameterized by direction and gait mode.
- **API:** Commands T:1-T:7, T:10 fully spec'd - see [api.md](api.md). T:7 firmware works; mobile app has no clips UI yet.
- **Mobile app:** Gait control, leg calibration, invert-robot. No "Clips" screen.

### Main open threads

1. **Next-gen `.fhc` foot-space pipeline:** Designed but not built. Replaces joint-angle baking with foot XYZ + Bezier handles, enabling runtime IK and adaptability.
2. **Mobile app clips UI:** T:7 backend is ready; front-end integration needed.
3. **Runtime adaptability:** IMU correction, terrain adaptation, and clip mirroring designed; no implementation yet.
4. **Servo numbering alignment:** Proposal in `animation/SERVO_ID_CONVENTION.md` needs firmware confirmation.

---

## Next-gen clip format: `.fhc` and foot-space animation

The key architectural step is shifting from baked joint angles to foot positions + Bezier handles, unlocking runtime IK. Full spec: [animation/fhc-format.md](animation/fhc-format.md).

### Why foot-space matters

- **Animator-friendly:** Define motion as "foot here," not joint angles.
- **Portable:** The same clip works across servo calibrations - IK runs at runtime.
- **Composable:** Overlays with IMU corrections, terrain sensing, and clip mirroring without re-baking.
- **Current limitation:** Recalibrating any servo invalidates all baked-angle clips.

### Format overview

Per-frame data:
- **Foot XYZ in body frame (mm):** position of each leg's foot relative to chassis.
- **Cubic Bezier handles (value-axis only):** 8 bytes per keyframe per axis; linear is the degenerate case.
- **Pole sign (optional):** sparse track for knee-inversion at full extension.
- **Track layout:** `SHARED_WITH_OFFSETS_MIRROR` (locomotion: one FL trajectory + phase offsets + mirror) or `PER_LEG_INDEPENDENT` (expressive clips).

Binary format structure is in `doc/animation-pipeline/leg-coordinates.md` §8. Task sequencing is in `doc/animation-pipeline/animation-pipeline-roadmap.md`.

### Migration path

1. Keep `clips_all.h` as fallback for backward compatibility.
2. Add a `.fhc` loader: boot-time enumerator loads every `.fhc` from `/clips/` into SRAM.
3. Task sequence:
   - T1: Port `ik_v2` from `code/simulation/kinematics.py` to C. Target: ~5 µs per leg at 100 Hz.
   - T2: `.fhc` binary structs + loader (1 day).
   - T3: Bezier evaluator (0.5 day).
   - T4: Blender exporter `export_fhc.py` (2 days, includes validation).
   - T5: Servo writer + calibration system (1-2 days + assembly time).
   - T6: URDF -> `LegGeom[]` header generator (0.5 day).
   - T7: Runtime gait engine (2-3 days): FSM, per-leg phase clocks, blend logic.
   - T8: WebSocket glue (1 day after T7).
   - T9: Bench test sequence (gates T1-T8).

### Authoring constraints

- **Body control must be static in v1:** No keyframes on `body_ctrl` within a clip's range. Body movement bakes into foot positions; animated body is a v2 feature.
- **First and last keyframes near neutral:** `<`1 mm tolerance for smooth transitions (exporter validates).
- **All foot positions IK-reachable:** Exporter runs `ik_v2` on every keyframe and rejects clips with unreachable poses.
- **Pole flips near full extension only:** Exporter checks joint limits to prevent servo strain.

---

## Runtime adaptability

With foot-space clips and runtime IK, these features become possible at no extra compute cost.

### IMU body-pose correction

Designed in `doc/animation-pipeline/leg-coordinates.md` §6. Hook in `animation-pipeline-roadmap.md` T7 and T10.

- Read MPU6050 at ~200 Hz; compute chassis-correction matrix from gravity vector.
- Call `gait_engine_set_pose_correction(R_correction, dh_mm)` at ~50 Hz.
- Runtime pipeline: `foot_corrected = R_correction * foot_clip + (0, 0, dh)` -> IK.
- T7 ships with identity-matrix stub; IMU integration is a separate workstream.

### Terrain adaptation

Hooks designed, not implemented. Per-foot Z offset when a foot sensor detects uneven ground; plugs into the foot-target pipeline after IMU correction.

### Clip mirroring

Designed in `doc/animation-pipeline/leg-coordinates.md` §5 (`SHARED` layout with `mirror_mask`).

Play a "wave right leg" clip mirrored to "wave left leg" by flipping the Y foot coordinate, halving the clip count. The exporter generates `mirror_mask = [bool; 4]` at export time; runtime applies the transform per frame. This is a clip property, not a separate firmware feature.

---

## Mobile app: clips UI (T:7 integration)

T:7 (`CMD_PLAY_CLIP`) is fully implemented in firmware. The mobile app has no clips UI yet.

### Implementation checklist

- [ ] Add a "Clips" tab or button in the Actions screen.
- [ ] Read clip metadata from `animation/exported_clips/clips_manifest.json` at app startup (clip ID, name, duration).
- [ ] Button per clip: send `{T:7, c:<id>}` on tap. Transition to `STATE_ACTION` first (same pattern as T:6).
- [ ] Show playback progress using the `pc` field from T:10 telemetry (`pc * 100%`). Auto-hide once clip finishes.

Code location hints: gait control UI is in `code/remote-control-app/`; T:6 (invert-robot) is a good reference for the `STATE_ACTION` transition pattern.

---

## Servo numbering alignment

**Status:** Proposal in `animation/SERVO_ID_CONVENTION.md`. Pending firmware confirmation.

The proposed numbering (`servo_id = leg_idx * 3 + joint_idx`, FL->FR->BL->BR order) disagrees with the current firmware leg ordering (FR->FL->RR->RL). This matters because `.gait` files bake servo IDs at export time - wrong numbering means every leg moves wrong on hardware. See [conventions.md](conventions.md) for leg-naming context.

### Resolution checklist

- [ ] Confirm firmware's `SERVO_CONFIG[]` ordering in `code/firmware/.../config.h`.
- [ ] If it differs from the proposal, update the proposal table and `servo_mapping.yaml` `servo_id:` values.
- [ ] Mark `SERVO_ID_CONVENTION.md` as confirmed (no longer PROPOSAL).
- [ ] Regenerate any `.gait` or `.fhc` files against the finalized numbering.

`servo_mapping.yaml` is the single point of agreement between Blender/Python and firmware. Keep it in sync.

---

## Gait authoring and config files

**Status:** Designed in `doc/animation-pipeline/leg-coordinates.md` §7. Not yet implemented.

Gaits are currently hardcoded in `spinal_cord.cpp`. Proposed direction:

1. **Move gait profiles to a config file** (JSON or YAML) read at boot - editable without recompiling.
2. **Blender-authored gaits (v2+):** Once foot-space clips work, gaits can be authored as cyclic `.fhc` clips in Blender instead of procedural parameters. Handles complex gaits (gallop, bound, canter) more naturally.

This is a v2+ feature. Pursue only if authoring new gaits becomes a bottleneck.

---

## Known rough edges and quick wins

### No fixed tick rate

**Issue:** Main firmware loop runs as fast as possible; I2C latency to PCA9685 dominates, making motion bursty.

**Fix:** Pin main loop to 50 Hz using `vTaskDelayUntil()` or a hardware timer. Makes playback more predictable and `pc` from T:10 more accurate.

**Effort:** ~2 hours. High value.

---

### Gait interruption: no graceful handoff

**Issue:** When a clip pre-empts a gait, the gait stops mid-stride (leg may hang in the air).

**Fix:** Before starting a clip, blend the current pose to nearest neutral stance over ~200 ms. In the `playClip(id)` handler, add a brief `STATE_BLENDING` phase that lerps to neutral, then enter `STATE_ACTION`.

**Effort:** ~4 hours. Noticeable UX improvement.

---

### Servo numbering ambiguity

**Issue:** The `config.h` channel mapping was set empirically; no formal cross-check against the physical wiring or URDF.

**Fix:**
1. Document the current firmware servo order in a table (like Table 1 of `SERVO_ID_CONVENTION.md`).
2. Compare against the physical wiring diagram. If they agree, mark the document confirmed. If not, fix firmware or the document, then re-export clips.

**Effort:** ~2 hours + inspection time. Prevents silent servo-order bugs.

---

### No servo command rate limiter

**Status:** Deliberately absent (YAGNI until hardware data exists). See `CONTEXT.md` §"Safety layers."

If servo overheating or current spikes become an issue, add a slew-rate limiter in `setServoAngle()` (e.g., 180 deg in 200 ms max). Existing tests will catch violations.

---

## Suggested next steps

| Priority | Task | Effort |
|---|---|---|
| Short (1-2 wk) | Align servo numbering against `SERVO_ID_CONVENTION.md` | 2 hrs |
| Short (1-2 wk) | Mobile app clips UI: read manifest, send T:7, show progress | 1-2 days |
| Short (1-2 wk) | Pin main loop to 50 Hz | 2 hrs |
| Medium (3-4 wk) | Port `ik_v2` to C | 1-2 days |
| Medium (3-4 wk) | `.fhc` format + loader + Bezier eval | 1.5 days |
| Medium (3-4 wk) | Blender exporter `export_fhc.py` | 2 days |
| Medium (3-4 wk) | Runtime gait engine | 2-3 days |
| Polish | Gait interruption graceful blend | 4 hrs |
| Polish | Battery voltage safety gate | 2 hrs |
| Polish | Servo numbering cross-check with CAD | 2 hrs + inspection |

---

## Design documents as reference

| Document | Covers |
|---|---|
| `CONTEXT.md` | Shared terminology (Gait vs. Clip, math-space vs. servo-space, translation layers). |
| `code/API_SPEC.md` | WebSocket protocol (T:1-T:10), all commands and telemetry. |
| `doc/animation-pipeline/leg-coordinates.md` | Spec for `.fhc` format, foot-space, IK, Bezier, pole signs, track layouts, IMU correction. Locked design. |
| `doc/animation-pipeline/onboard-clip-player-design.md` | Earlier joint-angle clip design (some parts stale; end-behavior spec is still good). |
| `doc/animation-pipeline/animation-pipeline-roadmap.md` | Task breakdown, ownership, sequencing, effort estimates. |
| `animation/SERVO_ID_CONVENTION.md` | Servo numbering proposal and firmware-alignment checklist. |
| `code/simulation/kinematics.py` | `ik_v2` canonical implementation (to port to C for T1). |
| `code/firmware/CLIP_PLAYER_TESTING.md` | Test suite for the current joint-angle clip player (reference for `.fhc` test structure). |

All design decisions in these documents are locked - settled through grill-me sessions and ADRs. Implement as written; if you find a contradiction, check `CONTEXT.md` for the latest resolution.
