# FaceHugger pipeline reflow — state & plan

Living document so the next session picks up without rebuilding context. Read this first, then [PIPELINE_SPEC.md](PIPELINE_SPEC.md) and [ASSEMBLY_HIERARCHY.md](ASSEMBLY_HIERARCHY.md) for the authoritative CAD/URDF spec. End-to-end "how the pipeline works" docs live in [URDF_PIPELINE.md](URDF_PIPELINE.md).

The full original plan with rationale lives at `~/.claude/plans/peppy-cooking-charm.md`. This file is the operational summary: where we are, what to do next, what to skip.

---

## Where we are

Branch: `feat/urdf-pipeline`. Recent commits (newest last):

| Commit | What |
|---|---|
| `f8671b6` | Phase 0 diagnostic + spec docs + first re-export from restructured CAD |
| `a670244` | Phase A: new MESH_EXPORTS, joint capture, post-export checklist |
| `6716c63` | Fix asBuiltJoints walk + visibility-tolerant occurrence checks |
| `19be7e1` | UTF-8 file writes + ASCII-only joints formatting |
| `1e49d08` | Phase B: generate_urdf consumes CAD joints + per-side meshes + bracket visuals |
| `44b2036` | Alignment fix: leg-assembly normalization (Phase E) + URDF gen rewrite (Phase G/H/I) + standalone servos + view_urdf.py + URDF_PIPELINE.md |

### Phase 0 — DONE ✓

[phase0_verify.py](phase0_verify.py) reads `fusion_export.json` and PASS/FAILs against ASSEMBLY_HIERARCHY. Visibility-tolerant: hidden Link1R/MotorMountR are accepted via `mesh_files` evidence. Run anytime with `python code/simulation/phase0_verify.py` — must pass before Phase A.

### Phase A — DONE ✓ (verified by user re-export)

[cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py](../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py) rebuilt:

- **Visibility filtering removed entirely.** Toggle CAD light bulbs however you want — what gets captured is driven by four explicit lists: `EXPORT_RULES` (bodies → STLs), `CONSTRUCTION_POINTS` (named cpoints), `CONSTRUCTION_AXES` (named caxes), `JOINTS` (named joints). The JSON records each entity's `visible` flag as informational metadata only.
- **The export is the raw material; filtering happens at consumption time.** The `occurrences` tree in `fusion_export.json` is the *complete* design tree (every PCB, OLED, capacitor, etc.) — that's by design, so downstream tools (URDF generator, Blender visualizer) can resolve any occurrence path they need to look up world transforms. The four whitelists above shape **specific slices** of the JSON output: STLs (`EXPORT_RULES`), per-occurrence `points[]` / `axes[]` arrays, and the top-level `joints[]` array. The tree itself is unfiltered; consumers pick out the bits they care about.
- Chassis rule is now `type: "combined"` listing the `QuadrupedBody` and `LipoCage` bodies explicitly (electronics excluded by NOT being in the rule, regardless of CAD visibility).
- Per-side body rules with optional `component` filter and scoped landmark lookup.
- `combined` rule with optional `origin_landmark` (re-origin in world frame).
- `collect_joints()` walks **both** `Component.joints` and `Component.asBuiltJoints` across `design.allComponents`, filtered by the `JOINTS` whitelist. Output goes to top-level `joints` array in JSON + `=== Joints ===` block in TXT.
- Post-export message box runs the same checklist as `phase0_verify.py`.

**Latest export confirms**: 8 STLs, 3 joints captured (`Link1Revolute -105°/+45°`, `Link2Revolute -135°/+45°`, `Link3Revolute -90°/+90°`), all checklist rows ✓.

### Known gaps (not blockers)

- **`axis_construction_name` is None** for all three joints — Fusion's AsBuiltJoint API resolves axis-aligned construction axes to principal-axis enum values rather than exposing the originating entity name. `axis_dir_local_unit` is correctly populated (matches BodyToLink1Axis / Link1ToLink2Axis / Link2ToLink3Axis directions exactly), so the URDF generator has everything it needs. Leave for now.
- **`origin_construction_name` is None** for all three joints — same reason. `axis_origin_local_mm` is exact (e.g. `(11.97, 0, -4.20)` matches `BodyToLink1Point`).
- **R-side hip/knee servos not modeled** in CAD. `leg_shoulder_R.stl` is just `Link1R` body; the FR/BL legs render without their hip/knee servo bodies in the URDF until the user adds R servos in Fusion. Per PIPELINE_SPEC §5 this is an accepted gap.
- **`MotorMountR:1/LegMountFixedPoint`** is unverifiable while the user keeps that occurrence hidden. The R bracket lands at `LegMountPointFR` / `LegMountPointBL` purely via the URDF's instancing logic; no CAD-side cross-check.

### What's actually in `mesh_files` after the latest export

```
QuadrupedBody.stl   = FlexibleSkeleton:1 (chassis + LipoCage)
leg_mount_L.stl     = MotorMount:1/LegMountL,    re-origined to LegMountFixedPoint (local)
leg_mount_R.stl     = MotorMountR:1/LegMountR,   re-origined to LegMountFixedPoint (local)
leg_shoulder_L.stl  = combined: Link1L body + Servo_Mouser_Model:2/ServoBase,
                        re-origined to BodyToLink1Point (world)
leg_shoulder_R.stl  = Link1R body, re-origined to BodyToLink1Point (local)
leg_upper.stl       = Link2L/Link2 body, re-origined to Link1ToLink2Point
leg_lower.stl       = combined: Link3L body + Servo_Mouser_Model:3/ServoBase,
                        re-origined to Link2ToLink3Point (world)
servo.stl           = ServoBase, re-origined to ServoMountPoint
```

`_servo_role_assignment` is preserved across re-runs:

```
shoulder: FaceHuggerLegAssembly:1/Servo_Mouser_Model:1
hip:      FaceHuggerLegAssembly:1/Servo_Mouser_Model:2
knee:     FaceHuggerLegAssembly:1/Servo_Mouser_Model:3
```

---

### Phase B — DONE ✓ (commit `1e49d08`)

[generate_urdf.py](generate_urdf.py) consumes the CAD-sourced joints from `fusion_export.json` (`axis_dir_local_unit`, `axis_origin_local_mm`, `limits_rad`); emits 4 bracket visuals on `base_link`, per-side L/R link1 meshes, per-leg shoulder limits from yaml. yaml's `leg_template.joints` reduced to `cad_name → urdf_name + parent/child` mapping only.

### Phase E + G + H + I — Alignment fix DONE ✓ (commit `44b2036`)

Two compounding bugs were silently misplacing the legs in PyBullet: (1) the shoulder joint origin sat at the bracket's mounting tab instead of the rotation axis (38mm off), and (2) the leg-assembly's 90°-about-Z world rotation in CAD was never compensated for, so meshes and joint axes were 90° off. Fixed by:

- **Phase E** (exporter): compute `R_la` and apply to `axis_dir` / `axis_origin` for leg-assembly-internal joints. Convert all leg-internal body-rules to combined-rule (which auto-bakes world-frame vertices). Drop servo bake-ins from link STLs; standalone `servo.stl`. Add `landmark_occurrence` field to scope landmark lookups. JSON gets a top-level `leg_assembly_normalization` block.
- **Phase G** (URDF gen): per-corner shoulder placement = `mount + Rz(rpy_z) · side_offset`. Hip/knee origins X-flipped for R pair. Per-side L/R link1 meshes. Link2/link3 mesh visual rpy `(0, π, 0)` for R pair (shared meshes).
- **Phase H** (uniform joint sign convention): R-pair hip/knee axis flip + negate-and-swap limits. Same stance value drops every leg uniformly.
- **Phase I** (standalone servos): 12 servo visuals (4 shoulder + 4 hip + 4 knee). Per-role rpy = `R_role · R_servo1^T` lifts the shoulder-baked mesh's `+Z` shaft to world `+Y` for hip/knee. R pair gets X-flipped position only — no rotation (servo is the same physical part on every leg).

URDF kinematically verified via PyBullet FK. **Visually inspected via `view_urdf.py`** (new minimal viewer) — geometry looks correct from all angles.

Full rationale + math in [ALIGNMENT_FIX_PLAN.md](ALIGNMENT_FIX_PLAN.md). User-facing pipeline docs in [URDF_PIPELINE.md](URDF_PIPELINE.md).

---

## Next: Phase J — `simulate_v2.py` FK rewrite (the only thing blocking stand/walk)

The URDF is geometrically correct, but `simulate_v2.py`'s analytic FK/IK is calibrated against the OLD URDF (hip/knee axes were `+X`, single sign convention, etc.). With the new URDF:

1. **Drop `R_L1` per-leg rotation layer** (lines ~538–565 in simulate_v2). Shoulder rpy now encodes per-leg orientation directly; we don't need the per-leg matrix machinery.
2. **Switch hip/knee axis from old `+X` (leg-assembly local) to normalized `+Y`** in the analytic FK code.
3. **Sign convention is uniform across all 4 legs** (Phase H gave us this). FK/IK collapses to one function — no per-side branching.
4. **Foot tip is now side-dependent**: link2/link3 have `mesh_rpy = (0, π, 0)` for R pair, so the mesh-local foot tip `(x, y, z)` becomes `(-x, y, -z)` in URDF link3 frame for R pair. IK target needs to respect this.
5. **Stance height re-calibration**: at `hip=-40°, knee=-60°` the foot-tip world Z is roughly -85 mm relative to body origin. Default `body_height: 1 mm` puts feet 85 mm below ground at startup — bump initial body z to ~90 mm so PyBullet doesn't fight an interpenetration condition.

After Phase J: stance pose lands all 4 feet on the ground, IK resolves without `[FAIL]` markers, and gait controllers can be built on top.

## Then: Phase C — Blender visualizer

**Goal**: regenerate `facehugger.urdf` from the new CAD outputs; the URDF must match what `simulate_v2.py` and the Blender visualizer expect.

Files to edit:
- [generate_urdf.py](generate_urdf.py)
- [facehugger_config.yaml](facehugger_config.yaml)

### B1 — `leg_mount_{L,R}.stl` visual on `base_link`

Mirror the existing chassis-fixed shoulder-servo logic:
- 4 bracket `<visual>` elements on `base_link`, one per `LegMountPointXX`.
- Mesh = `leg_mount_L.stl` for FL/BR, `leg_mount_R.stl` for FR/BL (per the diagonal-pair rule).
- xyz from each leg's `mount_point`. rpy = identity.

### B2 — yaml schema

Per [PIPELINE_SPEC.md](PIPELINE_SPEC.md) §1, §3, §4. Update `legs[]`:

```yaml
legs:
  - { id: fl, mount_point: LegMountPointFL, side: L, rpy_z_deg:   0,
      shoulder_limits_deg: [-105,  +45], shoulder_neutral_deg: 0 }
  - { id: br, mount_point: LegMountPointBR, side: L, rpy_z_deg: 180,
      shoulder_limits_deg: [ -45, +105], shoulder_neutral_deg: 0 }
  - { id: fr, mount_point: LegMountPointFR, side: R, rpy_z_deg:   0,
      shoulder_limits_deg: [ -45, +105], shoulder_neutral_deg: 0 }
  - { id: bl, mount_point: LegMountPointBL, side: R, rpy_z_deg: 180,
      shoulder_limits_deg: [-105,  +45], shoulder_neutral_deg: 0 }
```

`leg_template.links.link1.mesh` becomes a `"leg_shoulder_{side}.stl"` template; generator substitutes `{side}` per leg. Link2 / Link3 stay shared (`leg_upper.stl`, `leg_lower.stl`).

### B3 — Diagonal-pair shoulder-joint rpy

Per PIPELINE_SPEC §9, the 180° Z rotation for BR/BL **rotates around the leg's own mount point**. Already encoded by per-leg `rpy_z_deg = 180`. Stop assuming uniformity in the generator.

### B4 — Per-leg shoulder limits

Drop the old "neutral ± 90°" derivation. Use each leg's `shoulder_limits_deg` directly when emitting shoulder joints.

### B5 — Consume CAD-sourced joint info (graceful)

Read `joints` array from `fusion_export.json`. Per-property precedence: `JSON > yaml`. Three joints to consume: `Link1Revolute` (shoulder — but per-leg rpy_z still applies post hoc), `Link2Revolute` (hip), `Link3Revolute` (knee). Limits in radians from JSON convert to URDF directly.

If `joints` empty/missing, fall back to yaml — keeps a transition window. Log a warning if both define a property and disagree.

### B6 — Sanity-keep chassis-fixed shoulder-servo visuals

The committed chassis-fixed shoulder-servo logic (uniform rpy = source FL rpy, 4 translations, 1 visual per `LegMountPointXX`) stays as-is. Confirm `_servo_rot_by_role` and `_servo_role_assignment` still resolve in the new manifest (they do — the assignment block was preserved across re-export).

### Phase B verification

```bash
python code/simulation/generate_urdf.py
grep -c '<visual>' code/simulation/facehugger.urdf       # >= 21
grep -c 'leg_shoulder_L.stl' code/simulation/facehugger.urdf   # = 2 (FL, BR)
grep -c 'leg_shoulder_R.stl' code/simulation/facehugger.urdf   # = 2 (FR, BL)
grep -c 'leg_mount_L.stl'    code/simulation/facehugger.urdf   # = 2
grep -c 'leg_mount_R.stl'    code/simulation/facehugger.urdf   # = 2
```

Per-leg shoulder limits in the URDF match PIPELINE_SPEC §4 table.

---

## Then: Phase C — Blender visualizer

File: [../../animation/scripts/visualize_fusion_export.py](../../animation/scripts/visualize_fusion_export.py).

### C1 — Mount-point name lookup

Update the `mounts` dict construction to read `LegMountPoint{XX}` instead of `LegMount{XX}`.

### C2 — Drop bracket-rotation lookup

`_legmount_rot_3x3()` reads bracket world rotations. Bracket occurrences moved into the leg assembly — function returns None for all 4 corners. Replace with the diagonal-pair `rpy_z_deg` table from yaml: `Rz(0°)` for FL/FR, `Rz(180°)` for BR/BL.

### C3 — Per-side mesh in duplication

`instance_four_legs()` imports both L and R leg-link meshes, picks per corner:

```
Legs/FL — L mesh, source position.
Legs/BR — L mesh, transform = T(LegMountPointBR) @ Rz(180°) @ T(-LegMountPointFL) @ M_src_L.
Legs/FR — R mesh, transform = T(LegMountPointFR) @ Rz(  0°) @ T(-anchor_R)         @ M_src_R.
Legs/BL — R mesh, transform = T(LegMountPointBL) @ Rz(180°) @ T(-anchor_R)         @ M_src_R.
```

`anchor_R` = world position of the R leg's `BodyToLink1Point` reference. Pull from JSON at runtime (it's the mirrored counterpart of the L-side anchor).

### C4 — Bracket visuals

Add 4 bracket meshes to chassis (`Meshes` collection): `leg_mount_L.stl` at FL/BR, `leg_mount_R.stl` at FR/BL. Chassis-fixed.

### C5 — Shoulder servos

Already handled by the committed `Shoulders` sub-collection logic; nothing to change.

---

## Then: Phase D — CLI orchestrator

New file: `code/simulation/facehugger.py` (~150 lines). One argparse-based entry point:

| Command | Wraps | Notes |
|---|---|---|
| `verify` | `phase0_verify.py` | Pass-through, exits non-zero on fail. |
| `urdf` | `generate_urdf.py` | Pass-through `--export` `--config` `--out`. |
| `blender [--headless] [--save PATH] [--dump]` | Blender + visualizer | Auto-detect path; honor `BLENDER_BIN`. |
| `sim [--walk\|--trot] [--headless] [--settle SEC]` | `simulate_v2.py` | Mirror simulator argparse. |
| `inspect <STL> [--top N] [--grid F]` | `inspect_stl.py` | Pass-through. |
| `all` | verify → urdf → sim | Smoke shortcut. |

Implementation: `subprocess.run(["python", str(SCRIPT_PATH), *args], cwd="code/simulation/")`. Blender bin: `BLENDER_BIN` env → fallback `/Applications/Blender-3.3-LTS.app/Contents/MacOS/Blender` on darwin → clear error otherwise.

Update [README.md](README.md) so the four-step sequence becomes `facehugger.py` invocations.

---

## Transition T — yaml → CAD as joint source-of-truth

Once Phase B's CAD-joint consumption (B5) is verified byte-equivalent against the prior URDF (`diff` empty or float-only), drop yaml `leg_template.joints[]`. Generator requires `joints` block from then on. yaml shrinks to leg-instance + servo config only.

Final yaml shape:

```yaml
robot_name: facehugger
mesh_dir: exported_meshes/
base_link: { name: base_link, mesh: QuadrupedBody.stl }
leg_template:
  leg_assembly_occurrence: "FaceHuggerLegAssembly:1"
  links:
    link1: { mesh: "leg_shoulder_{side}.stl" }
    link2: { mesh: "leg_upper.stl" }
    link3: { mesh: "leg_lower.stl" }
legs: [ ... per-leg from B2 ... ]
servo: { mass_kg: 0.060, effort_nm: 2.94, velocity_rad_s: 5.0,
         visual_flip_rpy_deg: [180, 0, 0] }
```

Update [README.md](README.md) troubleshooting: **"wrong joint limits in URDF" → edit the joint in Fusion, re-export** (not yaml).

---

## Order of execution from here

1. **Phase J** — `simulate_v2.py` FK rewrite. Unblocks stance + walk + IK on the new URDF.
2. **Phase C** — Blender visualizer (independent; can land any time).
3. **Phase D** — CLI orchestrator (independent; can land any time).
4. **Transition T** — Drop yaml joint config (now safe — Phase B already wires CAD-joint consumption).
5. **Optional fix** — `axis_construction_name` lookup for AsBuiltJoints (cosmetic; direction vectors are already correct).

---

## Quick reference — files

| File | Role |
|---|---|
| [PIPELINE_SPEC.md](PIPELINE_SPEC.md) | Authoritative spec (rpy_z_deg, limits, mount points, mirror plane). |
| [ASSEMBLY_HIERARCHY.md](ASSEMBLY_HIERARCHY.md) | Authoritative CAD tree (component / body / construction-point names). |
| [phase0_verify.py](phase0_verify.py) | Static check that the JSON matches the spec. |
| `fusion_export.json` | Generated by the Fusion add-in. Source of truth for downstream. |
| `fusion_export.txt` | Human-readable mirror of JSON, with `=== Joints ===` block. |
| [facehugger_config.yaml](facehugger_config.yaml) | Per-leg placement config (id, mount, side, rpy_z, limits, neutral). |
| [generate_urdf.py](generate_urdf.py) | Reads JSON + yaml → emits `facehugger.urdf`. |
| [simulate_v2.py](simulate_v2.py) | Loads URDF in PyBullet. |
| [../../animation/scripts/visualize_fusion_export.py](../../animation/scripts/visualize_fusion_export.py) | Blender debug scene. |
| [../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py](../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py) | The Fusion add-in. |

User reruns the Fusion add-in via *Shift+S → Scripts and Add-Ins → ExportBodiesToURDF → Run* whenever the CAD changes meaningfully.
