# FaceHugger pipeline — state & plan

Living document so the next session picks up without rebuilding context. Read this first, then [PIPELINE_SPEC.md](PIPELINE_SPEC.md) (rpy/limits/mount points), [ASSEMBLY_HIERARCHY.md](ASSEMBLY_HIERARCHY.md) (CAD tree), and [URDF_PIPELINE.md](URDF_PIPELINE.md) (end-to-end how the pipeline works).

The full original sprint plan with rationale lives at `~/.claude/plans/peppy-cooking-charm.md` (historical; not maintained).

---

## Where we are

Branch: `feat/urdf-pipeline`. Latest commits, newest last:

| Commit | What |
|---|---|
| `f8671b6` | Phase 0 diagnostic + spec docs + first re-export from restructured CAD |
| `a670244` | Phase A: new `EXPORT_RULES`, joint capture, post-export checklist |
| `6716c63` | Fix `asBuiltJoints` walk + visibility-tolerant occurrence checks |
| `19be7e1` | UTF-8 file writes + ASCII-only joints formatting |
| `1e49d08` | Phase B: `generate_urdf` consumes CAD joints + per-side meshes + bracket visuals |
| `44b2036` | Alignment fix (Phase E + G + H + I): leg-assembly normalization, standalone servos, `view_urdf.py`, URDF_PIPELINE.md |
| `f988fae` | docs sync — alignment fix done, Phase J next |
| `d837e91` | Drop superseded helper scripts (phase0_verify, dump_blender_scene, inspect_stl) |
| `1fd913b` | Drop v1 simulator and URDF |
| `32bcea4` | Move pipeline docs into `docs/` |
| `b98b758` | Move generated artifacts into `generated/` |
| `62a6bd4` | Split `simulate_v2.py` into per-topic modules |
| `d8956ed` | Phase J: rewrite `kinematics.py` for new URDF |
| `0b03fcb` | Phase C minimal: adapt `visualize_fusion_export.py` to new CAD names |
| `748bec3` | Phase D: `facehugger.py` CLI entry point |
| `354f2ee` | Transition T: drop `leg_template.joints` from yaml |

---

## What's done

**Phase 0 — DONE ✓.** Verification logic was folded into the Fusion add-in's post-export message box (`_verify_against_assembly_hierarchy()` in [ExportBodiesToURDF.py](../../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py)), so the standalone `phase0_verify.py` is gone.

**Phase A — DONE ✓.** [ExportBodiesToURDF.py](../../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py) rebuilt around four explicit whitelists (`EXPORT_RULES` / `CONSTRUCTION_POINTS` / `CONSTRUCTION_AXES` / `JOINTS`) instead of CAD-visibility filtering. The full design tree lands in `fusion_export.json` so downstream tools can resolve any occurrence path; the whitelists shape specific slices (STLs, points, axes, joints). `collect_joints()` walks `Component.joints` and `Component.asBuiltJoints` across `design.allComponents`, capturing axis direction / origin / limits in radians; output goes to `joints[]` in JSON and a `=== Joints ===` block in TXT. Latest export: 8 STLs, 3 joints (`Link1Revolute -105°/+45°`, `Link2Revolute -135°/+45°`, `Link3Revolute -90°/+90°`), all checklist rows ✓.

**Phase B — DONE ✓** (commit `1e49d08`). [generate_urdf.py](../generate_urdf.py) consumes CAD-sourced joints from `fusion_export.json` (`axis_dir_local_unit`, `axis_origin_local_mm`, `limits_rad`); emits 4 bracket visuals on `base_link`, per-side L/R link1 meshes, per-leg shoulder limits from yaml.

**Phase E + G + H + I — DONE ✓** (commit `44b2036`). Two compounding bugs were silently misplacing legs in PyBullet: shoulder origin sat at the bracket's mounting tab (38 mm off the rotation axis), and the leg-assembly's 90° world rotation in CAD was never compensated. Fixed via E (exporter computes `R_la` and applies to leg-assembly-internal joint axes/origins), G (URDF gen places shoulders as `mount + Rz(rpy_z) · side_offset`, X-flips hip/knee origins for R pair, applies `mesh_rpy=(0, π, 0)` on R-pair link2/link3), H (R-pair joint axis flip + negate-and-swap limits → uniform user-facing convention across all 4 legs), I (12 standalone servo visuals with per-role rpy `M_role = R_role · R_servo1^T`). URDF kinematically verified via PyBullet FK; visually inspected via `view_urdf.py`. Math + rationale in [ALIGNMENT_FIX_PLAN.md](ALIGNMENT_FIX_PLAN.md), end-to-end pipeline in [URDF_PIPELINE.md](URDF_PIPELINE.md).

**Phase J — DONE ✓** (commit `d8956ed`). The analytic FK/IK in [kinematics.py](../kinematics.py) was calibrated against the old URDF (hip/knee axes `+X`, single sign convention, chain along `+Y`). Rewritten:

- Dropped the `R_L1` per-leg rotation layer; joint xyz/axis come straight from the URDF.
- Hip/knee rotations use `_ry`. Per-leg `hip_axis_sign` / `knee_axis_sign` (read from URDF `<axis>` Y component) keep user-facing angles uniform while internal FK applies the right rotation.
- Per-leg `joint_limits` on `LegGeom` (FL/BR have hip range `[-135°, +45°]`, FR/BL have `[-45°, +135°]`; old code clamped everyone with FR's limits — bug fixed).
- Side-dependent foot tip in link3 frame: L pair = mesh-local `FootTip` from URDF metadata; R pair = `(-x, y, -z)` (link3 visual rpy = `(0, π, 0)` rotates the mesh by `Ry(π)`).
- `body_height` recomputes from neutral-foot Z (~85.9 mm below body origin at stance).

FK→IK round-trip at stance is `[OK]` for every leg (error < 0.1°). All 4 legs land at symmetric `(±176.0, ±55.8, -85.9)` mm.

**Known workspace edge**: knee URDF limit is `±90°`; trot's `step_height=25mm` clamps at the swing apex (foot lifts ~4 mm short of commanded). Fixable by lowering `step_height` to 20 mm, more crouched stance, or bumping the CAD knee limit.

**Phase C minimal — DONE ✓** (commit `0b03fcb`). [visualize_fusion_export.py](../../../animation/scripts/visualize_fusion_export.py) updated for new CAD: mount points read as `LegMountPointXX` (with the `Point` infix), and `_legmount_rot_3x3` was replaced with the diagonal-pair table (`Rz(0°)` for FL/FR, `Rz(180°)` for BR/BL — same table the URDF generator uses). The dropped `_mat3_*` helpers / `LegMountXX:1` lookups are gone. **Known limitation**: per-side mesh handedness is not implemented — FR/BL render with L-handed meshes in Blender. See "What's left".

**Phase D — DONE ✓** (commit `748bec3`). [facehugger.py](../facehugger.py) is the single argparse-driven entry point with `urdf` / `sim` / `view` / `blender` / `all` subcommands. Each wraps the underlying script via `subprocess.run`. Multi-version Blender resolver: `--blender-version` flag (default 3.3) tries the four common `/Applications` app-name conventions, then `blender{V}` and `blender` on `$PATH`. `BLENDER_BIN` env var still wins as a full override.

**Transition T — DONE ✓** (commit `354f2ee`). yaml's `leg_template.joints` is gone; the `cad_name → urdf_name + parent/child` mapping is fixed convention, so it lives in `_JOINT_TOPOLOGY` in [generate_urdf.py](../generate_urdf.py). yaml is now strictly leg-instance + servo properties. URDF byte-equivalent before/after.

**Repo organization (housekeeping)** — DONE ✓:
- `d837e91` dropped 3 superseded helper scripts.
- `1fd913b` dropped v1 simulator and `facehugger_v1.urdf`.
- `32bcea4` moved 5 pipeline docs into `code/simulation/docs/`.
- `b98b758` moved generated artifacts (`fusion_export.{json,txt}`, `exported_meshes/`, `facehugger.urdf`) into `code/simulation/generated/`.
- `62a6bd4` split `simulate_v2.py` (877 lines) into 5 modules: `constants.py` (22), `helpers.py` (210), `kinematics.py` (~315), `gaits.py` (~315), `simulate.py` (49).

### Known gaps (not blockers)

- `axis_construction_name` / `origin_construction_name` are `None` for all three joints — Fusion's AsBuiltJoint API resolves axis-aligned construction axes to principal-axis enums rather than the originating entity. `axis_dir_local_unit` and `axis_origin_local_mm` are exact, so the URDF generator has everything it needs; this is cosmetic.
- R-side hip/knee servos not modeled in CAD. `leg_shoulder_R.stl` is just `Link1R`; FR/BL render without their hip/knee servo bodies until the user adds R servos in Fusion. Per PIPELINE_SPEC §5 this is accepted.
- `MotorMountR:1/LegMountFixedPoint` is unverifiable while the user keeps that occurrence hidden. The R bracket lands at `LegMountPointFR` / `LegMountPointBL` purely via the URDF generator's instancing logic.

### What's actually in `mesh_files` after the latest export

```
QuadrupedBody.stl   = FlexibleSkeleton:1 (chassis + LipoCage)
leg_mount_L.stl     = MotorMount:1/LegMountL,    re-origined to LegMountFixedPoint (local)
leg_mount_R.stl     = MotorMountR:1/LegMountR,   re-origined to LegMountFixedPoint (local)
leg_shoulder_L.stl  = Link1L body,               re-origined to BodyToLink1Point
leg_shoulder_R.stl  = Link1R body,               re-origined to BodyToLink1Point
leg_upper.stl       = Link2L/Link2 body,         re-origined to Link1ToLink2Point
leg_lower.stl       = Link3L/Link3 body,         re-origined to Link2ToLink3Point
servo.stl           = ServoBase,                 re-origined to ServoMountPoint
```

`_servo_role_assignment` is preserved across re-runs:

```
shoulder: FaceHuggerLegAssembly:1/Servo_Mouser_Model:1
hip:      FaceHuggerLegAssembly:1/Servo_Mouser_Model:2
knee:     FaceHuggerLegAssembly:1/Servo_Mouser_Model:3
```

---

## What's left

| Priority | Item | Effort | Notes |
|---|---|---|---|
| **High** | **Animation pipeline** — `urdf_to_blender.py` + `animation_export.py` | 1–2 sessions | Project-critical. `urdf_to_blender.py` loads `facehugger.urdf` into Blender as a rigged armature (12 controllable bones, one per joint) for keyframing. `animation_export.py` exports the timeline as `(t, 12 servo angles)` rows that the firmware on `main` can stream to its 12 PWM channels. This is what turns the simulator into actual robot motion. |
| Low | Per-side mesh handedness in `visualize_fusion_export.py` (C3 + C4) | ~2 hours | Currently FR/BL show L-handed meshes. Either implement properly (route Link1L vs Link1R + flip Link2/Link3 visuals + add bracket visuals) or drop the script in favor of `urdf_to_blender.py` once the latter exists. |
| Low | Gait registry expansion: `bound`, `crab` from teammate's `main` | ~30 min | Phase J's uniform IK means teammate's `splayed_foot_ik` / `leg_ik_fixed_yaw` aren't strictly needed — vanilla `ik_v2` per leg with the right `foot_target()` shape suffices. |
| Low | Gait parameter tuning | ~30 min visual | Trot's 25 mm step_height clamps at the URDF's ±90° knee limit (~4 mm short at swing apex). Drop to 20 mm, or bump the CAD knee limit. Walk hasn't been visually validated end-to-end. |
| Cosmetic | `axis_construction_name` lookup for AsBuiltJoints in the Fusion add-in | ~15 min | Direction vectors are already correct; this just gives the human-readable name in `fusion_export.txt`. |

---

## How to test

All commands assume `cd code/simulation` first.

### The CLI itself (Phase D)

```bash
python facehugger.py --help                       # 5 subcommands listed
python facehugger.py sim --help                   # --walk / --trot / --headless / --settle
python facehugger.py blender --help               # --blender-version flag present
```

### URDF generation + Transition T byte-equivalence

```bash
cp generated/facehugger.urdf /tmp/urdf_pre.urdf
python facehugger.py urdf
diff /tmp/urdf_pre.urdf generated/facehugger.urdf            # empty (or float-noise only)
grep -c '<visual>' generated/facehugger.urdf                 # >= 21
grep -c 'leg_shoulder_L.stl' generated/facehugger.urdf       # 2 (FL, BR)
grep -c 'leg_shoulder_R.stl' generated/facehugger.urdf       # 2 (FR, BL)
grep -c 'leg_mount_L.stl'    generated/facehugger.urdf       # 2
grep -c 'leg_mount_R.stl'    generated/facehugger.urdf       # 2
grep -E "axis_key|point_key|limits_deg" facehugger_config.yaml   # nothing
```

### Phase J FK→IK round-trip

```bash
python facehugger.py sim --headless 2>&1 | head -25
```

Expected in the banner:

- `body_height: 85.9 mm`
- per-leg `foot = (±176.0, ±55.8, -85.9) mm` (sign pattern per corner)
- `IK[fl] / [br] / [fr] / [bl]: ds=+0.00 dh=+0.02 dk=+0.09 [OK]` — all `[OK]`, errors well below 0.5°.

### Phase J visual stand pose + gaits

```bash
python facehugger.py view                                    # PyBullet viewer (no physics)
python facehugger.py sim                                     # gravity on, holds stance pose
python facehugger.py sim --walk                              # walk gait
python facehugger.py sim --trot                              # trot gait (knee clamps slightly at swing apex; expected)
```

`view` mouse: left-drag orbit, ctrl+left-drag pan, scroll zoom, close window or Ctrl+C to quit.

### Phase C minimal Blender visualizer + multi-version resolver

```bash
# Default: looks for Blender 3.3 LTS in /Applications
python facehugger.py blender

# Specific version — tries Blender-{V}-LTS.app, "Blender {V}.app" (with space),
# Blender-{V}.app, Blender{V}.app, then blender{V}/blender on $PATH
python facehugger.py blender --blender-version 4.2
python facehugger.py blender --blender-version 5.1 --headless --save /tmp/scene.blend

# Full override — wins over --blender-version
BLENDER_BIN=/Applications/Blender.app/Contents/MacOS/Blender python facehugger.py blender
```

Console should print `[instance_legs] per-corner rotation — FR=Rz(+0°), BR=Rz(+180°), BL=Rz(+180°)`.

Known visual quirk: FR / BL legs render with L-handed meshes (no per-side mesh routing yet — see "What's left"). If Blender isn't found, the CLI prints which paths were tried and exits non-zero.

### Modularization sanity check

```bash
python -c "import constants, helpers, kinematics, gaits, simulate; print('imports OK')"
```

No circular imports. Module sizes ~22 / 210 / 315 / 315 / 49 lines.

### `all` smoke run

```bash
python facehugger.py all --headless
```

Runs `urdf` → `sim` (default stand) sequentially. Useful for CI; both should exit 0.

---

## Quick reference — files

| File | Role |
|---|---|
| [PIPELINE_SPEC.md](PIPELINE_SPEC.md) | Authoritative spec (rpy_z_deg, limits, mount points, mirror plane). |
| [ASSEMBLY_HIERARCHY.md](ASSEMBLY_HIERARCHY.md) | Authoritative CAD tree (component / body / construction-point names). |
| [URDF_PIPELINE.md](URDF_PIPELINE.md) | End-to-end "how the pipeline works" doc (math + diagrams). |
| [ALIGNMENT_FIX_PLAN.md](ALIGNMENT_FIX_PLAN.md) | Historical: rationale + math for Phases E/G/H/I. |
| `generated/fusion_export.json` | Generated by the Fusion add-in. Source of truth for downstream. |
| `generated/fusion_export.txt` | Human-readable mirror of JSON, with `=== Joints ===` block. |
| `generated/exported_meshes/*.stl` | 8 re-origined meshes. |
| `generated/facehugger.urdf` | Generated robot description. |
| [facehugger.py](../facehugger.py) | CLI entry point (`urdf` / `sim` / `view` / `blender` / `all`). |
| [facehugger_config.yaml](../facehugger_config.yaml) | Per-leg placement (id, mount, side, rpy_z, limits, neutral) + servo physical props. |
| [generate_urdf.py](../generate_urdf.py) | Reads JSON + yaml → emits URDF. `_JOINT_TOPOLOGY` constant lives here. |
| [constants.py](../constants.py) | Paths + `TIMESTEP` + `STANCE_DEG`. |
| [helpers.py](../helpers.py) | Math utils, URDF/STL parsing, joint plumbing. |
| [kinematics.py](../kinematics.py) | `LegGeom`, `RobotConfig`, `fk_v2` / `ik_v2`, `build_config`. |
| [gaits.py](../gaits.py) | `GAITS` registry + `run_stand` / `run_gait`. |
| [simulate.py](../simulate.py) | argparse main only. |
| [view_urdf.py](../view_urdf.py) | Lightweight PyBullet URDF viewer (no physics). |
| [../../animation/scripts/visualize_fusion_export.py](../../../animation/scripts/visualize_fusion_export.py) | Blender debug scene (corners detected; per-side mesh routing TODO). |
| [../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py](../../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py) | The Fusion add-in. |

User reruns the Fusion add-in via *Shift+S → Scripts and Add-Ins → ExportBodiesToURDF → Run* whenever the CAD changes meaningfully.
