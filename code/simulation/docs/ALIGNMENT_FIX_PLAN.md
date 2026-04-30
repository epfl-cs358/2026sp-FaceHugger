# Alignment fix plan — export-side normalization + standalone servos

## Problem statement

Two compounding bugs put the leg meshes in the wrong place in PyBullet:

1. **Shoulder joint origin is at the mounting tab, not the rotation axis.**
   `LegMountPointXX` (chassis-side anchor at the mounting tab) is **38 mm
   away** from `BodyToLink1Point` (the actual rotation axis). The
   `leg_shoulder_*.stl` meshes are re-origined to `BodyToLink1Point`, so
   when the URDF places the joint at `LegMountPointXX`, the mesh's local
   `(0,0,0)` lands at the wrong world location.

2. **`FaceHuggerLegAssembly:1` is rotated 90° about Z relative to world.**
   Its `world_transform_rm_cm` rotation = `[[0,-1,0],[1,0,0],[0,0,1]]`,
   so leg-assembly-local `+Y` maps to world `-X`. All exported leg
   meshes and JSON `axis_origin_local_mm` / `axis_dir_local_unit` values
   are in this rotated frame; the URDF generator does not compensate, so
   meshes end up rotated 90° from where they belong, and hip/knee joint
   axes point in wrong world directions.

Verified from JSON:
- `BodyToLink1Point` world = `(-57.9, 46.22, -9.0)` mm
- `LegMountPointFL` world = `(-41.5, 49.0, 26.0)` mm
- L-offset (mount → axis, world frame) = `(-16.4, -2.78, -35.0)` mm
- `MotorMountR:1` `LegMountFixedPoint` world = `(-74.3, 49.0, 26.0)`
- R-offset = `(+16.4, -2.78, -35.0)` (X-mirrored from L-offset, as
  expected for a Y_la-mirror composed with `R_la`)

Hand-computed clean-target (post-normalization):
- Link1Revolute axis (world-aligned) = `(0, 0, 1)` ✓
- Link2Revolute axis (world-aligned) = `(0, 1, 0)` ✓
- Link3Revolute axis (world-aligned) = `(0, 1, 0)` ✓
- Hip joint origin in normalized link1 frame = `(-50.92, 22.595, 14.446)` mm

## Goal

Re-orient leg-assembly-internal data **at export time** so the JSON's
"leg-assembly local" frame is world-aligned. The URDF generator then
treats the JSON as if the leg-assembly were placed in CAD with
identity rotation. Servos are emitted as **standalone visuals** (no
bake-in to link STLs), so per-leg mirror handling is just per-leg
visual rpy with no CAD work.

No CAD changes required. User re-runs the Fusion exporter, then
`generate_urdf.py`.

---

## Phase E — Exporter changes (`ExportBodiesToURDF.py`)

### E1. Compute the leg-assembly's normalization rotation `R_la`

In `run()`, after `traverse(root.occurrences)`, find
`FaceHuggerLegAssembly:1` in the JSON occurrence tree. Extract its
`world_transform_rm_cm` 3×3 rotation block as `R_la` (row-major list of
lists). Pass `R_la` to `collect_joints()`.

If the leg-assembly cannot be located, **fail fast** — the rest of the
pipeline assumes its presence.

### E2. Convert all leg-assembly-internal body-rule entries to combined-rule, drop servo bake-ins

Combined-rule output produces world-frame vertices (origin at landmark
world position) via the existing `_transform_triangle` step. World
frame == world-aligned axes. So switching to combined-rule implicitly
normalizes mesh orientation **for free** — no extra `R_la` math on
mesh vertices.

Servos become **standalone visuals** (no bake-in). The shared
`servo.stl` is emitted as a single combined-rule for one occurrence,
and the URDF generator instances it per leg with appropriate rpy.

Final `EXPORT_RULES`:

```python
EXPORT_RULES = [
    # Chassis
    {"type": "combined", "stl": "QuadrupedBody.stl", "parts": [
        {"occurrence": "FlexibleSkeleton:1/QuadrupedBody:1", "body": "QuadrupedBody"},
        {"occurrence": "FlexibleSkeleton:1/LipoCage:1",      "body": "LipoCage"},
    ]},

    # Brackets — each landmark lives in its own bracket component, so
    # we add an optional `landmark_occurrence` field to scope the
    # landmark lookup (otherwise the L lookup would hit the first
    # LegMountFixedPoint in the tree, which may be either bracket).
    {"type": "combined", "stl": "leg_mount_L.stl",
     "origin_landmark": "LegMountFixedPoint",
     "landmark_occurrence": "FaceHuggerLegAssembly:1/MotorMount:1",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/MotorMount:1",
                "body": "LegMountL"}]},
    {"type": "combined", "stl": "leg_mount_R.stl",
     "origin_landmark": "LegMountFixedPoint",
     "landmark_occurrence": "FaceHuggerLegAssembly:1/MotorMountR:1",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/MotorMountR:1",
                "body": "LegMountR"}]},

    # Link1 — L and R variants. NO servo bake-in; standalone servos
    # come from servo.stl below.
    {"type": "combined", "stl": "leg_shoulder_L.stl",
     "origin_landmark": "BodyToLink1Point",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/Link1L:1",
                "body": "Link1"}]},
    {"type": "combined", "stl": "leg_shoulder_R.stl",
     "origin_landmark": "BodyToLink1Point",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/Link1R:1",
                "body": "Link1R"}]},

    # Link2, Link3 — shared meshes.
    {"type": "combined", "stl": "leg_upper.stl",
     "origin_landmark": "Link1ToLink2Point",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/Link2L:1",
                "body": "Link2"}]},
    {"type": "combined", "stl": "leg_lower.stl",
     "origin_landmark": "Link2ToLink3Point",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/Link3L:1",
                "body": "Link3"}]},

    # Single shared servo. Emitted as a standalone visual on each leg
    # by the URDF generator (12 visuals total: 4 shoulder + 4 hip + 4
    # knee).
    {"type": "combined", "stl": "servo.stl",
     "origin_landmark": "ServoMountPoint",
     "parts": [{"occurrence": "FaceHuggerLegAssembly:1/Servo_Mouser_Model:1",
                "body": "ServoBase"}]},
]
```

**Schema additions:**
1. `landmark_occurrence` field on combined rules — when set, the
   landmark lookup is scoped to that occurrence's `points[]`. When
   unset, the existing tree-wide walk is used.
2. (No mirror flag needed — standalone-servo design avoids it.)

Implementation hook: extend `_find_landmark_world_pos_mm` (or add a
`_find_landmark_in_occurrence_world` helper) that takes the JSON
occurrence path. Modify `export_stls()`'s combined-rule branch to
prefer this scoped lookup when `landmark_occurrence` is present.

### E3. Normalize joint axis dir and origin in `collect_joints()`

After extracting `axis_dir_local_unit` and `axis_origin_local_mm` for
each joint, apply `R_la` so they're expressed in the world-aligned
frame. Apply only to leg-assembly-internal joints (currently all 3 —
`Link1Revolute`, `Link2Revolute`, `Link3Revolute`).

```python
def _apply_R(R_3x3, v3):
    return [
        R_3x3[0][0]*v3[0] + R_3x3[0][1]*v3[1] + R_3x3[0][2]*v3[2],
        R_3x3[1][0]*v3[0] + R_3x3[1][1]*v3[1] + R_3x3[1][2]*v3[2],
        R_3x3[2][0]*v3[0] + R_3x3[2][1]*v3[1] + R_3x3[2][2]*v3[2],
    ]

# In collect_joints, for joints whose owner_component is
# 'FaceHuggerLegAssembly', rotate axis_dir + axis_origin by R_la:
joint_entry["axis_dir_local_unit"]  = _apply_R(R_la, axis_dir)
joint_entry["axis_origin_local_mm"] = _apply_R(R_la, axis_origin)
```

Also store servo positions for the URDF generator's standalone
visuals. Specifically, in the JSON's occurrence tree, `Servo_Mouser_Model:2`
and `Servo_Mouser_Model:3` already carry `world_origin_mm`. The URDF
generator reads those and computes their position in the link1 / link3
frame post-normalization (subtract the link's joint origin world,
no further rotation needed since the values are already in world frame).

### E4. Document the normalization in JSON output + txt

```json
"leg_assembly_normalization": {
  "applied": true,
  "rotation_3x3_row_major": [[...], [...], [...]],
  "owner_component": "FaceHuggerLegAssembly"
}
```

`fusion_export.txt` gets a one-paragraph note:

```
=== Leg-assembly normalization ===
Rotated leg-assembly-internal joint axes and joint origins by R_la
(the assembly's world rotation) so downstream consumers see a
world-aligned frame. Mesh STLs are produced via combined-rule which
already bakes world-frame vertices.
```

### E5. (Defer) Construction-point normalization

`pos_world_mm` is already in world frame and needs no change.
`pos_mm` (component-local) is consumed by very few downstream paths —
leave it un-normalized for now. The URDF generator should prefer
`pos_world_mm` for any post-normalization lookups.

---

## Phase G — `generate_urdf.py` simplification

After Phase E lands and the user re-exports, the URDF generator
changes:

### G1. Drop `mount_point` as the shoulder joint origin

`mount_point` in yaml stays useful as a chassis-side anchor for
**bracket visuals** on `base_link`. It is no longer used as the
shoulder joint origin.

### G2. Compute per-corner shoulder rotation-axis world positions

Read once from the JSON:

```python
mount_FL_world = LegMountPointFL.pos_world_mm   # (-41.5, +49.0, +26.0)
mount_FR_world = LegMountPointFR.pos_world_mm   # (+41.5, +49.0, +26.0)
mount_BR_world = LegMountPointBR.pos_world_mm   # (+41.5, -49.0, +26.0)
mount_BL_world = LegMountPointBL.pos_world_mm   # (-41.5, -49.0, +26.0)

axis_FL_world  = MotorMount   's LegMountFixedPoint pos_world_mm  # source CAD
axis_R_in_CAD  = MotorMountR's LegMountFixedPoint pos_world_mm   # source CAD

# Derive offsets (constant per-side; in world frame)
L_offset = (BodyToLink1Point.pos_world_mm - mount_FL_world)
R_offset = (BodyToLink1Point.pos_world_mm - axis_R_in_CAD)
        ≈ (-16.4, -2.78, -35.0) and (+16.4, -2.78, -35.0)
```

### G3. Per-leg shoulder joint placement

```python
for leg in cfg["legs"]:
    side    = leg["side"]                  # "L" or "R"
    rpy_z   = math.radians(leg["rpy_z_deg"])  # 0 or π
    mount   = mount_world_by_id[leg["id"]]
    offset  = L_offset if side == "L" else R_offset
    if rpy_z == math.pi:
        offset = (-offset[0], -offset[1], offset[2])  # Rz(π)

    shoulder_origin_xyz = (mount[0] + offset[0],
                           mount[1] + offset[1],
                           mount[2] + offset[2])

    urdf.joint(
        name=f"{leg['id']}_shoulder_joint",
        ...
        origin_mm=shoulder_origin_xyz,
        rpy_z_deg=leg["rpy_z_deg"],            # corner flip only
        axis=Link1Revolute.axis_dir_local_unit, # already (0,0,1) post-norm
        ...
    )
```

### G4. Hip/knee joint origins in normalized frame

```python
hip_origin_in_link1 = subtract(
    Link2Revolute.axis_origin_local_mm,   # post-norm
    Link1Revolute.axis_origin_local_mm,
)
# = approx (-50.92, +22.595, +14.446) mm

knee_origin_in_link2 = subtract(
    Link3Revolute.axis_origin_local_mm,
    Link2Revolute.axis_origin_local_mm,
)
# = approx (-94.7, 0, 0) mm
```

Both are emitted directly with `rpy=(0,0,0)`. Axes
(`Link2/3Revolute.axis_dir_local_unit`, both `(0,1,0)` post-norm)
emit directly.

### G5. Bracket visuals on `base_link`

4 bracket visuals on `base_link`, mesh = `leg_mount_{L,R}.stl` per
side, with:
- xyz = `LegMountPointXX_world`
- rpy = `(0, 0, rpy_z_deg)` per leg (back-of-pair flip)

### G6. Standalone servo visuals (12 total)

Read from JSON:
```python
servo_shoulder_world = Servo_Mouser_Model:1.world_origin_mm  # source position
servo_hip_world      = Servo_Mouser_Model:2.world_origin_mm  # rigid w/ Link1
servo_knee_world     = Servo_Mouser_Model:3.world_origin_mm  # rigid w/ Link3
```

Compute servo positions relative to their parent link's joint origin
**in the source-CAD placement** (which is the FL placement for L pair):

```python
hip_servo_in_link1_FL  = sub(servo_hip_world,  BodyToLink1Point.pos_world_mm)
knee_servo_in_link3_FL = sub(servo_knee_world, Link2ToLink3Point.pos_world_mm)
```

Per-leg emission:

| Visual                                | xyz                                   | rpy           |
| ------------------------------------- | ------------------------------------- | ------------- |
| 4× shoulder servo on `base_link`      | `LegMountPointXX_world` (+ servo offset from mount tab in normalized frame) | `(0, 0, rpy_z_deg)` |
| 4× hip servo on `<leg>_link1`         | `hip_servo_in_link1_FL` (X-flipped for R pair) | identity for L pair, `(0, π, 0)` for R pair |
| 4× knee servo on `<leg>_link3`        | `knee_servo_in_link3_FL` (X-flipped for R pair) | identity for L pair, `(0, π, 0)` for R pair |

### G7. Link2 / Link3 visual rpy for R pair

```python
link2_visual_rpy = (0, math.pi, 0) if side == "R" else (0, 0, 0)
link3_visual_rpy = (0, math.pi, 0) if side == "R" else (0, 0, 0)
```

`leg_shoulder_{L,R}.stl` (link1) does NOT need this rpy — they're
already side-specific meshes from the export.

---

## Phase V — Verification

After Phase E + G:

1. **STL spot-checks**:
   - `leg_shoulder_L.stl` — bounding-box max should extend in **world
     -X** direction from origin.
   - `leg_shoulder_R.stl` — bounding-box max in **world +X** direction.
   - `leg_upper.stl` and `leg_lower.stl` — long axis in world `-X`
     (the L-pair-baked direction).
   - `servo.stl` — small bounding box, roughly cylindrical, oriented
     along world Z.

2. **URDF inspection**:
   - 4 shoulder joints with origins at world `(±57.9, ±46.22, -9.0)`
     mm in the appropriate combinations:
     - FL: `(-, +, -)`  → `(-57.9, +46.22, -9.0)`
     - BR: `(+, -, -)`  → `(+57.9, -46.22, -9.0)`
     - FR: `(+, +, -)`  → `(+57.9, +46.22, -9.0)`
     - BL: `(-, -, -)`  → `(-57.9, -46.22, -9.0)`
   - Shoulder rpy = `(0, 0, 0)` for FL/FR, `(0, 0, π)` for BR/BL.
   - Hip joint origin (in link1 frame) ≈ `(-50.92, +22.595, +14.446)`
     mm for L pair, `(+50.92, +22.595, +14.446)` mm for R pair.
   - Hip + knee axes = `(0, 1, 0)`. Shoulder axis = `(0, 0, 1)`.
   - 12 servo.stl visuals: 4 on `base_link` + 4 on
     `<leg>_link1` + 4 on `<leg>_link3`.
   - Link2/Link3 visual rpy = `(0, π, 0)` for FR/BL, identity for FL/BR.

3. **PyBullet stand**:
   - `python simulate.py` — robot stands with feet on the
     ground, body upright, legs splaying outward in the four
     quadrants.
   - IK should resolve (no `[FAIL]` markers).

4. **Phase 0 verifier** — required checks still pass.

---

## Order of execution

1. **E1** — read & store `R_la`.
2. **E2** — `EXPORT_RULES` rewrite + `landmark_occurrence`
   support.
3. **E3** — rotate joint `axis_dir` and `axis_origin` in
   `collect_joints()`.
4. **E4** — write the normalization metadata.
5. **User re-exports** in Fusion. Spot-check STLs (V1).
6. **G1–G7** — `generate_urdf.py` rewrite.
7. **User regenerates URDF**. Inspect URDF (V2).
8. **PyBullet test** (V3).

### Phase H — Fix B: uniform joint sign convention across legs (post-G)

After G lands, R-pair joint angles for hip/knee require opposite sign
from L-pair to produce the same physical motion (because Link1R is
mirrored, the hip/knee origins are X-flipped, and the shared joint
axis = +Y in normalized frame). This forces every downstream consumer
(simulate, IK, gait controllers) to do per-side sign flips.

**Fix B**: in `generate_urdf.py`, when emitting hip + knee joints for
the R pair (FR, BL), negate the joint axis (`(0, 1, 0)` → `(0, -1, 0)`)
and negate-and-swap the joint limits. This makes the URDF describe the
sign convention so downstream code stays uniform.

Verified via PyBullet FK: with `hip=-40°, knee=-60°` applied to all
four legs, every link3 frame lands at world Z = -55.4 mm.

### Phase I — Standalone-servo placement (post-G)

The combined-rule export drops servo bake-ins (Phase E2). The URDF
generator emits 12 standalone servo visuals:

- 4 shoulder servos on `base_link`, one per leg, at
  `mount_world + Rz(rpy_z) · side_offset` with rpy = `(0, 0, rpy_z_deg)`.
- 4 hip servos on `<leg>_link1`, at `hip_servo_offset_in_link1` (X-flipped
  for R pair), rpy = `(0, 0, 0)`.
- 4 knee servos on `<leg>_link3`, at `knee_servo_offset_in_link3` (X-flipped
  for R pair), rpy = `(0, 0, 0)`.

**No mirror-approximation rotations on the servo visuals.** Servo is
the same physical part on every leg; we only translate it (and let the
parent link's frame inherit the back-of-pair `Rz(π)` cascade). The
CAD's L-bracket vs R-bracket nested servos differ in world rotation
(`R_R · R_L^T = Ry(π)`), but applying that to the visual would flip
the servo's Z direction (its shaft) — geometrically wrong for a
real-world unflippable servo body. Visual fidelity is sacrificed to
preserve the "single physical part" semantics.

### Phase J — `simulate.py` FK rewrite (later)

Once the URDF is geometrically correct, `simulate.py`'s FK/IK
needs updating to consume the new conventions. Specifically:

1. **Drop the `R_L1` per-leg rotation layer** (lines ~538–565). With
   shoulder rpy now encoding per-leg orientation directly, downstream
   FK can use the URDF kinematic chain as-is.
2. **Switch hip/knee axis from old `+X` (leg-assembly local) to
   normalized `+Y`** in any analytic FK code.
3. **Sign convention is uniform across legs** thanks to Phase H, so
   FK/IK can be one function — no per-side branching.
4. **Foot tip is side-dependent** because link2/link3 have
   `mesh_rpy = (0, π, 0)` for R pair. The mesh-local foot-tip vector
   `(x, y, z)` becomes `(-x, y, -z)` in URDF link3 frame for R pair.
   IK target should respect this.
5. **Stance height re-calibration**: with the new geometry, foot tip
   world Z at `hip=-40°, knee=-60°` is approximately -85 mm relative
   to body origin. Default `body_height: 1 mm` puts feet below the
   ground at start-up — bump initial body z to ~90 mm so PyBullet
   doesn't fight an interpenetration condition before settling.

This is **deferred** until the URDF + servo work is validated visually.

---

## Non-goals

- Not changing CAD. The 90° rotation of `FaceHuggerLegAssembly:1`
  stays; we absorb it in the export. No 4th servo, no Link3R, no
  rigid-group additions.
- Not touching the chassis (`FlexibleSkeleton:1`) export — its world
  transform is identity.
- Not creating per-corner CAD instantiations — the URDF generator
  produces 4 legs from one CAD source-FL placement plus the corner
  symmetry rules in `facehugger_config.yaml`.
- Not addressing the Blender visualizer (Phase C in the older
  pipeline plan) in this fix — Phase C lands separately once the
  URDF format settles.

---

## Risks

- **`landmark_occurrence` parsing**: must walk the path
  (`FaceHuggerLegAssembly:1/MotorMount:1`) and only look at that
  occurrence's `points[]`. Mirror the `_find_occ_and_world_transform`
  pattern.
- **Mirror-component combined rule**: Link1R has its own
  `world_transform_rm_cm` that includes the mirror. The existing
  combined-rule code applies that transform via
  `_find_occ_and_world_transform` + `_transform_triangle`, so a
  Y-mirror in leg-assembly-local frame composed with `R_la` produces
  the correct R-mesh in world-aligned coords. No special handling
  needed.
- **Knee/hip servo position visual precision**: the per-leg
  standalone-servo placement uses 180° rotations as a stand-in for
  true mirrors. For typical roughly-axisymmetric servo bodies, this
  is visually indistinguishable. If a future servo model has a
  prominent off-axis feature, switch to a per-pair STL bake.
