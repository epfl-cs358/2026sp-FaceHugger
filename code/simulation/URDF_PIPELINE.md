# How the FaceHugger URDF pipeline works

## TL;DR

> Each STL is given a **mesh-local origin at a meaningful CAD construction
> point**, so the URDF generator can place it with a single `<origin xyz>`
> (no per-mesh rotation guesswork). Construction points double as
> **placement anchors** (where to put each mesh) and **joint origins**
> (where rotation axes pivot). Construction axes give the **direction**
> of each rotation. Everything is composed via 3×3 rotation matrices and
> 3-vector translations — no quaternions, no inverse kinematics, just
> linear algebra.

This document explains the design end-to-end. Read it once, then refer
to specific sections as needed.

---

## 1. The big idea: meshes know where to attach themselves

Most URDF authors build a robot by writing each link's geometry origin
(`<origin xyz="..." rpy="..."/>`) by hand. That's tedious and
error-prone — you have to compute, for each link, where the mesh's
geometric origin happens to be relative to the joint that attaches it.

We flip this. **The exporter pre-shifts every STL's vertices** so that
the mesh's local `(0, 0, 0)` already coincides with a meaningful
attachment point — typically the CAD construction point that the next
joint pivots around. Then in the URDF, every mesh visual is just:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry><mesh filename="exported_meshes/leg_upper.stl"/></geometry>
</visual>
```

The `<origin>` is identity; all the placement work has already been
done at export time.

The "meaningful attachment point" for each STL is its
`origin_landmark`:

| STL                     | Origin landmark                | Meaning                           |
| ----------------------- | ------------------------------ | --------------------------------- |
| `QuadrupedBody.stl`     | (none — already at origin)     | chassis frame = world frame       |
| `leg_mount_{L,R}.stl`   | `LegMountFixedPoint`           | bracket-side mating point         |
| `leg_shoulder_{L,R}.stl`| `BodyToLink1Point`             | shoulder rotation axis position   |
| `leg_upper.stl`         | `Link1ToLink2Point`            | hip rotation axis position        |
| `leg_lower.stl`         | `Link2ToLink3Point`            | knee rotation axis position       |
| `servo.stl`             | `ServoMountPoint`              | top of servo (base of shaft)      |

So when the URDF says "place `leg_upper.stl` at the FL hip joint
location," the mesh's `(0, 0, 0)` lands on `Link1ToLink2Point` in
world — exactly where the next joint pivot is. Perfect alignment by
construction.

---

## 2. Construction points: anchors and joint origins

Construction points in CAD have **two distinct roles** in this
pipeline. Same name, same physical point, different consumers.

### Role A — Mesh placement anchors

The exporter's combined-rule code subtracts the landmark's world
position from every vertex. Result: mesh-local `(0,0,0)` ≡ landmark
world position.

When the URDF generator places the mesh on a link, it just sets
`<visual><origin xyz>` to the landmark's world position. Done.

### Role B — Joint origins for the kinematic chain

Fusion's revolute joints carry a `geometryOrOriginOne` (or
`.geometry` for as-built joints) which is set to a construction point.
We capture this in `fusion_export.json`'s `joints[]` array:

```json
{
  "name": "Link2Revolute",
  "axis_origin_local_mm": [-50.92, 34.565, 10.246],
  "axis_dir_local_unit":  [0, 1, 0],
  "limits_rad": { "min": -2.36, "max": +0.79 },
  ...
}
```

The URDF generator uses `axis_origin_local_mm` as the joint's
`<origin xyz>` (relative to the parent link's frame). So the joint
**pivots exactly where its construction point sits** — and because the
mesh is also re-origined to that point, the link's frame attaches to
the mesh's `(0, 0, 0)` automatically.

This duality is what makes the system "click together." Move the
construction point in CAD → both the joint pivot AND the mesh origin
follow.

---

## 3. Construction axes: rotation directions

A revolute joint also needs a *direction* for its rotation. Fusion
revolute joints reference a construction axis (like `BodyToLink1Axis`,
`Link1ToLink2Axis`, `Link2ToLink3Axis`) as the rotation axis.

The exporter captures the axis's unit-vector form into
`axis_dir_local_unit`. The URDF generator emits it as
`<joint><axis xyz="..."/>`.

For the FaceHugger:

| Joint           | Construction axis (CAD)  | `axis_dir` after normalization |
| --------------- | ------------------------ | ------------------------------ |
| Shoulder        | `BodyToLink1Axis` (Z)    | `(0, 0, 1)`                    |
| Hip             | `Link1ToLink2Axis` (X)   | `(0, 1, 0)`                    |
| Knee            | `Link2ToLink3Axis` (X)   | `(0, 1, 0)`                    |

(Note that hip/knee axes were `+X` in leg-assembly-local before
normalization. After applying `R_la`, they become `+Y` in
world-aligned coordinates. See §5.)

---

## 4. The matrix math, in one place

There are exactly four kinds of 3D transforms in this pipeline:

| Symbol      | Meaning                                                                |
| ----------- | ---------------------------------------------------------------------- |
| `T_world`   | Per-occurrence 4×4 world transform from `world_transform_rm_cm`        |
| `R_la`      | Leg-assembly's 3×3 world rotation (currently 90° about Z in this CAD)  |
| `R_role`    | Per-servo world rotation (`R_servo1`, `R_servo2`, `R_servo3`)          |
| `Rz(rpy_z)` | Per-leg back-of-pair flip (0 for FL/FR, π for BR/BL)                   |

Everything else is composition of these.

### 4a. Mesh export — bake world transforms into vertices

Combined-rule export, for each part:

```python
for vertex_body in body_local_vertices:
    vertex_world = R_world @ vertex_body + t_world      # 3×3 · 3 + 3
    vertex_mesh  = vertex_world - landmark_world         # 3-vector subtract
```

After all parts are written, the mesh has vertices in world frame
relative to the landmark. Mesh-local axes ≡ world axes.

### 4b. Leg-assembly normalization (`R_la`)

The CAD has `FaceHuggerLegAssembly:1` placed with a 90°-about-Z world
rotation. That means joint `axis_dir_local_unit` and
`axis_origin_local_mm` (which Fusion reports in the *owner component's*
local frame) are 90° off from world. We absorb that at export time:

```python
# In collect_joints():
if owner_component == "FaceHuggerLegAssembly":
    axis_dir   = R_la @ axis_dir_local
    axis_origin = R_la @ axis_origin_local
```

After normalization, the JSON's "leg-assembly local" values are
already in world-aligned coordinates. The URDF generator can use them
verbatim — no conditional rotation logic.

### 4c. Per-leg shoulder placement (corner symmetry)

Each leg's shoulder joint pivots at `BodyToLink1Point` for that
corner. We don't have `BodyToLink1Point` in CAD for FR/BR/BL; we
derive them from the source-FL placement plus the chassis-corner
geometry:

```python
# Constants (from JSON, source FL placement):
L_offset = BodyToLink1Point_world − MotorMount.LegMountFixedPoint_world
         = (-16.4, -2.78, -35.0) mm
R_offset = BodyToLink1Point_world − MotorMountR.LegMountFixedPoint_world
         = (+16.4, -2.78, -35.0) mm     # X-mirror of L_offset

# Per leg:
side_offset    = L_offset if side == "L" else R_offset
rotated_offset = Rz(rpy_z_deg) @ side_offset
shoulder_xyz   = LegMountPointXX_world + rotated_offset
```

That's the entire per-leg shoulder math. Three lines. The hip/knee
joints below use leg-assembly-local deltas (with X-flip for R pair —
see §5b) and inherit the per-leg orientation through the kinematic
chain.

### 4d. Per-role servo orientation

The shared `servo.stl` is exported from `Servo_Mouser_Model:1` (the
shoulder servo) and inherits *its* world rotation. Mesh-local `+Z` =
world `+Z` = shoulder shaft direction.

For hip / knee placements, the servo body should have shaft along
world `+Y` (matching the hip and knee joint axes). We rotate the mesh
to match each role's CAD orientation:

```python
M_role = R_role @ R_shoulder.T          # 3×3 · 3×3

# In rpy form (URDF Z·Y·X Euler decomposition):
M_hip  = Rx(-π/2)             →  rpy = (-π/2,  0,    0)
M_knee = Rz(-π/2) · Ry(-π/2)  →  rpy = ( 0,   -π/2, -π/2)
```

These rpy values are *literally* the visual `<origin rpy>` on each
hip-servo / knee-servo `<visual>` block. The shoulder servo gets
identity rpy because the mesh already matches.

---

## 5. The diagonal-pair scheme — same mesh, four placements

The robot has four legs but only one leg modeled in CAD (the FL leg).
We instantiate it four times via two symmetries.

### 5a. Diagonal pairs (mesh handedness)

| Leg | Mesh side | Why                                                              |
| --- | --------- | ---------------------------------------------------------------- |
| FL  | L         | source CAD placement                                              |
| BR  | L         | back-of-FL diagonal (180°-rotated about its `LegMountPointBR`)    |
| FR  | R         | mirror of source CAD about `FaceHuggerLegAssembly`'s XZ plane     |
| BL  | R         | back-of-FR diagonal                                               |

The R mesh (`leg_shoulder_R.stl`) is exported from the user's mirrored
`Link1R` body. Link2 / Link3 / servos are **shared** — same STL on
both sides.

### 5b. Joint origin X-flip for R pair

`Link1ToLink2Point` and `Link2ToLink3Point` live in the L assembly.
For the R pair, the leg geometry is mirrored about the leg-assembly
XZ plane (`y_la → -y_la`). After `R_la` normalization, that maps to a
**world X-flip**. So:

- L pair: hip joint origin in link1 frame = `(-50.92, +22.6, +14.4)` mm
- R pair: hip joint origin in link1 frame = `(+50.92, +22.6, +14.4)` mm

Same for knee. The URDF generator does `offset[0] = -offset[0]` for R
pair.

### 5c. R-pair joint axis flip (Phase H)

Without intervention, the R pair joint convention would be inverted
from the L pair (same numerical angle = opposite physical motion).
We negate the joint axis sign for R pair to keep the convention
uniform:

```python
if side == "R":
    axis_dir = [-a for a in axis_dir]   # +Y → -Y
    lim_lo, lim_hi = -lim_hi, -lim_lo   # negate-and-swap limits
```

Result: `hip = -40°, knee = -60°` drops *every* leg uniformly.

### 5d. Back-of-pair Rz(π) cascade

For BR and BL, the leg is rotated 180° about the chassis Z-axis at
the corner's `LegMountPointXX`. We encode this as
`shoulder_joint.rpy = (0, 0, π)`. The kinematic chain propagates the
rotation through link1 → link2 → link3 automatically; no per-link
adjustment needed.

### 5e. Link2 / Link3 visual rpy for R pair

Shared meshes (Link2L body) extend in the mesh's `-X` direction. For
L-pair legs that's the correct chassis-out direction. For R pair we
need them to extend in `+X`, so we apply `<visual><origin rpy="0 π
0">` on link2 and link3 of the R pair. The mesh rotates 180° about
Y; knee-end goes from `-X` to `+X`. Inertial CoM rotates with the
mesh.

---

## 6. The whole pipeline, end to end

```
┌──────────────────────────────────────────────────────────────────┐
│  Fusion 360 CAD                                                   │
│    bodies, occurrences, construction points/axes, revolute joints │
└───────────────┬───────────────────────────────────────────────────┘
                │ ExportBodiesToURDF.py (Fusion add-in)
                │
                │  • EXPORT_RULES → 8 STLs, each re-origined to a
                │    construction-point landmark (combined-rule
                │    bakes world transforms into vertex coords).
                │
                │  • collect_joints() walks Component.joints +
                │    Component.asBuiltJoints, captures axis_dir +
                │    axis_origin + limits per joint, and applies
                │    R_la to leg-assembly-internal joints.
                │
                │  • Construction points / axes captured with their
                │    pos_world_mm / dir_world.
                │
                ▼
┌──────────────────────────────────────────────────────────────────┐
│  fusion_export.json + fusion_export.txt                            │
│    occurrence tree (with world transforms),                       │
│    joints[] array (normalized),                                   │
│    leg_assembly_normalization metadata (R_la),                    │
│    mesh_files manifest (origin shifts).                           │
└───────────────┬───────────────────────────────────────────────────┘
                │ generate_urdf.py
                │
                │  • Reads facehugger_config.yaml for per-leg
                │    placement (mount_point, side, rpy_z_deg).
                │
                │  • For each leg:
                │      - Compute shoulder origin via §4c.
                │      - X-flip hip/knee origins for R pair (§5b).
                │      - Negate joint axis + swap limits for R
                │        pair (§5c, Phase H).
                │      - Apply mesh_rpy = (0,π,0) on link2/link3
                │        for R pair (§5e).
                │      - Emit standalone servo visuals on link1
                │        and link3 with M_hip / M_knee rpy (§4d).
                │
                ▼
┌──────────────────────────────────────────────────────────────────┐
│  facehugger.urdf                                                  │
│    1 base_link + 4 legs × (link1 + link2 + link3)                 │
│    12 revolute joints (4×{shoulder,hip,knee})                     │
│    29 visuals (1 chassis + 4 brackets + 12 link meshes +          │
│                12 standalone servos).                             │
└───────────────┬───────────────────────────────────────────────────┘
                │
                ├─→ PyBullet (simulate_v2.py, view_urdf.py)
                ├─→ Blender (visualize_fusion_export.py)
                └─→ ROS, Gazebo, etc.
```

---

## 7. What lives where (file-level summary)

| File                                                                           | Role                                                                                   |
| ------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------- |
| `cad/scripts/ExportBodiesToURDF/.../ExportBodiesToURDF.py`                     | Fusion add-in. EXPORT_RULES, collect_joints, R_la normalization, JSON+TXT output.      |
| `code/simulation/fusion_export.json`                                           | Ground-truth structured export. Authoritative for downstream tools.                    |
| `code/simulation/fusion_export.txt`                                            | Human-readable preview of the same data.                                               |
| `code/simulation/exported_meshes/*.stl`                                        | 8 re-origined meshes.                                                                  |
| `code/simulation/facehugger_config.yaml`                                       | Per-leg placement: `mount_point`, `side` (L/R), `rpy_z_deg`, shoulder limits override. |
| `code/simulation/generate_urdf.py`                                             | Reads JSON + yaml, writes URDF. All per-leg placement math lives here.                 |
| `code/simulation/facehugger.urdf`                                              | Generated robot description.                                                           |
| `code/simulation/simulate_v2.py`                                               | Full PyBullet sim with stance/IK/gait. Heavyweight — see Phase J in ALIGNMENT_FIX_PLAN |
| `code/simulation/view_urdf.py`                                                 | Lightweight viewer. `--orbit`, `--joints`, `--stance`. Use this to eyeball the robot.  |
| `code/simulation/PIPELINE_SPEC.md`                                             | What the pipeline should do (decisions, conventions).                                  |
| `code/simulation/ASSEMBLY_HIERARCHY.md`                                        | What the CAD looks like (occurrences, bodies, points, axes).                           |
| `code/simulation/ALIGNMENT_FIX_PLAN.md`                                        | The plan we executed to land all this. Has Phase J (FK rewrite) for later.             |
| `code/simulation/URDF_PIPELINE.md`                                             | This document.                                                                         |

---

## 8. When to update what

| Change in CAD                                | Action                                                |
| -------------------------------------------- | ----------------------------------------------------- |
| Move a construction point                    | Re-run Fusion export. URDF auto-updates.              |
| Edit a body's geometry                       | Re-run Fusion export. STL changes; URDF auto-updates. |
| Add a new joint                              | Re-run Fusion export. Add to `JOINTS` whitelist if needed. |
| Move `FaceHuggerLegAssembly:1` in CAD        | Re-run Fusion export. `R_la` updates automatically.   |
| Mirror a body in CAD                         | Re-run Fusion export. New STL appears.                |
| Change leg corner positions                  | Move the chassis-side construction points (e.g. `LegMountPointFL`) in CAD, re-export.  |
| Change a joint's resting limits              | Edit it in Fusion; re-export.                         |
| Change L vs R diagonal-pair assignment       | Edit `facehugger_config.yaml` `legs[].side`.          |
| Tune per-leg shoulder limits                 | Edit `facehugger_config.yaml` `shoulder_limits_deg`.  |

Conceptually: **CAD is the source of truth for geometry; yaml is the
source of truth for placement.**

---

## 9. Why this design (vs. the alternatives)

We considered three approaches before settling on this one:

1. **Hand-write the URDF**: Brittle. Every CAD change requires manual
   re-derivation of joint origins. Failed at the bracket-vs-axis 38mm
   offset in our previous round.

2. **Use a CAD-to-URDF tool like fusion2urdf or onshape-to-urdf**:
   These tools work for "one rigid body per link" assemblies. Our
   robot has shared meshes (Link2/Link3 used twice), per-leg mirroring,
   and per-corner placement — none of which these tools handle. Worth
   another look if/when the design simplifies.

3. **Generate URDF from CAD ourselves** (this design): More upfront
   investment, but each piece is small and inspectable. The URDF is
   regenerable in ~1 second. Per-leg math is centralized in
   `generate_urdf.py`'s leg loop. Adding a 5th leg or changing the
   corner geometry is a yaml edit + a re-run.

The cost of (3) is the math you've now read. The benefit is that
moving any construction point in CAD propagates to the URDF without
human intervention.
