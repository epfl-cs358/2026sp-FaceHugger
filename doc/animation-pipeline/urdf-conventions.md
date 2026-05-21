# URDF Conventions for the Animation Pipeline

The URDF is the single source of truth for kinematic geometry — it
feeds the simulation IK, the firmware IK (via the `leg_geom.h`
generator in T6), the Blender rig builder, and any future RL or
physics-simulator handoff. This document captures the conventions
the URDF must follow for all of those consumers to agree.

Authoritative neighbouring docs:

- [`code/simulation/docs/MERGE_AND_CONVENTION.md`](../../code/simulation/docs/MERGE_AND_CONVENTION.md)
  — joint-angle convention (Phase H, axis-sign uniformization).
- [`animation/SERVO_ID_CONVENTION.md`](../../animation/SERVO_ID_CONVENTION.md)
  — servo numbering authority.
- [`leg-coordinates.md`](leg-coordinates.md) — IK math + how the URDF
  feeds it.

---

## URDF joint origin convention

Joint `<origin>` must be placed at the **physical rotation axis** —
the servo shaft. This is the standard ROS/URDF convention and what
physics simulators (Gazebo, MuJoCo, Isaac Gym) require for correct
dynamics.

The `<visual><origin>` offset on the child link then shifts the mesh
to appear in the right place relative to the joint frame, without
affecting kinematics.

For this robot's leg structure:

```xml
<!-- Yaw joint: origin at yaw servo shaft, Z = rotation axis -->
<joint name="leg_fl_yaw" type="revolute">
  <origin xyz="hip_x hip_y hip_z"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="..." velocity="..."/>
</joint>

<!-- Hip pitch joint: origin at hip servo shaft -->
<!-- w_y offset here is the real lateral distance between yaw axis
     and hip shaft -->
<joint name="leg_fl_hip" type="revolute">
  <origin xyz="0 w_y 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="0.2" effort="..." velocity="..."/>
</joint>

<!-- Knee pitch joint: origin at knee servo shaft -->
<joint name="leg_fl_knee" type="revolute">
  <origin xyz="0 0 -l2"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="0.0" effort="..." velocity="..."/>
</joint>
```

The `w_y` value is **not a workaround** — it is the physically real
lateral offset between the yaw servo shaft and the hip servo shaft.
`ik_v2` accounts for it analytically (see
[`leg-coordinates.md` §3](leg-coordinates.md)). It is read from the
URDF by T6 and compiled into `LegGeom.w_y`.

---

## Frames, axes, and angle sign convention

This section is intentionally explicit — every angle is named together
with the frame it is expressed in, and every axis is named together
with the physical feature it points along, so there is no ambiguity
when extending or debugging the pipeline.

### Frames

- **Body frame** (≡ world frame when the robot is at rest): origin at
  the chassis centroid, located at URDF `base_link`.
  - **+X** = robot's right side
  - **+Y** = robot's front
  - **+Z** = up
- **Mount-point frame** (4 of them, one per leg corner): origin at the
  leg's `LegMountPointXX` construction point on the chassis. Orientation
  = body frame composed with that leg's *geometric body-Z rotation*
  (0° for the front pair `FL`/`FR`, 180° for the back pair `BL`/`BR`).
  This frame is rigidly attached to `base_link`; nothing about it
  depends on joint angles.
- **Joint frame** (one per revolute joint): origin at the joint's
  rotation axis. Orientation = parent-link frame composed with the
  URDF `<origin rpy>` of that joint. **At joint angle θ = 0 the child
  link's frame coincides with the joint frame** — that is the
  definition of "rest" for that joint.

### Reuse of physical parts across the four legs

Only **two bracket variants** and **two Link1 variants** exist in CAD
(L-shape and R-shape, where R is the YZ-plane mirror of L). Link2 and
Link3 are **shared across every leg** — there is only one `Link2L` and
one `Link3L` mesh in the export. The four corners are reached by
combining mirror-status with a body-Z rotation of the mount-point
frame:

| Leg | Bracket | Link1 | Link2 | Link3 | Mount-point frame body-Z rotation |
|-----|---------|-------|-------|-------|-----------------------------------|
| FL  | L-shape | L     | L     | L     | 0°   |
| BR  | L-shape | L     | L     | L     | 180° |
| FR  | R = L mirrored about body's YZ plane | R | L (rotated 180° about own Y) | L (rotated 180° about own Y) | 0°   |
| BL  | R       | R     | L (rotated 180° about own Y) | L (rotated 180° about own Y) | 180° |

The "rotated 180° about own Y" for Link2/Link3 of the R-pair is the
`link_mesh_rpy` dict in
[`generate_urdf.py`](../../code/simulation/generate_urdf.py) (search for
`link_mesh_rpy` — it sets `mesh_rpy = (0, π, 0)` on link2/link3 when
`side == "R"`). It is applied to the visual mesh **and** to the
inertial CoM so the leg geometry extends in the right direction on the
right side of the body.

Restating the geometric reasoning that justifies this part reuse:

- **FL → BR** (same parts): when the L-bracket's mount-point frame is
  rotated 180° about body Z, the surface that faced the front of the
  robot now faces the back, and the surface that faced the body's
  outside (left for FL) now faces the body's outside on the opposite
  diagonal corner (right for BR). The L-bracket therefore fits BR
  unchanged.
- **FL → FR** (mirror of bracket + Link1): only the chirality-bearing
  parts (bracket and Link1) need a CAD-level mirror. Link2 and Link3
  are chirality-neutral and can be reused, provided they are spun 180°
  about their own Y axis to undo the orientation mismatch introduced
  by attaching them to the mirrored Link1.
- **FR → BL** (same parts as FR): same logic as FL → BR — a 180°
  body-Z rotation of the mount-point frame moves the FR layout to the
  BL corner.

### Joint axes — what direction they point and what positive θ means

By URDF (and ROS) convention, every revolute joint declares an axis
vector inside `<axis xyz="...">`, expressed in the joint frame. The
joint angle θ is **right-hand-rule rotation about that axis vector**:
curl the fingers in the direction of rotation, the thumb points along
the axis vector.

This robot uses the following physical conventions for the three
joint axes per leg:

| Joint                      | Axis direction (physical) | Axis vector in joint frame (L-side) | Axis vector in body frame at rest (L-side) | Positive θ (RHR) |
|----------------------------|---------------------------|-------------------------------------|--------------------------------------------|------------------|
| Shoulder (`Link1Revolute`) | along the shoulder servo shaft, from servo base **bottom toward horn** — physically vertical | `(0, 0, 1)` | body **+Z** (up) | leg yaws CCW viewed from above |
| Hip (`Link2Revolute`)      | along the hip servo shaft, from base bottom toward horn — physically along the leg's longitudinal direction at rest | `(1, 0, 0)` | body **+Y** at rest (forward, since the L-side leg extends along +Y at rest) | foot lifts up (the leg pitches up about the hip) |
| Knee (`Link3Revolute`)     | along the knee servo shaft, base bottom toward horn — same direction as hip at rest | `(1, 0, 0)` | body **+Y** at rest | knee flexes, foot lifts |

The "axis points from servo base bottom toward servo horn" rule is
baked into the Fusion CAD: every servo's mount frame is constructed
with its local +Z exiting through the horn, so the joint
`axis_dir_local_unit` captured by the export already points that way.

> **Sign mnemonic for hip / knee:** with the axis along bottom→horn,
> positive θ rotates the foot **up** (toward the chassis), negative θ
> drops it **down** (away from the chassis). This is invariant across
> legs once the R-side correction below is applied.

### Why R-side legs (FR + BL) need axis negation

Mirroring the L-bracket about the body's YZ plane to make the R-bracket
flips the chirality of that bracket. After mirroring, the same physical
servo, when bolted into the R-bracket, has its **base-bottom-to-horn
direction reversed in body frame** compared to the L-side mounting.
That is the geometric meaning of the user-side observation: "from back
to front the vector is pointing differently — that's why for FR and BL
the servos 2 and 3 flip their angles."

If we left every leg with the same axis vector
`(0, 1, 0)` in body frame for hip / knee, then for FR / BL the URDF's
axis vector would point in the **opposite** direction of physical
bottom→horn. By RHR, the same `θ_hip = +30°` command on FL (foot up)
would push the FR foot **down**, because the URDF axis is reversed
relative to the physical convention on that leg.

To restore the property that **the same θ value produces the same
physical motion on every leg**, the URDF generator negates the joint
axis on the R-side at
[`generate_urdf.py:1007`](../../code/simulation/generate_urdf.py#L1007):

```python
if side == "R":
    axis_dir = [-a for a in axis_dir]
```

After this flip, the URDF's axis once again points along physical
bottom→horn for every leg. RHR with that axis then makes positive θ
mean "lift" everywhere, and IK / gait / animation can stay
leg-agnostic.

### Why the limits also need a negate-and-swap

URDF joint limits are signed bounds on θ: `lower ≤ θ ≤ upper`. The
sign is interpreted in the joint's axis convention. When we flip the
axis vector (R-side correction above), the sign convention flips along
with it: a configuration that was `θ = +45°` under the un-flipped axis
is `θ = −45°` under the flipped axis.

The bounds therefore transform as:

```
new_lower = -(old_upper)
new_upper = -(old_lower)
```

That is the negate-and-swap pattern at
[`generate_urdf.py:1010`](../../code/simulation/generate_urdf.py#L1010):

```python
if lim_lo is not None and lim_hi is not None:
    lim_lo, lim_hi = -lim_hi, -lim_lo
```

Combined with the axis flip, the URDF expresses the same physical
sweep range on every leg, with consistent θ semantics. This is what
makes "the same θ value produces the same physical motion" hold even
when the Fusion limits are asymmetric (e.g. `[−135°, +45°]`).

### Shoulder asymmetry: same logic, currently incomplete

The shoulder (`Link1Revolute`) has its own subtlety. Convention A in
[`_shoulder_rest_for`](../../code/simulation/generate_urdf.py#L152-L176)
sets each leg's shoulder `<origin rpy>` to a different yaw so that
`θ_shoulder = 0` always means **this leg is at its mechanical zero**.
But Convention A does **not** flip the shoulder axis or
negate-and-swap the shoulder limits on the R-side.

When the FL shoulder window happens to be symmetric (e.g. `±90°` about
FL's mechanical zero — true with the previous CAD's `fl_rest = −45°`
and Fusion limits `[−135°, +45°]`), the asymmetry-correction is
invisible because `[−90°, +90°]` is its own negate-and-swap.

When the window becomes asymmetric — as in the current export with
`fl_rest = 0°` and Fusion limits `[−135°, +45°]` — applying
`[−135°, +45°]` identically to every leg gives FR and BL a sweep range
that is mirrored relative to FL/BR. The fix is the same three-flip
pattern hip and knee already use:

1. Negate the shoulder axis on R-side.
2. Negate-and-swap the shoulder limits on R-side.
3. The shoulder origin position does **not** need an X-flip — it is
   already computed correctly from `LegMountPointXX_world + Rz(rpy_z) ·
   side_axis_offset` in `generate_urdf.py`.

---

## URDF requirements for RL / physics-simulator handoff

For a simulation or RL framework to load and simulate the robot
correctly, the URDF must include:

- **`<inertial>` per link**: `<mass>` and `<inertia>` tensor.
  Without these, the simulator uses default/zero inertia and produces
  garbage dynamics. PyBullet and Gazebo will silently accept missing
  inertials with default values; MuJoCo errors out.
- **`<limit>` per revolute joint**: `lower`, `upper` (rad), `effort`
  (N·m), `velocity` (rad/s). These are used by RL frameworks to clamp
  actions and define the action space.
- **Mesh scale in metres** (ROS convention). If STLs are exported
  from Fusion in mm, use `<mesh scale="0.001 0.001 0.001"/>` on each
  `<visual>` and `<collision>`.

> **TODO:** Verify inertial properties are populated before handing
> the URDF to anyone doing RL or simulation. A URDF with missing
> `<inertial>` tags will load without error but simulate
> incorrectly. Check
> [`code/simulation/generated/facehugger.urdf`](../../code/simulation/generated/facehugger.urdf)
> by `grep -c '<inertial>'` — should equal the number of links.

---

## Stability invariants — what must NOT change

The following are **coordinate contracts** between the URDF and every
stored `.fhc` clip. Changing any of these invalidates all stored
clips:

1. **`base_link` origin position in Fusion.** Anchored to a named
   construction point per
   [`leg-coordinates.md` §1](leg-coordinates.md). This is the body
   frame origin for every clip.
2. **Per-leg shoulder mount positions** (URDF `leg_*_yaw`
   `<origin>`). Changing these moves the leg's coordinate origin and
   shifts every stored foot position relative to it.
3. **Joint axis directions.** Changing a joint's `<axis>` flips the
   sign of stored joint angles, which the IK won't catch (it just
   produces unreachable targets).
4. **Link lengths** (`L1.x`, `L2.x`, `foot_L3` distances).
   Changing the geometry changes the reachable workspace; old clips
   may now be unreachable or differently shaped.

What's **safe** to change:

- `<visual>` origins and meshes (cosmetic — won't affect IK).
- `<collision>` geometry (only affects physics simulation, not IK).
- `<inertial>` values (only affects dynamics, not kinematics).
- Joint `<limit>` ranges (clamps tighten/loosen, but stored clips
  in the old range are still reachable).

When any of the "must NOT change" items must change anyway:

1. Bump the `.fhc` magic to `FHC2`.
2. Re-export every `.fhc` from the source `.blend` files with the new
   URDF.
3. The firmware boot loader rejects `FHC1` files when running against
   the new URDF (catches the version mismatch loudly).

---

## Cross-references

- [`leg-coordinates.md`](leg-coordinates.md) — design canon, including
  IK math that consumes URDF values.
- [`animation-pipeline-roadmap.md`](animation-pipeline-roadmap.md) —
  T6 generates `leg_geom.h` from the URDF.
- [`firmware-research.md`](firmware-research.md) — ESP32 IK budget.
- [`code/simulation/kinematics.py`](../../code/simulation/kinematics.py)
  — canonical IK that reads the URDF the same way T6 must.
- [`code/simulation/generate_urdf.py`](../../code/simulation/generate_urdf.py)
  — generates the URDF from `fusion_export.json`.
