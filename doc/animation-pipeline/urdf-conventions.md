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
