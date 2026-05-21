<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  code/simulation/docs/SIM_PIPELINE.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/firmware/kinematics.md
-->

> **Reference material.** Verbatim copy of `code/simulation/docs/SIM_PIPELINE.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `code/simulation/docs/SIM_PIPELINE.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [drift] Joint names are stale. Doc steps 4a-c and §C use fl_shoulder_joint / fl_hip_joint / fl_knee_joint. The actual URDF (generated/facehugger.urdf) emits fl_link1_joint / fl_link2_joint / fl_link3_joint (×4 legs) — see MERGE_AND_CONVENTION §3 rename. name[:2]→leg still holds; "shoulder|hip|knee" by name does NOT.
- [drift] §C verification numbers are off. Doc: fl_link1 world = (-0.0579, +0.0462, -0.0090). URDF fl_link1_joint origin = (-0.050401, +0.043222, +0.036844); Z is +0.0368 not -0.009, X/Y also differ (likely older export). rpy rest angles DO match (FL=-45°, FR=+45°, BL=-135°, BR=+135°).
- [ok]    STANCE_DEG hip=-40°, knee=-60° and shoulder=0 match constants.py STANCE_DEG and the Convention-A "shoulder neutral = 0" rule. fk_v2 / build_config / reset_to_stance / run_stand / run_gait all exist in kinematics.py + gaits.py.
- [drift] Line refs drifted: doc cites gaits.py:176 (_connect_and_setup) and gaits.py:242 (run_stand); actual lines are 209 and 284.
- [stale] Inputs list (step "facehugger_config.yaml — ... per-leg rpy_z_deg ... shoulder limits and neutral") is stale: rpy_z_deg, shoulder_limits_deg AND shoulder_neutral_deg were all dropped from facehugger_config.yaml (now derived; see yaml comment lines 56-68). Step 1e/2's yaml.legs[*].shoulder_neutral_deg no longer exists.
- [ok]    8 STLs in generated/exported_meshes/ match (QuadrupedBody, servo, leg_upper, leg_lower, leg_shoulder_{L,R}, leg_mount_{L,R}); LEG ASSEMBLY METADATA block (BodyToLink1Point/FootTip) present in URDF.
- [todo] PyBullet numeric foot-landing claims (fl_link3 ≈ (-0.181,+0.069,-0.055), feet z≈-0.118, body_height≈0.12) not recomputed here; verify after a fresh `facehugger.py urdf` + sim run.
-->
# SIM_PIPELINE — generation and consumption recipes

This is the verifiable algorithmic walkthrough for **(A)** how the URDF is built
from CAD + yaml, and **(B)** what PyBullet does between `loadURDF` and the
"robot stands on the floor" image. Use it to check that any *other* renderer
(e.g. Blender) reproduces the same world transforms.

The two pipelines are independent and can be checked separately:
- A: inputs → URDF text. The output is the URDF on disk.
- B: URDF + yaml → PyBullet world state. The output is per-link world poses
  after stance angles are applied.

---

## A. URDF generation

**Entry point:** [code/simulation/generate_urdf.py](../generate_urdf.py) (run via `python facehugger.py urdf`).

**Inputs:**
- `generated/fusion_export.json` — produced by Fusion's ExportBodiesToURDF.
  Has the per-mesh `origin_shift_mm` (the CAD-construction-point landmark
  each STL was re-origined onto), an occurrence tree, joint axes/limits.
- `facehugger_config.yaml` — handedness per leg (`side: L|R`), per-leg
  `rpy_z_deg` (0 for the "front" leg of a diagonal pair, 180 for the
  "back" one), shoulder limits and neutral, servo physical properties.

**Outputs:**
- `generated/facehugger.urdf`. One `<link>` for `base_link` plus three per
  leg (`fl_link1`, `fl_link2`, `fl_link3`, ×4 legs). Twelve revolute joints.
- A `<!-- LEG ASSEMBLY METADATA -->` comment block near the top with
  `BodyToLink1Point`, `Link1ToLink2Point`, `Link2ToLink3Point`, `FootTip`
  in mm, leg-assembly-local frame. Other tools read this — see helpers.py
  `_parse_leg_points_from_urdf`.

**Algorithm:**

```
1. Load fusion_export.json and facehugger_config.yaml.

2. Resolve per-leg world-frame mount points from the export tree:
     for each leg in yaml.legs:
         mount_world_mm[leg] = find_point_world_in_tree(
             export.occurrences,
             leg.mount_point     # e.g. "LegMountPointFL"
         )

3. Emit base_link:
   a. Visual + collision: QuadrupedBody.stl at <origin xyz="0 0 0" rpy="0 0 0"/>.
   b. For each leg in yaml.legs:
        - leg_mount_{side}.stl visual at LegMountFixedPoint_world,
          rpy = (0, 0, leg.rpy_z_deg).
        - servo.stl visual at the chassis-fixed shoulder servo location:
          xyz = mount_world + Rz(rpy_z) @ ShoulderServoOffset
          rpy = (0, 0, leg.rpy_z_deg)  + servo.visual_flip_rpy_deg from yaml
        These are 4 mounts + 4 servos = 8 chassis-fixed extras on base_link.
   c. Inertial: from fusion_export's QuadrupedBody physics block (or
      fallback mass).

4. Emit each leg in turn (4 of them):
   a. Joint fl_shoulder_joint, type="revolute":
        parent=base_link, child=fl_link1
        <origin xyz=mount_world rpy="0 0 leg.rpy_z_deg"/>
        <axis xyz="0 0 1"/>                    (shoulder = Z)
        <limit lower=… upper=… effort=… velocity=…> from yaml
   b. Joint fl_hip_joint:
        parent=fl_link1, child=fl_link2
        <origin xyz=Link1ToLink2_offset rpy="0 0 0"/>     (mm → m)
        <axis xyz="0 ±1 0"/>                   (hip = ±Y; +Y for L, -Y for R)
   c. Joint fl_knee_joint:
        parent=fl_link2, child=fl_link3
        <origin xyz=Link2ToLink3_offset rpy="0 0 0"/>
        <axis xyz="0 ±1 0"/>                   (knee = ±Y, same sign as hip)
   d. Link fl_link1: visual leg_shoulder_{side}.stl at <origin xyz="0 0 0"/>.
       Plus a hip-servo visual: servo.stl at
         xyz = Link1ToLink2_offset_in_link1
         rpy = (-π/2, 0, 0)                    (rotate the chassis-baked
                                                 servo into hip orientation)
   e. Link fl_link2: visual leg_upper.stl at <origin xyz="0 0 0" rpy=mesh_rpy/>
        - mesh_rpy = (0, 0, 0)        for L pair (FL, BR)
        - mesh_rpy = (0, π, 0)        for R pair (FR, BL) — Y-flip the mesh
   f. Link fl_link3: visual leg_lower.stl at the same mesh_rpy as link2.
        Plus a knee-servo visual: servo.stl at
         xyz = (0, -0.000206, 0)
         rpy = (0, -π/2, -π/2)

5. Insert the LEG ASSEMBLY METADATA comment block.

6. Write the URDF text to disk.
```

**Key invariants the URDF carries (Blender / any other importer must respect):**
- *Every* mesh has its vertices pre-shifted in CAD so that mesh-local
  `(0,0,0)` is the link frame's origin (the next joint's pivot, mostly).
  → primary visual `<origin>` is identity; placement is via the joint chain.
- Joint axes use ROS conventions: shoulder = +Z always; hip/knee = +Y for L
  pair, -Y for R pair. Numerical noise (`1e-17` etc.) is harmless — both
  consumers normalise via sign-of-Y.
- Visual `<origin xyz rpy>` IS used for the *non-primary* visuals (the
  base_link mounts/servos and the per-link servo extras). Don't ignore them.
- Mesh `scale="0.001 0.001 0.001"` always — STLs are mm.

---

## B. PyBullet setup

**Entry points:** [code/simulation/simulate.py](../simulate.py) → [gaits.py](../gaits.py) `run_stand` / `run_gait`.

**Inputs:**
- `generated/facehugger.urdf`
- `facehugger_config.yaml` (for shoulder neutrals and servo gains).

**Outputs:**
- A connected PyBullet world with a plane, the robot at body_height, joints
  set to per-leg stance angles, motor targets running.

**Algorithm:**

```
1. build_config() — kinematics.py:
   a. Parse URDF joints (xyz, rpy, axis, limits) via helpers._load_urdf_joints.
   b. Parse LEG ASSEMBLY METADATA block (foot tip in link3-mesh frame, mm).
   c. Convert foot tip to metres. For R-pair legs, apply Ry(π) → (-x, y, -z)
      because their leg_upper / leg_lower visuals carry mesh_rpy=(0, π, 0).
   d. Build LegGeom per leg:
        mount       = shoulder joint xyz
        yaw_offset  = shoulder joint rpy[2]              (0 or π)
        L1_vec      = hip joint xyz                       (link1-frame)
        L2_vec      = knee joint xyz                      (link2-frame)
        foot_L3     = foot tip in link3-frame             (per side)
        hip_axis_sign  = +1 if hip axis Y >= 0  else -1
        knee_axis_sign = +1 if knee axis Y >= 0 else -1
        joint_limits   = per-joint (lo, hi) from URDF
   e. Stance per leg:
        shoulder_rad = radians(yaml.legs[*].shoulder_neutral_deg)   # all 0
        hip_rad      = radians(constants.STANCE_DEG["hip"])        # -40°
        knee_rad     = radians(constants.STANCE_DEG["knee"])        # -60°
   f. Compute neutral_foot[leg] = fk_v2(cfg, leg, *stance_rad[leg]).
      body_height = max(1e-3, -min(z) over neutral_foot).

2. _connect_and_setup() — gaits.py:176:
   a. p.connect(GUI or DIRECT)
   b. p.setGravity(0, 0, -9.81); p.setTimeStep(1/240)
   c. p.loadURDF("plane.urdf")                         (ground plane)
   d. robot_id = p.loadURDF(
          cfg.urdf_path,
          basePosition=[0, 0, cfg.body_height + 0.02],  (lift 2 cm above)
          useFixedBase=False
      )
      ── PyBullet now has the robot at REST POSE (all joints = 0). The
         legs are extended STRAIGHT OUT — the "starfish" layout. This is
         the visual a Blender importer that *only* walks the URDF tree
         will produce. ──
   e. joint_map = build_joint_map(robot_id) — maps URDF joint names to
      PyBullet joint indices.
   f. reset_to_stance(robot_id, joint_map, cfg.stance_rad):
        for each non-fixed joint j of role (shoulder|hip|knee) on leg L:
            p.resetJointState(robot_id, joint_idx, stance_rad[L][role])
      ── THIS is the step that bends hip/knee to the standing pose.
         Without it the robot stays starfished. ──
   g. apply_leg_pose(...) — also sends motor targets so gravity doesn't
      pull the bent joints back to zero.
   h. Knee-link friction tweak: lateralFriction=1.0, restitution=0.1.

3. run_stand() — gaits.py:242: settle for `settle_s` then idle-step the sim.
   run_gait() — same, but per-tick computes foot targets via IK and feeds
   them to apply_joint_targets() through the gait period.
```

**The key step a flat URDF importer is missing:** `reset_to_stance`. Without
it the rendered model is rest pose (starfish).

---

## C. Recipe — making Blender match PyBullet's "settled stand" image

Blender doesn't have a `loadURDF` that respects joint state. To reproduce
PyBullet's standing pose visually, do this in the importer:

```
1. Parse URDF → links{name: visuals[]}, joints{child: parent, origin, axis}.

2. Read stance angles:
     shoulder_rad[leg_id] = radians(yaml.legs[leg_id].shoulder_neutral_deg)
     hip_rad   = radians(-40)        # constants.STANCE_DEG["hip"]
     knee_rad  = radians(-60)        # constants.STANCE_DEG["knee"]

3. For each joint, compute its stance angle:
     name = joint.name              # e.g. "fl_shoulder_joint"
     leg  = name[:2]
     role = "shoulder" | "hip" | "knee"   (from name)
     theta = (shoulder_rad[leg] | hip_rad | knee_rad)
     joint_q[name] = theta

4. Compute link world transforms with stance applied:
     link_world[root] = Identity
     for each child link in topological order:
         T_origin = joint.origin                 # 4×4 from <origin xyz rpy>
         T_rot    = Rotation(joint.axis, joint_q[joint.name])
         link_world[child] = link_world[parent] @ T_origin @ T_rot

5. For each <visual> in each link:
     mesh_world = link_world[link] @ visual_origin
     import STL with global_scale=0.001
     obj.matrix_world = mesh_world

There is no rigging required — a single FK pass is enough for a static
"matches PyBullet" snapshot. Add a rig later for keyframable animation.
```

**Verification checklist** (numeric, against the URDF):
- `fl_link1` world translation = `(-0.0579, +0.0462, -0.0090)`.
- `bl_link1` = `(-0.0579, -0.0462, -0.0090)`; `fr_link1` = `(+0.0579, +0.0462, -0.0090)`;
  `br_link1` = `(+0.0579, -0.0462, -0.0090)`. (Body shoulders at the four corners.)
- With stance applied (shoulder=0, hip=-40°, knee=-60°), `fl_link3` world ≈
  `(-0.181, +0.069, -0.055)` and the four feet land in the four quadrants
  with z ≈ -0.118 m (cfg.body_height ≈ 0.12 m → spawn = 0.14 m).
- The four base_link servos are at the four corners' Z=+0.037, NOT bunched.
- Each `leg_lower` mesh's tip should sit on the ground if you place the
  body at z = body_height.

**Things to *not* mistake for bugs:**
- Rest-pose import (no stance) shows legs HORIZONTAL — that's correct
  for joint angles = 0. PyBullet's image was post-`reset_to_stance`.
- The chassis-fixed servos sit *inside* the body cavity (the body shell is
  hollow). When viewed from above through the cutouts they look like they
  overlap the body — that's the actual physical layout.
