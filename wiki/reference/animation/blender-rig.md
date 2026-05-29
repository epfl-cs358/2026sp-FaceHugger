# URDF to animated character in Blender

This stage turns the kinematic source of truth (`facehugger.urdf`) into a posable Blender armature an animator can drive. The rig stays byte-for-byte faithful to the placement that PyBullet and the CAD export agree on, which is what lets a clip authored in Blender play correctly on the robot.

Three scripts in `animation/scripts/` cooperate, all targeting Blender 5.x:

- **`visualize_fusion_export.py`** is the CAD-side cross-check. It drops the chassis and legs as bare STLs at their exported world poses, validating that the URDF generator started from the right geometry.
- **`visualize_urdf.py`** is the placement baseline. It walks the URDF joint chain at rest (all angles zero) and places each visual STL exactly where PyBullet would on `loadURDF`. This is the gold-standard reference the rig must reproduce.
- **`urdf_to_blender_rigged.py`** is the animatable rig. It uses the same URDF parser and the same rest-pose link matrices as the baseline, then builds a real armature on top.

Build the rig through the simulation entry point:

```bash
python code/facehugger.py blender --rigged
```

It reads `code/simulation/generated/facehugger.urdf`, the STLs in `generated/exported_meshes/`, and `fusion_export.json`, and rebuilds from scratch every run. Nothing kinematic persists in a `.blend` between runs, so re-export from Fusion and regenerate the URDF before rebuilding if the CAD moved. For clip authoring and export see [the clip panel](clip-panel.md); for the angle spaces see [Conventions](../conventions.md).

## Armature anatomy

The armature is named `FaceHuggerRig`: one root (`base_link`) plus three bones per leg.

```
FaceHuggerRig
├── base_link              (root, chassis, fixed)
├── fl_link1               (FL shoulder, URDF axis world +Z)
│   └── fl_link2           (FL hip,      URDF axis world ±Y)
│       └── fl_link3       (FL knee,     URDF axis world ±Y)
├── fr_link1 / fr_link2 / fr_link3
├── bl_link1 / bl_link2 / bl_link3
└── br_link1 / br_link2 / br_link3
```

Bone names match URDF link names exactly. The export tooling uses these names as dict keys with no translation table, so do not rename them.

A bone is a rotation control, not the visible limb. Each bone's head sits at its joint pivot and its tail aims at the next joint pivot (or the foot tip for `*_link3`). The geometry is attached separately by `attach_visuals`, which forces each mesh's `matrix_world = link_world @ visual_origin` so it overlays the placement baseline regardless of where the bone points.

### Bone roll and the uniform-Z contract

At build time `EditBone.align_roll(joint_axis_world)` is called for every bone. After this, **bone-local Z is the URDF joint axis for all 12 joint bones**, including the per-side axis sign flip on the right-side legs (`<axis>0 -1 0</axis>`). The practical result: all pose-mode rotation lives on `rotation_euler[2]` uniformly, with no per-joint axis branching.

This is the rig's half of the L/R mirror erasure. The URDF carries the physical left/right mirror as signed joint axes; aligning each bone-local Z onto its signed axis means a positive `rotation_euler[2]` is uniform CCW across all 12 bones. The exporter therefore feeds the same uniform math-space the firmware gaits use. The mirror lives in two independent places (the URDF/rig and the firmware `translateToServo`), not as one chained application. See [Conventions](../conventions.md) for the firmware side.

The contract relies on `_bone_endpoints_world_mm` projecting each bone's tail onto the plane perpendicular to its joint axis before `align_roll` is called. Without the projection, `align_roll` would be only approximate (around 13 degrees of error on `*_link1`, around 23 degrees on `*_link3`), and `rotation_euler[2]` would no longer be a clean joint angle.

### Joint-limit constraints

A `LIMIT_ROTATION` constraint with `use_limit_z = True` is applied to every joint bone, with `min_z`/`max_z` read from the URDF `<limit lower upper>` at build time. The URDF is authoritative; if joint ranges are retuned in Fusion, regenerate the URDF and rebuild the rig.

## FK shoulder, IK hip and knee

The rig mirrors the physical three-servo-per-leg arrangement:

- **Shoulder (`*_link1`), FK.** It yaws the leg laterally about world Z and is not in the IK chain. A scripted analytic driver writes each `*_link1.rotation_euler[2]` from the foot position expressed in `body_ctrl`-local space, so the shoulder tracks the foot target and the result is invariant to chassis pitch, roll, yaw, and translation.
- **Hip and knee (`*_link2`, `*_link3`), IK with `chain_count=2`.** An IK constraint on the knee bone targets the `foot_target_{leg}` Empty, and the solver drives hip and knee to bring the foot to the target. IK X and Y are locked on all joint bones so the solver only rotates on bone-local Z, preventing lateral bending that has no physical servo counterpart.

### Why chain_count=2 and not 3

Each foot target is parented to its leg's `*_link1` bone. If the shoulder were in the IK chain (`chain_count=3`), the solver would rotate `*_link1`, which moves the parented target, which changes the goal, an unstable feedback loop with around 180 mm of rest drift. Excluding the shoulder is the only stable configuration for this parenting.

### Animator workflow

1. Rotate `{leg}_link1` in pose mode (or let the foot-tracking driver set it) to aim the shoulder.
2. Drag the `foot_target_{leg}` Empty to place the foot. Because the target is parented to `*_link1`, the shoulder carries it along automatically.

IK-solved hip and knee angles live only in the evaluated pose, so the export step bakes them to `rotation_euler[2]` FCurves before converting. See [the clip panel](clip-panel.md).

## Zero-pose baseline check

At `rotation_euler = (0, 0, 0)` on all bones, the rigged scene must match the `visualize_urdf.py` placement baseline within **0.5 mm** at every joint landmark. The per-leg shoulder rest orientations are encoded in the URDF `<joint><origin rpy>` (derived from the FL rest), so they do not appear as non-zero Blender rotations. If the rigged scene drifts beyond 0.5 mm, the rig is composing transforms incorrectly (typically a bone-roll/projection error, a bad parent-inverse, or a stale URDF) and must be regenerated. This single deterministic check catches the whole class of "rig looks right but kinematics are subtly off" bugs.

## Object name contract

Actions store keyframe data as FCurve paths that reference object names. If `urdf_to_blender_rigged.py` changes these names between runs, existing Actions silently stop animating. The stable names are:

| Object | Name |
|---|---|
| Armature | `FaceHuggerRig` |
| Body control | `body_ctrl` |
| Foot targets | `foot_target_fl`, `foot_target_fr`, `foot_target_bl`, `foot_target_br` |
| Joint bones | `{leg}_link1`, `{leg}_link2`, `{leg}_link3` |

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Leg bends laterally when dragging a foot target | IK X/Y locks missing | Rebuild with `python code/facehugger.py blender --rigged`; check `lock_ik_x = lock_ik_y = True`. |
| Foot does not follow shoulder rotation | Foot target lost its parent | Re-parent: Parent = Armature, Bone = `{leg}_link1`, Type = Bone. |
| Mesh drifts vs PyBullet | Rig not rebuilt after URDF regeneration | Rebuild with `python code/facehugger.py blender --rigged`. |
| Joint exceeds its URDF limit | `LIMIT_ROTATION` constraint disabled | Check the pose bone constraints. |
| Foot target starts in the wrong place | foot-tip landmark missing from `fusion_export.json` | Re-export from Fusion, then regenerate the URDF. |
