# Blender rig

The FaceHugger Blender rig is built automatically by
`animation/scripts/urdf_to_blender_rigged.py`. Run it via the simulation entry
point:

```bash
python code/simulation/facehugger.py blender --rigged
```

The script reads `code/simulation/generated/facehugger.urdf`, the STLs in
`code/simulation/generated/exported_meshes/`, and `fusion_export.json`. It
rebuilds the rig from scratch on every run - nothing is persisted in a `.blend`
between runs.

For clip authoring and export, see [clip-panel.md](clip-panel.md).

---

## Armature anatomy

The armature is named `FaceHuggerRig` and contains 13 bones - one root
(`base_link`) plus three per leg:

```
FaceHuggerRig
├── base_link              (root, chassis - fixed)
├── fl_link1               (FL shoulder - URDF axis: world +Z)
│   └── fl_link2           (FL hip      - URDF axis: world +Y)
│       └── fl_link3       (FL knee     - URDF axis: world +Y)
├── fr_link1 / fr_link2 / fr_link3
├── bl_link1 / bl_link2 / bl_link3
└── br_link1 / br_link2 / br_link3
```

Bone names match URDF link names exactly (lowercase, e.g. `fl_link1`). The
export tooling uses these names as dict keys with no translation table - do not
rename them.

### Bone roll and the uniform-Z contract

At rig-build time, `EditBone.align_roll(joint.axis)` is called for every bone.
After this, **bone-local Z is the URDF joint axis for all 12 joint bones**,
including the per-side axis sign flip on FR/BL hip and knee (`<axis>0 -1 0</axis>`
in the URDF). The practical result: all pose-mode rotation lives on
`rotation_euler[2]` uniformly - no per-joint axis branching.

This relies on `_bone_endpoints_world_mm` projecting each bone's tail onto the
plane perpendicular to its joint axis before `align_roll` is called. Without the
projection, `align_roll` would only be approximate on `*_link1` (approx. 13 deg
error) and `*_link3` (approx. 23 deg error), breaking the contract. Mesh
geometry is attached via `attach_visuals` using explicit `matrix_world` and is
unaffected by the tail projection.

### Joint-limit constraints

A `LIMIT_ROTATION` constraint with `use_limit_z = True` is applied to every
joint bone. The `min_z`/`max_z` values are read from the URDF `<limit lower
upper>` at rig-build time - they are not hardcoded here. The URDF is the
authoritative source; if joint ranges are retuned in Fusion, regenerate the URDF
and rebuild the rig.

---

## FK shoulder + IK hip/knee chain

The rig implements the physical 3-servo-per-leg arrangement directly:

- **Shoulder (`*_link1`) - FK only.** Rotate it in pose mode to aim the leg
  laterally (world-Z yaw). It is not included in the IK chain.
- **Hip + knee (`*_link2`, `*_link3`) - IK, chain_count=2.** An `IK` constraint
  on the knee bone (`*_link3`) targets the foot Empty
  `foot_target_{fl,fr,bl,br}`. The solver drives hip and knee to bring the foot
  to the target position.

IK X and Y are locked on all joint bones (`lock_ik_x = lock_ik_y = True`,
`ik_stiffness_x/y = 1.0`) so the solver only rotates on bone-local Z. This
prevents lateral bending that has no physical servo counterpart.

### Why chain_count=2 and not 3

Each foot target is parented to its leg's `*_link1` bone (`parent_type='BONE'`).
If the shoulder were also in the IK chain (`chain_count=3`), the solver would
rotate `*_link1`, which moves the parented foot target, which changes the IK
goal - producing an unstable feedback loop with approximately 180 mm of drift at
rest pose. `chain_count=2` excludes the shoulder and is the only stable
configuration for this parenting arrangement.

### Animator workflow

1. Rotate `{leg}_link1` in pose mode to set the shoulder yaw.
2. Drag the `foot_target_{leg}` Empty to set the foot position. Because the foot
   target is parented to `*_link1`, rotating the shoulder automatically carries
   the foot target with it - no manual repositioning required.

---

## Zero-pose baseline check

At `rotation_euler = (0, 0, 0)` on all bones, the rig must match the placement
baseline produced by `visualize_urdf.py` within **0.5 mm** at every joint
landmark. The per-leg shoulder offsets (FL -45 deg, FR +45 deg, BL -135 deg,
BR +135 deg, world frame) are encoded in the URDF `<joint><origin rpy>` per
Convention A and do not appear as non-zero Blender rotations. If the rigged
scene drifts beyond 0.5 mm from the baseline, the rig is composing transforms
incorrectly and must be regenerated.

---

## Object name contract

Actions store keyframe data as FCurve paths that reference object names. If
`urdf_to_blender_rigged.py` changes these names between runs, existing Actions
silently stop animating anything. The stable names are:

| Object | Name |
|---|---|
| Armature | `FaceHuggerRig` |
| Body control | `body_ctrl` |
| Foot targets | `foot_target_fl`, `foot_target_fr`, `foot_target_bl`, `foot_target_br` |
| Joint bones | `{leg}_link1`, `{leg}_link2`, `{leg}_link3` |

These are constants in the script, not derived from URDF link names.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Leg bends laterally when dragging foot target | IK X/Y locks missing | Re-run `facehugger.py blender --rigged`; check `lock_ik_x = lock_ik_y = True` on pose bones. |
| Foot does not follow shoulder rotation | Foot target lost its parent | Re-parent in Object Properties: Parent = Armature, Bone = `{leg}_link1`, Type = Bone. |
| Mesh drifts vs PyBullet | Rig not rebuilt after URDF regeneration | Re-run `facehugger.py blender --rigged`. |
| Joint exceeds URDF limit | `LIMIT_ROTATION` constraint disabled | Check pose bone constraints in the Properties panel. |
| Foot target starts in wrong place | `Link3TipAxis` missing from `fusion_export.json` | Re-export from Fusion, then regenerate the URDF. |
