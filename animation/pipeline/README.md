# animation/pipeline — Blender scene builders

Blender 5.x scripts that build the FaceHugger scene from the URDF or
Fusion export. All three share the same conventions (1 Blender unit = 1 mm,
viewport clip 0.1 → 10000) and the same red/orange marker palette, so you
can open multiple `.blend` outputs and overlay them for cross-validation.

| File | Kind | Source of truth | What it's for |
|---|---|---|---|
| `urdf_to_blender_rigged.py` | scene builder (`--python`) | `facehugger.urdf` + `fusion_export.json` | **Animator-facing**. Pose-able rig: FK shoulder + IK on hip+knee, foot-target Empties. |
| `visualize_urdf.py` | scene builder (`--python`) | `code/simulation/generated/facehugger.urdf` | Cross-check: does the URDF chain walk reproduce PyBullet's `loadURDF` rest pose? |
| `visualize_fusion_export.py` | scene builder (`--python`) | `code/simulation/generated/fusion_export.json` | Cross-check: does the CAD's raw landmark/world data match the URDF generator's output? |

## urdf_to_blender_rigged.py — animation rig

Builds a real Blender Armature on top of the URDF: 13 bones (12 leg +
`base_link`), one bone per URDF link, parented in chain. Each bone's
roll is set by `EditBone.align_roll(joint.axis)` so bone-local Z is the
URDF joint axis for every joint uniformly — including the per-side
±Y sign on hip/knee. Visual STLs are parented to their bone via
`parent_type='BONE'`, so the meshes follow pose-mode rotations.

**Animation workflow** — built around the physical 3-servo layout:
1. **FK shoulder**: rotate `{leg}_link1` in pose mode to aim the leg
   laterally (servo 1, world Z yaw).
2. **IK foot reach**: drag the `{leg}_foot_target` Empty. The IK
   constraint solves hip + knee (servos 2 + 3) to bring the leg's
   foot tip to the empty's position. Shoulder stays at its FK pose.
3. The foot target is **parented to the link1 bone**, so when you
   rotate the shoulder in step 1, the target follows automatically —
   no driver / no driver-update step.

**Constraints set up automatically:**
- `LIMIT_ROTATION` on every joint bone, `use_limit_z=True`,
  `min_z` / `max_z` from URDF `<limit lower upper>`. Pose-mode rotation
  clamped to the URDF's per-joint range.
- `IK` constraint on each `{leg}_link3` (knee) bone, `chain_count=2`
  (hip + knee solve), `use_rotation=False`, `use_stretch=False`
  (rigid bones), targeting `{leg}_foot_target`.
- Per-bone IK locks: `lock_ik_x = lock_ik_y = True`,
  `ik_stiffness_x = ik_stiffness_y = 1.0`. After `align_roll`, the
  joint axis is bone-local Z; locks force the IK solver to rotate
  only around Z (= the URDF joint axis), no lateral bending.

**Foot tip source**: read from `Link3TipAxis` in `fusion_export.json`
when present (CAD source-of-truth), with a hardcoded link3-frame
fallback if the export hasn't been re-run with the axis added to
the whitelist.

**Run:**
```bash
python code/simulation/facehugger.py blender --rigged
# or, headless + save:
python code/simulation/facehugger.py blender --rigged --headless --save /tmp/fh_rigged.blend
```

CLI flags after `--`: `--urdf PATH`, `--meshes PATH`, `--json PATH`,
`--save PATH`, `--ik-chain {2,3}` (default 2; 3 is unstable in
combination with the parented foot target — see the script's
docstring for the why).

**Markers**: same red/orange spheres as the placement-only visualizer,
but parented to their bone so they follow pose rotations. Hidden by
default (`hide_viewport=True` on `Joint Origins/` and `Joint Axes/`
collections); toggle the eye in the outliner to see them.

**Object hierarchy** (top-down, post-build):

```
world_origin   (PLAIN_AXES Empty, world (0,0,0) — never animated)
  ├── body_ctrl   (CUBE Empty — animator handle for chassis position + rotation)
  │     └── FaceHuggerRig   (armature, hide_viewport=True; bones drive limbs)
  └── foot_target_{fl,fr,bl,br}   (SPHERE Empties — animator handles)

foot_ik_{fl,fr,bl,br}   (PLAIN_AXES, hidden, UNPARENTED;
                          COPY_LOCATION pulls them to foot_target.world + static offset)
```

`world_origin` is a deterministic anchor for downstream exporters and
is not intended to be animated. It sits at world identity, so each
child's local transform equals its world transform. `foot_ik_*` are
deliberately NOT parented: the COPY_LOCATION offset would double under
world_origin motion if they were (target.world + owner.pre_constraint_world
both shift by the same delta). foot_ik follows foot_target purely via
the constraint, which is correct regardless of anchor motion.

**Regression vs visualize_urdf.py**: at all-zero pose, every visual
mesh's world position matches the placement-only baseline within
0.5 mm (0.001 rad rotation tolerance). Any deviation means the rig is
composing transforms wrong and is worth investigating.

See [code/simulation/docs/API_ANIMATION_SPEC.md](../../code/simulation/docs/API_ANIMATION_SPEC.md)
for the full animator-facing API reference (baking, export to
`.gait`, servo conversion).

---

## visualize_urdf.py — current baseline

Reads the URDF directly and walks the joint chain at rest pose (all joint
angles 0°), the same way PyBullet does on `loadURDF`. One pass:

```
link_world[root]  = Identity
link_world[child] = link_world[parent] @ joint.origin   (rest pose)
mesh.matrix_world = link_world[link]   @ visual.origin
```

Translation is in metres throughout the chain; the script multiplies by
1000 once at the end to land in the mm-scale Blender scene. Rotations
pass through unchanged (they're unitless).

**Markers:**
- **Red spheres** in `Joint Origins/` — one per joint, at the world position
  of that joint's pivot (the child link's frame origin).
- **Orange spheres** in `Joint Axes/` — one per joint, 20 mm out from the
  pivot along the unit axis vector. Direct visualization of `<axis xyz>` —
  spot a bad sign or a misnamed axis instantly.

**Run:**

```bash
/Applications/Blender-5.1.app/Contents/MacOS/Blender \
    --python animation/pipeline/visualize_urdf.py
# or, headless + save:
/Applications/Blender-5.1.app/Contents/MacOS/Blender --background \
    --python animation/pipeline/visualize_urdf.py \
    -- --save /tmp/fh_urdf.blend
```

CLI flags after `--`: `--urdf PATH`, `--meshes PATH`, `--save PATH`.

### Known minor issues (revisit before rigging if they bite)

- **No "above link1" construction-point markers.** The URDF only carries
  joint origins, not raw CAD landmarks. So the script puts a red sphere at
  each joint pivot (where the joint rotates) but cannot show
  `BodyToLink1Point`, `LegMountFixedPoint`, `ServoMountPoint`, etc. — those
  only exist in `fusion_export.json`. If you need the richer landmark set,
  open the fusion-export visualizer's .blend alongside.
- **Axis tips aren't always visually obvious.** All 12 joints get an orange
  sphere placed at `pivot + 20mm · unit_axis`, but for the shoulder joints
  whose axis is +Z, the tip sits inside / above the body shell where it can
  be hard to see. Increasing `AXIS_LENGTH_MM` or hiding the chassis mesh
  helps. (Eventually we may want oriented arrows / line objects instead of
  spheres so direction is unambiguous.)

## visualize_fusion_export.py — JSON cross-check

Reads `fusion_export.json` directly and places each STL at its
`origin_landmark`'s `pos_world_mm`. The 4-leg replication uses
`LegMountPointXX` plus per-corner `Rz(rpy_z_deg)` to instance the source
FL leg at the other three corners. Pure mesh placement; the `*_root`
Empties were dropped per "no rigging" — there's no rig parent.

Useful for diagnosing CAD-side issues (missing landmarks, wrong
`origin_world_mm` on a construction point, mesh in wrong frame). When the
URDF visualizer disagrees with this one, the disagreement points at the
URDF generator (`code/simulation/generate_urdf.py`).

## Scene conventions (both visualizers)

- `unit_settings.scale_length = 0.001` — 1 BU = 1 mm. Robot reads as
  ~200 BU, comfortable for the default viewport.
- STL imports use `bpy.ops.wm.stl_import(global_scale=1.0)` since both the
  STL vertex coordinates AND the scene are in mm — no conversion.
- `clear_scene` manually wipes data-blocks instead of calling
  `bpy.ops.wm.read_factory_settings(use_empty=True)`. The factory-reset
  call leaves `wm.stl_import.poll()` returning False on Blender 5.x until
  the GUI redraws, which kills imports done from `--python`.
- No Empties, no parenting, no armatures. `obj.matrix_world` is set
  directly per object. This sidesteps every `matrix_parent_inverse` /
  depsgraph-timing trap the rigging-aware versions hit. Rigging belongs in
  a successor script, not these.

## What's next

1. **Optional polish** for the visualizer:
   - Replace orange spheres with oriented arrows / line objects so axis
     direction is unambiguous (currently you have to read the orange-tip
     position relative to the red pivot).
   - Add a CLI flag to apply stance angles (shoulder=0°, hip=-40°,
     knee=-60°) as a static snapshot — useful for comparing against
     PyBullet's settled-stand image without running the simulator.
2. **CAD-side cleanup** if you want symmetric leg meshes:
   currently the URDF generator emits `<visual><origin rpy="0 π 0"/>` on
   `fr_link2`/`fr_link3`/`bl_link2`/`bl_link3` to mirror the L mesh onto R
   legs. Exporting native R-pair STLs from Fusion drops that flip and
   makes any future rigging code one branch simpler.
