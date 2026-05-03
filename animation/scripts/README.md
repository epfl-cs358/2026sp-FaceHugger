# animation/scripts — Blender visualizers

Two debug scene builders that import the FaceHugger model into Blender 5.x.
They share a scene convention (1 Blender unit = 1 mm, viewport clip
0.1 → 10000) and the same red/orange marker palette, so you can open both
.blend outputs and overlay them for cross-validation.

| Script | Source of truth | What it answers |
|---|---|---|
| `visualize_urdf.py` | `code/simulation/generated/facehugger.urdf` | Does the URDF chain walk reproduce PyBullet's `loadURDF` rest pose? |
| `visualize_fusion_export.py` | `code/simulation/generated/fusion_export.json` | Does the CAD's raw landmark/world data match the URDF generator's output? |
| `urdf_to_blender.py` | (legacy — superseded by `visualize_urdf.py`) | — |

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
    --python animation/scripts/visualize_urdf.py
# or, headless + save:
/Applications/Blender-5.1.app/Contents/MacOS/Blender --background \
    --python animation/scripts/visualize_urdf.py \
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

## urdf_to_blender.py — legacy

Earlier flat-placement attempt at the URDF→Blender import. Same chain-walk
math as `visualize_urdf.py`, but used the Blender default m-scale scene,
which makes the 0.2-m robot 0.2 BU wide — close to the default viewport
clip range and visually hard to interpret. Superseded by
`visualize_urdf.py`. Schedule for deletion.

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

1. **Build the rig on top of `visualize_urdf.py`.** Same chain walk, but
   create one Empty per link parented per the joint chain, and parent each
   visual mesh to its link Empty via `obj.parent + matrix_local =
   visual_origin`. Keyframing `rotation_euler` on the 12 joint Empties
   (around their respective `joint.axis`) animates the rig. Snapping all
   joint angles to 0 should reproduce this baseline exactly.
2. **Delete `urdf_to_blender.py`** once the rigging successor exists, and
   remove the `--debug` flag from `facehugger.py` if it's only used by
   that script. Avoids drift between three URDF importers.
3. **Optional polish** for the visualizer:
   - Replace orange spheres with oriented arrows / line objects so axis
     direction is unambiguous (currently you have to read the orange-tip
     position relative to the red pivot).
   - Add a CLI flag to apply stance angles (shoulder=0°, hip=-40°,
     knee=-60°) as a static snapshot — useful for comparing against
     PyBullet's settled-stand image without running the simulator.
4. **CAD-side cleanup** if you want symmetric leg meshes:
   currently the URDF generator emits `<visual><origin rpy="0 π 0"/>` on
   `fr_link2`/`fr_link3`/`bl_link2`/`bl_link3` to mirror the L mesh onto R
   legs. Exporting native R-pair STLs from Fusion drops that flip and
   makes any future rigging code one branch simpler.
