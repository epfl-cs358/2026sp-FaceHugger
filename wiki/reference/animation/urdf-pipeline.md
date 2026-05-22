# URDF pipeline

`facehugger.urdf` is the kinematic source of truth. It is generated, not
hand-authored; every downstream consumer (PyBullet, the Blender rig builder,
the firmware IK generator) is expected to agree with it.

## Fusion 360 export

The `ExportBodiesToURDF` Fusion add-in produces two artefacts:

- `code/simulation/generated/fusion_export.json` - the structured export:
  occurrence tree with world transforms, a `joints[]` array of normalized
  revolute-joint data (axis direction, axis origin, limits), and a
  `mesh_files` manifest that records per-mesh `origin_shift_mm` landmarks.
- `code/simulation/generated/exported_meshes/*.stl` - eight re-origined STLs.

The `EXPORT_RULES` list inside the add-in controls which Fusion bodies are
exported and under which filenames. Edits to the chassis "combined parts"
belong here, not downstream.

### Mesh re-origining

Rather than computing per-visual `<origin xyz>` offsets in the URDF, the
exporter pre-shifts every STL's vertices so that mesh-local `(0, 0, 0)`
coincides with a meaningful CAD construction point, typically the joint
pivot that attaches the next link. The URDF generator can therefore emit
identity origins on every visual:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry><mesh filename="exported_meshes/leg_upper.stl"/></geometry>
</visual>
```

The construction point used for each mesh is recorded as `origin_shift_mm` in
`fusion_export.json`'s `mesh_files` array. The same construction point serves
a dual role: it is the mesh placement anchor and the joint origin that the URDF
generator reads for the corresponding `<joint><origin xyz>`.

### Leg-assembly normalization

`FaceHuggerLegAssembly:1` is placed in Fusion with a 90-degree Z rotation,
so joint axis directions and origins that Fusion reports in the owner
component's local frame are 90 degrees off from world. The exporter's
`collect_joints()` function absorbs this rotation (`R_la`) before writing to
JSON, so the values in `fusion_export.json` are already in world-aligned
coordinates. `generate_urdf.py` uses them verbatim.

## generate_urdf.py and facehugger_config.yaml

`generate_urdf.py` reads `fusion_export.json` together with
`facehugger_config.yaml` (per-leg placement: `mount_point`, `side` L/R,
`rpy_z_deg`, and shoulder limits override) and writes
`code/simulation/generated/facehugger.urdf`.

Run it via the entry point:

```bash
python facehugger.py urdf
```

The URDF is regenerated in roughly one second. The generator preserves any
user edits to `mesh_files._servo_role_assignment` in the JSON across reruns.

## Joint zero and limits

Joint angle theta=0 corresponds to the Fusion rest pose, the configuration
the joint is in when the CAD assembly is at its definition state. Limits are
read from the Fusion revolute joint definitions via `collect_joints()` and
written into each `<limit lower="..." upper="..."/>` element.

For the shoulder joint, Convention A applies: `generate_urdf.py`'s
`_shoulder_rest_for` function derives each leg's shoulder `<origin rpy>` yaw
from `fl_rest_rad = -pi/4` (-45 degrees), so that `theta_shoulder = 0` always
means the leg is at its mechanical zero. The derivation is:

- FL: `fl_rest_rad`
- FR: `-fl_rest_rad`
- BL: `wrap_pi(fl_rest_rad + pi)`
- BR: `-wrap_pi(fl_rest_rad + pi)`

Do not add per-leg rest values to `facehugger_config.yaml`; the per-leg
derivation from FL is the intended pattern.

## Mirror plane and diagonal pairs

Only one leg is modelled in CAD (the FL leg). The four placements are reached
via two symmetries.

**Mirror plane for R mesh.** `Link1R` and `MotorMountR` are mirrored through
the XZ plane of the `FaceHuggerLegAssembly` origin. `BodyToLink1Point` lies
on this plane (Y=0 in assembly-local coordinates), so re-origining R-side
vertices to that point works cleanly without an additional offset.

**Diagonal pairs.**

| Leg | Mesh side | rpy_z_deg |
|-----|-----------|-----------|
| FL  | L         | 0         |
| BR  | L         | 180       |
| FR  | R         | 0         |
| BL  | R         | 180       |

FL and BR share the L mesh; FR and BL share the R mesh. The 180-degree
back-of-pair rotation is encoded as `shoulder_joint.rpy = (0, 0, pi)` at the
mount point; the kinematic chain propagates it through all child links
automatically.

Link2 and Link3 are shared across all four legs. For the R pair, `generate_urdf.py`'s
`link_mesh_rpy` dict applies `mesh_rpy = (0, pi, 0)` on link2 and link3 so
that the shared mesh extends in the correct chassis-outward direction. The
inertial CoM rotates with the mesh.

**R-pair joint axis and limit corrections.** Mirroring the bracket flips the
chirality of the hip and knee servo mounting. Without correction, the same
joint angle would produce opposite physical motion on L-side and R-side legs.
`generate_urdf.py` restores uniform semantics for every R-side hip and knee
joint:

```python
if side == "R":
    axis_dir = [-a for a in axis_dir]
    lim_lo, lim_hi = -lim_hi, -lim_lo
```

After this correction, positive theta lifts the foot on every leg regardless
of side. See [`../conventions.md`](../conventions.md) for the full
angle-space canon and `translateToServo` mapping.

## File summary

| File | Role |
|------|------|
| `cad/scripts/ExportBodiesToURDF/` | Fusion add-in: `EXPORT_RULES`, `collect_joints`, `R_la` normalization, JSON+STL output |
| `code/simulation/generated/fusion_export.json` | Structured export - authoritative for downstream tools |
| `code/simulation/generated/exported_meshes/*.stl` | Eight re-origined meshes |
| `code/simulation/facehugger_config.yaml` | Per-leg placement: mount point, side, rpy_z_deg, shoulder limits override |
| `code/simulation/generate_urdf.py` | Reads JSON + yaml, writes URDF; all per-leg math lives here |
| `code/simulation/generated/facehugger.urdf` | Generated kinematic source of truth |
| `code/simulation/simulate.py` | PyBullet sim with stance/IK/gait |
| `code/simulation/docs/PIPELINE_SPEC.md` | CAD-side decisions and conventions |
