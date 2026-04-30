# FaceHugger simulation pipeline

End-to-end: from the Fusion 360 design, produce a URDF that's faithful to the
CAD (re-origined meshes, per-leg quadrant-limited shoulder joints, servo
visuals in the right orientation) and drive it in PyBullet.

## Overview

```
Fusion 360 CAD
   │   ExportBodiesToURDF add-in (cad/scripts/ExportBodiesToURDF/)
   ▼
fusion_export.json    (CAD tree + mesh_files manifest)
fusion_export.txt     (human-readable tree)
exported_meshes/*.stl (5 files: chassis + 3 leg links + servo)
   │
   ├── generate_urdf.py ──► facehugger.urdf ──► simulate_v2.py  (PyBullet)
   │
   └── animation/scripts/visualize_fusion_export.py    (Blender 3.3 LTS)
```

## Prerequisites

- Python env: `uv`, `pyyaml`, `pybullet`, `numpy`.
- Fusion 360 with the ExportBodiesToURDF add-in installed (`cad/scripts/ExportBodiesToURDF/`).
- Blender 3.3 LTS (optional — only needed for the debug visualizer).

## Step 1 — Run the Fusion exporter

In Fusion 360, open your assembly and run the add-in from **Scripts and Add-Ins**. It writes to `code/simulation/generated/`:

| File | Content |
|---|---|
| `fusion_export.json` | Full CAD tree + `mesh_files` manifest (source bodies, `origin_shift_mm`, servo role assignment, per-occurrence world transforms). |
| `fusion_export.txt` | Same tree in a human-readable form. |
| `exported_meshes/*.stl` | 5 STLs: `QuadrupedBody.stl` (chassis + 4 brackets stitched in world frame), `leg_shoulder.stl`, `leg_upper.stl`, `leg_lower.stl`, `servo.stl`. Leg meshes are re-origined to their URDF joint landmarks; `servo.stl` is re-origined to `ServoMountPoint`. |

Re-runs preserve user edits to `mesh_files._servo_role_assignment` in the JSON.

## Step 2 — (optional) Inspect exports

```bash
# Open the export in Blender: chassis + 4 legs (instanced) + construction-point spheres
/Applications/Blender-3.3-LTS.app/Contents/MacOS/Blender \
    --python animation/scripts/visualize_fusion_export.py
```

## Step 3 — Generate the URDF

```bash
cd code/simulation
uv run generate_urdf.py
```

Writes `generated/facehugger.urdf`. The generator:

- reads the `mesh_files` manifest to place meshes with the right `origin_shift`;
- emits per-leg shoulder joint limits as `[neutral ± 90°]` from `facehugger_config.yaml`'s `shoulder_neutral_deg`;
- emits 12 servo `<visual>` elements (4 shoulder on `base_link`, 1 hip on each `link2`, 1 knee on each `link3`).

## Step 3.5 — View the URDF (optional)

```bash
cd code/simulation
python view_urdf.py
```

Loads the URDF in standing pose and lets you orbit around it.

| Action | How |
| --- | --- |
| Orbit | left-drag |
| Pan | ctrl + left-drag |
| Zoom | scroll |
| Quit | close window or Ctrl+C |

The body is fixed and gravity is off — nothing moves on its own.

## Step 4 — Simulate

```bash
cd code/simulation

python simulate_v2.py              # GUI, standing pose (default)
python simulate_v2.py --walk       # walk gait
python simulate_v2.py --trot       # trot gait
python simulate_v2.py --headless   # no GUI — CI smoke-check
```

The simulator reads geometry from the URDF + `facehugger_config.yaml`; no
hardcoded leg lengths or stances in Python.

## Config

All semantic parameters live in [facehugger_config.yaml](facehugger_config.yaml):

- `base_link.mesh` — chassis STL filename.
- `leg_template.joints` — per-joint axis/point keys, default limits.
- `legs[]` — per-leg `mount_point`, `rpy_z_deg`, `shoulder_neutral_deg` (quadrant center; standing pose).
- `servo` — mass, effort, velocity.

Angles are in degrees in the yaml; `generate_urdf.py` converts to the URDF's radians.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Chassis STL has electronics/PCBs baked in | `EXPORT_RULES` `combined.parts` list in [../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py](../../cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/ExportBodiesToURDF.py) is out of date — add or remove the right occurrences. |
| Servos misaligned in Blender / PyBullet | Either the CAD's `ServoMountPoint` is not on the shaft axis, or the URDF's per-role rpy is off — inspect in Blender. |
| `simulate_v2.py --walk` spawns through floor | Settling / body-height issue — see `--settle SECONDS` flag and the gait-aware `body_height` computation. |
| `URDF is missing LEG ASSEMBLY METADATA` | Regenerate the URDF; the metadata comment block is emitted by `generate_urdf.py`. |

## File map

```
code/simulation/
  facehugger_config.yaml        semantic config (hand-edited)
  generate_urdf.py              URDF generator
  simulate_v2.py                PyBullet simulator
  view_urdf.py                  PyBullet URDF viewer (no physics)
  README.md                     this file
  docs/                         pipeline docs (PIPELINE_SPEC, ASSEMBLY_HIERARCHY, …)
  generated/                    artifacts produced by the Fusion add-in / generator
    fusion_export.json          CAD tree (do not edit)
    fusion_export.txt           human-readable tree
    exported_meshes/*.stl       generated STLs (do not edit)
    facehugger.urdf             generated URDF (do not edit)

../../animation/scripts/
  visualize_fusion_export.py    Blender 3.3 scene builder
```

## Design notes

- The `origin_shift_mm` field in `mesh_files` records the landmark each STL was re-origined against — the URDF generator consumes it so leg meshes sit at their joint origins with `<origin xyz="0 0 0"/>`.
- Per-leg `shoulder_neutral_deg` values (FR=+45, FL=+135, BR=−45, BL=−135) place each leg into its body quadrant, and the joint-limit window `[neutral ± 90°]` prevents the servo from being commanded across the chassis.
- All four legs share `rpy_z_deg = −90°`, keeping `R_L1 = I` so the URDF generator doesn't need per-leg rotation fudges on joint offsets or meshes.
