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
   ├── generate_urdf.py ──► facehugger.urdf ──► simulate.py  (PyBullet)
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

## Driving the pipeline — `facehugger.py`

[facehugger.py](facehugger.py) is the single CLI entry point. Each subcommand wraps one of the underlying scripts:

```bash
cd code/simulation

python facehugger.py urdf                 # regenerate generated/facehugger.urdf
python facehugger.py view                 # open URDF in PyBullet's viewer (no physics)
python facehugger.py sim                  # GUI, standing pose
python facehugger.py sim --walk           # walk gait
python facehugger.py sim --trot           # trot gait
python facehugger.py sim --headless       # no GUI — CI smoke-check
python facehugger.py blender                          # Blender debug scene (default 3.3 LTS)
python facehugger.py blender --blender-version 5.1    # specific Blender version
python facehugger.py blender --headless --save /tmp/scene.blend
python facehugger.py all                  # urdf → sim
```

The `blender` subcommand resolves the Blender executable in this order: `BLENDER_BIN` env var → macOS `/Applications` candidates for the requested `--blender-version` (`Blender-{V}-LTS.app`, `Blender {V}.app` with a space, `Blender-{V}.app`, `Blender{V}.app`) → `blender{V}` on `$PATH` → plain `blender` on `$PATH`. If nothing matches, the CLI prints what it tried and exits non-zero. Set `BLENDER_BIN=/path/to/blender` to bypass the search entirely.

`view` mouse controls:

| Action | How |
| --- | --- |
| Orbit | left-drag |
| Pan | ctrl + left-drag |
| Zoom | scroll |
| Quit | close window or Ctrl+C |

The viewer holds the body fixed with gravity off — nothing moves on its own.

The `sim` simulator reads geometry from the URDF + `facehugger_config.yaml`; no hardcoded leg lengths or stances in Python.

The URDF generator (`urdf` subcommand):

- reads the `mesh_files` manifest in `generated/fusion_export.json` to place meshes with the right `origin_shift`;
- emits per-leg shoulder joint limits from `facehugger_config.yaml`'s `shoulder_limits_deg`;
- emits 12 servo `<visual>` elements (4 shoulder on `base_link`, 4 hip on each `link1`, 4 knee on each `link3`).

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
| `simulate.py --walk` spawns through floor | Settling / body-height issue — see `--settle SECONDS` flag and the gait-aware `body_height` computation. |
| `URDF is missing LEG ASSEMBLY METADATA` | Regenerate the URDF; the metadata comment block is emitted by `generate_urdf.py`. |

## File map

```
code/simulation/
  facehugger.py                 CLI entry point — wraps the scripts below
  facehugger_config.yaml        semantic config (hand-edited)
  generate_urdf.py              URDF generator
  simulate.py                   PyBullet simulator (constants/helpers/kinematics/gaits)
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
