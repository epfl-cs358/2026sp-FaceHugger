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
exported_meshes/*.stl (8 files: chassis + per-side L/R brackets + per-side L/R shoulder + shared upper/lower + servo)
   │
   └── urdf_gen/generate_urdf.py ──► facehugger.urdf ──┬─► pybullet_sim/  (PyBullet sim + clip interpreter)
                                                       │
                                                       └─► visualize_urdf.py  (Blender 5.x)
```

The code is split into packages: **`pybullet_sim/`** (the runtime — `simulate`,
`gaits`, `kinematics`, `helpers`, `constants`, `sim_monitor`), **`firmware_port/`**
(the firmware-faithful clip re-port — `servo_convention`, `clip_loader`,
`clip_player`; the Python fallback / parity reference), and **`urdf_gen/`**
(`generate_urdf`, `verify_export_parity`). `facehugger.py` + `facehugger_config.yaml`
+ `generated/` stay at the top. Run modules with `python -m pybullet_sim.simulate`
from `code/simulation/`, not by path.

## Prerequisites

- Python env: `uv`, `pyyaml`, `pybullet`, `numpy`.
- Fusion 360 with the ExportBodiesToURDF add-in installed (`cad/scripts/ExportBodiesToURDF/`).
- Blender 5.0+ (optional — only needed for the URDF visualizer; 3.3 LTS no longer supported).

## Step 1 — Run the Fusion exporter

In Fusion 360, open your assembly and run the add-in from **Scripts and Add-Ins**. It writes to `code/simulation/generated/`:

| File | Content |
|---|---|
| `fusion_export.json` | Full CAD tree + `mesh_files` manifest (source bodies, `origin_shift_mm`, servo role assignment, per-occurrence world transforms). |
| `fusion_export.txt` | Same tree in a human-readable form. |
| `exported_meshes/*.stl` | 8 STLs: `QuadrupedBody.stl` (chassis), `leg_mount_L.stl` / `leg_mount_R.stl` (bracket per side), `leg_shoulder_L.stl` / `leg_shoulder_R.stl` (link1 per side), `leg_upper.stl` (link2, shared), `leg_lower.stl` (link3, shared), `servo.stl`. Leg meshes are re-origined to their URDF joint landmarks; `servo.stl` is re-origined to `ServoMountPoint`. |

Re-runs preserve user edits to `mesh_files._servo_role_assignment` in the JSON.

## Driving the pipeline — `facehugger.py`

[../facehugger.py](../facehugger.py) is the single CLI entry point. It lives at `code/facehugger.py` and resolves its own paths, so run it from the repo root (it shells into this `code/simulation/` package for you). Each subcommand wraps one of the underlying scripts:

```bash
python code/facehugger.py urdf                 # regenerate generated/facehugger.urdf
python code/facehugger.py sim                  # GUI, standing pose
python code/facehugger.py sim --walk           # walk gait (EXACT firmware tickGait, via the SIL)
python code/facehugger.py sim --trot           # trot gait (EXACT firmware tickTrot, via the SIL)
python code/facehugger.py sim --clip "wave" --python-port   # the Python clip re-port (no C++ toolchain; gaits not supported here)
python code/facehugger.py sim --headless       # no GUI — CI smoke-check
python code/facehugger.py sim --clip "wave" --headless   # play a baked clip via the interpreter
python code/facehugger.py blender                          # URDF in Blender, placement-only
python code/facehugger.py blender --rigged                 # animator-facing rig (armature + IK)
python code/facehugger.py blender --blender-version 5.2    # specific Blender version
python code/facehugger.py blender --headless --save /tmp/scene.blend
python code/facehugger.py sim --app            # sim + WebSocket API + Expo web app
python code/facehugger.py flash                # build + upload the firmware
```

### `blender` subcommand

Loads `generated/facehugger.urdf` into Blender. Requires **Blender 5.0+**. Two modes selectable via `--rigged`:

| Mode | Wrapped script | What you get | When to use |
|---|---|---|---|
| **placement-only** (default) | [animation/scripts/visualize_urdf.py](../../animation/scripts/visualize_urdf.py) | Walks the joint chain at rest pose (the same math PyBullet uses on `loadURDF`) and places each of the 29 STL visuals at `link_world @ visual_origin`. No armature, no Empties, no parenting. | Cross-check: does the URDF chain reproduce PyBullet's `loadURDF` rest pose? Spot bad joint origins / axes visually. |
| **rigged** (`--rigged`) | [animation/scripts/urdf_to_blender_rigged.py](../../animation/scripts/urdf_to_blender_rigged.py) | Real Armature: 13 bones, FK shoulder + IK on hip+knee, foot-target Empties parented to each `link1`. URDF `<limit>` clamps applied per bone. Matches the placement-only baseline within 0.5 mm at zero pose. | Animator workflow — pose the rig in pose mode, drag foot targets, bake clips. |

| Flag | Default | What it does |
|---|---|---|
| `--rigged` | off | Use the rigged armature scene instead of placement-only. |
| `--blender-version VERSION` | `5.1` | Major.minor of the Blender to launch (e.g. `5.1`, `5.2`). Drives the `/Applications` search; ignored if `BLENDER_BIN` is set. |
| `--headless` | off | Run Blender in `--background` mode — no GUI window. Pair with `--save` for CI / batch use. |
| `--save PATH` | mode-dependent (see below) | Save the built scene to `PATH` (passed as `-- --save PATH` to the inner script). Works with or without `--headless`. |
| `--reset` | off | Start from a blank scene, discarding any existing animations (rigged mode only — placement-only always starts blank). |

**Default save target.** If `--save` is omitted:

- **`--rigged`** defaults to `animation/fh_rigged_latest.blend` — the animation **library** (rig + all authored clips). The rigged builder reopens it (unless `--reset`) and stash/restores your clips across the rebuild, so re-running `blender --rigged` is safe and keeps your work.
- **placement-only** (no `--rigged`) does **not** save anywhere — it opens a blank scene for viewing and discards on exit. This is deliberate: `visualize_urdf.py` clears the scene with **no** stash/restore, so it must never write over the rigged library and destroy clips. Pass an explicit `--save PATH` if you want to keep a placement-only scene (use a path other than the library).

⚠️ **Do not point a placement-only `--save` at `animation/fh_rigged_latest.blend`** — it would overwrite the library with a clip-less placement scene.

The CLI resolves the Blender executable in this order: `BLENDER_BIN` env var → macOS `/Applications` candidates for the requested `--blender-version` (`Blender-{V}-LTS.app`, `Blender {V}.app` with a space, `Blender-{V}.app`, `Blender{V}.app`) → `blender{V}` on `$PATH` → plain `blender` on `$PATH`. If nothing matches, the CLI prints every path it tried and exits non-zero. Set `BLENDER_BIN=/path/to/blender` to bypass the search entirely.

Examples:

```bash
python facehugger.py blender                                   # placement-only, Blender 5.1, GUI
python facehugger.py blender --rigged                          # rigged scene, Blender 5.1, GUI
python facehugger.py blender --rigged --headless \             # rig built in background, no GUI
    --save /tmp/fh_rigged.blend
python facehugger.py blender --blender-version 5.2 --rigged    # use Blender 5.2 instead
BLENDER_BIN=/opt/blender/blender python facehugger.py blender  # explicit binary override
```

Common pitfalls:

- **"Could not locate Blender 5.1"** — your install path isn't in the `/Applications` candidates. Set `BLENDER_BIN` or pass `--blender-version` matching what you actually have installed.
- **STL imports silently fail in `--background`** — make sure you're on Blender 5.0+. The placement-only script's `clear_scene` works around a `wm.read_factory_settings(use_empty=True)` quirk that bricked STL import on older versions.
- **Rigged scene drifts from placement baseline** — at all-zero pose the two should match within 0.5 mm. If they don't, the rig is composing transforms wrong; open both `.blend` outputs and overlay. See [animation/scripts/README.md](../../animation/scripts/README.md) for the rig-build details.

The `sim` simulator reads geometry from the URDF + `facehugger_config.yaml`; no hardcoded leg lengths or stances in Python.

The URDF generator (`urdf` subcommand):

- reads the `mesh_files` manifest in `generated/fusion_export.json` to place meshes with the right `origin_shift`;
- emits per-leg shoulder joint limits from `facehugger_config.yaml`'s `shoulder_limits_deg`;
- emits 12 servo `<visual>` elements (4 shoulder on `base_link`, 4 hip on each `link1`, 4 knee on each `link3`).

## Config

All semantic parameters live in [facehugger_config.yaml](facehugger_config.yaml):

- `base_link.mesh` — chassis STL filename.
- `leg_template.links.<role>.mesh` — per-link STL filename template (`{side}` substituted from `legs[].side`).
- `legs[]` — per-leg `id`, `mount_point`, `side`. Shoulder rest, limits, and back-of-pair flip are no longer in yaml — they're derived from the Fusion JSON's `Link1Revolute.limits_rad` and from `leg_id` per [wiki/reference/firmware/kinematics.md](../../wiki/reference/firmware/kinematics.md).
- `servo` — mass, effort, velocity, optional `visual_flip_rpy_deg`.

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
  facehugger.py                 CLI entry point — runs the packages below via `python -m`
  facehugger_config.yaml        semantic config (hand-edited)
  README.md                     this file
  docs/                         (empty after the 2026-06 wiki migration; kept as a placeholder)
  pybullet_sim/                 runtime package
    simulate.py                 PyBullet simulator front-end (constants/helpers/kinematics)
    kinematics.py helpers.py constants.py sim_monitor.py
  firmware_port/                firmware-faithful clip re-port (Python fallback / parity ref)
    servo_convention.py clip_loader.py clip_player.py gait_interpreter.py (+ tests/)
  urdf_gen/                     build package
    generate_urdf.py            URDF generator
    verify_export_parity.py     sim↔firmware export parity check
  generated/                    artifacts produced by the Fusion add-in / generator
    fusion_export.json          CAD tree (do not edit)
    fusion_export.txt           human-readable tree
    exported_meshes/*.stl       generated STLs (do not edit)
    facehugger.urdf             generated URDF (do not edit)

../../animation/scripts/
  visualize_urdf.py             Blender 5.x scene builder (URDF → placement + joint markers)
  visualize_fusion_export.py    Blender 5.x scene builder (fusion_export.json → meshes + landmarks)
```

## Design notes

- The `origin_shift_mm` field in `mesh_files` records the landmark each STL was re-origined against — the URDF generator consumes it so leg meshes sit at their joint origins with `<origin xyz="0 0 0"/>`.
- Per-leg shoulder rest is derived from the FL Fusion-export rest via the formula in [wiki/reference/firmware/kinematics.md](../../wiki/reference/firmware/kinematics.md): `FR = -FL`, `BR = wrap_pi(FL + π)`, `BL = -wrap_pi(FL + π)`. With the current CAD's `FL = -π/4`, the four legs sit at `FL=-45°, FR=+45°, BR=+135°, BL=-135°` in world space. URDF limits are symmetric `[-90°, +90°]` about each leg's rest.
- The geometric back-of-pair flip (chassis bracket positioning) is `0°` for front legs (`fl`, `fr`) and `180°` for back legs (`bl`, `br`), derived from `leg_id` rather than carried in yaml.
