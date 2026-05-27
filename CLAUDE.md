# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

FaceHugger is a quadruped robot built for EPFL's *Making Intelligent Things* course (2026sp). The repo is multi-domain: **CAD** (Fusion 360), **firmware** (ESP32/PlatformIO), **simulation** (PyBullet + URDF), **animation tooling** (Blender), and a **remote-control mobile app** (Expo / React Native). The four worlds are tied together by a single source-of-truth pipeline; understanding that pipeline is the fastest way to be productive.

## The pipeline (read this first)

```
Fusion 360 assembly
    │  cad/scripts/ExportBodiesToURDF/  (Fusion add-in)
    ▼
code/simulation/generated/
    fusion_export.json     CAD tree + mesh manifest + per-occurrence world transforms
    fusion_export.txt      human-readable tree
    exported_meshes/*.stl  8 STLs, re-origined to URDF joint landmarks
    │
    │  code/simulation/urdf_gen/generate_urdf.py  +  facehugger_config.yaml
    ▼
    facehugger.urdf  ──┬──► pybullet_sim/            (PyBullet sim, gaits, clip interpreter)
                       └──► animation/scripts/       (Blender 5.x: placement + rigged)
```

Everything downstream of the Fusion export (URDF, PyBullet, Blender rig, IK reference cases, eventually the `.fhc` animation files) is **derived**. The URDF is the kinematic source of truth — every other component is expected to agree with it, not the other way around. Anything in `code/simulation/generated/` is regenerated; never hand-edit those files.

The single entry point for the simulation half of the pipeline is [code/simulation/facehugger.py](code/simulation/facehugger.py). Subcommands: `urdf`, `sim` (`--walk` / `--trot` / `--headless` / `--clip NAME` / `--monitor` / `--log` / `--list-clips`, plus the interface flags `--serve` / `--app` / `--panel`), `blender` (`--rigged` for the animation rig; auto-refreshes a stale URDF), `app` (Expo web), and `flash` (PlatformIO build + upload). `serve` survives as a deprecated alias for `sim --serve`; `all` was removed (use `urdf` then `sim`). The runtime lives in the `pybullet_sim/` package, the URDF build step in `urdf_gen/`, and the firmware-faithful clip re-port in `firmware_port/`. **Clip playback defaults to the EXACT compiled firmware** (`firmware_sil/`, software-in-the-loop via pybind11; auto-built); pass `sim --clip NAME --python` to use the Python re-port instead (no C++ toolchain). `facehugger.py sim --serve` (or `--app`) runs the firmware-backed WebSocket robot API (API_SPEC `T:` protocol on :8081) so the app or the browser control panel (`code/remote-control-app/control-panel/robot_control_panel.html`, hostable via `sim --panel`) can drive the sim exactly like the real robot. See [code/simulation/README.md](code/simulation/README.md) and [code/simulation/firmware_sil/README.md](code/simulation/firmware_sil/README.md).

## Repo layout (non-obvious bits)

- **`cad/scripts/ExportBodiesToURDF/`** — Fusion 360 add-in (Python). Edits to the chassis "combined parts" list (`EXPORT_RULES`) belong here, not downstream — see the troubleshooting table in `code/simulation/README.md`.
- **`cad/scripts/ExportPrintableSTLs/`** — separate Fusion add-in that exports print-ready STLs (per-body up-axis rotation). Unrelated to the URDF pipeline.
- **`code/firmware/`** — PlatformIO ESP32 project. Three logical layers under `src/`: `brain/` (network + sensors), `nervous_system/` (kinematics, legs, servos, spinal_cord, movements/poses), `shared/` (config + data). See [code/firmware/src/nervous_system/README.md](code/firmware/src/nervous_system/README.md) for the "80% hardcoded pose / 20% calibration" philosophy.
- **`code/simulation/`** — Python, split into packages: `pybullet_sim/` (the runtime — `simulate`, `gaits`, `kinematics`, `helpers`, `constants`, `sim_monitor`), `firmware_port/` (the firmware-faithful clip re-port — `servo_convention`, `clip_loader`, `clip_player`, `gait_interpreter`; the Python fallback / parity reference), and `urdf_gen/` (`generate_urdf`, `verify_export_parity`). `facehugger.py` + `facehugger_config.yaml` + `generated/` sit at the top. The `docs/` subfolder is the authoritative spec for the CAD↔URDF↔sim contract. Run modules as `python -m pybullet_sim.simulate` (from `code/simulation/`), not by path.
- **`code/remote-control-app/MyApp/`** — Expo (React Native + TypeScript) app, uses Zustand for state. Talks to the ESP32 over WebSocket on port 81 per [code/API_SPEC.md](code/API_SPEC.md).
- **`animation/scripts/`** — Blender 5.x tooling. Two kinds of files live side by side:
  - *Scene builders* (run as `blender --python …`): `visualize_urdf.py` (placement-only, cross-check baseline), `visualize_fusion_export.py` (CAD-side cross-check), `urdf_to_blender_rigged.py` (real armature with FK shoulder + IK on hip/knee). The rigged scene must match the baseline within 0.5 mm at zero pose — if it doesn't, the rig is composing transforms wrong.
  - *Animator-facing helpers*: `fh_clip_panel.py` (Blender N-panel add-on; manages named 5-Action clip bundles on the rig via the layered Action API) and `fh_rename_actions.py` (one-shot `--background` migration from legacy `*Action` names to the `base_anim__<target>` clip convention). See [animation/scripts/README.md](animation/scripts/README.md) for the table mapping each file to its role + source-of-truth.
- **`animation/blend-iterations/`** — local-only `.blend` work files (gitignored, large).
- **`doc/animation-pipeline/`** — design docs for the upcoming on-board animation engine (`.fhc` format, ESP32 playback). The `.gait` baked-angle format and textbook IK in firmware are scheduled to be replaced by this. Implementation has not started.

## Common commands

### Simulation (Python, run from `code/simulation/`)

```bash
python facehugger.py urdf                       # regenerate generated/facehugger.urdf
python facehugger.py sim                        # GUI, standing pose
python facehugger.py sim --walk                 # walk gait
python facehugger.py sim --trot                 # trot gait
python facehugger.py sim --headless             # CI smoke check
python facehugger.py sim --clip "wave" --headless   # play a baked clip through the interpreter
python facehugger.py blender                    # placement-only, default Blender 5.1
python facehugger.py blender --rigged           # animator-facing rig
python facehugger.py blender --headless --save /tmp/scene.blend
python facehugger.py sim --app                  # sim + WebSocket API + Expo web app
python facehugger.py flash                       # build + upload the firmware
```

Override the Blender executable with `BLENDER_BIN=/path/to/blender`. The CLI searches `/Applications/Blender-{V}-LTS.app`, `/Applications/Blender {V}.app`, etc. — see `_resolve_blender_bin` in [code/simulation/facehugger.py](code/simulation/facehugger.py:62) if it can't find your install.

### Firmware (PlatformIO, run from `code/firmware/`)

```bash
pio run                                         # build for the upesy_wroom ESP32
pio run -t upload                               # flash
pio device monitor                              # 115200 baud, exception decoder on
pio test -e native -f test_kinematics           # run the native C++ IK tests (Unity)
python test/gen_ik_reference.py                 # regen reference_data.h from sim's leg_ik
```

The `native` environment builds `nervous_system/kinematics.cpp` only and runs Unity tests on the host — no hardware required. The IK reference cases are sourced from the Python `kinematics.py` to keep firmware C++ and sim Python in lockstep.

### Mobile app (Expo, run from `code/remote-control-app/MyApp/`)

```bash
npm install
npm run start                                   # Metro / Expo dev server
npm run ios | npm run android | npm run web
```

## Toolchain notes

- **Python 3.12** (`.python-version`). No `pyproject.toml` in the repo yet; the simulation deps live in `code/simulation/requirements.txt`. Prefer `uv run script.py` per the global convention.
- **PyBullet on macOS** has no PyPI wheel. Use a conda env (`conda install -c conda-forge pybullet`) before `pip install -r requirements.txt`. Linux/CI can `pip install` directly.
- **Blender 5.0+ is required** for the animation scripts. 3.3 LTS is no longer supported (factory-reset behavior breaks STL import from `--python`).
- **Fusion 360 add-ins** are Python scripts that run inside Fusion; they're not part of the host Python environment.

## Conventions to respect

- **Leg naming**: URDF/Python use `fl`, `fr`, `bl`, `br`. The firmware historically uses `fr`, `fl`, `rr`, `rl` (with `rr`/`rl` = "rear" = `br`/`bl`). Translation table is in [code/simulation/docs/MERGE_AND_CONVENTION.md §5](code/simulation/docs/MERGE_AND_CONVENTION.md). Don't propagate firmware leg names into Python or Blender code.
- **Per-leg shoulder rest** is derived from FL via `FR = -FL`, `BL = wrap_pi(FL + π)`, `BR = -wrap_pi(FL + π)`. Don't add per-leg rest values to `facehugger_config.yaml` — that pattern was removed during the convention merge.
- **Servo numbering** is bridged through `servo_mapping.yaml` (URDF link name ↔ firmware servo_id ↔ `direction`). See [animation/SERVO_ID_CONVENTION.md](animation/SERVO_ID_CONVENTION.md). The numbering is still a **proposal** pending firmware `SERVO_CONFIG[]` confirmation — flag the gap if you're about to bake `.gait` files.
- **Mesh placement**: leg STLs are re-origined in the Fusion exporter to their URDF joint landmarks, so the URDF emits `<origin xyz="0 0 0"/>` on visuals. The `origin_shift_mm` field in `fusion_export.json`'s `mesh_files` records which landmark was used.
- **`generate_urdf.py` re-runs preserve user edits** to `mesh_files._servo_role_assignment` in the JSON. Don't blow it away unless you mean to.

## Files that look usable but aren't

- **`doc/gait-design/`** — superseded by `doc/animation-pipeline/`; reorg plan deletes it outright. Treat as historical only.

(The `teleop.py`, `terrain.py`, and `view_urdf.py` stubs — and the `facehugger.py view` subcommand — were removed; don't expect them.)

## Where the architecture decisions live

Deep "why" content is in markdown files alongside the code, not in the code itself:

- [code/simulation/docs/MERGE_AND_CONVENTION.md](code/simulation/docs/MERGE_AND_CONVENTION.md) — leg-naming, shoulder convention A, URDF θ=0 = Fusion rest pose, why a hand-written `kinematics.py` was rejected in favor of URDF-derived geometry.
- [code/simulation/docs/PIPELINE_SPEC.md](code/simulation/docs/PIPELINE_SPEC.md) — CAD-side decisions: mirror plane, mount-point naming, joint zero/limits, mesh orientation.
- [code/simulation/docs/API_ANIMATION_SPEC.md](code/simulation/docs/API_ANIMATION_SPEC.md) — animator-facing reference for the Blender rig.
- [animation/scripts/README.md](animation/scripts/README.md) — per-script role table, rig anatomy (bone roll / IK chain / foot-target parenting), and the `fh_clip_panel` clip model.
- [doc/animation-pipeline/leg-coordinates.md](doc/animation-pipeline/leg-coordinates.md) — design canon for the upcoming `.fhc` on-board animation format. Read before writing any code touching the new format.
- [doc/reorg-plan-2026-05-13.md](doc/reorg-plan-2026-05-13.md) — drafted but deferred layout reshuffle of `code/simulation/` and `animation/`. Worth a glance before suggesting structural changes so you don't propose a conflicting layout.
- [code/API_SPEC.md](code/API_SPEC.md) — WebSocket JSON protocol between mobile app and ESP32.

## Project hooks

- `.claude/hooks/block-dangerous-git.sh` runs on every Bash invocation and blocks dangerous git operations (force push, `reset --hard`, etc.). If a git command unexpectedly fails, check whether the hook caught it before retrying.
