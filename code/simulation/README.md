# FaceHugger simulation pipeline

End-to-end: from the Fusion 360 design, produce a URDF that's faithful to the
CAD and drive it in PyBullet with the exact firmware code compiled to the host.

## Quick start

```bash
# Regenerate the URDF from CAD export
python code/facehugger.py urdf

# Run the sim (standing pose, GUI)
python code/facehugger.py sim

# Play a clip through the exact firmware SIL
python code/facehugger.py sim --clip "wave"

# Walk gait (firmware SIL)
python code/facehugger.py sim --walk

# Open URDF in Blender (rigged armature for animation)
python code/facehugger.py blender --rigged
```

`facehugger.py` is the single CLI entry point — run it from the repo root.

## Packages

| Package | Role |
|---------|------|
| `pybullet_sim/` | PyBullet runtime — kinematics, scene lifecycle, motor control, torque monitoring |
| `firmware_sil/` | Software-in-the-loop bridge — compiles the real firmware C++ to a Python module via pybind11 |
| `urdf_gen/` | URDF generator — reads `generated/fusion_export.json` + `facehugger_config.yaml` → `facehugger.urdf` |
| `animation_tools/` | Firmware-faithful clip math (servo convention, clip loader, export parity) |
| `tests/` | Test suite (173 tests) |

## Documentation

Everything else — full CLI reference, config details, the blender pipeline,
controlling the simulation, torque analysis, and limitations — lives in the wiki:

→ **[Simulation documentation](https://epfl-cs358.github.io/2026sp-FaceHugger/reference/simulation/)**

## Prerequisites

```bash
cd code/simulation
conda env create -f environment.yml   # first time only (macOS: conda, Linux: pip works too)
conda activate facehugger             # every session
```

Also needed:
- Fusion 360 with the ExportBodiesToURDF add-in (`cad/scripts/ExportBodiesToURDF/`)
- For the SIL: CMake ≥ 3.15 + C++17 compiler + pybind11 (auto-builds on first use)

Other project envs (wiki, app, firmware) — see the [main README](../../README.md).
