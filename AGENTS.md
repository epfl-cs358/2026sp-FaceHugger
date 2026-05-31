# Agent Instructions — FaceHugger

## About

FaceHugger is a quadruped robot built for EPFL's *Making Intelligent Things* course (2026sp). The repo spans CAD (Fusion 360), firmware (ESP32/PlatformIO), simulation (PyBullet + URDF), Blender-based animation tooling, and a React Native remote-control app. These four worlds are tied together by a single source-of-truth pipeline: Fusion 360 → URDF → PyBullet sim + Blender rig → firmware clips.

## Working Here

- Read `AGENTS.md` and `CONTEXT.md` at session start
- Check `.work/todos/` and `.work/plans/` for active work
- Write plans to `.work/plans/<topic>.md` before significant changes
- Update `CONTEXT.md` when terminology or understanding is settled
- Research findings go in `.work/research/<topic>.md`
- Architecture decisions go in `docs/decisions/<slug>.md`
- Agent session logs go in `agent-log/<date>-<slug>.md`

## Python Environment

Use `conda activate facehugger` before any Python scripts (PyBullet is installed via conda on macOS — the global `uv` default does not apply here).

## Stack

- **Languages**: C++ (firmware), Python (sim + tooling), TypeScript (app)
- **Firmware**: PlatformIO, ESP32 (upesy_wroom)
- **Simulation**: PyBullet, URDF
- **Animation**: Blender 5.x (bpy), custom N-panel add-on
- **App**: Expo / React Native, Zustand
- **CAD add-ins**: Fusion 360 Python add-ins

## Commands

```bash
# Simulation (from repo root)
conda activate facehugger
python code/facehugger.py urdf
python code/facehugger.py sim
python code/facehugger.py sim --clip "wave" --headless

# Firmware (from code/firmware/)
pio run
pio run -t upload
pio test -e native -f test_kinematics

# App (from code/remote-control-app/MyApp/)
npm install && npm run start

# Wiki / docs
.venv-docs/bin/mkdocs build --strict
```

## Conventions

- **Leg naming**: `fl/fr/bl/br` everywhere in Python/Blender/URDF. The firmware uses `FR/FL/RR/RL` — never propagate those into Python or Blender code.
- **Angle spaces**: math-space (symmetric, authored) → `translateToServo` → servo-space (hardware). The `constrain(0,180)` clamp is the electrical backstop, a separate layer.
- **URDF is the kinematic source of truth** — everything downstream (sim, rig, IK) must agree with it.
- Files under `code/simulation/generated/` are regenerated — never hand-edit.

## Skills to Use

- `/skill:tdd` — for firmware/simulation feature work
- `/skill:diagnose` — when something is broken
- `/skill:grill-with-docs` — before significant design decisions
- `/skill:improve-codebase-architecture` — when modularising a component
- `/skill:handoff` — before ending a long session
