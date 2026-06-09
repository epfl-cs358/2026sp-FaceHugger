# FaceHugger - a quadruped robot for MIT Things 2026sp

CAD, firmware, simulation, and animation tooling for the FaceHugger quadruped.

## See it in action

![FaceHugger at rest](wiki/assets/img/general-robot-pics-videos/main-robot-stance.jpg)
*FaceHugger at rest: 12 servos, ESP32 brain, PCA9685 driver*

\[reupload needed\]
*General moves: walk, trot, and wave clips*

\[reupload needed\]
*Dancing clip*

\[reupload needed\]
*Auto-flip: the robot detects when it's upside-down and mirrors its pose

## Documentation

Project docs are a MkDocs Material wiki under [`wiki/`](wiki/).

- **Live site:** _[FaceHugger Wiki](https://epfl-cs358.github.io/2026sp-FaceHugger/)_
- **How to contribute / fill it out:** see [`wiki/README.md`](wiki/README.md) - structure, local preview,
  adding images/video/3D models/notebooks, and the per-member assignments.

## Repo layout

- `cad/` - Fusion 360 design files and export add-ins
- `code/simulation/` - URDF generator + PyBullet sim ([README](code/simulation/README.md))
- `code/firmware/` - ESP32 firmware (PlatformIO)
- `code/remote-control-app/` - Expo / React Native control app
- `animation/` - Blender rig and animation tooling
- `wiki/` - project documentation (published to GitHub Pages)

## Getting started

```bash
# Simulation
cd code/simulation && conda env create -f environment.yml && conda activate facehugger
python code/facehugger.py sim

# Wiki (local preview)
python -m venv .venv-docs && source .venv-docs/bin/activate && pip install -r requirements-docs.txt
mkdocs serve

# Remote-control app
cd code/remote-control-app/MyApp && npm install && npx expo start --web

# Firmware
cd code/firmware && pio run -t upload
```
