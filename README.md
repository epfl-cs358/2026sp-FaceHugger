# FaceHugger - a quadruped robot for MIT Things 2026sp

CAD, firmware, simulation, and animation tooling for the FaceHugger quadruped.

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
