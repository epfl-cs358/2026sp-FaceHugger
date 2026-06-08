# FaceHugger - a quadruped robot for MIT Things 2026sp

CAD, firmware, simulation, and animation tooling for the FaceHugger quadruped.

## See it in action

![FaceHugger at rest](wiki/assets/img/general-robot-pics-videos/main-robot-stance.jpg)
*FaceHugger at rest — 12 servos, ESP32 brain, PCA9685 driver*

<video src="https://raw.githubusercontent.com/epfl-cs358/2026sp-FaceHugger/main/wiki/assets/img/general-robot-pics-videos/general-moves.MOV" controls width="100%"></video>
*General moves — walk, trot, and wave clips*

<video src="https://raw.githubusercontent.com/epfl-cs358/2026sp-FaceHugger/main/wiki/assets/img/general-robot-pics-videos/robot-dancing.MOV" controls width="100%"></video>
*Dancing clip*

<video src="https://raw.githubusercontent.com/epfl-cs358/2026sp-FaceHugger/main/wiki/assets/img/general-robot-pics-videos/robot-flipping.MOV" controls width="100%"></video>
*Auto-flip — the robot detects when it's upside-down and mirrors its pose*

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
