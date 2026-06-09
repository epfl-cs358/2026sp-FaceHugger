# FaceHugger: An Open-Source Quadruped Robot that Walks, Dances, and Flips

<img width="50%" alt="main-robot-stance" src="https://github.com/user-attachments/assets/3c2586f5-037b-4c2a-b8e2-0f888ed73f07" />

In this repo you will find the CAD, firmware, app, simulation, and animation tooling for the FaceHugger quadruped.
This robot was designed and built from scratch by a team of 6 students at EPFL in the [Making Intelligent Things course (CS-358)](https://edu.epfl.ch/coursebook/en/making-intelligent-things-b-CS-358-B) during the Spring 2026 semester.

Find out how to build it and how it works in the _[FaceHugger Wiki](https://epfl-cs358.github.io/2026sp-FaceHugger/)_.

## See it in action

https://github.com/user-attachments/assets/f48a0bcb-4729-4eee-8e89-9a81a8c70e2d

*General moves: walk and wave clips*

https://github.com/user-attachments/assets/28df8788-2bca-48f2-9b82-b47c927adb80

*Dancing clip*

https://github.com/user-attachments/assets/d9c9cdf4-f84c-4cda-ada7-795cacb76790

*Auto-flip: the robot detects when it's upside-down and mirrors its pose*

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
