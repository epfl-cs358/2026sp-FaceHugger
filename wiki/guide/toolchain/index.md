# Toolchain

FaceHugger's build pipeline connects four tools in a fixed order. Understanding
each stage prevents surprises when you change something upstream and wonder why
nothing moved downstream.

**Fusion 360 -> URDF export.** The two Fusion scripts in
`cad/scripts/ExportBodiesToURDF/` and `cad/scripts/ExportPrintableSTLs/` are the
first stage. The URDF exporter writes a JSON manifest and STL meshes that the
Python generator (`generate_urdf.py`) turns into `facehugger.urdf`. Everything
downstream is derived from that URDF, so never hand-edit the generated files. See
[Fusion 360 export scripts](fusion-export.md) for the step-by-step.

**URDF -> Blender rig.** Running `python code/facehugger.py blender --rigged`
ingests the URDF and builds an armature in Blender 5.x. The rigged scene
(`animation/fh_rigged_latest.blend`) is the starting point for clip authoring.

**Blender rig -> clip authoring and export.** The FH Clip Panel (N-panel add-on
in Blender) lets you pose the rig, record keyframe bundles, and export them as C
headers and JavaScript players that the firmware can play back. See
[Blender clip authoring and export](blender-clips.md) for the workflow.

**Exported clips -> firmware flash.** The final step compiles the PlatformIO
project (which embeds the exported clip headers) and flashes it to the ESP32 over
USB. After flashing, the robot joins its own Wi-Fi network and accepts commands
over WebSocket. See [Flashing the firmware](flashing.md) for the commands.

## Setup: installing each component

Each component has its own toolchain; there is no single installer. Install only
the ones you need. The [`facehugger.py` CLI](../../reference/simulation/cli.md)
prints the relevant command if it hits a missing dependency. For the deeper
"which environment runs what" picture, see [Software](../software.md#software-environments).

### Simulation + the `facehugger.py` CLI (Python)

The only host-Python project, one environment for everything Python. On macOS use
conda (pybullet has no PyPI wheel there); the `environment.yml` is a one-shot:

```bash
cd code/simulation
conda env create -f environment.yml
conda activate facehugger
```

Linux / CI can skip conda and `pip install -r code/simulation/requirements.txt`.
Dependencies: Python 3.12, `pybullet`, `pyyaml`, `matplotlib` (only for `--log`
plots), and for the firmware software-in-the-loop build `pybind11` + CMake >= 3.15
+ a C++17 compiler, plus `websockets` / `sse-starlette` / `uvicorn` for
`sim --serve` and its telemetry. Without the C++ toolchain, the sim still runs via
`--python` (the Python re-port).

### Remote-control app (Node / Expo)

Needed to run the app, including `sim --app` (which serves it on the web):

```bash
cd code/remote-control-app/MyApp
npm install
```

Dependencies: Node.js 18+ and npm; Expo Go on a phone for the mobile flow. See
[Remote control app](../remote_control_app.md).

### Firmware (PlatformIO)

```bash
pip install platformio        # or the VS Code PlatformIO extension
```

PlatformIO pulls the ESP32 toolchain and the `lib_deps` from `platformio.ini` on
first build. Flash with `python code/facehugger.py flash`. See
[Flashing the firmware](flashing.md).

### Animation (Blender) and CAD (Fusion 360)

Blender 5.0+ (5.1 recommended) for the rig and clip authoring; the FH Clip Panel
add-on ships in the repo. Fusion 360 (with the two export scripts in
`cad/scripts/`) for the CAD and the URDF/print exports. See
[Blender clip authoring](blender-clips.md) and
[Fusion 360 export scripts](fusion-export.md).
