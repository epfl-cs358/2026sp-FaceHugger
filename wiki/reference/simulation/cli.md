# FaceHugger CLI

`facehugger.py` is the single entry point for the simulation half of the project. It is a thin dispatcher: each subcommand shells out to the right module (`urdf_gen.generate_urdf`, `pybullet_sim.simulate`, the Blender scripts, or the firmware-backed WebSocket server) so you never invoke those by path.

Run it from `code/simulation/`:

```bash
cd code/simulation
python facehugger.py <subcommand> [options]
```

## Subcommands at a glance

| Subcommand | What it does |
|------------|--------------|
| `urdf`   | Regenerate `generated/facehugger.urdf` from the Fusion export + config |
| `sim`    | Run the PyBullet simulator (stand, gait, or clip) |
| `blender`| Open the URDF in Blender (placement cross-check, or the animation rig) |
| `serve`  | Run the firmware-backed WebSocket robot API (drive from the app / control panel) |
| `all`    | `urdf` → `sim` in one go (smoke run) |

## `urdf`: regenerate the URDF

Rebuilds `generated/facehugger.urdf` from `generated/fusion_export.json` (the CAD tree + mesh manifest) and `facehugger_config.yaml`. Everything downstream (the sim, the Blender rig, the IK reference cases) is derived from this URDF, so regenerate it whenever the CAD export or the config changes.

| Flag | Meaning |
|------|---------|
| `--export PATH` | path to `fusion_export.json` (default: `generated/`) |
| `--config PATH` | path to `facehugger_config.yaml` |
| `--out PATH`    | output URDF path |

```bash
python facehugger.py urdf
```

## `sim`: run the simulator

With no mode flag, the robot just stands. Add one mode and any number of instrumentation flags. **By default, clips and gaits are driven by the exact compiled firmware** (see [Controlling the simulation](pybullet-control.md)); `--python` switches to the Python re-port.

| Flag | Meaning |
|------|---------|
| `--walk` / `--trot` | run that gait (the exact firmware `tickGait`/`tickTrot`) |
| `--clip NAME` | play a baked animation clip by name (from `clips_all.h`) |
| `--loop` | replay the clip continuously (GUI only) to watch cumulative behaviour |
| `--float` | no gravity/floor, body pinned; inspect pure joint geometry without falling/slipping |
| `--monitor` | print a periodic torque + estimated-current status line (peak τ, total A, `[CONT]`/`[STALL]` joints) |
| `--log` | record per-step torque/current → summary + `sim_log.csv` + `sim_log.png` (additive to `--monitor`) |
| `--python` | drive clips **and** gaits with the Python re-port instead of the firmware (no C++ toolchain needed) |
| `--headless` | no GUI window; CI smoke check |
| `--settle SECONDS` | hold the stance this long before the main loop (lets gravity resolve initial overlap) |

```bash
python facehugger.py sim                       # GUI, standing pose
python facehugger.py sim --trot                # trot gait (firmware)
python facehugger.py sim --clip "wave"         # play a clip (firmware)
python facehugger.py sim --trot --float        # joint geometry only, no balance
python facehugger.py sim --clip "wave" --headless --log   # CI + torque capture
```

## `blender`: open the URDF in Blender

Two modes are available. A **placement-only** scene (the default, via `visualize_urdf.py`) cross-checks the URDF rest pose against PyBullet, and a **rigged** scene (`--rigged`) gives a posable armature, IK, and foot-target Empties for animation. Requires Blender 5.0+.

| Flag | Meaning |
|------|---------|
| `--rigged` | build the animation armature (FK shoulder + IK hip/knee) instead of placement-only |
| `--blender-version V` | Blender major.minor to launch (e.g. `5.2`); default 5.1. Override the path with `BLENDER_BIN` |
| `--headless` | run Blender in `--background` mode |
| `--save PATH` | save the built `.blend` to this path |
| `--reset` | start from a blank scene, discarding existing animations |

```bash
python facehugger.py blender                   # placement-only cross-check
python facehugger.py blender --rigged           # animation rig
python facehugger.py blender --headless --save /tmp/scene.blend
```

See [URDF → Blender rig](../animation/blender-rig.md) for what the rig is and how it is built.

## `serve`: firmware-backed WebSocket API

Runs the [WebSocket robot API](../remote-control/websocket-api.md) on a PyBullet robot, with the command dispatch handled by the **compiled firmware itself**. The mobile app or `tools/robot_control_panel.html` can then drive the sim exactly as they drive the real robot.

| Flag | Meaning |
|------|---------|
| `--host HOST` | bind address; default `localhost`. Use `--host 0.0.0.0` so a phone/emulator on the LAN can connect |
| `--port PORT` | default `8081` (the robot uses 81, which is privileged on macOS/Linux) |
| `--gui` | show the PyBullet window |

```bash
python facehugger.py serve --gui                       # ws://localhost:8081
python facehugger.py serve --host 0.0.0.0 --gui        # reachable from a phone
```

It also streams a live per-joint telemetry frame over Server-Sent Events on **:8082**. See [Controlling the simulation](pybullet-control.md#serve-drive-the-sim-like-the-robot).

## `all`: urdf → sim

Convenience: regenerate the URDF then immediately run the sim, sharing the `sim` flags above. Useful as a one-shot smoke check after a CAD or config change.

```bash
python facehugger.py all --headless
```
