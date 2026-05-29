# FaceHugger CLI

`facehugger.py` is the single entry point for the simulation half of the project. It is a thin dispatcher: each subcommand shells out to the right module (`urdf_gen.generate_urdf`, `pybullet_sim.simulate`, the Blender scripts, the firmware-backed WebSocket server, or PlatformIO) so you never invoke those by path.

Run it from the repo root:

```bash
python code/facehugger.py <subcommand> [options]
```

It resolves its own paths and shells into the `code/simulation/` packages, so the working directory does not matter.

## Subcommands at a glance

| Subcommand | What it does |
|------------|--------------|
| `urdf`   | Regenerate `generated/facehugger.urdf` from the Fusion export + config |
| `sim`    | Run the robot in software (compiled firmware + PyBullet + the hardware WebSocket API) |
| `blender`| Open the URDF in Blender (placement cross-check, or the animation rig) |
| `app`    | Launch the Expo web app (the mobile remote-control client in a browser) |
| `flash`  | Build and upload the firmware to the ESP32 via PlatformIO |

`serve` still works as a **deprecated alias** for `sim --serve` (it prints a note). `all` was removed; run `urdf` then `sim` instead.

## `urdf`: regenerate the URDF

Rebuilds `generated/facehugger.urdf` from `generated/fusion_export.json` (the CAD tree + mesh manifest) and `facehugger_config.yaml`. Everything downstream (the sim, the Blender rig, the IK reference cases) is derived from this URDF, so regenerate it whenever the CAD export or the config changes.

| Flag | Meaning |
|------|---------|
| `--export PATH` | path to `fusion_export.json` (default: `generated/`) |
| `--config PATH` | path to `facehugger_config.yaml` |
| `--out PATH`    | output URDF path |

```bash
python code/facehugger.py urdf
```

## `sim`: the robot in software

`sim` runs **the robot in software**: the exact compiled firmware driving a PyBullet robot, fronted by the same hardware WebSocket API. With no drive flag the robot just stands. Add one drive mode and any number of the other flags. **By default, clips and gaits are driven by the exact compiled firmware** (see [Controlling the simulation](pybullet-control.md)); `--python` switches to the Python re-port.

**Drive** (pick at most one; none = stand):

| Flag | Meaning |
|------|---------|
| `--walk` / `--trot` | run that gait (the exact firmware `tickGait`/`tickTrot`) |
| `--clip NAME` | play a baked animation clip by name (from `clips_all.h`) |
| `--loop` | replay the clip continuously (GUI only) to watch cumulative behaviour |

**Backend**:

| Flag | Meaning |
|------|---------|
| `--python` | drive clips **and** gaits with the Python re-port instead of the firmware (no C++ toolchain needed) |

**Inspect**:

| Flag | Meaning |
|------|---------|
| `--float` | no gravity/floor, body pinned; inspect pure joint geometry without falling/slipping |
| `--monitor` | print a periodic torque + estimated-current status line (peak τ, total A, `[CONT]`/`[STALL]` joints) |
| `--log` | record per-step torque/current → summary + `sim_log.csv` + `sim_log.png` (additive to `--monitor`) |
| `--headless` | no GUI window; CI smoke check |
| `--settle SECONDS` | hold the stance this long before the main loop (lets gravity resolve initial overlap) |

**Info**:

| Flag | Meaning |
|------|---------|
| `--list-clips` | print the clip ids/names compiled into `clips_all.h` and exit |

**Interface** (drive `sim` from an external client instead of the CLI):

| Flag | Meaning |
|------|---------|
| `--serve` | expose the [WebSocket robot API](../remote-control/websocket-api.md) (`T:` protocol) plus SSE telemetry on `:8082`, so the app or control panel can drive the sim |
| `--host HOST` | bind address; default `localhost`. Use `--host 0.0.0.0` so a phone/emulator on the LAN can connect |
| `--port PORT` | WebSocket port; default `8081` (the robot uses 81, privileged on macOS/Linux) |
| `--app` | implies `--serve` **and** launches the Expo web app as a child process |
| `--app-port PORT` | port for the web app; default `8080` |
| `--panel` | implies `--serve` **and** hosts the browser [control panel](../remote-control/control-panel.md) over HTTP at `http://localhost:8082/panel` |

```bash
python code/facehugger.py sim                       # GUI, standing pose
python code/facehugger.py sim --trot                # trot gait (firmware)
python code/facehugger.py sim --clip "wave"         # play a clip (firmware)
python code/facehugger.py sim --trot --float        # joint geometry only, no balance
python code/facehugger.py sim --clip "wave" --headless --log   # CI + torque capture
python code/facehugger.py sim --list-clips          # print compiled clip ids and exit
python code/facehugger.py sim --serve               # WebSocket API on :8081 + SSE on :8082
python code/facehugger.py sim --app                 # sim + API + the web app (full session)
python code/facehugger.py sim --panel               # sim + API + browser control panel
```

The full interactive session is `python code/facehugger.py sim --app`: it brings up the sim, the WebSocket API, and the web app together. Drive the app at the sim by tapping the app's **Settings** "Simulator" preset.

## `blender`: open the URDF in Blender

Two modes are available. A **placement-only** scene (the default, via `visualize_urdf.py`) cross-checks the URDF rest pose against PyBullet, and a **rigged** scene (`--rigged`) gives a posable armature, IK, and foot-target Empties for animation. Requires Blender 5.0+.

Before opening, `blender` **auto-refreshes a stale URDF**: it regenerates `generated/facehugger.urdf` if that file is older than `fusion_export.json` or `facehugger_config.yaml`, so the scene always reflects the current CAD and config. Pass `--skip-urdf-check` to opt out.

| Flag | Meaning |
|------|---------|
| `--rigged` | build the animation armature (FK shoulder + IK hip/knee) instead of placement-only |
| `--rebuild-rig` | force a from-scratch rig build; does **not** reopen `fh_rigged_latest.blend`, so its clips are discarded |
| `--skip-urdf-check` | skip the stale-URDF refresh and open the existing URDF as-is |
| `--blender-version V` | Blender major.minor to launch (e.g. `5.2`); default 5.1. Override the path with `BLENDER_BIN` |
| `--headless` | run Blender in `--background` mode |
| `--save PATH` | save the built `.blend` to this path |
| `--reset` | start from a blank scene, discarding existing animations |

```bash
python code/facehugger.py blender                   # placement-only cross-check
python code/facehugger.py blender --rigged           # animation rig
python code/facehugger.py blender --headless --save /tmp/scene.blend
```

See [URDF → Blender rig](../animation/blender-rig.md) for what the rig is and how it is built.

## `app`: launch the web app

Starts the Expo web app, the mobile remote-control client running in a browser. On its own it is just the app; point it at a running sim (`sim --serve`) or the real robot from the app's **Settings** screen. For the all-in-one session that brings up the sim and the app together, use `sim --app`.

## `flash`: build and upload firmware

Builds and uploads the firmware to the ESP32 via PlatformIO.

| Flag | Meaning |
|------|---------|
| `--build-only` | compile but do not upload |
| `--monitor` | open the serial monitor after uploading |
| `--env ENV` | PlatformIO environment; default `upesy_wroom` |

```bash
python code/facehugger.py flash                     # build + upload
python code/facehugger.py flash --build-only        # compile only
python code/facehugger.py flash --monitor           # upload then watch serial
```

## Driving `sim` like the real robot

The `T:` WebSocket API, the web app, and the browser control panel are all exposed through `sim`'s interface flags (`--serve`, `--app`, `--panel`) documented above. The standalone `serve` subcommand is a deprecated alias for `sim --serve`. See [Controlling the simulation](pybullet-control.md#sim-serve-drive-the-sim-like-the-robot) for what to connect and how.
