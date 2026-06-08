# Other command-line tools

`facehugger.py` is the front door for the everyday commands (see the [CLI](cli.md)), but it wraps only the common cases. The pipeline has a handful of other tools you run directly: the URDF generator, two verifiers, the firmware-build check, the WebSocket server, and the Blender scripts. This page is the reference for those.

All paths are relative to the repo root. The Python modules run from `code/simulation/` (so the packages import), and the Blender scripts run through a Blender binary.

## Building the URDF directly

`facehugger.py urdf` wraps this, but you can call the generator yourself when you want non-default paths:

```bash
cd code/simulation
python -m urdf_gen.generate_urdf [--export PATH] [--config PATH] [--out PATH]
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--export` | `generated/fusion_export.json` | the CAD export to read |
| `--config` | `facehugger_config.yaml` | the build config |
| `--out` | `generated/facehugger.urdf` | where to write the URDF |

It prints a per-leg shoulder-axis world-position check as it runs, so you can confirm the geometry is sane.

## Verifying the export pipeline

Two independent checks guard the clip export. Run them before flashing a new clip.

**Cross-format parity** confirms that the three servo-angle derivations for every clip frame agree (the firmware header, the browser `.js`, and the link1-corrected bone header):

```bash
cd code/simulation
python -m animation_tools.export_parity [--export-dir PATH]
```

`--export-dir` defaults to the directory holding `clips_all.h`. Exit code `0` means all formats agree, `1` means a mismatch, `2` means the files could not be read.

**Export consistency** confirms each clip's `.h` round-trips to its `.js` through the panel's `_frame_to_servo` converter, and that the standing and flat poses hit the expected servo values:

```bash
python3 animation/scripts/check_export_consistency.py [--export-dir PATH]
```

`--export-dir` defaults to `animation/exported_clips`. Same exit-code scheme (`0` pass, `1` failures, `2` file error).

## Checking the firmware build (SIL)

The compiled-firmware module (`fh_sim`) auto-rebuilds when the firmware sources change, so you rarely call this by hand. When you want to check or pre-build it:

```bash
cd code/simulation
python -m firmware_sil.sil_bridge [--check]
```

With no flag it reports freshness and rebuilds if stale. With `--check` it only reports and exits `1` if a rebuild is needed (useful in CI).

## Running the WebSocket server directly

`facehugger.py sim --serve` is the wrapper for this (and `facehugger.py serve` is a deprecated alias for `sim --serve`). The module exposes one extra flag, `--sse-port`, that the wrapper does not forward. Run the module directly if you need to move the telemetry stream:

```bash
cd code/simulation
python -m firmware_sil.ws_sim [--host HOST] [--port PORT] [--gui] [--sse-port PORT] [--panel]
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--host` | `localhost` | bind address (`0.0.0.0` to reach it from a phone) |
| `--port` | `8081` | WebSocket port (the robot uses `81`, privileged) |
| `--gui` | off | show the PyBullet window |
| `--sse-port` | `8082` | Server-Sent Events telemetry port |
| `--panel` | off | also host the browser [control panel](../remote-control/control-panel.md) over HTTP at `http://localhost:8082/panel` |

See [Controlling the simulation](pybullet-control.md#sim-serve-drive-the-sim-like-the-robot) for what to connect to it.

## Blender scripts

These run inside Blender, not the Python env. The pattern is the same for all of them: pass the script with `--python`, and pass the script's own arguments after a bare `--` (everything after `--` is handed to the script instead of Blender). Set `BLENDER_BIN` to your Blender 5.x executable.

`facehugger.py blender` already wraps the first two; document here is for running them directly or with non-default paths.

**Placement-only URDF scene** (cross-check the URDF rest pose against PyBullet; not animatable):

```bash
"$BLENDER_BIN" --python animation/scripts/visualize_urdf.py -- \
  [--urdf PATH] [--meshes PATH] [--save PATH]
```

**Rigged URDF scene** (the posable armature with IK and foot targets used for clip authoring):

```bash
"$BLENDER_BIN" --python animation/scripts/urdf_to_blender_rigged.py -- \
  [--urdf PATH] [--meshes PATH] [--json PATH] [--save PATH]
```

`--json` points at `fusion_export.json` and is used only to cross-check the foot tip; `--save` defaults to `animation/fh_rigged_latest.blend` so a rebuild always refreshes the library file. See [URDF to Blender rig](../animation/blender-rig.md).

**CAD-side cross-check scene** (imports straight from the Fusion export, before the URDF step):

```bash
"$BLENDER_BIN" --python animation/scripts/visualize_fusion_export.py -- \
  [--export PATH] [--meshes PATH] [--save PATH]
```

**Headless re-export of every clip** (writes per-clip `.h`/`.js`/`.csv` plus the bundled `clips_all.h` and `clips_manifest.json`):

```bash
"$BLENDER_BIN" --background --factory-startup \
  animation/fh_rigged_latest.blend \
  --python animation/addons/export_all_clips.py
```

