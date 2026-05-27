# Trying it out

This page is a hands-on tour of what you can run and what each thing should look like when it works. It is the practical companion to the [CLI](cli.md) reference: the CLI page lists every flag, this page walks through the scenarios you actually test. Run the `facehugger.py` commands from the repo root as `python code/facehugger.py ...`, with the `facehugger` conda env active (see [Software](../../guide/software.md#host-side-simulation)). The standalone `python -m ...` tools and `pytest tests/` still run from `code/simulation/`.

## Smoke check

Start by confirming the pipeline builds and the sim runs end to end:

```bash
python code/facehugger.py urdf
python code/facehugger.py sim --headless
```

This regenerates the URDF and then runs the simulator with no window. A clean run prints the URDF shoulder-axis check and exits `0` without throwing. The first run also compiles the firmware module (`fh_sim`), which takes a moment; later runs reuse it.

Then open the GUI on the standing pose:

```bash
python code/facehugger.py sim
```

You should see the robot holding its `NEUTRAL[]` stance, steady, with no leg sliding or sinking.

## Watching gaits

```bash
python code/facehugger.py sim --walk     # walk gait
python code/facehugger.py sim --trot     # trot gait
```

Both are driven by the exact firmware. The robot should step in place / forward without falling. Add `--monitor` to print a periodic torque and current line, or `--float` to pin the body weightless so you can study the leg motion without balance getting in the way:

```bash
python code/facehugger.py sim --trot --float --monitor
```

If you have no C++ toolchain, add `--python` to drive the gait from the pure-Python re-port instead of the compiled firmware.

## Playing a clip

Clips are baked gestures compiled into the firmware. To see which names are available, run `python code/facehugger.py sim --list-clips`, read `animation/exported_clips/clips_manifest.json`, or start `sim --serve` (below) and send `T:8` (list clips). Then:

```bash
python code/facehugger.py sim --clip "wave"            # play once
python code/facehugger.py sim --clip "wave" --loop     # repeat (GUI only)
python code/facehugger.py sim --clip "wave" --float    # geometry only, no balance
```

A clip plays, then eases back to the neutral stance. `--loop` carries physics state across repeats so you can watch for drift. If you just re-exported a clip and a running `sim --serve` does not list it, restart it so it reloads the freshly compiled firmware (see [Controlling the simulation](pybullet-control.md#sim-serve-drive-the-sim-like-the-robot)).

## Torque and current

To capture rather than just watch, add `--log`. On exit it prints a summary table and writes `sim_log.csv` and `sim_log.png` to wherever you ran the command:

```bash
python code/facehugger.py sim --trot --log
```

Joints are colored against two references: green below the continuous torque, amber up to stall, red at the stall cap. Standing should be solidly green; brief fast-clip frames reading amber are honest "burst" load, not a bug. See [Torque instrumentation](pybullet-control.md#torque-instrumentation).

## Driving it like the real robot

`sim --serve` runs the same `T:` WebSocket protocol the robot uses, dispatched by the compiled firmware:

```bash
python code/facehugger.py sim --serve                 # ws://localhost:8081
python code/facehugger.py sim --serve --host 0.0.0.0  # reachable from a phone
python code/facehugger.py sim --app                   # also launches the web app
python code/facehugger.py sim --panel                 # also hosts the browser control panel
```

Two clients can drive it:

- **The browser [control panel](../remote-control/control-panel.md)** (`code/remote-control-app/control-panel/robot_control_panel.html`) is a no-build debug client for poking the sim: run `sim --panel` and open the printed `http://localhost:8082/panel`, or open the file directly and point it at `ws://localhost:8081`. Use one button per command; direction buttons auto-repeat while held.
- **The mobile app** (`code/remote-control-app/MyApp`): run `sim --app` (which launches the app for you) or `sim --serve --host 0.0.0.0`, then in the app's **Settings** screen tap the **Simulator** preset (or type the dev machine's LAN IP and port `8081`); the app drives the sim exactly as it drives the robot. Tap the **Robot** preset to target hardware.

Two faithful firmware behaviors to expect: a held direction stops after ~500 ms unless re-sent (the deadman), and moving does nothing until you select a gait (`T:5`). The panel and app handle the re-send for you. See [Controlling the simulation](pybullet-control.md#sim-serve-drive-the-sim-like-the-robot).

## Animation and export

To inspect the model in Blender or author motion:

```bash
python code/facehugger.py blender            # placement-only cross-check
python code/facehugger.py blender --rigged    # the animation rig
```

The placement scene should match PyBullet's rest pose. The rigged scene is where you pose and bake clips with the FH Clip Panel (see [Blender clip authoring](../../guide/toolchain/blender-clips.md)). After exporting, verify before flashing:

```bash
python3 animation/scripts/check_export_consistency.py   # .h/.js round-trip + pose conventions
python -m urdf_gen.verify_export_parity                  # all three formats agree
```

Both exit `0` when everything agrees. Only then wire the clip into the firmware and reflash. The full set of standalone tools and their flags is on the [other tools](scripts-cli.md) page.
