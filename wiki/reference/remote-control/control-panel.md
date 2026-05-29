# Browser control panel

The control panel is a no-build, single-file browser debug client for the robot. It is one HTML page with one button per `T:` command (list and play clips, set the gait, move, invert, calibrate), plus a sticky rail that shows all-leg telemetry. There is nothing to install: it talks the same [WebSocket `T:` protocol](websocket-api.md) the firmware speaks, so it drives the simulator and the real robot identically.

The [mobile app](index.md) is the primary client. The control panel exists for quick debugging from a laptop: poking individual commands, watching telemetry, and sanity-checking a clip without the app.

## Where it lives

```
code/remote-control-app/control-panel/robot_control_panel.html
```

## Opening it

The quickest way is to let the simulator host it:

```bash
python code/facehugger.py sim --panel
```

`--panel` implies `--serve` (so the WebSocket API comes up on `:8081`) and serves the panel over HTTP. Open the printed URL, `http://localhost:8082/panel`, and it is already pointed at the sim.

Otherwise open the file directly in a browser (double-click it, or use a `file://` URL) and set the target to `ws://localhost:8081` to reach a running `sim --serve`.

## Driving the real robot

The panel is not sim-only. Point its target at the robot's WebSocket instead and it drives hardware:

```
ws://<robot-ip>:81
```

On the ESP32 access point that is `ws://192.168.4.1:81`.

## The deadman caveat

The firmware deadman is faithful in the sim, so the panel must keep a held direction alive:

- Set a gait first with `T:5`. A move command does nothing until a gait is selected.
- Re-send held directions. A single move command stops after about 500 ms; the panel's direction buttons auto-repeat while held to beat the timeout.

This is real firmware behaviour, not a panel or sim quirk. See [Controlling the simulation](../simulation/pybullet-control.md#sim-serve-drive-the-sim-like-the-robot) for the same notes from the sim side.
