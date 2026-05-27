# firmware_sil — run the exact firmware code in the sim (SIL)

Software-in-the-loop: the **actual flashed firmware C++** (`SpinalCord`, compiled
unchanged from `code/firmware/src/`) runs on the host and computes the 12 servo
angles; `sil_bridge.py` turns those into URDF joint targets and drives PyBullet.
No Python re-port is in this path, so a clip that looks right here issues the same
servo commands on the real robot.

## How it works

```
clips_all.h (firmware copy) ──► fh_sim (compiled SpinalCord) ──► 12 servo degrees
                                       │  tick(t_ms) advances an injected clock
                                       ▼
                          sil_bridge.servo_angles_to_joint_targets
                                       ▼
                          pybullet.setJointMotorControl2  (POSITION_CONTROL)
```

The firmware sources are compiled against host shims in `hal/` (zero firmware
changes):

| `hal/` file | shadows | role |
|---|---|---|
| `Arduino.h` | `<Arduino.h>` | `millis()` reads an **injected** sim clock (`fh_sim::clock_ms`, set per tick); `map`/`constrain` real; `Serial` no-op; `String = std::string`. |
| `Wire.h` | `<Wire.h>` | empty stub. |
| `Adafruit_PWMServoDriver.h` | the PCA9685 driver | recording mock; `setPWM` stores the pulse per channel. |
| `network_stubs.h` | WiFi/WebSockets | placeholder; unused in Step 1 (`network.cpp` isn't compiled). |

> The SIL plan (D1) names ByteNana/ArduinoMock for a fuller Arduino surface. The
> compiled control files use only `millis`/`map`/`constrain`/`Serial`/`String`, so
> this hand-rolled shim is the smaller, network-free, deterministic equivalent.
> Swap in ArduinoMock via `FetchContent` if a future file needs more of Arduino.

## Build

Requires a C++17 compiler, CMake ≥ 3.15, and `pybind11` (in the conda env:
`conda install -n facehugger -c conda-forge pybind11`).

```bash
cd code/simulation/firmware_sil
cmake -S . -B build -DPython_EXECUTABLE=$(which python)
cmake --build build
```

Produces `build/fh_sim.<ext>.so`. It is **not committed** (rebuilt locally / in
CI). Contributors without a C++ toolchain skip this and use the Python re-port
(`firmware_port/`); `tests/test_sil_poc.py` skips automatically when `fh_sim` is absent.

### Auto-rebuild (no stale firmware)

The firmware driver never runs a stale `.so`: `sil_bridge.load_fh_sim()` compares
the built module's mtime against every source under `code/firmware/src/`, `hal/`,
`bindings.cpp`, and `CMakeLists.txt`, and **recompiles before importing** if any
is newer (or the `.so` is missing). So editing firmware and re-running
`facehugger.py sim --clip ...` always reflects the change.

- Skip the auto-rebuild (warn loudly instead): set `FH_SIL_NO_BUILD=1`.
- Optional pre-run freshness gate (no GitHub CI is set up for this):
  ```bash
  python -m firmware_sil.sil_bridge --check   # exit 1 if stale (no build)
  python -m firmware_sil.sil_bridge           # rebuild if stale, then report
  ```

## Use it

```bash
cd code/simulation
python facehugger.py sim --clip "wave" --headless            # DEFAULT: exact firmware (auto-built)
python facehugger.py sim --clip "wave" --headless --python   # force the Python re-port (no toolchain)
```
Clip playback **defaults to the exact firmware**; `--python` forces the re-port
(`firmware_port/`). Only `--clip` uses this driver — stand/`--walk`/`--trot` are
unaffected, so a toolchain-less machine can still run those.

## Verify (run from code/simulation/, not this dir)

```bash
cd code/simulation
conda run -n facehugger python -m pytest tests/ -v
```

- `tests/test_sil_poc.py` — the firmware plays a clip with all angles in `[0,180]`; SIL
  matches the Python re-port at frame 0 within the firmware's whole-degree
  truncation (≤ 1°); the bridge drives PyBullet headless; the default clip path
  (no flag) runs end-to-end through the firmware.
- `tests/test_sil_clip_suite.py` — replays **every** clip through the firmware and asserts
  an **exact** match to its committed golden trace (`golden/*.json`). A firmware or
  clip-export change that shifts any servo angle fails here.

## Golden traces

`golden/<clip>.json` is the recorded servo-angle trace (10 Hz, whole degrees) the
firmware produces for each clip — the exporter/firmware regression baseline.
Regenerate **only** for intentional changes, then review the diff:

```bash
cd code/simulation && conda run -n facehugger python -m firmware_sil.gen_golden
```

## WebSocket robot API (drive from the app / a browser panel)

`ws_sim.py` serves the API_SPEC `T:` protocol and drives a PyBullet robot with the
firmware. **The command routing is the compiled firmware**: each message is handed
to the firmware's own `network.cpp::handleParsedMessage` (compiled into `fh_sim`
alongside ArduinoJson + `clip_list_serializer`), so any change to the firmware's
API handling reflects automatically — no Python mirror to drift. Only the
WebSocket transport and the periodic telemetry are Python.

```bash
cd code/simulation
python facehugger.py serve                 # ws://localhost:8081  (or: python -m firmware_sil.ws_sim)
```

Then open `tools/robot_control_panel.html` in a browser (no build/deps): set the
target to `ws://localhost:8081` and Connect. One button per API command (clip
discovery T:8, play T:7, state T:2, move T:1, gait T:5, invert T:6, calibrate T:4);
it shows the exact JSON sent and logs replies. The **same panel drives the real
robot** — point it at `ws://<robot-ip>:81`. The unmodified app can connect too.

Port note: the real robot uses **81** (privileged on macOS/Linux), so the sim
defaults to **8081**. `--port 81` works with sudo for true parity.

### Drive the real Expo app against the sim

The full mobile app (`code/remote-control-app/MyApp`) can drive the sim exactly as
it drives the robot. The app's WebSocket endpoint is configurable in
`MyApp/config/config.ts`:

1. Set `webSocketPort = 8081` (it defaults to `81`, the robot).
2. Set `webSocketIP` to the dev machine's **LAN IP** (not `localhost`, unless you
   run the app's web build on the same machine).
3. Run the server bound to all interfaces so a phone/emulator can reach it:
   ```bash
   python facehugger.py serve --host 0.0.0.0   # WS API on :8081 (+ SSE telemetry on :8082)
   ```
4. Start the app (`npm run web` / `npm run ios` / `npm run android`).

The app and the Mac must be on the same network. Revert `webSocketPort` to `81`
to target the robot again.

## Sim telemetry (SSE, local debug only)

Alongside the WS API, `serve` streams a per-joint telemetry frame over
**Server-Sent Events on :8082** (`http://localhost:8082/telemetry`) at ~20 Hz. This
is sim-only — the real robot doesn't serve it — and it's best-effort: if
`sse-starlette`/`uvicorn` aren't installed the WS API still runs, telemetry just
disabled. Each frame is `{t, joints:[12 × {…}]}` in order FL, FR, BL, BR ×
shoulder/hip/knee, where every joint carries **both** spaces:

- `commanded_servo_deg` — the firmware's 0–180 output (what the real servos receive, 90 = neutral),
- `commanded_joint_deg` — that same command in URDF-joint degrees (0 = neutral), comparable to actual,
- `actual_joint_deg` — the measured PyBullet joint angle,
- `delta_deg` — `commanded_joint − actual` (a true tracking error),
- `torque_nm` / `current_a` — from PyBullet + `sim_monitor`,
- `pre_clamp_deg` — the firmware's pre-clamp `[OOR]` request for that servo this tick, else null.

`tools/robot_control_panel.html` renders this in its "Sim telemetry" table (Δ red
when |Δ|>5°, τ coloured by `sim_monitor.band()` — green = safe continuous
(<~0.98 N·m), amber = burst (under stall), red = saturated (≥2.94 N·m stall) — a
Clamp column from `pre_clamp_deg`, and a stale banner when frames stop). With
`--gui`, the PyBullet links are torque-tinted the same way every 12th step. The `pre_clamp_deg` value comes from a real firmware
feature: `Leg::setJointAngles` / `Servo::setServoAngle` print `[OOR] servo <ch>
requested <deg>` before clamping to [0,180] (visible on the bench serial monitor),
which the SIL's Serial mock captures.

```bash
python facehugger.py serve --gui            # ws://localhost:8081 + SSE http://localhost:8082/telemetry
```

## Status

Steps 1–3 of `docs/.work/pybullet-sim/EXACT-FIRMWARE-SIL-PLAN.md` done: SIL clip
playback, the golden-trace suite, the `--python` fallback, and the firmware-backed
WebSocket API + control panel. Plus: firmware `[OOR]` pre-clamp warnings (captured by
the SIL) and the SSE sim-telemetry stream (:8082) + panel table.
