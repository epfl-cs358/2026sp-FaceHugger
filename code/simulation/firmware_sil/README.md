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
(`firmware_port/`); `test_sil_poc.py` skips automatically when `fh_sim` is absent.

### Auto-rebuild (no stale firmware)

`--sil` never runs a stale `.so`: `sil_bridge.load_fh_sim()` compares the built
module's mtime against every source under `code/firmware/src/`, `hal/`,
`bindings.cpp`, and `CMakeLists.txt`, and **recompiles before importing** if any
is newer (or the `.so` is missing). So editing firmware and re-running
`facehugger.py sim --sil` always reflects the change.

- Skip the auto-rebuild (warn loudly instead): set `FH_SIL_NO_BUILD=1`.
- CI / pre-run gate:
  ```bash
  python -m firmware_sil.sil_bridge --check   # exit 1 if stale (no build)
  python -m firmware_sil.sil_bridge           # rebuild if stale, then report
  ```

## Use it

```bash
cd code/simulation
python facehugger.py sim --clip "wave" --headless --sil   # play via the exact firmware
```
Default (no `--sil`) uses the Python re-port (`firmware_port/`).

## Verify (run from code/simulation/, not this dir)

```bash
cd code/simulation
conda run -n facehugger python -m pytest test_sil_poc.py test_sil_clip_suite.py -v
```

- `test_sil_poc.py` — the firmware plays a clip with all angles in `[0,180]`; SIL
  matches the Python re-port at frame 0 within the firmware's whole-degree
  truncation (≤ 1°); the bridge drives PyBullet headless; `--sil` runs end-to-end.
- `test_sil_clip_suite.py` — replays **every** clip through the firmware and asserts
  an **exact** match to its committed golden trace (`golden/*.json`). A firmware or
  clip-export change that shifts any servo angle fails here.

## Golden traces

`golden/<clip>.json` is the recorded servo-angle trace (10 Hz, whole degrees) the
firmware produces for each clip — the exporter/firmware regression baseline.
Regenerate **only** for intentional changes, then review the diff:

```bash
cd code/simulation && conda run -n facehugger python -m firmware_sil.gen_golden
```

## Status

Steps 1–2 of `docs/.work/pybullet-sim/EXACT-FIRMWARE-SIL-PLAN.md` done. Next:
Step 3 (WebSocket server on :81 + `tools/robot_control_panel.html`).
