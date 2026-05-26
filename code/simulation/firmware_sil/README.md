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

## Verify

```bash
conda run -n facehugger python -m pytest test_sil_poc.py -v
```

Proves: the firmware plays a clip with all servo angles in `[0,180]`; SIL output
matches the Python re-port's `translate_to_servo` at frame 0 to within the
firmware's whole-degree truncation (≤ 1°); and the bridge drives PyBullet joints
through a full clip headless.

## Status

Step 1 of `docs/.work/pybullet-sim/EXACT-FIRMWARE-SIL-PLAN.md` (proof of concept).
Not yet wired into `facehugger.py sim --sil` — that's Step 2.
