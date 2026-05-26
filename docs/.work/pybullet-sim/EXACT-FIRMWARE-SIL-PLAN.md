# Plan — run the *exact* firmware code in the PyBullet sim (SIL)

**Status:** PLAN ONLY (2026-05-26). **Decisions D1–D6 LOCKED** (§9) — still nothing
implemented; this is the agreed spec to build from, step by step. It compiles the
**actual flashed firmware C++** and drives the PyBullet robot with it — so a
clip/gait that looks right in sim is *guaranteed* to issue the same servo commands
on the real robot, and any firmware change is testable in sim before flashing.
Extends `GAIT-AND-PARITY-PLAN.md` (its "Approach B", expanded with the JS/WebSocket
layer + mass tuning).

### Locked decisions (the short version)
- **D1 — Mock the hardware, zero firmware changes.** **ByteNana/ArduinoMock** (via
  CMake `FetchContent`) for the `Arduino.h` surface, with `millis()` backed by an
  injected sim-clock variable in `hal/` (not a mock expectation); a recording
  `Adafruit_PWMServoDriver` mock; `WiFi`/`WebSocketsServer`/`network.cpp` **stubbed**
  (log + return, not compiled). All shims isolated in `firmware_sil/hal/`; firmware
  source untouched. (`ArduinoFake` was the original idea but isn't a CMake package —
  see §2.)
- **D2 — Python `websockets` server on :81** + a **standalone single-file HTML
  control panel** that drives *both* the sim and the real robot.
- **D3 — `pybind11`** for the bindings.
- **D4 — Clips first** (validates the exporter), then gait, then WS/JS — but the WS
  server + HTML panel are built alongside from the start.
- **D5 — Keep the Python re-port**, relocated to `code/simulation/firmware_port/`
  as the no-C++-toolchain fallback.
- **D6 — `code/simulation/firmware_sil/`**, built with **CMake** (points at the
  firmware sources directly), as a sibling of `pybullet_sim/` and `firmware_port/`.
- **`--sil` flag** switches the joint driver to the firmware SIL bridge; default
  (no flag) keeps the Python re-port so nothing breaks without a C++ toolchain.

---

## 1. What you want, and why it's the right call

> Run the same C++ I flash to the ESP32 inside the sim, so testing the exported
> clips (and gaits, and any firmware change) in simulation proves they'll do the
> right thing on the real robot — and flag any angle that shouldn't run.

This is **software-in-the-loop (SIL)**: the firmware's *logic* runs on the host,
its *hardware* is replaced by the simulator. It's the right call because it kills
the one weakness of the current approach — the sim's clip interpreter is a
hand-written **Python re-port** of the firmware, so it can silently drift from the
C++. SIL removes the re-port entirely: there is **one** implementation (the
firmware), exercised in both places. If the exporter feeds bytes that the firmware
mis-handles, the sim mis-behaves the same way — which is exactly the bug you want
to catch before flashing.

---

## 2. Can we use the firmware code directly? (the honest mechanics)

**No — not as an import.** It's C++; Python can't run it directly. The path is
**compile → load → call**:

1. **Compile** the firmware control sources into a host shared library
   (`.dylib`/`.so`) — *the same `.cpp` files* that compile to the ESP32, just for
   the host CPU.
2. **Load** that library from Python via a foreign-function interface (FFI).
3. **Call** it each sim step: feed a command + a synthetic clock, read back the 12
   servo angles, and drive PyBullet with them.

The obstacle is that the interesting files are **hardware-coupled**. Concretely
(verified in the tree):

| File | Hardware/timing deps | In `native` build today? |
|---|---|---|
| `motion_math.cpp` | none (`#ifndef ARDUINO`-clean) | ✅ yes |
| `kinematics.cpp` | none | ✅ yes |
| `spinal_cord.cpp` (**holds `tickGait`/`tickClip`**) | `<Arduino.h>`, `<Wire.h>`, `<Adafruit_PWMServoDriver.h>`, `millis()`, `Serial` | ❌ no |
| `servo.cpp` | same + `pwm.setPWM()`, `map()` | ❌ no |
| `leg.cpp` | same | ❌ no |
| `network.cpp` | `WebSocketsServer`(81), `WiFi`, `ArduinoJson` | ❌ no |

So `translateToServo` (the math) already compiles on the host — but the *control
logic that calls it* (`tickGait`, `tickClip`, the state machine) does not, because
it pulls in Arduino + the PCA9685 driver + `millis()`. To run that exact code on
the host we must satisfy those dependencies **without changing the firmware logic**.

### How we make the exact code compile on the host — **Option A (LOCKED)**

Mock the hardware layer; **zero firmware changes**. All shims live in
`firmware_sil/hal/` and are put on the include path *ahead of* the real Arduino
libs, so the exact `spinal_cord.cpp` / `servo.cpp` / `leg.cpp` compile unmodified.

| Firmware include | Host shim in `firmware_sil/hal/` | Behaviour |
|---|---|---|
| `<Arduino.h>` | **ByteNana/ArduinoMock** (via CMake) + our own millis backing | `Arduino.h`/`Stream`/`HardwareSerial`/`WString` shims from ArduinoMock; **`millis()`/`micros()` backed by an injected sim-clock variable** (see note); `delay()` no-op; `map()`/`constrain()` real. |
| `<Wire.h>` | `Wire.h` | empty stub (only `#include`d, never used for logic). |
| `<Adafruit_PWMServoDriver.h>` | `Adafruit_PWMServoDriver.h` | **recording mock** — `setPWM(channel, 0, pulse)` stores `(channel, pulse)` in a buffer the Python side can read; `setPWMFreq`/`begin` no-ops. |
| `<WiFi.h>`, `<WebSocketsServer.h>` (network) | `network_stubs.h` | **stub out completely** — log the call and return; **`network.cpp` is not compiled at all** (the Python `websockets` server replaces it, §5). |
| `<ArduinoJson.h>` | the real lib | cross-platform; only needed if we later compile the command dispatch (not in scope now). |

#### The Arduino mock — verified choice (D1 detail)
*Correction:* `ArduinoFake` is **not** a CMake package (it's a PlatformIO-registry /
FakeIt-based lib), so it can't be pulled by CMake. Verified the two clean options:
- **ByteNana/ArduinoMock** — has a real `CMakeLists.txt`; integrate via
  `FetchContent` (or `add_subdirectory(arduino)`) and
  `target_link_libraries(<tgt> PRIVATE ArduinoNativeMocks)`; exposes `src/` as a
  public include dir; ships `Arduino.h`/`WString.h`/`Stream.h`/minimal
  `HardwareSerial`/`times.h(millis,delay)`. GoogleTest/GoogleMock-based (only for
  *its* tests — as a consumer we just link the shim lib).
- **Vendor ArduinoFake's `src/` headers** into `hal/` — header-based (FakeIt
  single-header), no PlatformIO tooling; `millis` mocked via
  `When(Method(ArduinoFake(), millis)).AlwaysDo(λ)`.

**Decision: ByteNana/ArduinoMock via `FetchContent`** for the Arduino surface — it's
the CMake-native one. **But `millis()` is *not* taken from a mock-framework
expectation** (gmock/FakeIt expectations are for unit tests, wrong tool for a clock
called every tick in a long-running sim). Instead `hal/` provides a tiny millis
backing:
```cpp
// firmware_sil/hal/sim_clock.h  (concept, not final)
namespace fh_sim { extern uint32_t clock_ms; }
inline uint32_t millis() { return fh_sim::clock_ms; }   // shadows ArduinoMock's millis
```
The pybind11 bridge sets `fh_sim::clock_ms = step * 1000 / 240` before each
`tick()`. Deterministic, framework-free in the hot loop. (If ArduinoMock's own
`millis()` turns out to be a settable plain stub, use that directly; otherwise our
backing wins because `hal/` is first on the include path.)

Notes:
- **`millis()` is the determinism hinge** — it comes from the injected sim clock,
  never the host wall clock. This is why we own its backing (above) rather than
  leaving it to the mock framework.
- **The recording PCA9685 mock is the seam.** Read servo *degrees* two ways:
  (a) invert the firmware's `map(deg,0,180,150,600)` from the stored pulse, or
  (b) call `Servo::getServoAngle()` (the firmware stores the post-clamp angle).
  The bridge exposes both; (b) avoids integer-PWM truncation, (a) also verifies the
  electrical map if wanted.
- **Why not the HAL-seam alternative:** cleaner firmware but requires touching
  firmware; you chose A so the SIL provably runs *exactly* the flashed source.

### Why this guarantees what you want
Either way, the bytes from the exporter → `clips_all.h` → **the exact firmware
`tickClip`/`translateToServo`** → 12 servo angles → PyBullet. The sim and the
robot run the *same code* on the *same data*, so "passes in sim" ⇒ "issues the
same commands on hardware." That is the guarantee the current Python re-port
cannot make.

---

## 3. The FFI binding — how Python calls the firmware

**`pybind11` (LOCKED, D3).** Bind the C++ `SpinalCord` class directly to a Python
object — the control core is class-based, so binding the object is the natural fit
(richer than `ctypes`: call methods, return arrays). `pybind11` is the build
dependency and the CMake target links it. (`ctypes` was the lighter alternative;
not chosen.)

### Entry-point contract (the seam the sim drives)
```
class FirmwareControl:                 # pybind11 wrapper over SpinalCord
    def command(self, json_str): ...   # feed an API_SPEC T: command (or a typed call)
    def tick(self, t_ms: int): ...     # advance the firmware one logical tick at sim time
    def servo_angles(self) -> list[12] # the 12 recorded servo degrees (FR,FL,RR,RL × sh,th,kn)
```
The sim loop becomes: per 240 Hz step, `fw.tick(step * 1000/240)`, read
`servo_angles()`, convert to URDF radians via the **existing**
`servo_convention.servo_to_radians` + `LEG_ID_TO_URDF_AXIS_SIGN` (the sim-only
axis correction), and `setJointMotorControl2`. The Python re-port's
`frame_to_joint_targets` kernel is reused **only for the servo-deg→radian→axis
step**; the servo *degrees themselves now come from the firmware*, not the port.

---

## 4. Where it lives + how a firmware change gets tested

**The agreed layout (D5/D6, LOCKED):**
```
code/simulation/
  pybullet_sim/                    physics runtime (simulate/gaits/kinematics/
                                   helpers/constants/sim_monitor) — unchanged
  firmware_port/                   ← the Python re-port, RELOCATED here (D5)
    servo_convention.py            (was pybullet_sim/interpreter/servo_convention.py)
    clip_player.py  clip_loader.py  gait_interpreter.py  __init__.py  tests/
  firmware_sil/                    ← NEW (D6), the exact-firmware SIL
    hal/
      Arduino.h                    millis() injectable, delay() no-op, Serial→stdio
      Wire.h                       empty stubs
      Adafruit_PWMServoDriver.h    recording mock — setPWM writes to a buffer
      network_stubs.h              WiFi/WebSocketsServer no-ops that log calls
    bindings.cpp                   pybind11: the FirmwareControl class over SpinalCord
    CMakeLists.txt                 points at code/firmware/src/... by path + pybind11
    sil_bridge.py                  load the .so, tick, read servo angles → PyBullet
tools/
  robot_control_panel.html         standalone API tester for sim AND real robot (§5)
```

Key points:
- **CMake (not PlatformIO native)** because we need `pybind11`; CMake points at the
  firmware sources **by path** (`code/firmware/src/nervous_system/*.cpp`) — no
  copies, so nothing drifts — with `firmware_sil/hal/` first on the include path so
  the shims shadow the real Arduino/Adafruit/network headers. `network.cpp` is
  excluded from the source list.
- **The re-port relocation (D5)** is a rename `pybullet_sim/interpreter/ →
  firmware_port/`. It carries an import-rewiring sub-task: `pybullet_sim/gaits.py`
  (`from .interpreter.clip_loader import …`) and `urdf_gen/verify_export_parity.py`
  (`from pybullet_sim.interpreter… import …`) must repoint to `firmware_port`, and
  the interpreter tests' `sys.path`/imports update accordingly. Guard it with the
  existing `test_pipeline_regression.py` net (it must stay green across the move).
- **The `--sil` flag (LOCKED):** `facehugger.py sim --sil` (and the underlying
  `simulate.py`) switches the joint driver from `firmware_port` (the Python
  re-port) to `firmware_sil.sil_bridge`. **Default = no flag = Python re-port**, so
  contributors without a C++ toolchain are unaffected.

**Testing any firmware change:** rebuild `fh_sim` (CI step), then:
- **Clip validation:** play every clip in `clips_all.h` through the SIL; assert no
  servo leaves its mechanical range, and (regression) that the servo-angle
  sequence matches a stored golden trace. A bad exporter or a firmware edit that
  drives an out-of-range angle **fails the test**.
- **Out-of-range flagging:** the firmware already clamps to `[0,180]` in
  `setServoAngle`; the SIL records the *pre-clamp* requested angle too, so the sim
  can flag "servo X was commanded to 195° (clamped)" — surfacing authoring/firmware
  bugs that the robot would silently swallow.
- **Gait/changes:** same harness, driven by pinned `(gait, X/Y/Yaw, t)` inputs.

This is the "any change on the firmware is testable in the sim" property, for real.

---

## 5. Simulating the JS app + the WebSocket API entrypoint (full-stack SIL)

The robot's control surface is a WebSocket server on **port 81** speaking the
`T:`-tagged JSON protocol (`code/API_SPEC.md`). To test the JS/app against the
simulated robot, stand up that same entrypoint in front of the SIL core:

```
  React-Native/Expo app (unmodified)                 ← the real JS, unchanged
        │  ws://localhost:81   (API_SPEC T: commands)
        ▼
  sim WebSocket server  ──►  FirmwareControl.command()  ──►  exact firmware dispatch
        │                                                     │
        │                                          tick(t) → servo_angles()
        ▼                                                     ▼
   telemetry/state back to app                        PyBullet robot moves
```

**Command-handling layer — Python `websockets` server (LOCKED, D2).** A Python
`websockets` server on :81 parses the `T:` JSON and calls
`FirmwareControl.command(...)`. The motion code is exact; only the transport+parse
glue is Python. The app can't tell it's a sim. (We do **not** compile
`network.cpp`; it's stubbed per D1.)

### The standalone HTML control panel (D2) — primary test tool for sim **and** robot
A single self-contained file `tools/robot_control_panel.html` — **no framework, no
build, no deps** — that talks raw WebSocket to either target:
- **IP/port field** at the top, default `localhost:81` (sim); type the robot's IP to
  drive the real hardware over the same protocol. A Connect/Disconnect + status dot.
- **One button per API command** in `code/API_SPEC.md` (`T:1` move, `T:2` state,
  `T:5` gait, `T:6` invert, `T:7` play clip, `T:8` list clips, `T:4` calibrate, …),
  with the few inputs each needs (direction, gait id, clip name, channel/angle).
- **Shows the exact JSON** it will send on each click (so you can eyeball the wire
  format), and **shows the response/ack** streamed back.
- Because it speaks the protocol, not an SDK, it works **identically against the sim
  WS server and the real robot** — your one tool for both. It also doubles as living
  documentation of `API_SPEC.md` and a manual regression check for protocol drift.

This panel is built **alongside** the WS server from the start (D4), even before the
gait/WS motion paths are fully wired — it's immediately useful for poking the sim
and the robot by hand.

**The JS app itself:** the unmodified Expo app (or a headless Playwright run of the
web build) can also point at `ws://localhost:81` and drive the simulated robot
end-to-end — app → protocol → firmware command handling → motion, on the desktop,
before touching hardware.

---

## 6. Mass & physical tuning (the part that is *not* guaranteed)

SIL guarantees the **commands** (servo angles) match. It does **not** make the
robot *move* like reality — that depends on the physics model. To make the sim's
motion trustworthy, tune the model against measurements:

### 6.1 Mass budget
- **Per-servo mass:** `facehugger_config.yaml` currently has `mass_kg: 0.300` as an
  *unverified placeholder* (you noted you haven't weighed them). → weigh one
  QYRC DSS-230MG; set it.
- **Per-link inertials:** the URDF gets inertials from the Fusion *physics* export
  when available; where it falls back to servo mass, the link masses are rough. →
  audit `generate_urdf.py`'s inertial path; prefer CAD density-based mass per body.
- **Chassis/electronics:** the `0.8 kg` `BODY_MASS_KG` (ESP32 + PCA9685 + battery +
  frame) is an estimate. → weigh the assembled chassis; split into base-link mass +
  battery position (affects CoM, which dominates balance).

Expose all of these in one place (config) so tuning doesn't touch code.

### 6.2 Servo dynamics
PyBullet's `POSITION_CONTROL` is a PD model capped by `force` (2.94 N·m) and
`maxVelocity` (5 rad/s). The real DSS-230MG has a torque/speed curve and finite
stiffness. → the `--log` torque/current trace is the instrument: run a known motion
on the **real** robot (measure current draw / timing), run the same clip in sim,
and tune `force`/`maxVelocity`/`positionGain` until the sim's `--log` trace matches
the measured current/timing. This is iterative; document a fixed calibration
routine (e.g. "single leg lift" clip) as the reference.

### 6.3 Friction / contact
`lateralFriction=1.2`, `spinningFriction=0.05`, `restitution=0` are guesses. → tune
against observed foot-slip on the real surface; a "push test" (does it slide when
it should grip?) calibrates lateral friction.

### 6.4 The protocol
1. Pick 2–3 reference motions (a clip, a gait, a static load).
2. Measure on hardware: joint angles (servo feedback if available), current draw,
   gross body motion (video / IMU).
3. Run identical in sim with `--log`; compare.
4. Tune mass → friction → servo force/gains, in that order (mass dominates).
5. Lock the tuned config; re-run the SIL clip suite to confirm commands unchanged.

This is **best-effort fidelity (G2)** — bounded by measurement quality, never
"proven." Be explicit with users about the two-tier guarantee (§7).

---

## 7. How strong is the guarantee? (two tiers)

- **Command fidelity (G1) — GUARANTEED, bit-for-bit.** With SIL, the sim runs the
  exact firmware `tickClip`/`tickGait`/`translateToServo` on the exact data. The
  servo angles are identical to the robot's by construction (same compiled logic).
  The only residual: the firmware's **per-tick smoothers** (clip EMA `0.75`, input
  `*0.1`) are loop-rate dependent — but SIL *runs the real smoother code*, so if the
  sim ticks at the firmware's cadence it matches; if it ticks at 240 Hz the
  transient differs (steady-state identical). You control the tick rate, so you can
  match it. And `uint16_t` degree quantisation is reproduced (it's the real code).
  → **A clip that passes SIL will issue the same commands on the robot. Guaranteed.**
- **Physical fidelity (G2) — APPROXIMATE.** Whether the robot *balances / slips /
  tips* the same is a tuning problem (§6), never exact. SIL + good mass/friction
  tuning gets you "plausible and useful for catching gross failures (falls,
  collisions, out-of-range)", not "physically certified."

The sentence to put on the tin: **"If it commands correctly in sim, it commands
correctly on the robot (guaranteed). Whether it *moves* the same is tuned, not
guaranteed."** For validating your exporter, G1 is exactly what you need.

---

## 8. Build & CI integration; risks

- **Build:** a `native`-style target (PlatformIO `env:native` already exists, or a
  standalone CMake) compiles the firmware sources + HAL/mock + `sim_api` into
  `fh_sim.{so,dylib}`. The sim's `uv` dev loop gains an optional "build the SIL lib"
  step; CI builds it once and runs the SIL clip suite. Keep the Python re-port as
  the no-build fallback so `facehugger.py sim` works without a C++ toolchain.
- **Risks / honest caveats:**
  - *float determinism:* host `double` == ESP32 soft-float `double` for IEEE-754
    ops; safe. (Don't rely on `float` extended precision — the code uses `float`
    intermediates; same on both.)
  - *third-party libs on host:* `Adafruit_PWMServoDriver` and `WebSocketsServer` are
    Arduino-oriented — hence mock them (A) or seam them out (B); don't try to build
    them on host.
  - *`millis()` source:* must be injected/synthetic for determinism — never the host
    wall clock.
  - *two `clips_all.h` copies* (firmware vs exporter) already drift by hand (memory
    note); SIL reading the *firmware* copy is actually the right anchor — make the
    SIL clip suite read `code/firmware/src/nervous_system/clips_all.h`, so it tests
    what the robot will run.
  - *toolchain:* contributors without a C++ compiler fall back to the Python
    re-port; document this.

---

## 9. Decisions — LOCKED (2026-05-26)

- **D1 — Option A, mock the hardware, zero firmware changes.** **ByteNana/ArduinoMock**
  (CMake `FetchContent`) for the `Arduino.h` surface; `millis()` backed by an injected
  sim-clock variable in `hal/`; recording `Adafruit_PWMServoDriver` mock (stores
  `(channel, pulse)`); `WiFi`/`WebSocketsServer`/`network.cpp` **stubbed out**
  (log + return, not compiled). All shims isolated in `firmware_sil/hal/`. (Not
  ArduinoFake — it isn't a CMake package; §2.)
- **D2 — Python `websockets` server on :81**, plus a standalone single-file
  `tools/robot_control_panel.html` (IP/port field, one button per API command,
  shows sent JSON + response) that drives **both** the sim and the real robot.
- **D3 — `pybind11`** for the bindings.
- **D4 — Clips first** (validates the exporter), then gait, then WS/JS; the WS
  server + HTML panel are built alongside from the start.
- **D5 — Keep the Python re-port**, relocated to `code/simulation/firmware_port/`;
  it stays as the no-C++-toolchain fallback (not deleted).
- **D6 — `code/simulation/firmware_sil/`** (sibling of `pybullet_sim/` and
  `firmware_port/`), built with **CMake** pointing at the firmware sources by path.
- **`--sil` flag** selects the SIL bridge; default keeps the Python re-port.

---

## 10. Sequencing (locked scope — implement step by step when we start)

**Step 0 — Relocate the re-port (D5). ✅ DONE.** `git mv pybullet_sim/interpreter/
→ firmware_port/`; repointed imports in `gaits.py`, `verify_export_parity.py`, and
the interpreter tests (`parents[3]→[2]`, `pybullet_sim.interpreter→firmware_port`);
`clip_loader` repo-root walk `parents[4]→[3]`; the structure test now pins
`firmware_port`. Pure move — 71 tests green, clip playback + parity unchanged. Gives
the SIL a clean sibling to slot next to.

**Step 1 — SIL proof of concept (Option A, one clip). ✅ DONE.** Scaffolded
`firmware_sil/`: `hal/` shims, `CMakeLists.txt` (CMake → `pybind11`, points at
`code/firmware/src/nervous_system/*.cpp` excluding `network.cpp`, `hal/` first on
the include path), `bindings.cpp` (`pybind11` `FirmwareControl`:
`tick`/`play_clip`/`servo_angles`), `sil_bridge.py`. The exact firmware
`SpinalCord` compiles unchanged and runs on the host; `test_sil_poc.py` proves a
clip plays with all servo angles in `[0,180]`, **SIL matches the Python re-port's
`translate_to_servo` at frame 0 to 0.888°** (= the firmware's whole-degree
truncation), and the bridge drives PyBullet headless. `millis()` is fed from an
injected sim clock.

> **Deviation from D1 (flagged):** for the PoC `hal/Arduino.h` is a ~50-line
> hand-rolled shim, *not* ByteNana/ArduinoMock. Reason: the compiled control files
> use only `millis`/`map`/`constrain`/`Serial`/`String`, so the hand-rolled shim is
> smaller, network-free (no `FetchContent`/GoogleTest), and deterministic — still
> "mock the hardware, zero firmware changes." Swap in ArduinoMock via `FetchContent`
> if a future compiled file needs a fuller Arduino surface.

**Step 2 — `--sil` flag + SIL clip suite.** Wire `facehugger.py sim --sil` /
`simulate.py` to switch the joint driver to `sil_bridge` (default stays
`firmware_port`). Drive **all** clips from the firmware's own `clips_all.h`; add
golden-trace + out-of-range (pre-clamp) assertions; CI builds the CMake lib and runs
the suite. **Headline deliverable — this validates the exporter.**

**Step 3 — WS server + HTML control panel (built alongside, D2/D4).** Python
`websockets` server on :81 → `FirmwareControl.command`; `tools/robot_control_panel.html`
(works against sim and real robot). Stand these up even though only clip commands
reach motion yet — immediately useful for manual poking.

**Step 4 — Gait via SIL.** Feed `(gait, X/Y/Yaw)` commands into `FirmwareControl`;
same harness and assertions as the clip suite.

**Step 5 — Full-stack JS loop.** Point the unmodified Expo app at the sim WS server;
validate app → protocol → firmware → motion; regression-guard `API_SPEC.md`.

**Step 6 — Mass/physics tuning** (§6): weigh servos + chassis, set inertials,
calibrate against real `--log` traces; lock the tuned config; re-run the SIL clip
suite to confirm commands unchanged.

Each step is independently useful and leaves the sim working (default driver is
always the Python re-port). **Step 2 already delivers the headline goal — test the
exported clips in the sim and know they'll run correctly on the robot.**
