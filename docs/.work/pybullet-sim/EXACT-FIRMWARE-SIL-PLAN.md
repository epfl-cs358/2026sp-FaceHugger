# Plan — run the *exact* firmware code in the PyBullet sim (SIL)

**Status:** PLAN ONLY (2026-05-26), for review. Nothing implemented. This is the
concrete design for compiling the **actual flashed firmware C++** and driving the
PyBullet robot with it — so that a clip/gait that looks right in sim is *guaranteed*
to issue the same servo commands on the real robot, and any firmware change is
testable in sim before flashing. Extends `GAIT-AND-PARITY-PLAN.md` (this is its
"Approach B", committed to and expanded with the JS/WebSocket layer + mass tuning).

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

### Two ways to make the exact code compile on the host

**Option A — Mock the hardware layer (zero firmware changes).**
Provide host implementations of the headers the firmware includes:
- `Arduino.h` → a shim (or the `ArduinoFake` PlatformIO lib): `millis()` returns an
  **injectable** sim clock; `map()`, `constrain()`, `Serial` as no-ops/host stdio.
- `Wire.h` → empty stub (only `#include`d, never used for logic).
- `Adafruit_PWMServoDriver` → a **recording mock**: `setPWM(ch, 0, pulse)` stores
  the pulse per channel. (Or tap one level up at `Servo::servoAngle` via
  `getServoAngle()` to avoid the integer PWM map.)
- `ArduinoJson` → the real lib (it's cross-platform; already a `native` dep).

The exact `spinal_cord.cpp` / `servo.cpp` / `leg.cpp` compile **unmodified**. This
is the standard PlatformIO native-testing pattern for Arduino code.
- *Pro:* firmware source is untouched — provably "the code you flash."
- *Con:* you maintain a faithful PCA9685 mock + Arduino shim; `millis()` injection
  needs a hook (ArduinoFake provides one; a custom shim is ~30 lines).

**Option B — Introduce a tiny hardware seam in the firmware (HAL).**
Refactor the firmware so the servo sink and clock are behind interfaces
(`IServoBus::writeAngle(ch, deg)`, `IClock::millis()`), injected at construction.
The ESP32 build injects the PCA9685 + real `millis`; the sim injects a recording
sink + a sim clock.
- *Pro:* clean, explicit seam; no mocking a third-party lib; better firmware
  architecture; the recorded value is servo **degrees** (no PWM-truncation noise).
- *Con:* it changes the firmware — but the changed firmware is still exactly what
  you flash, so SIL fidelity is preserved. It's a one-time, low-risk refactor
  (the choke points are already centralised: `Servo::setServoAngle`,
  `SpinalCord::update`'s `millis()` reads).

**Recommendation:** **Option B**, scoped tightly. It's the difference between
"perpetually mocking the Adafruit lib's surface" and "one clock + one servo-bus
interface." The seam is small because the firmware already funnels every write
through `Servo::setServoAngle` and reads time only via `millis()`. If you want
*zero* firmware change first, start with Option A (it proves the concept), then
migrate to B. **Decision needed on review (§9, D1).**

### Why this guarantees what you want
Either way, the bytes from the exporter → `clips_all.h` → **the exact firmware
`tickClip`/`translateToServo`** → 12 servo angles → PyBullet. The sim and the
robot run the *same code* on the *same data*, so "passes in sim" ⇒ "issues the
same commands on hardware." That is the guarantee the current Python re-port
cannot make.

---

## 3. The FFI binding — how Python calls the firmware

Two standard options:

- **`ctypes`** (stdlib): call C functions across a clean `extern "C"` boundary. We'd
  write a thin `sim_api.cpp` exposing C functions (`fh_sim_create()`,
  `fh_sim_command(json)`, `fh_sim_tick(t_ms)`, `fh_sim_read_servos(double out[12])`).
  Simple, no extra deps, but we hand-marshal structs.
- **`pybind11`**: bind the C++ `SpinalCord` class directly to a Python object.
  Richer (call methods, return arrays), nicer ergonomics, but adds a build dep and
  a compile step. **Recommended** — the control core is class-based (`SpinalCord`),
  so binding the object is the natural fit.

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

A small package, e.g. `code/simulation/firmware_sil/` (sibling to `pybullet_sim/`):
```
firmware_sil/
  hal/                 host Arduino shim + recording servo bus (+ sim clock)   [Option A]
                       OR  the injected sim backends                            [Option B]
  sim_api.cpp          extern "C" / pybind11 entry point over SpinalCord
  build.py / CMake     compile the exact firmware sources + hal -> fh_sim.{so,dylib}
  bridge.py            FirmwareControl wrapper + the PyBullet driver loop
```
The build pulls firmware sources **by path** from `code/firmware/src/...` (not
copies) so there is no second copy to drift. A `facehugger.py sim --firmware`
flag (TBD) selects the SIL driver over the Python re-port.

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

Two fidelity levels for the **command-handling** layer:
- **(i) Python WS shim (recommended first):** a Python `websockets` server on :81
  parses the `T:` JSON and calls `FirmwareControl.command(...)`. The motion code is
  exact; only the transport+parse glue is Python. The app can't tell it's a sim.
- **(ii) Exact network handler:** compile `network.cpp`'s `onWebSocketEvent`
  dispatch into the host core too (mock `WebSocketsServer`, feed it frames). Maximal
  fidelity of the command path, but the links2004 WebSockets lib is ESP-oriented and
  awkward on host — so the JSON→command **dispatch** is worth extracting/compiling;
  the socket transport is best left to Python. (Decision §9, D2.)

For the **JS itself**: run the Expo app (or a headless JS test harness / Playwright
against the web build) pointed at `ws://localhost:81`. The JS runs unmodified and
drives the simulated robot end-to-end. This validates: app → protocol → firmware
command handling → motion — the whole loop, on the desktop, before touching
hardware. It also becomes a regression test for `API_SPEC.md` drift.

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

## 9. Decisions for you to make on review

- **D1 — Mock (A) vs HAL seam (B)** for compiling the exact code. (Recommend: start
  A to prove it, land B for maintainability. Are you OK touching firmware?)
- **D2 — Network layer fidelity:** Python WS shim (i) vs compiling the exact
  `network.cpp` dispatch (ii). (Recommend i first.)
- **D3 — FFI:** `pybind11` (recommended) vs `ctypes`.
- **D4 — Scope of first cut:** clips only, or clips + gait + the WS/JS loop? (Recommend:
  clips via SIL first — directly validates your exporter — then gait, then WS/JS.)
- **D5 — Keep the Python re-port?** as a no-C++-toolchain fallback + a second
  independent implementation for the parity triangle, or retire it once SIL lands.
- **D6 — Where does `fh_sim` build live:** firmware repo (PlatformIO env) vs sim
  package (CMake)? (Affects who owns the build.)

---

## 10. Suggested sequencing (when approved — not now)

1. **Proof of concept (Option A, clips):** Arduino shim + recording PCA9685 mock;
   compile `spinal_cord.cpp` + deps natively; `extern "C"` `tick`/`read_servos`;
   ctypes; play one clip through PyBullet via SIL. Confirms the exact code runs.
2. **SIL clip suite:** drive all clips from the firmware `clips_all.h`; golden-trace
   + out-of-range assertions; wire into CI. **This validates your exporter.**
3. **Gait via SIL:** feed `(gait, X/Y/Yaw)` commands; same harness.
4. **HAL seam (Option B)** + `pybind11`: retire the mock for a clean injected seam.
5. **WebSocket entrypoint:** Python WS server on :81 → `FirmwareControl.command`;
   point the Expo app at it; full-stack SIL.
6. **Mass/physics tuning** (§6): calibrate against real measurements; lock config.

Each step is independently useful; step 2 already delivers the headline goal —
**"test the exported clips in sim and know they'll run correctly on the robot."**
