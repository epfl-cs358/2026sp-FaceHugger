# Plan — gait interpreter + a unified firmware↔sim control path

**Status:** PLAN ONLY (2026-05-26). Nothing here is implemented. It is the
thought-through design for (a) playing firmware-faithful **gaits** in the sim and
(b) a **unified** way to mirror *all* firmware servo/robot control in the sim, so
the simulation stays a faithful adaptation of the real robot as the firmware
evolves. It also answers the key question: **how well can that faithfulness be
guaranteed?**

Grounded in two research notes: `.work/research/fw-control-surface.md` (firmware
control architecture) and `.work/research/sim-parity-infra.md` (existing sim
parity infrastructure), plus a Context7 review of PlatformIO and Bullet/PyBullet
capabilities.

---

## 1. The goal, stated precisely

> No matter how the firmware changes, the simulation reflects the robot's
> commanded behaviour *as-is*, and we can **prove** the sim matches the firmware.

Two distinct properties are bundled in there; keep them separate, because they
have very different guarantee strengths:

- **(G1) Command parity** — given the same inputs (clip id, or gait + joystick
  X/Y/Yaw, at the same logical time), the sim computes the *same 12 servo angles*
  the firmware would. This is **provable** and is what the existing clip parity
  chain already guarantees for clips.
- **(G2) Physical fidelity** — the robot in the sim *behaves* like the real robot
  (balance, slip, tipping, contact). This is **never exactly guaranteed** — it's a
  modelling problem (masses, friction, servo dynamics, controller gains). The sim
  can be made *plausible*, not *exact*.

The whole parity strategy below is about maximising **G1** (which is achievable to
the bit) and being honest that **G2** is best-effort.

---

## 2. The decisive architectural fact

Every firmware motion path — gait (walk/crab), trot, in-place yaw, clip playback,
stand, invert — funnels through **one pure function**, `translateToServo()`
(`motion_math.cpp:32`), to produce 12 servo angles in degrees, then a single
hardware sink `Servo::setServoAngle()` maps degrees→PWM. (See
`fw-control-surface.md §1, §3`.)

So the **parity seam is "12 servo angles in degrees"** — pre-PWM, post-`translateToServo`.
The sim already meets the firmware at exactly this seam for clips
(`servo_convention.translate_to_servo` → `frame_to_joint_targets` →
`setJointMotorControl2`). **Everything new should target the same seam.** Anything
upstream of it (gait sweep math, clip interpolation) is pure arithmetic and
portable; everything downstream (PWM integer map, I2C, servo physics) is hardware
and is replaced by PyBullet's joint controller.

Corollary: the firmware's pure-math layer (`motion_math.cpp` + the three tick
geometry blocks) is **already compiled host-side** in the PlatformIO `native`
environment for its Unity tests (`motion_math.h:5` — "Arduino-free … compiled in
the native env"). That is the foundation for the strongest option below.

---

## 3. The spectrum of approaches (and what each guarantees)

Four ways to make the sim mirror the firmware, from weakest to strongest G1
guarantee. They are **not mutually exclusive** — the recommendation combines them.

### Approach A — Python re-port (the current pattern, extended to gait)
Re-implement each firmware function in Python (as the clip interpreter already
does), and guard it with the "verbatim firmware-formula copy in the test" pattern.

- **G1 guarantee:** *indirect.* Parity holds only as strongly as the test vectors.
  Two independent transcriptions (port + test copy) must agree; a missed case in
  both passes silently. The clip chain manages this with exact-int comparison on
  *real* data — strong, but it's still "transcription verified by tests."
- **Cost:** low; matches existing code; pure-Python, no build/FFI.
- **Drift risk:** a firmware change requires a manual Python re-port + test update.
  Forget it and the sim silently diverges until someone re-runs parity.

### Approach B — Compile firmware C++, call it from Python (SIL via FFI) ★
Build the firmware's pure control code (`motion_math.cpp`, and a thin
`control_core.cpp` exposing `tickGait/tickTrot/tickClip` geometry without Arduino
deps) into a **host shared library**, and call it from Python via `ctypes` or
`pybind11`. The sim feeds a synthetic millisecond clock and reads back the 12
servo angles, then drives PyBullet through the existing `frame_to_joint_targets`
seam.

- **G1 guarantee:** *exact, by construction.* It is **literally the same compiled
  code** the firmware runs (same source file, same compiler arithmetic). No
  transcription, so no transcription drift. This is the honest answer to "ideally
  we'd use the same code" — it **is** possible.
- **Enabler (Context7):** PlatformIO's `native` platform already builds this code
  on the host (used today for `test_kinematics` / `test_clip_parity`). The same
  toolchain produces a `.so`/`.dylib`; the firmware math is already
  `#ifndef ARDUINO`-clean.
- **Cost:** medium — a small extra build target + an FFI shim + struct marshalling.
  Adds a C++ build to the sim's dev loop (already required for firmware tests).
- **Residual risk:** the `native` build's `double` arithmetic must match the ESP32
  target's (it does for IEEE-754 `double`; the ESP32 uses soft-float `double`,
  same result). The only true divergence is the firmware's `uint16_t` servo-angle
  **quantisation to whole degrees** (`servo.h:24`) — reproduce it explicitly.

### Approach C — Co-simulation / hardware-in-the-loop (HIL)
Run the *actual* firmware (either compiled native, or the real ESP32) as a
separate process that connects to the PyBullet **server** and streams servo
commands in; PyBullet returns joint/IMU state to close the loop.

- **G1 guarantee:** *exact* (it's the firmware itself), and it additionally
  exercises the firmware's **timing** (the per-tick EMA/input smoothing whose lag
  is loop-rate dependent — see §6), which Approaches A/B can only approximate.
- **Enabler (Context7):** PyBullet supports `SHARED_MEMORY`, `TCP`, `UDP`
  transports and a C API (`b3ConnectSharedMemory`, `b3RobotSimulatorClientAPI`,
  `PhysicsClientC_API`), plus a server-side plugin manager (`pdControlPlugin`,
  `b3PluginManager`). An external C++ harness — or a bridge from the ESP32's
  WebSocket servo stream (`code/API_SPEC.md`, port 81) — can drive the sim
  synchronously. (The ESP32 itself can't link Bullet; it would stream servo
  degrees over serial/WS to a host bridge that calls `setJointMotorControl2`.)
- **Cost:** high — process orchestration, a transport bridge, clock
  synchronisation, and (for real-ESP32 HIL) the board + a PCA9685 command tap.
- **Use:** validation milestones, not the everyday dev loop. This is how you'd
  answer "does the *timed* behaviour match," not just the math.

### Approach D — Full ESP32 emulation (QEMU/Wokwi) driving the sim
Run the unmodified firmware binary in an ESP32 emulator, intercept the PCA9685 I2C
writes, and feed the resulting servo angles to PyBullet.

- **G1 guarantee:** *exact, including firmware timing and the integer PWM map.*
- **Cost:** very high; emulator I2C tap is fiddly; Wokwi/QEMU ESP32 support is
  partial. **Out of scope** — listed for completeness; not recommended now.

### Recommendation
**B as the backbone, A as the bridge, C as the validation gate.**

1. **Now / this milestone:** implement the **gait interpreter via Approach A**
   (Python port of `tickGait`), because it's the smallest step, reuses the proven
   clip pattern, needs no build changes, and is enough to *play gaits in the sim*
   and *prove command parity on sampled inputs*. (§4, §5.)
2. **Next:** introduce **Approach B** (compile-and-call) as the *parity oracle* —
   the FFI-wrapped firmware becomes the "independent ground truth" that today is a
   hand-copied formula in the tests. This upgrades G1 from "two transcriptions
   agree" to "the sim equals the actual firmware binary," and makes firmware
   changes self-propagating (rebuild → tests catch any divergence automatically).
   Eventually the Python ports can even be *generated/checked against* the FFI
   oracle, or replaced by it behind the unified API.
3. **Later / milestone validation:** stand up **Approach C** once to confirm the
   timed behaviour (EMA transients, command latency) under real cadence.

---

## 4. The gait interpreter (concrete design — Approach A)

A faithful port of `SpinalCord::tickGait` (+ `tickTrot`, `tickYawRotation`) into
`pybullet_sim/interpreter/gait_interpreter.py` (currently a stub). It is a **third
engine** alongside the sim-native `gaits.py` and the clip interpreter — it does
**not** replace `gaits.py` (that one is the Cartesian + URDF-IK engine; this one
is the degree-space firmware mirror). They coexist exactly as `run_gait` and
`run_clip` already do. (`sim-parity-infra.md §3`.)

### 4.1 What it computes
Mirror `fw-control-surface.md §2` verbatim:
- Phase clock: `t` (synthetic ms → s), `globalPhase = fmod(t/period, 1)`,
  `legPhase = fmod(globalPhase − offsets[i] + 1, 1)`.
- Per-leg `sweep`/`lift` (stance vs swing), in **degrees**, off `NEUTRAL[i]`.
- `fwdDir`/`yawDir` sign tables blending `activeX/activeY/activeYaw`.
- Sub-modes: `tick_yaw_rotation` (|yaw|>0.05), `tick_trot` (GAIT_TROT), `tick_gait`
  (walk/crab) — each producing a `a[12]` math-space frame (FR,FL,RR,RL × sh,th,kn).
- The `GAITS[]` table (`movements.h`) and constants ported as Python data.

### 4.2 How it reaches PyBullet — reuse the existing seam
The ported math's only job is to **produce `a[12]`**. Then hand it to the
*existing, already-correct* kernel `clip_player.frame_to_joint_targets(a)`
(`sim-parity-infra.md §3`), which does `translate_to_servo → clamp* →
servo_to_radians → LEG_ID_TO_URDF_AXIS_SIGN → {joint_name: rad}`. Drive with
`helpers.apply_joint_targets`. **Do not** re-derive the servo/axis math — funnel
through the one kernel so a gait can never render mirror-image legs.
(*Caveat:* `clamp_clip_servos` is **clip-only**; gaits use only the `[0,180]`
constrain in the firmware's `setJointAngles`. The kernel must allow skipping the
tight clip clamp for the gait path — small refactor: a `clamp=` parameter.)

### 4.3 Driver + CLI (no new physics)
Add a `run_gait_fw(...)` driver (or extend `run_gait` with an `engine=` switch)
that, per 240 Hz step, computes synthetic `t_ms`, calls the interpreter, applies
targets, steps. Surface it as `facehugger.py sim --gait-fw walk --xy 0,1` (exact
flag TBD). The existing settle / `--monitor` / `--log` scaffolding is reused.

### 4.4 Inputs
Gaits are driven by live joystick `activeX/Y/Yaw` (not fixed params). The CLI/API
must accept these as constants (held) or a scripted timeline. For parity testing,
they're pinned per sample.

---

## 5. The unified control API (the "no matter what the firmware does" part)

Define one interpreter boundary that mirrors the firmware's own `SpinalCord` seam:

```
            firmware-space command + synthetic clock
                          │
            ControlInterpreter.tick(t_ms) ──► a[12] math-space degrees
                          │                    (per active mode)
            frame_to_joint_targets(a)  ──► {joint_name: radians}
                          │
            apply_joint_targets ──► PyBullet setJointMotorControl2
```

### 5.1 Shape
A small state machine mirroring `SpinalCord::update()`'s dispatch
(`fw-control-surface.md §6`): modes `IDLE / WALK(gait) / ACTION(clip) / REST /
STAND / FAILSAFE`, with the **same transition rules** (clip pre-empts gait;
graceful-stop at phase boundary; `CLIP_RETURN_MS` ease). Commands map 1:1 to the
WebSocket protocol (`code/API_SPEC.md`: `T:1` move, `T:2` state, `T:5` gait,
`T:7` clip, `T:6` invert…). `tick(t_ms)` returns the active mode's `a[12]`.

This makes the sim accept *the same commands the app sends the robot* and produce
*the same servo angles* — the literal "faithful adaptation" the goal asks for.

### 5.2 What it must thread (the gaps to close — `sim-parity-infra.md §5`)
- **EMA smoothing** (clip-path only, `CLIP_EMA_ALPHA=0.75`) — apply per-mode, not
  globally. Gaits/poses don't smooth; clips do.
- **`isInverted` pitch-mirror** (`applyInvert`) — thread an `inverted` flag through
  all modes (currently ignored in sim).
- **Deadman / input smoothing** (`active += (target−active)*0.1`) — model only if
  going for timed parity (Approach C); for command parity, pin inputs.
- **A real `servo_mapping.yaml`** — see §7; the LegId↔URDF binding is currently
  hardcoded in `servo_convention.py`. A unified API is the moment to formalise it.

### 5.3 Why route everything through `a[12]` + the one kernel
Because that is the single seam where sim==firmware is *proven*. If every mode
(clip, gait, pose, calibration) emits `a[12]` (or, for the degree-direct paths
like `relax`/calibration, servo degrees), then **one** parity test design covers
all of them, and **one** sign/axis correction protects all of them.

---

## 6. How well can parity be guaranteed? (the honest answer)

Split by the two properties from §1.

### G1 — command parity (the 12 servo angles): **guaranteeable, with caveats**
- **Pure geometry (gait sweep/lift, clip interpolation, `translateToServo`):**
  guaranteeable to the **bit** under Approach B (same compiled code), or to
  **~1°/exact-int** under Approach A (transcription + test triangle), *as long as
  the comparison is on `t`/phase, not wall-clock*. Gait math is closed-form in `t`
  (`pose(t)` is loop-rate-independent), so sampling at 240 Hz reproduces it exactly
  at those instants. (`fw-control-surface.md §5`.)
- **Two things that break *exact* equality** and must be handled deliberately:
  1. **Per-tick smoothers** — the clip `EMA (0.75)` and input `*0.1` recurrences
     are applied *per loop iteration*, and the firmware loop is **free-running and
     jittery** (no fixed dt). Their transient lag depends on ticks-per-second, so a
     240 Hz sim will **not** match the ESP32's transient unless ticked at the same
     cadence. *Steady-state converges; transients diverge.* → Either (a) compare on
     the **pre-EMA** `a[12]` (the EMA is downstream of the parity seam for the
     geometry), (b) disable EMA for parity tests (as the clip port already does),
     or (c) use Approach C to get true timed behaviour. Exact timed-EMA parity is
     **only** achievable by running the firmware itself at its real cadence (C/D).
  2. **`uint16_t` whole-degree quantisation** of stored servo angles
     (`servo.h:24`) — a `double`-based sim differs in the fractional part. →
     truncate to whole degrees at the seam if you want bit-equality.
- **The integer PWM `map()`** (degrees→counts) adds truncation noise; stay at the
  **degree** seam and you avoid it. Only drop to PWM counts if validating the
  electrical layer.

**Bottom line for G1:** *exact* command parity is achievable for the geometry of
every mode (provably so with Approach B). The only places it is *not* bit-exact
without running the real firmware are the timing-dependent smoothers — and those
are isolated, well-understood, and can be excluded from the parity seam or matched
via HIL.

### G2 — physical fidelity: **best-effort, not guaranteeable**
The same servo *commands* can still produce different *motion* in sim vs reality,
because of: body mass/inertia (the `0.8 kg`/`0.3 kg` placeholders — note `mass_kg`
is unverified), friction coefficients, the PyBullet `POSITION_CONTROL` PD model vs
the real servo's torque/speed curve, contact softness, and the 240 Hz integrator.
These can be *tuned* against real measurements (the `--log` torque/current trace is
the instrument for this) but never *proven* equal. Be explicit with users: the
sim guarantees **what the firmware commands**, and *approximates* **how the robot
moves**.

### The guarantee, in one sentence
> We can guarantee the simulation issues the **same servo angles** the firmware
> would (exactly, if we compile-and-call the firmware; to ~1° if we re-port and
> test it), for every control mode, at every sampled input — **except** the
> firmware's timing-dependent smoothing transients, which only a
> hardware/emulator-in-the-loop run reproduces exactly. We **cannot** guarantee
> the physical motion matches reality; that is a tuning problem, bounded by the
> mass/friction/servo-model fidelity.

---

## 7. Parity test strategy (extend the existing chain)

The clip chain proves `sim == exporter == firmware` on a **baked file**
(`clips_all.h`). Gaits have **no baked artifact** — they're generated live — so
the test vector changes from `clip_frame → servo` to `(gait_id, X, Y, Yaw, t) →
a[12]/servo`. (`sim-parity-infra.md §4`.)

Plan:
1. **`gen_gait_parity_reference`** — sample a grid of `(gait_id, activeX/Y/Yaw, t)`
   tuples; run the firmware gait math (via the FFI oracle, Approach B — *or* a
   verbatim copy if staying on A) → bake expected `a[12]` into
   `gait_parity_reference.h` / `.json`.
2. **Firmware Unity test** `test_gait_parity` — assert C++ `tickGait` reproduces
   the reference at those samples (≤1° tol), mirroring `test_clip_parity`.
3. **Sim test** — assert `gait_interpreter.tick_gait` reproduces the same `a[12]`
   at the same `(inputs, t)` (exact or 1e-9 vs the oracle).
4. **The oracle upgrade (Approach B):** replace the hand-copied "independent ground
   truth" formula in the unit tests with a call into the compiled firmware lib.
   Then *any* firmware edit that changes behaviour makes the sim test fail on the
   next run — **drift becomes impossible to miss**, which is exactly the
   "no matter the firmware changes" property the goal wants.

Pin **phase, not wall-clock** in all references (`globalPhase`, `legPhase`
formulas), so the tests are timing-independent.

---

## 8. PyBullet / PlatformIO / ESP32 — what the ecosystem offers (Context7)

- **PyBullet has no built-in "firmware" notion** — it's a physics server with a
  client C API. But its **transport layer is the integration hook**: `DIRECT`,
  `GUI`, `SHARED_MEMORY`, `UDP`, `TCP` connections, a C API
  (`b3ConnectSharedMemory`, `b3RobotSimulatorClientAPI`, `PhysicsClientC_API`), and
  a server-side **plugin manager** (`b3PluginManager`, e.g. `pdControlPlugin`).
  → An external process (compiled firmware harness, or a bridge from the real
  ESP32's WebSocket servo stream) can drive the sim synchronously. This is the
  standard robotics co-sim pattern and is what makes Approach C viable without
  hacking PyBullet internals.
- **PlatformIO already gives us the SIL path for free:** the `native` platform
  builds/runs the firmware's Arduino-free code on the host (the repo's
  `test_kinematics` / `test_clip_parity` do exactly this). The same target emits a
  shared library for Approach B — no new toolchain, and the firmware math is
  already `#ifndef ARDUINO`-guarded.
- **ESP32 specifics:** the ESP32 cannot itself link Bullet/PyBullet; HIL with the
  *real board* means tapping its servo output (WebSocket `T:` stream on port 81,
  or a PCA9685 I2C sniff) and bridging those 12 angles into `setJointMotorControl2`
  on a host. Full-binary emulation (Wokwi/QEMU, Approach D) is possible but
  heavyweight and not recommended now.

**Net:** the ecosystem fully supports the "same code" ambition. The cleanest lever
is **PlatformIO native → shared lib → Python FFI (Approach B)** for everyday
provable parity, with **PyBullet's shared-memory/TCP C API (Approach C)** reserved
for timed/whole-system validation.

---

## 9. Suggested sequencing (when we implement — not now)

1. **Gait interpreter, Approach A** — port `tickGait/tickTrot/tickYawRotation`;
   reuse `frame_to_joint_targets` (add a `clamp=` skip for the gait path); add a
   `run_gait_fw` driver + CLI flag; mirror the clip-port docstring/`file:line`
   convention. Ship with a `gait_parity_reference` + sim/firmware tests.
2. **FFI oracle, Approach B** — `native` shared-lib target exposing
   `translateToServo` + the tick geometry; `ctypes`/`pybind11` shim; flip the
   parity tests' "ground truth" to the compiled firmware. Reproduce the `uint16_t`
   degree quantisation at the seam.
3. **Unified `ControlInterpreter`** — mode state machine + command mapping
   (`API_SPEC` `T:`), per-mode EMA/invert handling, formalise `servo_mapping.yaml`.
4. **HIL validation, Approach C** — one-time shared-memory/WS bridge to confirm
   timed behaviour; document the residual G2 (physical) gaps and tune
   mass/friction against `--log` traces.

Each step is independently shippable and leaves the sim working; none requires the
next. Step 1 alone already delivers "play firmware gaits in the sim, with proven
command parity on sampled inputs."

---

## 10. Open questions to resolve before building

- **Scope of "unified":** just clip+gait (the ticked paths), or also the one-shot
  paths (stand/relax/calibration/invert)? The seam supports all; the state machine
  is the extra work.
- **Joystick input model for gaits:** held constants, or a scripted
  `(t → X/Y/Yaw)` timeline? Affects the CLI and the parity grid.
- **Approach B build integration:** do we want the sim's `uv` dev loop to depend on
  a C++ build (and CI to build the native lib), or keep B as an opt-in parity tool?
- **`servo_mapping.yaml`:** formalise now (single source for LegId↔URDF↔servo_id)
  or keep the hardcoded maps until firmware `SERVO_CONFIG[]` is confirmed? (Memory:
  servo numbering is still a proposal.)
- **Quantisation:** do we want bit-exact (truncate to whole degrees) or is ≤1°
  tolerance acceptable for the sim's purpose (geometry validation)?
