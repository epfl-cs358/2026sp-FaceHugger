# Sim ↔ Firmware Parity Infrastructure — Study for a Unified Control Interpreter

Read-only study (2026-05-26). All paths absolute-relative to repo root
`code/simulation/` and `code/firmware/`. Nothing was edited.

---

## 1. The porting pattern established by the clip interpreter

### Which firmware functions are ported, and where the correspondence is documented

The `pybullet_sim/interpreter/` package is a **line-for-line Python port of the
firmware clip-playback path**, with the firmware source file:line cited in every
module docstring:

| Python (sim) | Firmware (C++) source cited in docstring |
|---|---|
| `servo_convention.translate_to_servo` (`servo_convention.py:84`) | `motion_math.cpp:32-59` |
| `servo_convention.clamp_clip_servos` (`servo_convention.py:123`) | `motion_math.cpp:10-18` |
| `servo_convention.NEUTRAL[]` (`servo_convention.py:61`) | `neutral_pose.h:13` |
| `clip_player._interpolate_frame` (`clip_player.py:69`) | `motion_math.cpp:62-90` (`clipPoseAt`) |
| `clip_player.frame_to_joint_targets` (`clip_player.py:36`) | `spinal_cord.cpp:427-437` (inner loop of `tickClip`) |
| `clip_loader.load_clips_all_h` (`clip_loader.py:59`) | parses `clips_all.h` emitted by `fh_clip_panel.to_clips_header` |

`translate_to_servo` is byte-identical to the C++ `switch` — the per-leg formulas
match exactly, including the 2026-05-25 "BR un-mirror" comment:

```python
# servo_convention.py:107-113  (LEG_RR / BR)
out.hip = 90.0 + (sh + 45.0)   # was 90.0 - (sh + 45.0)
out.thigh = 90.0 + th
out.knee = 90.0 - kn
```
```cpp
// motion_math.cpp:52-54
out.hip   = 90.0 + (sh + 45.0);  // was 90.0 - (sh + 45.0)
out.thigh = 90.0 + th;
out.knee  = 90.0 - kn;
```

### What it deliberately does NOT port — EMA smoothing

The module header is explicit (`clip_player.py:14-16`):

> "EMA smoothing (CLIP_EMA_ALPHA=0.75 in firmware) is intentionally NOT applied
> here. The simulator is used to validate clip **geometry**, not to reproduce
> smoothing artifacts."

The firmware applies a per-channel EMA *before* `translateToServo`
(`spinal_cord.cpp:431-436`, `emaStep` in `motion_math.cpp:20-22`):
`clipSmoothed_[i][j] = alpha*prev + (1-alpha)*target`. The sim skips that filter
entirely. Also deliberately **not** ported into the sim clip path: the
`applyInvert` pitch-mirror (`motion_math.cpp:24-29`, applied centrally in firmware
`applyServos` `spinal_cord.cpp:105-106`), and the firmware's `CLIP_RETURN_MS=500`
ease-to-NEUTRAL endgame (`clipPlayerStep` `motion_math.cpp:97+`). The sim's
`play_blocking` just *holds* the last frame (`clip_player.py:196-207`), it does
not run the smoothstep return.

### How clip_player drives PyBullet

The per-frame pipeline (`clip_player.py:4-10` docstring, body at
`frame_to_joint_targets` `:36-66` and `ClipPlayer.step` `:134-157`):

1. `_interpolate_frame(frames, elapsed_ms)` → `a[12]` math-space degrees, in
   **firmware LegId order** (`_LEG_IDS = (LEG_FR, LEG_FL, LEG_RR, LEG_RL)`,
   `clip_player.py:33`; layout `a[0..2]=FR … a[9..11]=RL`).
2. `translate_to_servo(leg_id, sh, th, kn)` → `ServoTriple` (0–180°).
3. `clamp_clip_servos(servo)` (the tighter clip-path clamp, *is* ported).
4. `servo_to_radians(deg) = radians(deg − 90)` (`servo_convention.py:158`):
   servo 90 → 0 rad, the URDF joint zero.
5. Joint mapping + the **URDF axis-sign correction**, then a motor command.

**Joint mapping**: `LEG_ID_TO_SIM_NAME` (`servo_convention.py:25-30`) maps firmware
LegId → URDF leg name (`FR→fr, FL→fl, RR→br, RL→bl`). Joint names are built as
`f"{urdf_name}_link{1,2,3}_joint"`. PyBullet indices come from
`helpers.build_joint_map(robot_id)` (`helpers.py:175-181`), which reads
`getJointInfo` and returns `{joint_name: joint_index}`.

**The `LEG_ID_TO_URDF_AXIS_SIGN` correction** (`servo_convention.py:38-43`):
diagonal pair (FL, BR) = +1, (FR, BL) = −1, derived from each joint's
`<axis xyz="0 ±1 0">` in `facehugger.urdf`. Applied in
`frame_to_joint_targets` (`clip_player.py:62-65`):

```python
axis = LEG_ID_TO_URDF_AXIS_SIGN[leg_id]
targets[f"{urdf_name}_link1_joint"] = axis * servo_to_radians(servo.hip)
targets[f"{urdf_name}_link2_joint"] = axis * servo_to_radians(servo.thigh)
targets[f"{urdf_name}_link3_joint"] = -axis * servo_to_radians(servo.knee)
```

Note `link1` (yaw) **also** needs the sign factor (docstring `:48-52`: "without
it fr/bl shoulder yaw renders backwards"), and `link3` (knee) gets the *negated*
sign. This sign correction is **sim-only** — it compensates for URDF `<axis>`
orientation and has *no firmware counterpart* (firmware drives physical motors;
`servo_convention.py:36-37` says so explicitly).

**The motor command**: `ClipPlayer.step` lazily imports pybullet and calls
`setJointMotorControl2(robot_id, joint_idx, POSITION_CONTROL, targetPosition=rad,
force, maxVelocity)` per joint (`clip_player.py:150-157`). This is the same shape
as the shared `helpers.apply_joint_targets` (`helpers.py:216-229`) — though
`ClipPlayer` inlines its own copy rather than calling the helper.

---

## 2. The parity chain: how sim == exporter == firmware is proven

There are **three independent implementations** of the `translateToServo`
contract, and the chain locks them pairwise so the loop closes:

- **Sim**: `servo_convention.translate_to_servo` (Python).
- **Exporter**: `fh_clip_panel._frame_to_servo` (Blender add-on, Python).
- **Firmware**: `translateToServo` (C++, `motion_math.cpp:32`).

### Link A — sim == exporter, on real exported data (`verify_export_parity.py`)

`verify_export_parity.py:62-110` derives the servo command for **every frame of
every clip** from two independent paths and asserts equality:
- Path 1: `clips_all.h` (math-space) → sim's `translate_to_servo`.
- Path 2: the per-clip `<clip>.js` (servo degrees the browser streams), parsed.

The comparison is **exact integer equality under the .js [0,180] clamp**
(`verify_export_parity.py:58-59, 102-109`):

```python
def _clamp_0_180(v): return int(round(max(0.0, min(180.0, v))))
...
if sim[leg] != js[leg]:   # exact, no tolerance
    errors.append(...)
```

The module header (`verify_export_parity.py:17-21`) states the closure logic:
"Path 1 and Path 2 are produced by SEPARATE implementations … the firmware C++ is
locked to the exporter separately by `test_clip_parity`. So agreement here closes
the loop: sim == exporter == firmware, on the actual exported data."

### Link B — firmware == exporter, on sampled inputs (`test_clip_parity`)

`gen_clip_parity_reference.py` runs the **exporter's** `_frame_to_servo` over
3 sampled math-space frames (`SAMPLES`, `:36-55`) and bakes the expected servo
triples into `clip_parity_reference.h`. The firmware Unity test
`test_clip_parity.cpp:14-26` calls C++ `translateToServo` on the same inputs and
asserts agreement with a **1° tolerance**:

```cpp
// test_clip_parity.cpp:7-9
static const double TOL = 1.0;   // _frame_to_servo rounds to int; firmware is
                                 // double — 1 deg absorbs the rounding
...
TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_hip, s.hip, m);
```

### The "independent ground truth" pattern (the model to extend)

The unit-test parity guard (`tests/test_servo_convention.py:33-48`) keeps a
**verbatim copy of the firmware formula** as `_firmware_translate`, with a
DO-NOT-FACTOR-OUT warning:

> "`_firmware_translate()` is copied verbatim from firmware source as independent
> ground truth — DO NOT factor it out into servo_convention.py. Divergence between
> the two is precisely the bug these tests guard against."

Tests then assert `translate_to_servo == _firmware_translate` to `< 1e-9` at
NEUTRAL, NEUTRAL+30°, and single-axis perturbations (`test_servo_convention.py:81-122`).
This is the model: **a second, deliberately-duplicated copy of the firmware
formula lives in the test, so the production port and the test cannot share a bug.**

**Tolerance summary**: sim↔exporter on real data = *exact int* (verify_export_parity);
firmware↔exporter on samples = *1°* (test_clip_parity); sim-port↔verbatim-firmware
in unit tests = *1e-9*.

---

## 3. Two gait engines: sim-native vs. firmware-faithful

### Sim-native `gaits.py` (exists, runs today)

`gaits.py` is **NOT a firmware port** — it is the sim's own procedural engine.
Its structure:
- `GAITS` dict (`gaits.py:32-49`) — `walk`/`trot` with `period, step_length` (in
  **metres**), `step_height` (m), `duty`, per-leg phase `offsets` keyed by **URDF
  leg names** (`fl/fr/bl/br`).
- `foot_target(...)` (`:64-80`) — produces a **Cartesian body-frame foot position**
  (x,y,z in metres), with a sine-arc swing.
- `gait_joint_targets(cfg, gait, t)` (`:83-101`) — for each leg, computes the foot
  target then calls **`cfg.leg_ik(cfg, foot, leg_id)`** (URDF-derived inverse
  kinematics) to get shoulder/hip/knee **radians directly**, keyed
  `f"{leg_id}_link{1,2,3}_joint"`.

So the sim path is: **foot Cartesian trajectory → URDF IK → joint radians**. It
never touches `translate_to_servo`, servo degrees, or `NEUTRAL[]`. It has its own
overlay drawing and a gait-aware spawn-height calc (`_body_height_for_gait`).

### Firmware-faithful `gait_interpreter.tickGait` (the STUB)

`gait_interpreter.py` is a stub (`tick_gait` raises `NotImplementedError`,
`:9-15`). A faithful port would mirror `SpinalCord::tickGait`
(`spinal_cord.cpp:168-244`), which is a **completely different model**:
- Works in **math-space degree deltas around `NEUTRAL[]`**, not Cartesian feet.
  `sh/th/kn` start at `NEUTRAL[i]` (`spinal_cord.cpp:209-211`) and accumulate
  `sweep`/`lift` terms.
- `sweep = step_length_deg * (0.5 − p)` in stance, `(-0.5 + p)` in swing;
  `lift = sin(p·π) * step_height_deg` (`spinal_cord.cpp:197-207`) — **degrees**,
  not metres, and applied directly to thigh/knee (`th += lift; kn -= lift`).
- Per-leg `fwdDir`/`yawDir` sign tables (`spinal_cord.cpp:222-225`) blend
  `activeX/activeY/activeYaw` joystick inputs — including the BR-un-mirror
  comment that ties `fwdDir[BR]=+1` to the servo convention change.
- Then `translateToServo(i, sh, th, kn)` → `applyServos` (`spinal_cord.cpp:242`).
- Has sub-modes: `tickYawRotation` (`:171, 337+`), `tickTrot`
  (`:173, 250+` — a *separate* validated 4-phase script with its own constants),
  `tickCrab`. `GAITS[]` table at `spinal_cord.cpp:22-27` carries
  `step_length_deg, step_height_deg, period_s, duty, offsets[4], label`.

### Where they differ / can they coexist

| | sim `gaits.py` | firmware `tickGait` |
|---|---|---|
| Foot representation | Cartesian (m) | math-space joint degrees |
| Kinematics | URDF-derived `cfg.leg_ik` | none — direct degree deltas off NEUTRAL |
| Units | metres | degrees |
| Leg keying | URDF names `fl/fr/bl/br` | LegId `0..3` (FR,FL,RR,RL) |
| Inputs | fixed gait params | live `activeX/Y/Yaw` joystick |
| Output to motors | radians via IK | servo deg via `translateToServo` |

**They can coexist**: they would be two separate registries / entry points,
exactly as `run_gait` (sim-native, `gaits.py:457`) and `run_clip` (firmware-port,
`gaits.py:402`) already coexist in the same file. A faithful `gait_interpreter`
would be a *third* engine alongside the sim-native one, not a replacement.

### What in helpers.py / clip_player.py is reusable for a gait interpreter

A firmware-faithful gait port reaches the *same servo→radian→PyBullet seam* the
clip player already owns, so it can reuse:
- **`servo_convention.translate_to_servo` / `clamp_clip_servos` / `servo_to_radians`**
  — directly, unchanged.
- **`LEG_ID_TO_SIM_NAME` and `LEG_ID_TO_URDF_AXIS_SIGN`** (`servo_convention.py:25-43`)
  — same LegId→URDF-name map and same axis-sign correction.
- **`clip_player.frame_to_joint_targets(a)`** (`:36`) — this is *exactly* the
  reusable kernel: it takes `a[12]` math-space degrees in LegId order and returns
  `{joint_name: rad}`. A gait port only needs to *produce the `a[12]`* (via the
  ported sweep/lift math) and then hand it to this function verbatim.
- **`helpers.build_joint_map`** (`:175`) and **`helpers.apply_joint_targets`**
  (`:216`) — joint indices + the `setJointMotorControl2` wrapper.
- The `run_gait` driver loop (`gaits.py:457-529`) — settle, monitor hook, overlay
  scaffolding — though it currently calls the sim-native `gait_joint_targets`.

---

## 4. The seam for parity

### Where clip parity compares today: servo degrees

The artifact compared across implementations is **servo degrees in [0,180]**, via
`translate_to_servo` (Link A in §2 compares integer servo degrees frame-by-frame;
Link B compares servo `ServoTriple` doubles to 1°). The math-space `a[12]` in
`clips_all.h` is the *common input*; **servo degrees are the common output** where
agreement is asserted. The baked artifact is `clips_all.h` itself — a static set
of exported keyframes.

### What the analogous gait parity artifact would be

**There is no baked file for gaits.** Clip data is authored in Blender and frozen
into `clips_all.h` (+ per-clip `.js`/`.h`); the sim, browser, and firmware all read
the *same static frames*. A gait is **generated live** from `(gait_id, activeX,
activeY, activeYaw, t)` — there is no exported keyframe table to diff against.

So gait parity must compare **generated sequences given identical inputs and
time**, not a static file. The natural analog of `gen_clip_parity_reference.py`
would be:

> Sample a grid of `(gait_id, activeX/Y/Yaw, t)` tuples → run the **firmware**
> `tickGait` math (or a verbatim copy) → bake expected `a[12]` (or expected
> `ServoTriple[4]`) into a `gait_parity_reference.h`. The sim's
> `gait_interpreter.tick_gait` must reproduce the same `a[12]`/servo triples at the
> same sample points, within tolerance.

Implication: the parity seam is still **servo degrees (or the pre-`translateToServo`
`a[12]` math-space frame)**, but the *test vector* is now `(inputs, t) → frame`
rather than `clip_frame → servo`. Because gait math is time-driven and uses
`millis()`/`fmodf` on a continuous `t`, the reference must pin `t` explicitly and
both sides must agree on the **phase formula** (`globalPhase = fmod(t/period, 1)`,
`legPhase = fmod(globalPhase − offset + 1, 1)`), not on wall-clock timing.

---

## 5. Gaps / risks for a unified "firmware-space command → faithful sim joint targets" API

1. **EMA smoothing is absent in the sim (by design).** `tickClip` smooths
   math-space angles with `CLIP_EMA_ALPHA=0.75` *before* `translateToServo`
   (`spinal_cord.cpp:431-436`); `gait`/calibration paths do **not** touch
   `clipSmoothed_`. A unified API must decide per-mode whether to apply EMA. The
   sim currently never does. For a *faithful* gait port this is fine (gaits aren't
   smoothed on firmware), but a unified "command → targets" call must not blanket-
   apply or blanket-skip EMA — it is **clip-path-only**.

2. **Timing / dt mismatch.** Sim runs a fixed `TIMESTEP = 1/240` s
   (`constants.py:14`); `ClipPlayer.play_blocking` advances on wall-clock
   `time.monotonic()` (`clip_player.py:183-209`). Firmware is driven by `millis()`
   on its own loop tick (`spinal_cord.cpp:176`, `gaitPhaseStartMs_`). There is **no
   shared notion of a "tick"**. A unified interpreter needs an explicit
   `tick(t_ms)` contract decoupled from both the 240 Hz physics step and the
   firmware loop rate, with parity asserted on *phase*, not on real time.

3. **Mode handling / state machine not modeled.** Firmware `tick()` dispatches
   `tickGait` vs `tickClip` by `robotState`/`currentGait_` (`spinal_cord.cpp:142-145`),
   with clip pre-empting gait (`STATE_ACTION`), graceful-stop at phase boundaries
   (`spinal_cord.cpp:181-188`), and the `CLIP_RETURN_MS` ease-to-NEUTRAL endgame
   (`clipPlayerStep`). The sim models *none* of this state machine — it just plays
   one clip or one gait in isolation. A unified API that accepts "firmware-space
   commands" would have to model command arbitration, the return-to-NEUTRAL ease,
   and gait start/stop, or scope itself explicitly to per-tick pose only.

4. **`isInverted` / `applyInvert` pitch-mirror not ported.** Firmware applies a
   centralized pitch mirror in `applyServos` (`spinal_cord.cpp:105-106`,
   `applyInvert` `motion_math.cpp:24-29`) that flips thigh/knee to `180−x` when the
   robot is flipped. The sim clip/gait paths ignore inversion entirely. A faithful
   unified path must thread an `inverted` flag through to match.

5. **No `servo_mapping.yaml`.** CLAUDE.md and `animation/SERVO_ID_CONVENTION.md`
   reference a `servo_mapping.yaml` (URDF link ↔ firmware servo_id ↔ direction),
   but **no such file exists in the repo** (searched; absent). The sim currently
   bridges LegId↔URDF-name with the hardcoded `LEG_ID_TO_SIM_NAME` dict and the
   axis-sign with `LEG_ID_TO_URDF_AXIS_SIGN`, both in `servo_convention.py`. The
   memory note "Servo numbering is still a proposal pending firmware
   `SERVO_CONFIG[]` confirmation" applies: a unified interpreter that wants real
   servo-id ↔ joint binding has no source-of-truth file yet.

6. **The sim's URDF axis-sign correction has no firmware equivalent** and is easy
   to get wrong (`clip_player.py:48-52` warns link1 needs it too). Any new path
   producing PyBullet targets must funnel through `frame_to_joint_targets` (or
   replicate the exact sign rules) or it will render mirror-image legs. This is a
   sim-side hazard, not a parity-with-firmware concern.

7. **Two gait engines will diverge silently.** The sim-native `gaits.py` (Cartesian
   + IK) and a faithful `gait_interpreter` (degree-space port) produce *different*
   motions and there is no test asserting they agree (nor should they — they're
   different models). Risk: a unified API entry point must make explicit which
   engine it dispatches to, or callers will assume firmware-faithfulness from the
   sim-native gait.

8. **Manual clips_all.h sync (memory theme).** Two `clips_all.h` exist — the
   exporter's (`animation/exported_clips/`) and the firmware copy — and they are
   synced **by hand after every re-export** (project memory:
   "firmware-clips-all-h-manual-sync"). `verify_export_parity.py` only checks the
   exporter-side `DEFAULT_CLIPS_H`. If the firmware copy drifts, the C++
   `test_clip_parity` (which only samples *formula* inputs, not the real clip table)
   would **not** catch it. A gait interpreter avoids this specific trap (no baked
   file), but a unified clip+gait API inherits the clip-sync hazard.

### Related project memory themes seen / relevant
- *firmware clips_all.h manual sync* — see gap #8.
- *Yaw/shoulder convention* (math +sh=CCW uniform; BR servo hardware-mirrored;
  export cancels rig bone `axis_sign`) — encoded in `translate_to_servo` BR case
  and in the gait `fwdDir/yawDir` tables; both sim and firmware already agree post
  2026-05-25 un-mirror.
- *Clip shoulder rest convention bug* (rig is the defect, sim/firmware faithful) —
  reinforces that the sim/firmware servo formulas are the trusted reference and the
  parity chain guards them.
- *Code over docs* — the absent `servo_mapping.yaml` (gap #5) is a case where the
  doc references a file the code does not have; trust the code's hardcoded maps.
