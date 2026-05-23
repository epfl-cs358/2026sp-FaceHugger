# FaceHugger servo & control conventions — TARGET (after changes B, C, D)

This is the **delta doc**: a copy of `DRAFT-current-reality-conventions.md` rewritten as
the *would-be reality* once the three agreed changes land:

- **B** — FL shoulder regularization (`servo = 90 + (sh − 135)`), so servo 90 = outward
  for all four legs (MIGRATE — needs hardware recalibration).
- **C** — a reachable reference-pose API: `T:2 s=4` → flat/all-90 calibration pose,
  `T:2 s=5` → standing/neutral (ADD).
- **D** — `isInverted` as a single live mirror at the servo write chokepoint (MIGRATE).

Every changed passage is tagged `CHANGED:` or `NEW:` with a one-line rationale, and each
states **which existing behaviours stay byte-for-byte identical** so the running robot
(gaits, clips, calibration) does not break. Untagged text is unchanged from the reality doc.

> **Status:** target / not-yet-landed. B and D must not ship without their migration
> checklists (see `DRAFT-planned-changelog.md` and
> `docs/superpowers/plans/2026-05-23-servo-convention-changes.md`). C is additive and safe.

All `file:line` references are against `feat/animation-flow-integration` at the time of
writing; the implementation plan owns keeping them current.

---

## 1. The control model: absolute servo degrees, every frame

*(Unchanged by B/C/D.)* Every motion source — gaits, the on-board clip player, and the
Blender exporter — ultimately produces **absolute servo angles in 0–180°** and writes them
straight to the PCA9685 servo driver. There are no deltas anywhere on the wire or in the
firmware: each frame fully specifies where each of the 12 servos should be.

All paths converge on a single chokepoint, `Servo::setServoAngle` (`servo.cpp:14`), which
clamps to `[0, 180]`, maps linearly to a pulse width
(`map(angle, 0, 180, MIN_PULSE=150, MAX_PULSE=600)`, `servo.cpp:22`, `config.h:13-14`), and
writes the PWM. The pulse map is strictly monotonic, so "a higher servo degree always pushes
one fixed physical direction" for any given servo.

The three producers reach that chokepoint slightly differently (gaits via
`translateToServo` → `setJointAngles`; clip player via `tickClip` → same; the exporter bakes
the equivalent `_frame_to_servo` on the host and ships finished servo degrees over T:4).

> **NEW (D):** change D adds *one more* transform between `setJointAngles` and the PCA write
> — the invert mirror — at this same chokepoint. It is the only place D touches the data
> path, so the "absolute servo degrees, single chokepoint" model is preserved, not widened.
> **Rationale:** putting invert at the choke means every producer inherits it identically
> instead of three partial implementations.

---

## 2. The global zero & rotation directions

*(Unchanged by B/C/D.)* Two convention images live in `code/simulation/docs/img/`. They are
canon for **rotation direction and the global zero only** — not the tuned neutral/standing
values or servo ranges. (Filenames are swapped vs content: `servo-1-…png` shows pitch,
`servos-2-3-…png` shows yaw.)

- **Shoulder yaw (servo 1):** viewed from above, front at top, positive is **CCW**. Each
  leg's *outward* (flat-spread) direction: **FL +135°, FR +45°, BR −45°, BL −135°**.
- **Thigh & knee pitch (servos 2–3):** legs flat = 0°, up = +, down = −.

The direction convention is **uniform across all four legs** in math-space. Per-leg hardware
mirroring is hidden one layer down (§3).

> Change B does **not** touch this section's directions. FL's *outward* direction is and
> stays +135°; B only changes which math angle servo 90 corresponds to (§4).

---

## 3. Math-space → servo: `translateToServo` is the single abstraction

The caller works in a **uniform, leg-agnostic math-space**: `+sh` = CCW yaw, `+th` / `+kn` =
up pitch, identically for all four legs. A single function, `translateToServo`
(`motion_math.cpp:4-31`), absorbs *all* hardware asymmetry. The exporter has a byte-identical
twin, `_frame_to_servo` (`fh_clip_panel.py:1902-1909`), locked to the firmware by
`test_servo_parity.py`.

**CHANGED (B):** the FL shoulder branch. **Rationale:** make servo 90 = outward for FL like
the other three legs, so an all-90 pose is symmetric and the FL/BL shoulder range stops
railing.

Today (reality):

```
FL:  hip = sh;              thigh = 90 + th;   knee = 90 - kn
FR:  hip = 90 + (sh - 45);  thigh = 90 - th;   knee = 90 + kn
BR:  hip = 90 - (sh + 45);  thigh = 90 + th;   knee = 90 - kn
BL:  hip = 90 + (sh + 135); thigh = 90 - th;   knee = 90 + kn
```

After B:

```
FL:  hip = 90 + (sh - 135); thigh = 90 + th;   knee = 90 - kn   ← CHANGED (was hip = sh)
FR:  hip = 90 + (sh - 45);  thigh = 90 - th;   knee = 90 + kn   ← unchanged
BR:  hip = 90 - (sh + 45);  thigh = 90 + th;   knee = 90 - kn   ← unchanged
BL:  hip = 90 + (sh + 135); thigh = 90 - th;   knee = 90 + kn   ← unchanged
```

Only the FL `hip` line changes, in **both** the firmware (`motion_math.cpp:13`) and the
exporter (`fh_clip_panel.py:1903`), so the byte-identical parity contract holds. FL's thigh
and knee branches, and **all three** other legs, are untouched.

The per-leg direction-sign table (`d(servo)/d(math_angle)`) is **identical before and after
B** — B is a pure *offset* change on FL's hip (a `+1` slope before and after), not a sign
flip:

| leg | shoulder | thigh | knee |
|-----|:--------:|:-----:|:----:|
| FL  | **+1** | **+1** | **−1** |
| FR  | **+1** | **−1** | **+1** |
| BL  | **+1** | **−1** | **+1** |
| BR  | **−1** | **+1** | **−1** |

**Unaffected by B (must stay byte-identical):** the BR shoulder sign-flip
(`tickYawRotation`'s `YAW_COEF[BR] = +1` vs the others' `−1`, `spinal_cord.cpp:336`) still
works — B does not touch BR or the slope table. The lift convention (`th += lift; kn -= lift`)
is unchanged.

The same physical L/R mirror is also carried in the URDF/rig; that representation is
unaffected by B (B is a firmware/exporter offset, not a kinematic axis change).

---

## 4. What servo 90 means: outward/flat-spread for **all** legs

**CHANGED (B).** Reality today: *"servo 90 corresponds to each leg's NEUTRAL (standing)
math-angle, not a universal outward direction,"* with FL the lone exception. After B, **servo
90 = the outward (flat-spread) direction for all four legs**, and FL's NEUTRAL shoulder moves
to its outward value. **Rationale:** removes the lone asymmetry so "all-12-servos-at-90 = a
symmetric flat pose" is finally true (which change C's flat-calibration command depends on).

The `NEUTRAL[]` table (`spinal_cord.cpp:17-22`) and `convention.json`'s `neutral_joint_deg`
still agree exactly; only FL's shoulder value changes:

| leg | NEUTRAL (sh, th, kn), math degrees | note |
|-----|------------------------------------|------|
| FR  | 45, −60, −37  | unchanged |
| FL  | **135**, −60, −40 | **CHANGED (was 75)** |
| BR  | −45, −50, −50 | unchanged |
| BL  | −135, −60, −35 | unchanged |

After B, all four standing shoulder math-angles equal their outward image directions
(135 / 45 / −45 / −135). The three corroborating sites for FL all move together:

- `config.h:32` `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE`: **75 → 90** (the *servo* boot default;
  90 because outward maps to servo 90 once the branch recenters).
- `spinal_cord.cpp:19` `NEUTRAL[FL].sh`: **75 → 135** (the *math* neutral).
- `convention.json` `"fl"[0]`: **75 → 135**.
- the FL `translateToServo` branch gains the standard recentring `90 + (sh − 135)` (§3).

Cross-check the arithmetic: FL standing math 135 → `90 + (135 − 135) = 90` servo, i.e. the
boot default of 90. Consistent.

> **CRITICAL — must not ship the formula change without the physical recalibration.** The
> running robot's FL horn is currently mounted so that servo 75 = FL's standing pose. After
> B, the firmware will command servo 90 for that same standing pose. If the formula+config
> change ships but the horn is not re-mounted, FL will sit ~15° (servo) off and every FL
> motion will be biased. The recalibration + FL clip re-bake + parity re-run are mandatory
> migration steps (see changelog/plan). **Until B fully lands, the reality doc's "FL is the
> exception" wording stays true.**

**Unaffected by B:** FR/BR/BL NEUTRAL values; the 2/3 amplitude scale; the absolute-angle
model; gait math; clip interpolation. Existing baked clips remain valid *for the legs other
than FL*; FL keys must be re-baked because their meaning shifts by the 75→135 reframing.

---

## 5. Defaults vs limits

*(Unchanged by B/C/D, except the one FL boot-default value noted in §4.)* Three distinct
concepts in three places:

- **Boot / neutral defaults** — `*_DEFAULT_ANGLE` macros in `config.h:23-52` (servo values,
  applied at boot by `returnToDefaultAngles`). **B changes exactly one of these:**
  `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE` 75 → 90. config.h still carries no range limits.
- **Runtime joint limits** (IK clamp) — `kinematics.cpp:28-33`/`:79-81`, radians; only bite
  on an IK path, which live gaits/clips do not use. The practical everyday clamp is the
  0–180 `constrain` at `servo.cpp:18` / `leg.cpp:64-66`. *(Unchanged.)*
- **Kinematic limits for sim/exporter** — URDF joint `<limit>` tags. *(Unchanged.)*

---

## 6. The T:4 wire value, and why it is special

A raw `T:4` (`CMD_CALIBRATE`) packet carries `{id, servo_id, a}`, is index-validated
(`isValidServoIndex`, `network.cpp:97`), and writes `a` **verbatim** to the resolved PCA
channel — it does **not** call `translateToServo`. *(This much is unchanged.)*

> **CHANGED (D):** with D, a T:4 angle still skips `translateToServo`, but — if the robot is
> currently inverted — it is now mirrored by the single `servo' = 180 − servo` transform at
> the write chokepoint, exactly like every other source. **Rationale:** today T:4 ignores
> `isInverted` entirely, which is part of the source-inconsistency D removes (§7).
> **Caveat for calibration use:** because calibration is normally done with the robot
> upright (not inverted), the everyday calibration flow is unaffected — invert is off, so
> `180 − servo` is the identity-free path. The behaviour only differs if someone calibrates
> while inverted, which is the consistent and intended result.

For a calibration client poking one channel upright: pick a number and watch the horn — the
convention is irrelevant, exactly as today.

(The orchestration's older note about an *unbounded* T:4 index is out of date: the
`isValidServoIndex` guard at `network.cpp:97-101` has landed.)

---

## 7. Robot-invert (`isInverted`) as a single live mirror

**CHANGED (D).** Reality today: invert is **partial and source-inconsistent** — gaits negate
`th`/`kn` in math-space (`spinal_cord.cpp:216`, `:310`, `:369`); `invertRobot()` writes a
hardcoded one-shot `180 − default` servo pose (`:434-438`); the clip player and T:4 ignore
`isInverted` entirely. After D there is **one** invert behaviour. **Rationale:** a flipped
robot should run *every* motion source identically, including clips and the exporter's
WebSocket stream.

After D:

- The invert is a single transform **`servo' = 180 − servo`** applied at the PCA-write
  chokepoint (`Servo::setServoAngle`, or a thin wrapper just before it), gated by the
  firmware-owned invert flag. Every producer — gaits, `tickClip`, the T:4 handler — inherits
  it automatically because they all funnel through that one write.
- The per-gait `if (isInverted) { th = -th; kn = -kn; }` lines are **removed** from all three
  tickers, otherwise the robot would double-invert (once in math-space, once at the choke).
- `invertRobot()` no longer writes a separate hardcoded pose; it only toggles the flag (and
  may snap the current pose through the new mirror). The hardcoded `180 − default` literals
  are retired.

> **NEW — flip-axis semantics decision (gates D).** `180 − servo` is a per-channel mirror. A
> *physically* upside-down quadruped may also need a left↔right leg-role swap for a true
> roll-over. **This must be decided before D writes code** — see the plan's Task D Step 0
> decision checkpoint. The delta doc records both options; it does not pick one.

> **CRITICAL — must not regress existing flip behaviour.** D must reproduce the OLD
> thigh/knee invert result for gaits (the negate-then-translate path and the
> mirror-at-choke path must agree for th/kn), and must leave the *upright* (invert-off) path
> byte-identical. Prereq: **B** — `180 − servo` is only a clean mirror once 90 = flat
> uniformly (otherwise FL's old 75-based servo values mirror to the wrong place). Gated
> behind the decision above + the regression tests in the plan.

**Unaffected by D:** all upright motion (invert flag off) is byte-identical to today — gaits,
clips, calibration, the deadman switch. The `{T:6, a:0}` `INVERT_ROBOT` trigger
(`network.cpp:136-143`) keeps the same wire API; only its internal effect is unified.

---

## 8. Pose & calibration surface — now with reachable reference poses

**CHANGED (C).** Reality today: there is **no API command to assume a named pose** —
`relax()` (all-90, `spinal_cord.cpp:93-99`) is dead code, `STATE_REST=4` is unreachable, and
`CMD_POSE`/`T:3` has no handler. After C, two reference poses are reachable via the existing
`CMD_STATE` (T:2). **Rationale (decided, option A):** smallest change, reuses existing FSM
semantics, no new command type, no clash with T:3's documented body-pose-IK meaning.

After C — the `CMD_STATE` (T:2) state table gains two reachable values:

| `s` | state | effect | status |
|-----|-------|--------|--------|
| 0 | IDLE | `rest()` | unchanged |
| 1 | WALK | `walk()` | unchanged |
| 2 | ACTION | `wallFlip()` | unchanged |
| 3 | FAILSAFE | (no-op fall-through today) | unchanged |
| **4** | **REST** | **`relax()` — all 12 servos to 90 (flat / calibration pose)** | **NEW (C)** |
| **5** | **STAND** | **`stand()` — drive legs to `NEUTRAL[]` (standing/neutral)** | **NEW (C)** |

Concretely, C:

- **NEW:** raises the `CMD_STATE` bounds check from `<= STATE_FAILSAFE` (3) to include the
  two new values (`network.cpp:80`), and adds the `case STATE_REST: spinalCord.relax();` and
  `case STATE_STAND: spinalCord.stand();` arms. **Rationale:** makes the existing-but-dead
  `relax()` reachable, and adds an explicit return-to-neutral.
- **NEW:** adds `STATE_STAND = 5` to the `RobotState` enum (`data.h:17-23`) and a new
  `SpinalCord::stand()` that drives each leg to `NEUTRAL[]` via `translateToServo` (the same
  path gaits use to reach standing today, just on demand). `relax()` already exists and is
  reused unchanged for the flat pose.
- **NEW:** documents `s=4` and `s=5` in `API_SPEC.md` §2.
- The selector `s` stays bounds-checked: out-of-range values remain a no-op.

> **T:3 left as-is (known gap, unchanged).** `CMD_POSE`/`T:3` keeps its documented
> body-pose / static-IK meaning (`API_SPEC.md:54-61`) and remains **unimplemented** — a T:3
> packet is still silently dropped. C deliberately does **not** repurpose it; that body-pose
> IK is future work.

**Unaffected by C:** every existing command and state (IDLE/WALK/ACTION, T:1/T:4/T:5/T:6/T:7,
the deadman switch). C is purely additive — nothing auto-calls the new poses (not on boot),
so a robot that never sends `s=4`/`s=5` behaves exactly as today. `T:7` clip playback still
auto-eases to the standing NEUTRAL pose at the end of a clip (`spinal_cord.cpp:411-414`); C
just makes that same standing pose reachable on demand too.

---

## 9. What an exporter must own vs. can ignore

*(Largely unchanged; B updates one number, D adds one note.)* An exporter emitting servo
degrees over T:4:

**Must own** (the firmware will not do it on the T:4 path):

- the **NEUTRAL anchor** — the per-leg standing math-angles servo 90 is built around.
  **CHANGED (B): FL's anchor is 135, not 75** — after B there is no FL special case; all four
  legs anchor at their outward direction.
- the **2/3 amplitude scale** toward neutral (`scale = 0.6667`, `convention.json:9`). *(Unchanged.)*
- the **lift / pitch sign convention** (`th += lift; kn -= lift`). *(Unchanged.)*

**Can ignore** (because `translateToServo` / `_frame_to_servo` absorbs it): the per-leg L/R
mirror and shoulder offsets. *(Unchanged.)*

> **NEW (D):** if the robot may be **inverted** while the exporter streams, the exporter no
> longer needs to pre-mirror anything — the firmware's single invert transform handles it.
> This is the foundation for change **E** (a keyframe-able invert toggle in the Blender
> exporter, design-only this round): once D centralizes the mirror, the exporter can emit an
> "invert" state change mid-clip and the firmware reinterprets every subsequent frame
> automatically.

In short: the exporter owns the *pose model* (neutral incl. FL=135 after B, scale, lift
sign); the per-leg *hardware mirror* and (after D) the *flip mirror* are handled for it.

---

## Summary of changed sites (for the implementer)

| Change | File:line | Before | After |
|--------|-----------|--------|-------|
| B | `motion_math.cpp:13` | `out.hip = sh;` | `out.hip = 90.0 + (sh - 135.0);` |
| B | `fh_clip_panel.py:1903` | `servo = [sh, 90 + th, 90 - kn]` | `servo = [90 + (sh - 135), 90 + th, 90 - kn]` |
| B | `spinal_cord.cpp:19` | `NEUTRAL[FL].sh = 75.0f` | `135.0f` |
| B | `config.h:32` | `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE 75` | `90` |
| B | `convention.json` `fl[0]` | `75` | `135` |
| C | `network.cpp:80` | `newState <= STATE_FAILSAFE` | include `STATE_REST`/`STATE_STAND` + new `case` arms |
| C | `data.h:17-23` | (no `STATE_STAND`) | add `STATE_STAND = 5` |
| C | `spinal_cord.{h,cpp}` | (no `stand()`) | add `stand()` driving `NEUTRAL[]` via `translateToServo`; reuse `relax()` |
| C | `API_SPEC.md` §2 | s = 0..3 | document s=4 (REST) and s=5 (STAND) |
| D | `servo.cpp` chokepoint | (no invert) | `servo' = 180 - servo` when invert flag set |
| D | `spinal_cord.cpp:216,310,369` | `if(isInverted){th=-th;kn=-kn;}` | removed |
| D | `spinal_cord.cpp:431-447` | hardcoded `180-default` pose | toggle flag only |

## Explicitly DO NOT change (carried from the ledger)

- The other per-leg `translateToServo` signs (BR shoulder flip; FR/BL vs FL/BR thigh/knee
  mirror) — correct and load-bearing.
- The 2/3 amplitude scale, the absolute-angle model, the standing NEUTRAL for FR/BR/BL.
- Gait math, clip interp, the deadman switch.
