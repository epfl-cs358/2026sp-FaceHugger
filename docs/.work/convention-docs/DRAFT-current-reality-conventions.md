# FaceHugger servo & control conventions — current reality

This describes how FaceHugger's control and servo conventions work **today**, as a
teammate or future-you wiring an animation exporter or an API client would need to
understand them. It is the baseline: it documents what the code does, including the
parts that are inconsistent, not what we wish it did. Anything genuinely broken or
half-wired is tagged `(known gap)`.

All claims here are verified against the live firmware and exporter on
`feat/animation-flow-integration`; key `file:line` references are listed at the end.

---

## 1. The control model: absolute servo degrees, every frame

Every motion source — gaits, the on-board clip player, and the Blender exporter —
ultimately produces **absolute servo angles in 0–180°** and writes them straight to
the PCA9685 servo driver. There are no deltas anywhere on the wire or in the
firmware: each frame fully specifies where each of the 12 servos should be.

All paths converge on a single chokepoint, `Servo::setServoAngle`
(`servo.cpp:14`), which clamps to `[0, 180]`, maps linearly to a pulse width
(`map(angle, 0, 180, MIN_PULSE=150, MAX_PULSE=600)`, `servo.cpp:22`,
`config.h:13-14`), and writes the PWM. The pulse map is strictly monotonic, so
"a higher servo degree always pushes one fixed physical direction" for any given
servo.

The three producers reach that chokepoint slightly differently:

- **Gaits** (`tickGait`, `tickTrot`, `tickYawRotation` in `spinal_cord.cpp`) compute
  per-leg math-space angles, convert them with `translateToServo`, then call
  `Leg::setJointAngles` → `setServoAngle` (`spinal_cord.cpp:220-221`, `:313-314`,
  `:371-372`).
- **The on-board clip player** (`tickClip`, `spinal_cord.cpp:395-429`) samples baked
  math-space frames, runs the same `translateToServo`, and calls `setJointAngles`
  (`spinal_cord.cpp:407-409`).
- **The Blender exporter** bakes `translateToServo`'s logic itself in
  `_frame_to_servo` (`fh_clip_panel.py:1879-1923`) and ships **already-converted
  servo degrees** over the WebSocket as `{T:4, id, servo_id, a}` packets
  (`fh_clip_panel.py:2121-2129`). The firmware T:4 handler then writes that number
  to the PCA channel verbatim — see §6.

So the exporter is already in the right model: it does the math→servo conversion on
the host and sends finished servo angles.

---

## 2. The global zero & rotation directions

Two convention images live in `code/simulation/docs/img/`. They are canon for
**rotation direction and the global zero only** — not for the tuned neutral/standing
values or servo ranges, which are physical calibration choices (see §4).

**Heads-up — the two filenames are swapped relative to their contents:**

- `servo-1-rotation-convention.png` actually shows the **pitch** convention (thigh
  and knee, servos 2–3): *"When the legs are laid down flat, the respective angles
  are 0 degrees… up ⇒ positive, down ⇒ negative."*
- `servos-2-3-rotation-convention.png` actually shows the **shoulder yaw** (servo 1)
  convention: a top-down unit circle, front of the robot at the top, angles measured
  CCW.

The conventions themselves:

- **Shoulder yaw (servo 1):** viewed from above with the robot's front at the top,
  positive is **counter-clockwise (CCW)**. The image gives each leg's *outward*
  (flat-spread) direction on that circle: **FL +135°, FR +45°, BR −45°, BL −135°**.
- **Thigh & knee pitch (servos 2–3):** legs laid flat = 0°, rotating **up = +**,
  **down = −**.

This direction convention is **uniform across all four legs** in math-space: a
positive `sh` is CCW-from-above for every leg, and positive `th`/`kn` is "up" for
every leg. That uniformity is the whole point — callers never branch per leg for
direction. The per-leg hardware mirroring is hidden one layer down (§3).

---

## 3. Math-space → servo: `translateToServo` is the single abstraction

The caller works in a **uniform, leg-agnostic math-space**: `+sh` = CCW yaw, `+th` /
`+kn` = up pitch, identically for all four legs. A single function,
`translateToServo` (`motion_math.cpp:4-31`), absorbs *all* hardware asymmetry and
maps that uniform math-space onto each physical servo's polarity and centring:

- the per-leg shoulder yaw offsets and the **BR shoulder sign-flip**,
- the **FR/BL vs FL/BR thigh & knee L/R mirror**,
- the 90° servo centring (and FL's no-offset special case, §4).

The exporter has a byte-identical twin, `_frame_to_servo` (`fh_clip_panel.py:1902-1909`),
locked to the firmware by `test_servo_parity.py`. Both look like this:

```
FL:  hip = sh;              thigh = 90 + th;   knee = 90 - kn
FR:  hip = 90 + (sh - 45);  thigh = 90 - th;   knee = 90 + kn
BR:  hip = 90 - (sh + 45);  thigh = 90 + th;   knee = 90 - kn
BL:  hip = 90 + (sh + 135); thigh = 90 - th;   knee = 90 + kn
```

Differentiating each branch gives the per-leg, per-joint direction sign —
`d(servo)/d(math_angle)` — i.e. whether driving a math angle up drives the servo up
(+1) or down (−1):

| leg | shoulder | thigh | knee |
|-----|:--------:|:-----:|:----:|
| FL  | **+1** | **+1** | **−1** |
| FR  | **+1** | **−1** | **+1** |
| BL  | **+1** | **−1** | **+1** |
| BR  | **−1** | **+1** | **−1** |

Two things to read off this table:

- **Shoulder is +1 for three legs and −1 only for BR.** BR is the single leg whose
  hip formula inverts `sh`. So to nudge a shoulder CCW-from-above by one degree, you
  send `+1` on FR/FL/BL but `−1` on BR. (This is exactly why `tickYawRotation` carries
  a per-leg `YAW_COEF` with BR flipped, `spinal_cord.cpp:334-336`.)
- **Thigh and knee are always anti-correlated within a leg** (+1/−1 or −1/+1). That
  matches the lift convention every gait uses: `th += lift; kn -= lift`
  (`spinal_cord.cpp:213-214`, `:302-303`, `:362-363`).

The same physical L/R mirror is *also* carried independently in the URDF joint
`<axis>` signs and in the Blender rig, but those are erased before the exporter: the
rig roll-aligns every bone-local Z onto its URDF joint axis and folds the axis sign
in at rig-build time, so a positive bone `rotation_euler[2]` is already uniform CCW
across all 12 bones. The exporter therefore feeds `translateToServo` the *same*
uniform math-space the gaits use. The mirror lives in both the URDF/rig and in
`translateToServo`, but as two independent representations of one physical fact, not
a single chained one.

---

## 4. What servo 90 means: each leg's NEUTRAL, not a universal "outward"

**Servo 90 corresponds to each leg's NEUTRAL (standing) math-angle, not to a
universal outward direction.** The firmware `NEUTRAL[]` table
(`spinal_cord.cpp:17-22`) and `convention.json`'s `neutral_joint_deg` agree
exactly:

| leg | NEUTRAL (sh, th, kn), math degrees |
|-----|------------------------------------|
| FR  | 45, −60, −37 |
| FL  | **75**, −60, −40 |
| BR  | −45, −50, −50 |
| BL  | −135, −60, −35 |

For **FR, BR, BL** the standing shoulder math-angle equals the image's outward
direction (45 / −45 / −135). **FL is the lone exception: it stands at math 75, while
its outward direction is +135.** This is deliberate and internally self-consistent,
corroborated in four places:

- `config.h:32` `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE = 75`,
- `spinal_cord.cpp:19` `NEUTRAL[FL] = { 75.0f, -60.0f, -40.0f }`,
- `convention.json` `"fl": [75, -60, -40]`,
- the FL branch of `translateToServo` uniquely has **no recentring** (`hip = sh`,
  `motion_math.cpp:13`), so math-neutral 75 maps directly to servo 75.

Introduced as the "standing crouched pose" in commit `e005f5a`
("feat(animation): add convention.json — hardware neutral/scale/channels"). The
other three legs need the `90 + (sh − const)` recentring precisely because their
math-neutral (45/−45/−135) is *not* 90; FL's math-neutral happens to equal its servo
value, so it needs none.

**Why this asymmetry is invisible in practice today.** Nothing in the live pipeline
requires FL's servo-90 to be its outward direction. The standing pose, all gaits, and
every baked clip were authored in this same 75-based frame, so it is self-consistent
end-to-end. FL being 15° off the nominal "90 = flat" only shows up if you ever drive
all servos to 90 at once (a symmetric flat-spread pose) — and the only thing that
does that is dead code (`relax()`, §7). It also contributes to shoulder values riding
the legal-range rail at travel extremes (e.g. BL's standing shoulder sits at servo
~179, FL near 75±sweep), which the clamp handles. `(known gap — the lone asymmetry,
deliberate but a trap for anyone assuming 90 = flat for every leg)`

---

## 5. Defaults vs limits

These are three distinct concepts living in three places; don't conflate them.

- **Boot / neutral defaults** are the `*_DEFAULT_ANGLE` macros in `config.h:23-52`.
  They are *servo* values, used to construct each `Servo` and applied at boot by
  `returnToDefaultAngles` (`spinal_cord.cpp:65-72`, via `servo.cpp:59-60`). For
  example FL hip default = 75, FR thigh default = 150. **config.h carries no range
  limits** — only these neutral defaults and the PCA channel/pulse constants.
- **Runtime joint limits** (the IK clamp) live in `kinematics.cpp:28-33`
  (`SHOULDER/HIP/KNEE_LIMIT_LO/HI`, in radians) and are applied by the IK clamp
  (`kinematics.cpp:79-81`). Note these only bite on an IK path; the live gaits and
  clips do not call IK — they anchor to `NEUTRAL[]` and convert directly — so the
  practical safety clamp for everyday motion is the 0–180 `constrain` at
  `servo.cpp:18` / `leg.cpp:64-66`.
- **Kinematic limits for the sim/exporter side** are carried by the URDF joint
  `<limit>` tags, used by PyBullet and the Blender rig.

---

## 6. The T:4 wire value, and why it is special

A raw `T:4` (`CMD_CALIBRATE`) packet carries `{id, servo_id, a}` and is the most
direct path to a servo. The handler (`network.cpp:90-109`) validates the indices
(`isValidServoIndex`, `network.cpp:97`), looks up
`LEG_SERVO_CHANNEL[id][servo_id]` (`config.h:64-69`), and calls
`applyCalibration` → `identifyAndMove` → `setServoAngle`, which writes `a`
**verbatim** to the PCA channel.

The key fact: **the T:4 path does NOT call `translateToServo`.** No per-leg mirror,
no centring, no scaling. `a` is just "drive this PCA channel to this 0–180 angle."
That is exactly why the exporter bakes `translateToServo` on the host and ships
finished servo degrees — the wire neither knows nor applies any convention.

For a calibration client poking one channel, this means you can ignore CCW/CW
entirely: just pick a number and watch which way the horn turns. If you do want a
specific physical CCW nudge, the only fact you need is the shoulder row of the §3
table — `+1` on `a` is CCW for FR/FL/BL, CW for BR.

(The orchestration's older note about an *unbounded* T:4 index is out of date: the
`isValidServoIndex` guard at `network.cpp:97-101` has landed, so a malformed
`id`/`servo_id` is now rejected with a warning rather than reading out of bounds.)

---

## 7. Robot-invert (`isInverted`) as it is today

The flip state is `isInverted` (`spinal_cord.cpp:60`), toggled by `invertRobot()`
(`spinal_cord.cpp:431-447`), which is reached from the API via `{T:6, a:0}`
(`CMD_ACTION_SELECTION` with `INVERT_ROBOT = 0`, `network.cpp:136-143`,
`movements.h:23`). **Invert is partial and source-inconsistent today** — there are
effectively two different invert behaviours that don't fully line up:

- **Gaits** apply `if (isInverted) { th = -th; kn = -kn; }` in math-space, *before*
  `translateToServo`, in all three tickers (`spinal_cord.cpp:216`, `:310`, `:369`).
  Only thigh and knee are negated; shoulder yaw is left alone.
- **`invertRobot()` itself** does not route through `translateToServo` at all — when
  toggled on it writes a **hardcoded one-shot servo pose** directly
  (`spinal_cord.cpp:434-438`, e.g. `leg1.setJointAngles(90, 30, 127)`), which is the
  `180 − servo` of each leg's default. When toggled off it just returns to defaults.
- **The clip player (`tickClip`) and the T:4 handler ignore `isInverted` entirely.**
  Clips play exactly as authored; calibration pokes are never mirrored.

So whether the robot's flip state actually affects motion depends on which source is
driving: gaits respect it (thigh/knee only), clips and direct calibration do not, and
the one-shot invert pose is a separate hardcoded thing. `(known gap)`

---

## 8. Pose & calibration surface today

There is currently **no API command to assume a named pose.** The relevant pieces:

- **`relax()`** (`spinal_cord.cpp:93-99`) drives all 12 servos to 90 (the flat
  pose, "safe to power off") and sets `STATE_REST`. It is **dead code — called from
  nowhere.** `(known gap)`
- **`CMD_POSE` / `T:3`** is in the command enum (`data.h:8`) and documented in
  `API_SPEC.md:54-61` as a chassis body-pose / static-IK command (`h`/`p`/`r`). But
  **`network.cpp` has no `case CMD_POSE`** — a `T:3` packet is silently dropped.
  `(known gap)`
- **`STATE_REST = 4`** (`data.h:22`) is **unreachable** from the API: the `T:2`
  (`CMD_STATE`) handler bounds-checks `STATE_IDLE..STATE_FAILSAFE` (0–3,
  `network.cpp:80`) and only wires IDLE/WALK/ACTION; even `STATE_FAILSAFE = 3` passes
  the bound but falls through to `default: break`, so it is also a no-op. `(known gap)`
- **`T:4`** (`CMD_CALIBRATE`) is the single-servo calibration poke (§6).
- **`T:7`** (`CMD_PLAY_CLIP`) plays a bundled clip once, then auto-eases to the
  neutral standing pose over 500 ms and returns to IDLE (`spinal_cord.cpp:376-393`,
  `network.cpp:145-153`).

The only way the robot reaches its standing NEUTRAL pose today is implicitly: a gait
or clip easing back to defaults at the end of motion (`returnToDefaultAngles` /
`returnToDefaultAnglesTimed`). There is no explicit "stand"/"neutral" or "flat" pose
command on the wire.

---

## 9. What an exporter must own vs. can ignore

For someone writing an animation exporter that emits servo degrees over T:4:

**Must own** (the firmware will not do it for you on the T:4 path):

- the **NEUTRAL anchor** — the per-leg standing math-angles servo 90 is built around,
  including FL's 75 special case (§4);
- the **2/3 amplitude scale** toward neutral (`scale = 0.6667`, `convention.json:9`,
  applied in `_frame_to_servo` step 1, `fh_clip_panel.py:1900`);
- the **lift / pitch sign convention** (`th += lift; kn -= lift`), so up-is-positive
  comes out right.

**Can ignore** (because `translateToServo` / `_frame_to_servo` absorbs it):

- the **per-leg L/R mirror** and shoulder offsets — the BR shoulder sign-flip and the
  FR/BL vs FL/BR thigh/knee mirror are all baked into the per-leg branch, so the
  exporter feeds uniform CCW-up math-space and lets the branch do the rest.

In short: the exporter owns the *pose model* (neutral, scale, lift sign); the per-leg
*hardware mirror* is handled for it, as long as it uses the same byte-identical
per-leg formulas the firmware does (kept honest by `test_servo_parity.py`).

---

## Sources

- `code/firmware/src/nervous_system/motion_math.cpp:4-31` — `translateToServo`, the
  per-leg math→servo branch (FL `hip=sh`; FR/BR/BL offsets; thigh/knee signs).
- `code/firmware/src/nervous_system/spinal_cord.cpp:17-22` — `NEUTRAL[]` table.
- `code/firmware/src/nervous_system/spinal_cord.cpp:93-99` — `relax()` (dead, all-90).
- `code/firmware/src/nervous_system/spinal_cord.cpp:216`, `:310`, `:369` — gait
  `isInverted` thigh/knee negation.
- `code/firmware/src/nervous_system/spinal_cord.cpp:334-336` — `tickYawRotation`
  `YAW_COEF` (BR shoulder flip).
- `code/firmware/src/nervous_system/spinal_cord.cpp:395-429` — `tickClip` (ignores
  invert).
- `code/firmware/src/nervous_system/spinal_cord.cpp:431-447` — `invertRobot()`
  hardcoded `180−servo` pose.
- `code/firmware/src/nervous_system/servo.cpp:14-25` — `setServoAngle` chokepoint,
  clamp + pulse map.
- `code/firmware/src/nervous_system/leg.cpp:63-66`, `:98-101` — `setJointAngles`,
  `identifyAndMove`.
- `code/firmware/src/nervous_system/kinematics.cpp:28-33`, `:79-81` — IK joint limits
  / clamp.
- `code/firmware/src/shared/config.h:13-14` — pulse range; `:23-52` —
  `*_DEFAULT_ANGLE` neutrals; `:64-75` — `LEG_SERVO_CHANNEL`, `isValidServoIndex`.
- `code/firmware/src/shared/data.h:5-23` — `CommandType` (incl. `CMD_POSE=3`),
  `RobotState` (incl. `STATE_REST=4`).
- `code/firmware/src/brain/network.cpp:77-153` — T:2/T:3(none)/T:4/T:5/T:6/T:7
  handlers; `:97-101` T:4 bounds guard.
- `code/firmware/src/nervous_system/movements.h:23` — `INVERT_ROBOT = 0`.
- `animation/scripts/fh_clip_panel.py:1879-1923` — `_frame_to_servo` (exporter twin
  + scale + range clamp); `:2121-2129` — T:4 wire emit.
- `animation/convention.json` — `neutral_joint_deg`, `scale = 0.6667`, channels.
- `code/API_SPEC.md:43-99` — wire packet docs (T:2..T:7; T:3 documented as body-pose
  IK).
- `code/simulation/docs/img/servo-1-rotation-convention.png` (pitch),
  `servos-2-3-rotation-convention.png` (yaw) — filenames swapped vs content.
- Commit `e005f5a` — convention.json / FL=75 standing-crouched pose.
