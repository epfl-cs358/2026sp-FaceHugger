# FaceHugger servo & control conventions

> **Branch note (read first):** this document describes the **post-Change-B** world on
> `feat/animation-flow-integration` — i.e. the FL shoulder *regularized* so servo 90 =
> outward for all four legs (`90 + (sh − 135)`, `NEUTRAL[FL]=135`, config FL hip = 90).
> The **`feat/safe-flash`** branch (what's flashed for testing) is **pre-B**: there FL is
> still `servo = sh`, `NEUTRAL[FL]=75`, `convention.json` fl = 75. Everything else
> (the C poses, D's pitch-only invert, 3a/3b smoothing, the direction signs, FR/BR/BL) is
> identical on both branches. On safe-flash the exporter and firmware still agree with each
> other (both old FL) — they're consistent; this doc is simply *ahead* on the FL bits.
> safe-flash also carries a **temporary `clampClipServos`** safeguard (clip-path only:
> shoulder→[38,142], thigh→[30,150]) not yet reflected below; it does not exist on the
> integration branch yet.

This is the reference for how FaceHugger's control and servo conventions work — what a
teammate (or future-you) wiring an animation exporter, an API client, or a new gait needs
to know. It documents the conventions **as the code is today**, on
`feat/animation-flow-integration`, after the servo-convention and smoothing work (changes
C, 3a, 3b, B, D) landed. Every claim is verified against the live firmware and exporter;
key `file:line` references are listed at the end.

One caveat up front: change B's *code* is committed, but its hardware step (re-mounting the
FL shoulder horn) is held. Until that remount happens and the FL clips are re-exported, do
not flash B-affected behaviour to the live robot. The conventions below describe the code
as written; the "known gaps" section calls out where hardware still lags.

---

## 1. The control model: absolute servo degrees, every frame

Every motion source — gaits, the on-board clip player, and the Blender exporter — ultimately
produces **absolute servo angles in 0–180°** and writes them to the PCA9685 servo driver.
There are no deltas anywhere on the wire or in the firmware: each frame fully specifies where
all 12 servos should be.

All paths converge on one electrical chokepoint, `Servo::setServoAngle` (`servo.cpp:15`),
which clamps to `[0, 180]` and maps linearly to a pulse width
(`map(angle, 0, 180, MIN_PULSE=150, MAX_PULSE=600)`, `servo.cpp:23`, `config.h:13-14`). The
pulse map is strictly monotonic, so a higher servo degree always pushes one fixed physical
direction for a given servo.

The three producers reach that chokepoint slightly differently:

- **Gaits** (`tickGait`, `tickTrot`, `tickYawRotation` in `spinal_cord.cpp`) compute per-leg
  math-space angles, convert with `translateToServo`, and write via `setJointAngles`.
- **The on-board clip player** (`tickClip`, `spinal_cord.cpp:423`) samples baked math-space
  frames, smooths them, runs the same `translateToServo`, and writes.
- **The Blender exporter** bakes `translateToServo`'s logic itself in `_frame_to_servo`
  (`fh_clip_panel.py:1879`) and ships **already-converted servo degrees** over the WebSocket
  as `{T:4, …, a}` packets. The firmware T:4 handler writes that number to the PCA channel
  verbatim (§6).

So the exporter is already in the right model: it does the math→servo conversion on the host
and sends finished servo angles.

There is now also a single **robot-side** transform between math→servo conversion and the
PCA write: the invert mirror, applied at the `applyServos` choke (§7). It is the only thing
the firmware layers on top of an absolute frame.

---

## 2. The global zero & rotation directions

Two convention images live in `code/simulation/docs/img/`. They are canon for **rotation
direction and the global zero only** — not for the tuned neutral/standing values or servo
ranges, which are physical calibration choices (§4).

The conventions:

- **Shoulder yaw (servo 1):** viewed from above with the robot's front at the top, positive
  is **counter-clockwise (CCW)**. Each leg's *outward* (flat-spread) direction on that circle
  is **FL +135°, FR +45°, BR −45°, BL −135°**.
- **Thigh & knee pitch (servos 2–3):** legs laid flat = 0°, rotating **up = +, down = −**.

This direction convention is **uniform across all four legs** in math-space: a positive `sh`
is CCW-from-above for every leg, and positive `th`/`kn` is "up" for every leg. That
uniformity is the whole point — callers never branch per leg for direction. The per-leg
hardware mirroring is hidden one layer down (§3).

> Heads-up: the two image filenames are swapped relative to their contents.
> `servo-1-rotation-convention.png` actually shows the **pitch** convention; and
> `servos-2-3-rotation-convention.png` actually shows the **shoulder yaw** convention. This
> is a known cosmetic issue (see "known gaps").

---

## 3. Math-space → servo: `translateToServo` is the single per-leg abstraction

The caller works in a **uniform, leg-agnostic math-space**: `+sh` = CCW yaw, `+th`/`+kn` = up
pitch, identically for all four legs. A single function, `translateToServo`
(`motion_math.cpp:22-50`), absorbs *all* hardware asymmetry and maps that uniform math-space
onto each physical servo's polarity and centring — the per-leg shoulder offsets, the BR
shoulder sign-flip, and the FR/BL vs FL/BR thigh/knee L/R mirror.

The exporter has a byte-identical twin, `_frame_to_servo` (`fh_clip_panel.py:1902-1911`),
locked to the firmware by `test_servo_parity.py`. Both now read identically across all four
legs:

```
FL:  hip = 90 + (sh - 135);  thigh = 90 + th;   knee = 90 - kn
FR:  hip = 90 + (sh - 45);   thigh = 90 - th;   knee = 90 + kn
BR:  hip = 90 - (sh + 45);   thigh = 90 + th;   knee = 90 - kn
BL:  hip = 90 + (sh + 135);  thigh = 90 - th;   knee = 90 + kn
```

Note FL's hip is now `90 + (sh - 135)` like the other three — the historic `hip = sh` special
case is gone (this is change B; see §4).

Differentiating each branch gives the per-leg, per-joint **direction sign** —
`d(servo)/d(math_angle)`, i.e. whether driving a math angle up drives the servo up (+1) or
down (−1):

| leg | shoulder | thigh | knee |
|-----|:--------:|:-----:|:----:|
| FL  | **+1** | **+1** | **−1** |
| FR  | **+1** | **−1** | **+1** |
| BL  | **+1** | **−1** | **+1** |
| BR  | **−1** | **+1** | **−1** |

Two things to read off this table:

- **Shoulder is +1 for three legs and −1 only for BR.** BR is the single leg whose hip
  formula inverts `sh`. So to nudge a shoulder CCW-from-above by one degree, you send `+1` on
  FR/FL/BL but `−1` on BR. (This is exactly why `tickYawRotation` carries a per-leg `YAW_COEF`
  with BR flipped, `spinal_cord.cpp:359`.)
- **Thigh and knee are always anti-correlated within a leg** (+1/−1 or −1/+1). That matches
  the lift convention every gait uses: `th += lift; kn -= lift`.

B is a pure *offset* change on FL's hip, not a sign flip — the table above is identical before
and after B.

The same physical L/R mirror is *also* carried independently in the URDF joint `<axis>` signs
and in the Blender rig, but those are erased before the exporter: the rig roll-aligns every
bone-local Z onto its URDF joint axis at rig-build time, so a positive bone
`rotation_euler[2]` is already uniform CCW across all 12 bones. The exporter therefore feeds
`translateToServo` the *same* uniform math-space the gaits use. The mirror lives in both the
URDF/rig and in `translateToServo`, as two independent representations of one physical fact —
not a single chained one.

---

## 4. What servo 90 means: each leg's NEUTRAL, and now also outward for all four legs

Two facts coincide today:

- **Servo 90 is each leg's flat-spread / outward direction** — for all four legs. After
  change B, FL was regularized to match FR/BR/BL: its outward shoulder direction (+135° math)
  now maps to servo 90, like the others.
- **Servo 90 is also each leg's standing NEUTRAL shoulder**, because the standing NEUTRAL
  shoulder math-angle for every leg is now its outward direction.

The firmware `NEUTRAL[]` table (`spinal_cord.cpp:21-26`) and `convention.json`'s
`neutral_joint_deg` agree exactly:

| leg | NEUTRAL (sh, th, kn), math degrees |
|-----|------------------------------------|
| FR  | 45, −60, −37 |
| FL  | **135**, −60, −40 |
| BR  | −45, −50, −50 |
| BL  | −135, −60, −35 |

All four standing shoulder math-angles now equal their outward image directions
(135 / 45 / −45 / −135). FL's value moved from 75 → 135 in change B; the three corroborating
sites move together:

- `spinal_cord.cpp:23` `NEUTRAL[FL].sh = 135.0f`,
- `convention.json` `"fl": [135, -60, -40]`,
- `config.h:32` `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE = 90` (the *servo* boot default — 90 because
  FL's outward maps to servo 90 once the branch recenters),
- the FL `translateToServo` branch gains the standard `90 + (sh − 135)` recentring (§3).

Arithmetic cross-check: FL standing math 135 → `90 + (135 − 135) = 90` servo, i.e. the boot
default of 90. Consistent.

**The all-90 pose is now the symmetric outward "X" calibration pose.** Driving all 12 servos
to 90 puts every leg flat and pointing outward — a symmetric diagonal X — which is exactly
what change C's flat-calibration command (`relax()`, §8) commands. Before B, FL would have sat
~45° off-symmetric in that pose; that asymmetry is now removed in code.

> Hardware caveat: the running robot's FL horn is still mounted for the old servo-75 standing
> pose. Until it is physically re-mounted so servo 90 = outward (and the FL clips re-exported),
> the all-90 "X" is symmetric *in code only*; on the current hardware FL will sit ~15° (servo)
> off. See "known gaps".

---

## 5. Defaults vs limits — three concepts, three places

Don't conflate these:

- **Boot / neutral defaults** are the `*_DEFAULT_ANGLE` macros in `config.h:23-52`. They are
  *servo* values, used to construct each `Servo` and applied at boot by
  `returnToDefaultAngles`. config.h carries **no range limits** — only neutral defaults and
  PCA channel/pulse constants. (Change B touched exactly one of these: FL hip 75 → 90.)
- **Runtime joint limits** (the IK clamp) live in `kinematics.cpp:28-33`
  (`SHOULDER/HIP/KNEE_LIMIT_LO/HI`, radians) and are applied by the IK clamp. These only bite
  on an IK path; the live gaits and clips do not call IK — they anchor to `NEUTRAL[]` and
  convert directly — so the practical safety clamp for everyday motion is the 0–180
  `constrain` at `servo.cpp:19` / `leg.cpp`.
- **Kinematic limits for the sim/exporter side** are carried by the URDF joint `<limit>`
  tags, used by PyBullet and the Blender rig.

---

## 6. The T:4 wire value, and why it is special

A raw `T:4` (`CMD_CALIBRATE`) packet carries `{id, servo_id, a}` and is the most direct path
to a servo. The handler (`network.cpp:92-111`) validates the indices (`isValidServoIndex`,
`network.cpp:99`; `config.h:73`), looks up `LEG_SERVO_CHANNEL[id][servo_id]`
(`config.h:64-69`), and writes `a` to that PCA channel.

The key fact: **the T:4 path does NOT call `translateToServo`.** No per-leg mirror, no
centring, no scaling. `a` is just "drive this PCA channel to this 0–180 angle." That is
exactly why the exporter bakes `translateToServo` on the host and ships finished servo degrees
— the wire neither knows nor applies any convention.

For a calibration client poking one channel, you can ignore CCW/CW entirely: pick a number and
watch which way the horn turns. If you do want a specific physical CCW nudge, the only fact you
need is the shoulder row of the §3 table — `+1` on `a` is CCW for FR/FL/BL, CW for BR.

Note: T:4 *does* now pass through the invert mirror at the write choke if the robot is inverted
(§7). In practice calibration is done upright, so this is the identity path; it only differs if
you calibrate while inverted.

---

## 7. Robot-invert (`isInverted`): a single pitch-only mirror at the choke

The flip state is `isInverted` (`spinal_cord.cpp:64`), toggled by `invertRobot()`
(`spinal_cord.cpp:467`), reached from the API via `{T:6, a:0}` (`CMD_ACTION_SELECTION` with
`INVERT_ROBOT = 0`, `network.cpp:138-146`). Change D unified what used to be three
inconsistent invert behaviours into one.

- Invert is a **pitch-only mirror**: `thigh' = 180 − thigh; knee' = 180 − knee`, shoulder
  untouched. It lives in `applyInvert` (`motion_math.cpp:14-20`), applied once at the
  `applyServos` choke (`spinal_cord.cpp:113-116`) just before the servo write.
- Because gaits, the clip player, and `stand()` all route their output through `applyServos`,
  they inherit the mirror identically. A flipped robot now runs every motion source the same
  way — including clips and the exporter stream — which is the foundation for a future
  keyframe-able invert toggle in the exporter.
- `invertRobot()` no longer writes a hardcoded one-shot pose; it just toggles the flag and
  re-assumes NEUTRAL through `applyServos`, so the mirror produces the inverted stand.
- Equivalence: for a pitch servo, `translateToServo` emits `90 ± angle`, and
  `180 − (90 ± x) = 90 ∓ x`, which is exactly the old math-space `th = −th; kn = −kn`. So
  routing gait output through the choke is bit-for-bit identical to the previous per-gait
  negation (host-tested in `test_invert_mirror`).

The clip-path EMA smoothing (3b) is applied to the math-space angle *before* `translateToServo`
and the invert, so the two compose cleanly.

All three gait tickers (`tickGait`, `tickTrot`, `tickYawRotation`), the clip player, and
`stand()` route their servo triple through `applyServos` — there is no remaining in-line
`th=-th; kn=-kn`. `invertRobot()` (`{T:6, a:0}`) just toggles the flag and re-assumes NEUTRAL
through the same choke (its old hardcoded inverted-pose table, which still held FL's pre-B
value, was removed).

When the invert flag is off, all motion is byte-identical to before change D. The `{T:6, a:0}`
wire API is unchanged; only its internal effect was unified.

---

## 8. Reference poses & calibration surface

Two reference poses are now reachable over the existing `CMD_STATE` (T:2) command (change C),
alongside the original three states. The full state table (`data.h:17-24`, `network.cpp:77-91`,
`API_SPEC.md` §2):

| `s` | state | effect |
|-----|-------|--------|
| 0 | IDLE | `rest()` |
| 1 | WALK | `walk()` |
| 2 | ACTION | `wallFlip()` |
| 3 | FAILSAFE | no-op fall-through |
| **4** | **REST** | `relax()` — all 12 servos to 90 (flat / outward-X calibration pose); clears `isInverted` |
| **5** | **STAND** | `stand()` — drive legs to `NEUTRAL[]` (standing / neutral) |

Wire shape: `{"T":2,"s":4}` and `{"T":2,"s":5}`. The selector is bounds-checked by
`isValidStateCommand` (`data.h:28-30`), which accepts `0–5`; out-of-range values (6, −1, 99…)
are rejected.

- **`s=4` → `relax()`** (`spinal_cord.cpp:97-107`) drives all 12 servos to 90 and sets
  `STATE_REST`. This is the **flat / all-90 calibration pose**: every servo at mechanical
  mid-travel, you mount the links to match. It explicitly clears `isInverted` (calibration is
  done right-side-up). This restores the firmware's original calibration pose — `relax()`
  predates the standing-NEUTRAL gait engine and was orphaned (dead code) until change C made it
  reachable.
- **`s=5` → `stand()`** (`spinal_cord.cpp:118-125`) drives each leg to `NEUTRAL[]` via
  `translateToServo` + `applyServos` — the same standing pose gaits launch from and ease back
  to, now commandable on demand.

C is purely additive: states `s=0..3` behave exactly as before, nothing auto-calls the new
poses on boot, and a client that never sends `s=4`/`s=5` sees no change. `T:7` clip playback
still auto-eases to the standing NEUTRAL pose at clip end; C just makes that same pose reachable
directly.

`CMD_POSE` / `T:3` is still documented as a chassis body-pose / static-IK command
(`API_SPEC.md` §3) but **has no handler** — a T:3 packet is silently dropped. C deliberately did
not repurpose T:3; the body-pose IK remains future work (see "known gaps").

---

## 9. Clip smoothing

Clip playback is smoothed in two independent ways, both confined to the clip / timed-move paths:

- **Smoothstep easing** (change 3a): the timed-move ease fraction is
  `frac = t·t·(3 − 2t)` (`easeFraction`, `motion_math.cpp:4-8`), used by `Servo::tickEase`
  (`servo.cpp:38-48`). This makes timed moves accelerate and decelerate smoothly, landing
  exactly on target (smoothstep(1) = 1). It affects the 500 ms glide back to the standing pose
  at clip end and any "move to angle over N ms" command. **Gaits never call `tickEase`**, so
  walking/trotting/crab are unaffected.
- **Per-channel EMA** (change 3b): inside `tickClip` only, each math-space angle is blended as
  `smoothed = α·prev + (1−α)·target` (`emaStep`, `motion_math.cpp:10-12`) with
  `CLIP_EMA_ALPHA = 0.75` (`spinal_cord.cpp:17`). The EMA state (`clipSmoothed_`) is seeded from
  the clip's frame 0 on `playClip` so playback starts on the true first pose. The EMA runs on
  the math-space angle, **before** `translateToServo` and the invert mirror, so it composes
  cleanly with both.

Both are **strictly clip/timed-move-only**: gaits, calibration (T:4), and the manual servo path
never touch the smoothing state. The gait input filter
(`activeX/Y/Yaw += (target − active)·0.1f`, `spinal_cord.cpp:144-146`) is a separate, unrelated
mechanism.

---

## 10. Calibration procedure

To calibrate the robot to its conventions:

1. Command **`{"T":2,"s":4}`** (REST / flat). All 12 servos go to 90 (mechanical mid-travel),
   and the invert flag is cleared — so the robot is in its upright, symmetric reference.
2. With the robot upright and invert cleared, mount each leg's links so that at servo 90 the
   leg points **outward and lies flat** (the symmetric outward "X").
3. The **FL shoulder horn must be mounted so servo 90 = outward** (post-change-B). This is the
   one joint whose mounting changed: previously FL's standing pose was servo 75, now it is the
   regular servo-90-outward like the other three.

Calibration correctness only requires a *known* reference pose; all-90 is chosen because every
servo sits at mid-travel. A roughly-mounted robot stays roughly correct.

---

## 11. What an exporter must own vs. can ignore

For someone writing an animation exporter that emits servo degrees over T:4:

**Must own** (the firmware will not do it for you on the T:4 path):

- the **NEUTRAL anchor** — the per-leg standing math-angles servo 90 is built around. After B
  there is no FL special case; all four legs anchor at their outward direction (FL = 135).
- the **2/3 amplitude scale** toward neutral (`scale = 0.6667`, `convention.json:9`, applied in
  `_frame_to_servo` step 1, `fh_clip_panel.py:1900`);
- the **lift / pitch sign convention** (`th += lift; kn -= lift`), so up-is-positive comes out
  right.

**Can ignore** (because `translateToServo` / `_frame_to_servo` absorbs it):

- the **per-leg L/R mirror** and shoulder offsets — the BR shoulder sign-flip and the FR/BL vs
  FL/BR thigh/knee mirror are baked into the per-leg branch;
- the **invert mirror** if the robot may be flipped mid-stream — the firmware's single invert
  transform handles it (the basis for the future keyframe-able invert toggle).

In short: the exporter owns the *pose model* (neutral incl. FL=135, scale, lift sign); the
per-leg *hardware mirror* and the *flip mirror* are handled for it, as long as it uses the same
byte-identical per-leg formulas the firmware does (kept honest by `test_servo_parity.py`).

---

## 12. Still true / unchanged

These predate this round of work and are untouched:

- **Gait math** — `tickGait`, `tickTrot`, `tickYawRotation` patterns, phase offsets, duty
  cycles, the per-leg sweep/lift logic. Invert-off output is bit-for-bit identical to before.
- **The 2/3 amplitude scale** around neutral (`scalePose`, `SCALE = 2/3`).
- **The other three legs (FR, BR, BL)** — their NEUTRAL values, their `translateToServo`
  branches, and their boot defaults are all unchanged. Only FL moved.
- **The absolute-servo-degrees model**, the single electrical chokepoint, the deadman switch,
  and the existing WebSocket wire shape for `T:1/2/4/5/6/7`.

---

## 13. Known gaps / not done

- **Change B hardware step is pending.** The B *code* is committed but **must not be flashed**
  until the FL shoulder horn is physically re-mounted so servo 90 = outward, and the FL-keying
  clips (`.js`/`.h`) are re-exported with the new FL mapping. Until then the live robot still
  expects the old servo-75 FL standing pose.
- **3c — rig shoulder limits deferred.** The Blender rig's shoulder limits were not updated for
  FL's 75→135 neutral move; deferred until B fully lands.
- **T:3 body-pose IK unimplemented.** `CMD_POSE` / `T:3` is documented (`API_SPEC.md` §3) but
  has no firmware handler — T:3 packets are silently dropped. Future work.
- **Exporter keyframe-able invert toggle (change E) is future work.** D laid the firmware
  foundation (centralized invert); the Blender N-panel toggle that emits an invert state change
  mid-clip is design-only this round.
- **Convention image filenames are swapped vs content** (§2): `servo-1-rotation-convention.png`
  shows pitch, `servos-2-3-rotation-convention.png` shows yaw. Cosmetic, not yet fixed.

---

## Sources

- `code/firmware/src/nervous_system/motion_math.cpp:4-8` — `easeFraction` (smoothstep);
  `:10-12` `emaStep`; `:14-20` `applyInvert` (pitch-only `180 − x`); `:22-50` `translateToServo`
  (per-leg branch incl. FL `90 + (sh − 135)`).
- `code/firmware/src/nervous_system/spinal_cord.cpp:17` `CLIP_EMA_ALPHA`; `:21-26` `NEUTRAL[]`
  (FL = 135); `:64` `isInverted`; `:97-107` `relax()` (all-90, clears invert); `:113-116`
  `applyServos` choke; `:118-125` `stand()`; all three gait tickers write via `applyServos`;
  `YAW_COEF` (BR flip) in `tickYawRotation`; `tickClip` (EMA before translate);
  `invertRobot()` (flag-only, re-poses NEUTRAL through the choke).
- `code/firmware/src/nervous_system/servo.cpp:15-26` `setServoAngle` chokepoint;
  `:38-48` `tickEase` (smoothstep).
- `code/firmware/src/nervous_system/spinal_cord.h:18-19` `relax()`/`stand()`; `:55`
  `clipSmoothed_`; `:77` `applyServos`.
- `code/firmware/src/brain/network.cpp:77-91` T:2 handler (s=0..5, incl. REST/STAND);
  `:92-111` T:4 handler + `:99` `isValidServoIndex` guard; `:138-146` T:6 `INVERT_ROBOT`.
- `code/firmware/src/shared/data.h:5-14` `CommandType` (incl. `CMD_POSE=3`); `:17-24`
  `RobotState` (incl. `STATE_REST=4`, `STATE_STAND=5`); `:28-30` `isValidStateCommand` (0..5).
- `code/firmware/src/shared/config.h:13-14` pulse range; `:32` `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE
  = 90`; `:64-69` `LEG_SERVO_CHANNEL`; `:73-75` `isValidServoIndex`.
- `code/firmware/src/nervous_system/kinematics.cpp:28-33`, clamp — IK joint limits.
- `code/API_SPEC.md` §2 (T:2 states incl. 4/5); §3 (T:3 body-pose IK, unimplemented);
  §4 (T:4).
- `animation/addons/fh_clip_panel.py:1879` `_frame_to_servo`; `:1902-1911` per-leg branch
  (FL `90 + (sh − 135)`); `:1900` 2/3 scale.
- `animation/convention.json` — `neutral_joint_deg` (fl = 135), `scale = 0.6667`, channels.
- `code/simulation/docs/img/servo-1-rotation-convention.png` (pitch),
  `servos-2-3-rotation-convention.png` (yaw) — filenames swapped vs content.
