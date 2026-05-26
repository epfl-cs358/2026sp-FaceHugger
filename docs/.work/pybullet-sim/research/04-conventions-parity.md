# Conventions, Joint/Servo Mapping, and Sim↔Firmware Parity

Scope: the conventions the PyBullet sim operates under and how it stays in
agreement with the URDF (kinematic source of truth) and the firmware
(`translateToServo`). All file:line refs are to this worktree.

> Several places where the **prose docs lag the actual code/URDF** are flagged
> inline with **⚠ DOC vs CODE**. Per the project's standing rule ("trust live
> code over design docs"), the code/URDF values are taken as authoritative.

---

## Conventions reference table (authoritative — from code + URDF)

| Aspect | Value | Source |
|---|---|---|
| World frame | +Y forward, +X right, +Z up | `MERGE_AND_CONVENTION.md:46` |
| Leg naming (Py/URDF/Blender) | `fl`, `fr`, `bl`, `br` | `MERGE_AND_CONVENTION.md:43` |
| Leg naming (firmware) | `fr=0`, `fl=1`, `rr=2 (=br)`, `rl=3 (=bl)` | `servo_convention.py:18-21` |
| URDF joint names | `{leg}_link1_joint` / `_link2_joint` / `_link3_joint` | `facehugger.urdf:68,75,82` |
| link1 = shoulder/yaw | axis Z, `+Z` for FL/BR, `−Z` for FR/BL | `facehugger.urdf:72,164,256,348` |
| link2 = hip/thigh | axis Y, `+Y` for FL/BR, `−Y` for FR/BL | `facehugger.urdf:79,171,263,355` |
| link3 = knee | axis Y, same sign as link2 per leg | `facehugger.urdf:86,178,270,362` |
| URDF θ=0 | = Fusion rest pose for every joint | `MERGE_AND_CONVENTION.md:39,106-122` |
| Shoulder rest (FL/FR/BL/BR) | −45° / +45° / −135° / +135° | `MERGE_AND_CONVENTION.md:18,151-156` |
| Shoulder rest derivation | `FR=−FL`, `BL=−wrap_pi(FL+π)`, `BR=wrap_pi(FL+π)` | `MERGE_AND_CONVENTION.md:20,140-145` |
| servo→radian map | `radians(servo_deg − 90)`; servo 90 = joint neutral = 0 rad | `servo_convention.py:145-158` |
| Clip-clamp ranges | hip [38,142], thigh [30,150], knee [0,180] | `servo_convention.py:123-142` |
| Servo numbering | `servo_id = leg*3 + joint`, **PROPOSAL** | `SERVO_ID_CONVENTION.md:21-38` |

---

## 1. Leg naming and the firmware translation table

Python/URDF/Blender use `fl/fr/bl/br`; the firmware uses `fr/fl/rr/rl` where
"rear" (`rr`,`rl`) means the same legs as "back" (`br`,`bl`). The translation
lives only at the firmware boundary.

`MERGE_AND_CONVENTION.md:184-189`:

```
| Firmware id | Firmware name | URDF id | URDF/Python name |
| 0 | fr | fr | front-right |
| 1 | fl | fl | front-left |
| 2 | rr | br | rear-right / back-right |
| 3 | rl | bl | rear-left / back-left |
```

The sim encodes this same enum in `servo_convention.py:18-30`:

```python
LEG_FR = 0
LEG_FL = 1
LEG_RR = 2  # also called BR in simulation / Blender
LEG_RL = 3  # also called BL in simulation / Blender

LEG_ID_TO_SIM_NAME = {LEG_FR: "fr", LEG_FL: "fl", LEG_RR: "br", LEG_RL: "bl"}
```

The parity tool's `.js` key map (`verify_export_parity.py:53`) is the same
bridge in the other direction: `{"fr": LEG_FR, "fl": LEG_FL, "br": LEG_RR,
"bl": LEG_RL}`.

`MERGE_AND_CONVENTION.md:193-198` warns: PyBullet `getJointInfo` returns
`bl`/`br` byte-for-byte; `joint_name[:2]` yields `bl`/`br`, never `rl`/`rr`.

---

## 2. URDF joint/link naming, axes, and θ=0 = Fusion rest

Each leg has three revolute joints named `{leg}_link{1,2,3}_joint`. link1 =
shoulder (yaw, Z), link2 = hip/thigh (pitch, Y), link3 = knee (pitch, Y).

> **⚠ DOC vs CODE (joint names).** `SIM_PIPELINE.md:62-69` and
> `MERGE_AND_CONVENTION.md:112` still call these `fl_shoulder_joint`,
> `fl_hip_joint`, `fl_knee_joint`. The **actual emitted names are
> `fl_link1_joint` etc.** (renamed in commit `80b8e98`, noted at
> `MERGE_AND_CONVENTION.md:126-127`). Code consumers use the link names:
> `kinematics.py:320` → `joints[f"{leg_id}_link1_joint"]`; the clip player
> writes `f"{urdf_name}_link1_joint"` (`clip_player.py:66-68`).

Representative joint block — FL shoulder (`facehugger.urdf:67-74`):

```xml
<!-- LEG: FL  (side=L, shoulder_rest=-45.0°, limits=[-52.0°, +90.0°]) -->
<joint name="fl_link1_joint" type="revolute">
  <parent link="base_link"/>
  <child link="fl_link1"/>
  <origin xyz="-0.050401 0.043222 0.036844" rpy="0 0 -0.785398"/>
  <axis xyz="0.0 0.0 1.0"/>
  <limit lower="-0.907571" upper="1.570796" effort="2.94" velocity="5.0"/>
</joint>
```

`rpy="0 0 -0.785398"` = −π/4 = **−45°** — the FL shoulder rest baked into the
joint origin, confirming "URDF θ=0 = Fusion rest pose" (the rest is in the
`<origin>`, not in the joint variable). FL hip/knee origins carry `rpy="0 0
0.0"` (`facehugger.urdf:78,85`) — their rest is 0°.

The θ=0 / limit-shift rule (`MERGE_AND_CONVENTION.md:106-122`):

> The Fusion joint's `restValue` … is the URDF's θ=0. … Limits in the URDF are
> **shifted relative to rest**: `[Fusion.min − Fusion.rest, Fusion.max −
> Fusion.rest]`.

> **⚠ DOC vs CODE (shoulder limit).** `MERGE_AND_CONVENTION.md:18,122` claim
> the shoulder ROM is symmetric `[−90°,+90°]` for all legs. The **emitted FL
> limit is `[-0.907571, 1.570796]` = `[−52°, +90°]`** (asymmetric; the URDF
> comment at `facehugger.urdf:67` itself says `limits=[-52.0°, +90.0°]`). The
> shifted-limit *mechanism* is real; the specific `±90°` figure in the doc is
> stale relative to the current CAD export.

---

## 3. Shoulder/yaw rest convention and the BR un-mirror

The authoritative quick-reference (`MERGE_AND_CONVENTION.md:15-22`):

```
SHOULDER (servo 1) — yaw joint, Z axis
- Viewed from above, 0° = pointing right (+X), CCW = positive.
- Rest angles per leg: FL: -45°  FR: +45°  BL: -135°  BR: +135°
- Derived from the FL Fusion export value via:
    FR = -FL,  BL = -wrap_pi(FL + π),  BR = wrap_pi(FL + π)
```

> **⚠ BRIEF vs DOC/CODE (sign of BL/BR derivation).** The research brief
> stated `BL = wrap_pi(FL+π)`, `BR = -wrap_pi(FL+π)`. The repo has the
> **opposite signs**: `BL = -wrap_pi(FL+π)`, `BR = wrap_pi(FL+π)`
> (`MERGE_AND_CONVENTION.md:20` and again at `:144-145`). For `FL=−45°` this
> yields `BL=−135°`, `BR=+135°` (`:151-156`). `MERGE_AND_CONVENTION.md:158-161`
> records that an earlier `_shoulder_rest_for()` had BL/BR swapped and splayed
> the back legs into the front quadrants — the current signs are the post-fix,
> visualizer-verified ones. **Report the repo signs as canonical.**

"math-space +shoulder = CCW uniform across all legs" is implemented as a
single uniform `translateToServo` shoulder formula family (`90 + (sh ±
offset)`), one offset per leg's rest, all **same sign on `sh`**
(`servo_convention.py:99-117`):

```python
if   leg_id == LEG_FR: out.hip = 90.0 + (sh - 45.0)
elif leg_id == LEG_FL: out.hip = 90.0 + (sh - 135.0)
elif leg_id == LEG_RR: out.hip = 90.0 + (sh + 45.0)   # BR un-mirrored
elif leg_id == LEG_RL: out.hip = 90.0 + (sh + 135.0)
```

The BR case carries the key history comment (`servo_convention.py:108-110`,
mirrored in `motion_math.cpp:48-53`):

```python
# BR shoulder un-mirrored (2026-05-25): +sh = +servo like fr/fl/bl
# (identical motor, yaw shaft on the same vertical axis). Byte-identical
# to firmware translateToServo / exporter _frame_to_servo.
out.hip = 90.0 + (sh + 45.0)   # was 90.0 - (sh + 45.0)
```

So the BR servo is **no longer** treated as hardware-mirrored at the
math→servo layer — `+sh` produces `+servo` on every leg uniformly. The
firmware compensates elsewhere (`YAW_COEF[BR]`, gait `fwdDir[BR]`) so gait
output is unchanged.

### URDF axis sign ≠ uniform +Z (a second subtlety)

`MERGE_AND_CONVENTION.md:80-84` and `:262` assert the shoulder axis is uniform
global `+Z`. **The actual URDF disagrees**: FL/BR shoulders are `0 0 1`,
FR/BL shoulders are `0 0 -1` (`facehugger.urdf:72,164` vs `:256,348`). This is
the Phase-H R-pair axis negation (`URDF_PIPELINE.md:250-263`) applied to link1
too. The sim handles it explicitly in the clip player
(`servo_convention.py:38-43`, `clip_player.py:61-68`):

```python
LEG_ID_TO_URDF_AXIS_SIGN = {LEG_FR: -1, LEG_FL: +1, LEG_RR: +1, LEG_RL: -1}
...
targets[f"{urdf_name}_link1_joint"] =  axis * servo_to_radians(servo.hip)
targets[f"{urdf_name}_link2_joint"] =  axis * servo_to_radians(servo.thigh)
targets[f"{urdf_name}_link3_joint"] = -axis * servo_to_radians(servo.knee)
```

> **⚠ DOC vs CODE within one file.** `clip_player.py:49-50` docstring says
> "Shoulder (link1) uses servo_to_radians directly — its axis is +Z …
> consistent across all legs and needs no sign correction." The code two lines
> below (`:61-66`) **does** apply the `axis` factor to link1, with a comment
> (`:62-65`) explaining the docstring is wrong: "without it the sim rendered
> fr/bl shoulder yaw backwards." The applied behavior (axis factor on link1)
> is the correct one.

knee gets `-axis` because link3's URDF axis has the same sign as link2 but the
servo convention negates knee (`90 - kn` on FL/BR; see `:106,113`).

---

## 4. Servo numbering / `servo_mapping.yaml` (PROPOSAL)

`SERVO_ID_CONVENTION.md:1-8` opens with a **STATUS: PROPOSAL** banner — the
numbering is pending firmware `SERVO_CONFIG[]` confirmation; the firmware side
is the single source of truth for servo numbering.

Proposed mnemonic (`SERVO_ID_CONVENTION.md:36-38`): `servo_id = leg_index*3 +
joint_index`, `leg_index ∈ {fl:0, fr:1, bl:2, br:3}`, `joint_index ∈ {link1:0,
link2:1, link3:2}`. So `fl_link1=0 … br_link3=11`.

`servo_mapping.yaml` is the bridge: **keys = URDF link names**, **values =
firmware per-PWM `servo_id`** (`SERVO_ID_CONVENTION.md:67-86`). The proposal
explicitly flags that **its leg order `fl→fr→bl→br` disagrees with the
firmware order `fr→fl→rr→rl`** (`:54-59`) — to be reconciled before any
`.gait` is baked. The `direction: ±1` field is **hardware-calibration-only**;
kinematic L/R mirroring is already absorbed into the rig bone roll, so default
is `direction: 1` everywhere (`SERVO_ID_CONVENTION.md:89-104`).

> **Note:** no `servo_mapping.yaml` exists in this worktree — it is described
> in the doc but not yet committed here (consistent with the "PROPOSAL" /
> "exporter when written" framing). The `.gait` path is future work; the
> *currently exercised* parity path is the clip format (§5), not `.gait`.

---

## 5. `verify_export_parity.py` — what it compares, tolerance, and the chain

This script proves the **sim interprets the Blender clip export identically to
what the firmware/browser receive**. It derives each servo command from
multiple *independent* paths and asserts agreement
(`verify_export_parity.py:2-26`):

- **Path 1** — `clips_all.h` (math-space) → the sim's own
  `servo_convention.translate_to_servo` (an independent Python port of the
  firmware switch).
- **Path 2** — the per-clip `<clip>.js` (servo degrees the browser streams),
  read directly.
- **Path 3** (when present) — per-clip `<clip>.h` cross-check.

Core comparison (`verify_export_parity.py:101-109`):

```python
for i, (frame, js) in enumerate(zip(clip.frames, js_frames)):
    sim = _sim_servo_js_equiv(list(frame.a))
    for leg in _JS_ORDER:
        if sim[leg] != js[leg]:
            errors.append(f"{clip.name} frame {i} {leg}: "
                          f"sim(clips_all.h)={sim[leg]} .js={js[leg]}")
```

`_sim_servo_js_equiv` (`:61-69`) runs `translate_to_servo` per leg and applies
the `.js`'s `[0,180]` clamp via `_clamp_0_180` (`:57-58`) so the two are
compared **like-for-like**. Tolerance is **exact integer equality** after the
shared `[0,180]` clamp (`sim[leg] != js[leg]` on rounded ints). The script also
guards frame counts (`:95-99`). Exit 0 = all agree, 1 = mismatch, 2 = file
error.

> The tighter `clampClipServos` (hip [38,142], thigh [30,150], knee [0,180];
> `servo_convention.py:123-142`) is deliberately **not** applied here — the
> tool compares the *convention* under the looser `.js` clamp; the tighter
> clip-clamp is a separate intentional firmware layer (`:22-26`).

### The chain that guarantees sim == exporter == firmware

`verify_export_parity.py:17-21`:

> Path 1 and Path 2 are produced by SEPARATE implementations of the
> `translateToServo` contract (the sim's `servo_convention.py` vs the
> exporter's `fh_clip_panel._frame_to_servo`); the firmware C++ is locked to
> the exporter separately by `code/firmware/test/test_clip_parity`. So
> agreement here closes the loop: **sim == exporter == firmware**, on the
> actual exported data.

The firmware end of the chain is the Unity test
`code/firmware/test/test_clip_parity/test_clip_parity.cpp:14-25`: it runs the
real firmware `translateToServo` against reference cases generated from the
**exporter's** `_frame_to_servo`, with a **1° tolerance** (`:7-9`) to absorb
int-rounding:

```cpp
static const double TOL = 1.0;   // design's stated round-trip tolerance
...
ServoTriple s = translateToServo(c.leg_id, c.sh, c.th, c.kn);
TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_hip,   s.hip,   m);
TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_thigh, s.thigh, m);
TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TOL, c.exp_knee,  s.knee,  m);
```

References come from `code/firmware/test/gen_clip_parity_reference.py`, which
imports the live exporter `animation/addons/fh_clip_panel.py` (with a bpy stub)
and emits `clip_parity_reference.h`.

Independently, the sim's `servo_convention.translate_to_servo`
(`servo_convention.py:84-120`) is **byte-identical** to the firmware
`translateToServo` (`motion_math.cpp:32-61`) — same per-leg formulas including
the 2026-05-25 BR un-mirror — and its docstring cites the exact firmware lines
(`servo_convention.py:8-12`: `motion_math.cpp:32`, `:10`, `neutral_pose.h:13`).

**Net chain:**
`clips_all.h → sim servo_convention` ≡ `.js (exporter _frame_to_servo)`
[`verify_export_parity.py`, exact-int]; and `exporter _frame_to_servo` ≡
`firmware translateToServo` [`test_clip_parity`, 1°]. Transitively the sim's
PyBullet servo output equals the firmware's physical servo command on every
exported clip frame.

---

## 6. "URDF is the kinematic source of truth; everything else is derived"

The principle and the rejection of a hand-written `kinematics.py`
(`MERGE_AND_CONVENTION.md` §6, `:202-221`). The teammate's `main`-branch
`kinematics.py` hardcoded link lengths and mounts:

| Quantity | THEIRS hardcoded | URDF (re-export) | Source-of-truth |
|---|---:|---:|---|
| `L1` | 0.080 m | ≈0.058 m | URDF |
| `L2` | 0.075 m | ≈0.095 m | URDF |
| `L3` | 0.077 m | ≈0.097 m | URDF |
| FL mount | (−0.040,+0.050,+0.025) | (−0.0579,+0.0462,−0.009) | URDF |

`MERGE_AND_CONVENTION.md:215-221`:

> These don't represent a sign or convention disagreement — theirs was just
> built against a different CAD revision … Using theirs' values would put the
> legs at the wrong physical positions and deliver IK solutions for the wrong
> robot. The URDF is treated as the single source of truth; any consumer that
> needs link lengths or mount positions reads them from there via
> `helpers._load_urdf_joints` or `kinematics.build_config`.

`URDF_PIPELINE.md:383-405` corroborates: hand-writing the URDF was rejected
(brittle, "failed at the bracket-vs-axis 38mm offset"); `fusion2urdf`-style
tools were rejected (can't handle shared meshes / per-leg mirroring); the
chosen design generates URDF from CAD so "moving any construction point in CAD
propagates to the URDF without human intervention." The `kinematics.py`
build reads everything from the URDF + the `LEG ASSEMBLY METADATA` comment
block (`SIM_PIPELINE.md:119-138`), never from constants. The §9 checklist
(`MERGE_AND_CONVENTION.md:273-274`) makes it a rule: "Link lengths and mount
positions are read from the URDF, never hardcoded."

---

## Summary of doc/code disagreements found

1. **Joint names**: docs say `*_shoulder/hip/knee_joint`; URDF + code use
   `*_link1/2/3_joint` (`facehugger.urdf:68`; renamed commit `80b8e98`).
2. **Shoulder ROM**: doc claims symmetric `±90°`; URDF emits `[−52°,+90°]` for
   FL (`facehugger.urdf:67,73`).
3. **Shoulder axis "uniform +Z"**: URDF has `+Z` for FL/BR, `−Z` for FR/BL
   (`facehugger.urdf:72` vs `:256`); the sim applies a per-leg axis sign to
   link1, contradicting `clip_player.py`'s own docstring (`:49-50` vs `:61-66`).
4. **Brief's BL/BR derivation signs** are the opposite of the repo's
   (repo: `BL=−wrap_pi(FL+π)`, `BR=+wrap_pi(FL+π)`).
5. **`servo_mapping.yaml`** is documented but not present in this worktree;
   servo numbering is an explicit PROPOSAL pending firmware confirmation.
