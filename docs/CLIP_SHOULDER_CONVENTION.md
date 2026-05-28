# The link1 (shoulder) delta-to-absolute fix

## Symptom

Playing an exported clip — `python facehugger.py sim --clip "tiny wiggle"`, or
the same clip on real hardware — made the robot do a small wiggle and then
collapse into a lopsided, frozen pose. It never settled back into the standing
pose the animation was authored around. The collapse reproduced identically in
PyBullet and on the physical robot, which was the first clue: the simulator was
faithfully replaying what the firmware would do, so the defect had to live
upstream of both, in the exported clip data.

## Two reference poses (do not conflate them)

Everything here turns on telling two poses apart:

- **Flat pose** (calibration, `T:2 s:4`): all twelve servos at 90°. The legs
  spread straight out horizontally — hips and knees straight, shoulders pointing
  outward. In math space this is shoulder = the per-leg neutral splay
  (±45° / ±135°), hip = 0, knee = 0.
- **Standing / neutral pose** (`T:2 s:5`, = firmware `NEUTRAL[]` =
  `convention.json`'s `neutral_joint_deg`): a crouched stand. The **shoulders
  are still at servo 90** — they do not move between flat and standing — but the
  hips and knees bend to their standing values (e.g. FR thigh 150, knee 53), so
  those servos are **not** 90.

So "servo 90 = shoulder outward" holds in both poses; only hip and knee differ.

## Root cause

The Blender rig drives the three joints of each leg differently:

- **link2 / link3 (hip, knee)** are solved by the IK constraint stack. The
  exporter's `_read_bone_angles` recovers their *absolute* joint angle.
- **link1 (shoulder/yaw)** is driven by an analytic yaw driver
  (`add_link1_yaw_drivers` in `urdf_to_blender_rigged.py`) whose expression
  subtracts the rest yaw. It therefore outputs a **delta from the flat/rest
  pose — 0° at rest** — not an absolute angle.

The export pipeline downstream (`_scale_from_neutral` and `_frame_to_servo`)
and the firmware `translateToServo` both assume every raw joint angle is
*absolute* and equals its `NEUTRAL` value at the standing pose, so that
`translateToServo(NEUTRAL)` puts each shoulder at servo 90. Feeding link1 in as
a delta (≈0) instead of its neutral broke that assumption. The scale-from-neutral
step then compressed each shoulder toward `NEUTRAL/3`:

- the ±45° legs (fr, br) landed ~30° off neutral (servo ≈ 60);
- the ±135° legs (fl, bl) were pushed past the servo clamp `[38, 142]` —
  servo 0 / 180 before clamping — pinning those shoulders to their mechanical
  stops at the standing pose.

Two shoulders jammed at their limits at "standing" is what tipped the robot.

## The fix

`fh_clip_panel.py` `_link1_delta_to_absolute()` adds the per-leg shoulder
neutral (degrees, from `convention.json` — never hardcoded) to each `*_link1`
angle as it comes off the bake, converting link1 from delta to absolute so it
matches link2/link3. Shoulders only — adding it to hip/knee would double-count.
It is applied once in `bake_clip`, so both export paths (`clips_all.h` via
`_scale_from_neutral`, and the live `.js` wire via `_frame_to_servo`) are fixed
from a single point.

The fix is deliberately in the exporter, not the rig: it is pure Python,
unit-testable, keeps `convention.json` as the single source of truth, and avoids
rebuilding the binary `.blend` and re-stashing every authored clip.

## Why the existing consistency check didn't catch it

`check_export_consistency.py` originally verified that each clip's `.h` bone
angles round-trip to its `.js` servo values. Both files are generated from the
same bake, so they agreed with each other *and were wrong together*. The bug was
invisible to a `.h`↔`.js` comparison.

The added `check_standing_neutral` / `check_flat_pose` assertions catch it
because they check against `convention.json` directly rather than against another
generated artifact:

- **standing**: `translateToServo(NEUTRAL)` → every shoulder = 90, every
  hip/knee = its `NEUTRAL[]` value and explicitly **not** 90;
- **flat** (scale bypassed, since the calibration pose is not a scaled clip
  frame): all twelve servos = 90.

If a shoulder neutral ever drifts so that `translateToServo` ≠ 90 — as happens
when the rig and `convention.json` disagree — the check fails loudly. This is the
machine-checkable form of the "servo 90 = outward for all four legs" guarantee.

## How to verify after re-exporting

The fix only changes newly baked clips; the clips committed on disk are stale
until re-exported in Blender:

```
BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
  "$BLENDER_BIN" --background --factory-startup \
  animation/fh_rigged_latest.blend \
  --python animation/scripts/export_all_clips.py
```

Watch the export log: there should be **zero** `WARNING: ... servo N out of
range` lines for shoulders. Then:

1. `cd animation/scripts && python check_export_consistency.py` — convention
   passes and all clips pass.
2. Frame 0 of a standing-start clip ("tiny wiggle", "lie down and stand up")
   maps to shoulder servos near 90 (math ≈ 45 / 135 / −45 / −135), not the old
   clamped 38 / 142.
3. `cd code/simulation && python facehugger.py sim --clip "tiny wiggle"` — the
   robot stands and wiggles instead of collapsing.

## Note: the "fl ~31° residual" was stale data, not a rig fault

An earlier draft of this doc flagged a suspected fl-specific ~31° baseline
residual (servo ≈ 69 at frame 0). That was an artifact of reading the
*pre-re-export* `clips_all.h` (baked under the old pre-Change-B fl convention).
After re-exporting with the delta-to-absolute fix, fl frame 0 reads raw ≈ −1.06
like the other three legs (the rig log confirms all `link1 yaw-only = 0.000°` at
rest), and fl frame-0 shoulder lands at servo ≈ 89 — clean, no residual. The
real fl-specific defect was the yaw *direction* inversion documented below, not
a baseline offset.

## Follow-up: link1 yaw exported a non-uniform math-space (axis_sign)

After the delta-to-absolute fix above, a body yaw (`body_ctrl` rotation) made the
shoulder servos disagree across legs. The canon (`DRAFT-delta-conventions.md` §2
and the yaw convention PNG) is: **math-space `+sh` = CCW yaw, the same direction
for all four legs** ("unit circle, same rotation" — the exact ±45/±135 angles
are illustrative; the *direction* is the convention, with flat = servo 90).

The rig's link1 yaw driver writes the **bone-local Z** rotation, i.e. the true
CCW foot-yaw delta multiplied by the bone axis sign (URDF `*_link1_joint`
`<axis z>`: fr −1, fl +1, br +1, bl −1). Reading that straight into math-space
makes the exported `sh` **non-uniform** across legs — the bug.

Fix (exporter, not the rig): `_link1_delta_to_absolute` multiplies each link1
delta by `_LINK1_DELTA_SIGN = {fr:−1, fl:+1, br:+1, bl:−1}` — **the axis sign
itself** — cancelling the bone-local sign so the exported math-space `sh` is
uniform CCW for all legs, matching the canon. Regression test:
`test_body_rotation_uniform_mathspace_yaw`.

### Why BR's *servo* still moves opposite (and that's correct)

With uniform math-space, `translateToServo` maps it to servos using the §3 slope
table — where **BR's shoulder is hardware-mirrored (slope −1)**. So for a body
yaw, FR/FL/BL servos move together and **BR's servo moves opposite**. That is the
documented convention, not a bug.

A natural expectation is "all four servos move the same physical direction." That
holds only if BR's shoulder is *un-mirrored* (slope +1) — a hardware question
(is BR's shoulder servo physically mirrored, or just rotated 180° about its yaw
axis?). Resolving it "all same" would mean changing BR's `translateToServo` slope
in **both** firmware (`motion_math.cpp`) and exporter (`_frame_to_servo`),
re-verifying `test_servo_parity.py`, **and** a physical BR remount — analogous to
Change B for FL. Until tested on hardware, BR's mirror stays as documented.

### Consistency

This fix is **export-only** — it corrects the rig→math-space layer the firmware
doesn't have. It does **not** touch `translateToServo`, which is byte-identical
between firmware and exporter and locked by `test_servo_parity.py`. So clip
playback stays consistent across exporter → `clips_all.h` → firmware/sim. The
firmware turn *gait* (`tickYawRotation`, with its own `YAW_COEF`) is a separate,
stepping motion — untouched here; verify its on-hardware turn direction
separately. Re-export clips for the fix to reach `clips_all.h`.
