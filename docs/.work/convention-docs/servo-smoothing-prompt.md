# Servo Smoothing Investigation + Implementation Prompt

> Read survey first (Task 1-2), then implement (Task 3).
> One commit per fix. TDD throughout.

---

## Context

The FaceHugger quadruped has a clip animation system. Clip
playback feels jerky between keyframes. The firmware already has
setServoAngleTimed + tickEase (linear lerp, non-blocking) and
clipPoseAt() for keyframe interpolation inside clips.

Convention facts confirmed by evidence gathering:
- servo 90 = neutral_joint_deg = outward/flat calibration pose
- FL neutral = 75 (deliberate gear-tooth alignment offset, being
  fixed in Change B — physical FL horn remount required first)
- BL shoulder at standing pose = servo ~179 (geometric, not a bug)
- link2 limits = ±60°, link3 limits = ±90° (symmetric, already in rig)
- link1 (shoulder) has no rig limits yet — Task 3c is DEFERRED
  until Change B (FL regularization) lands, because FL limits will
  change once FL neutral moves from 75 to 135
- Exporter and firmware are byte-identical (parity test GREEN)
- Working gaits are ground truth for sign conventions

SCOPE FOR THIS SESSION: Tasks 1, 2a-2c (survey), 3a and 3b only.
Task 3c is explicitly deferred — do NOT implement shoulder rig
limits until Change B (FL horn remount + formula change) is done.

Do NOT re-litigate W5/W6/W7 — those are resolved as correct design.
Only real open issues: G7 (handled separately) and W3 (minor).

---

## Task 1 — Read-only firmware survey

No code changes. Report back.

**1a. tickEase() in servo.cpp**
Paste the full implementation. Confirm it is a straight linear
lerp. Mark the exact line that changes to implement smoothstep
t*t*(3-2*t). Confirm servo still reaches target exactly at
t=easeDurMs with smoothstep (smoothstep(1.0) = 1.0 ✓).

**1b. tickClip() angle path**
Paste the exact lines from clipPoseAt() to the final
setServoAngle() call during CLIP_ACT_APPLY_POSE. How many hops?
Where exactly would a per-servo EMA filter slot in without
touching the gait path at all?

**1c. Gait 0.1f filter**
Paste the exact lines. What does it filter — foot target
positions, joint angles, or something else? What is the effect
of increasing coefficient from 0.1f to 0.2f on gait
responsiveness vs smoothness?

---

## Task 2 — Read-only rig/exporter survey

No code changes. Report back.

**2a. BL shoulder headroom at standing**
Using the exporter formulas and standing pose foot target
(-116.954, -113.740), confirm BL shoulder = servo ~179.
Then compute: what bone rotation produces servo 5 (lower stop)?
What produces servo 175 (upper stop)? How many degrees of
practical headroom does BL shoulder have above standing?

**2b. Per-leg shoulder limits for rig constraints**
For each leg (fl, fr, bl, br), compute the bone rotation_euler[2]
values that map to servo 5 and servo 175. Use actual _frame_to_servo
formulas. Account for FL neutral=75 offset.
Report as: leg | lower_bone_rad | upper_bone_rad | servo_at_standing

**2c. Confirm link2/link3 rig limits**
Are LIMIT_ROTATION constraints already set on link2 and link3
bones in urdf_to_blender_rigged.py? Do they match ±60° and ±90°
from the URDF? If yes, link1 is the only gap.

---

## Task 3 — Implement (TDD, one commit per item)

**3a. Smoothstep in tickEase**
Replace the linear lerp fraction in servo.cpp tickEase() with
smoothstep: frac = t * t * (3.0 - 2.0 * t)
The servo must still reach the target exactly at t=easeDurMs.
Update the existing tickEase Unity test to assert:
- midpoint angle is NOT linear midpoint (it overshoots toward
  target in middle, slower at start/end)
- final value equals target exactly
Affects: return-to-neutral tail, any setServoAngleTimed calls.
Does NOT affect gaits (they never call tickEase).
Commit: feat(firmware): smoothstep easing in tickEase

**3b. EMA filter in clip playback path only**
Inside tickClip() during CLIP_ACT_APPLY_POSE, after clipPoseAt()
computes the angle and before setServoAngle() is called, apply
a per-channel exponential moving average:

  smoothed[leg][joint] = alpha * smoothed[leg][joint]
                       + (1 - alpha) * target_angle

where alpha = 0.75 (named constant CLIP_EMA_ALPHA, tunable).
Store smoothed state as a float[4][3] on SpinalCord, reset to
the clip's first frame when playClip() is called.

CRITICAL: this must only run inside tickClip(). It must NOT
affect setServoAngle() directly, gaits, calibration, or T:4
manual commands. Gaits already have the 0.1f input filter.
Backwards compatibility: T:1/T:4/T:5/T:6/T:7 wire behaviour
must be bit-for-bit unchanged.

Note on Change D interaction: when the unified invert (180-servo
at setServoAngle) lands later, the EMA filter will naturally
sit on pre-invert values since it is inside tickClip. This is
correct — smooth the animation angle, then invert at the write
point. No conflict.

Add a native Unity test: constant input for N ticks converges
within tolerance; no overshoot. Confirm gait path is untouched
by reading the test for tickGait output — EMA state must not
appear in that path.
Commit: feat(firmware): EMA smoothing for clip playback (clip path only)

**3c. Rig shoulder rotation limits — DEFERRED**
Do NOT implement this task now. It depends on Change B
(FL shoulder regularization + physical horn remount). Once B
lands, FL neutral moves from 75 to 135, matching the other
three legs. At that point, all four shoulder limit calculations
use the same formula and 3c becomes simpler and correct.

Implementing 3c before B would require redoing it after B
with different FL values. Wait for B.

Flag this task as pending in the plan doc and move on.

---

## Self-check before committing each task

- 3a: existing tickEase test updated and green, pio run clean
- 3b: EMA alpha constant named, not hardcoded; gait test
  unaffected; clip convergence test green
- 3c: headless rig test green; values match Task 2b output
