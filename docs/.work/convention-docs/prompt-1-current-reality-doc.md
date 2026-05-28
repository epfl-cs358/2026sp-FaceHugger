# Agent prompt 1 — Current-reality conventions doc (DOCS ONLY)

You are writing documentation. **Do NOT modify any source code** (no .cpp/.h/.py/.json/.urdf).
You may only read code and write the ONE markdown draft named below. No commits.

## Inputs you are given
- `docs/.work/convention-docs/00-orchestration.md` — verified ground truth + change ledger.
- `docs/.work/convention-docs/handoff.md` — session context.
- Evidence (read as needed, treat as authoritative): `docs/.work/convention-chain/`
  step-a-urdf.md, step-b-blender.md, step-c-exporter.md, step-d-firmware.md,
  step-e-websocket.md, api-consistency.md, directions-table.md, followup.md.
- Convention images: `code/simulation/docs/img/servo-1-rotation-convention.png`,
  `servos-2-3-rotation-convention.png`.

Before writing, **re-verify every concrete claim against the live code** (cite `file:line`).
Trust code over the evidence docs if they ever disagree, and note the disagreement.

## What to write
A single doc describing **how the FaceHugger control/servo conventions work TODAY** — the
reality, not the ideal. Audience: a teammate or future-you wiring an animation exporter or
an API client who needs to know what +/- means for each joint and what the numbers on the
wire mean. Wiki-bound, so write in **clear, concise prose** (project doc style: prose-like,
short, not a wall of tables — but a couple of small reference tables are fine where they
genuinely help, e.g. the per-joint direction signs).

Cover, grounded in code:
1. **The control model:** every motion source sends absolute servo degrees (0–180) per
   frame; no deltas. Gait/clip/exporter all converge on `setServoAngle`. Cite the path.
2. **The global zero & rotation directions** (from the images, which are canon for
   *direction* only): shoulder yaw = CCW-from-above on a top-down unit circle, front-at-top;
   thigh/knee pitch = legs-flat is 0, up = +, down = −. Note the two image filenames are
   swapped vs their content.
3. **Math-space → servo: `translateToServo` is the single abstraction.** Explain that the
   caller works in a *uniform* per-leg-agnostic math space (+ = CCW yaw / up pitch for every
   leg) and that translateToServo absorbs all hardware mirroring (BR shoulder sign-flip, the
   FR/BL vs FL/BR thigh/knee L/R mirror, the 90° centring). Include the per-leg direction-sign
   table (d(servo)/d(math_angle)) from directions-table.md, verified against `motion_math.cpp`.
4. **What servo 90 means:** each leg's NEUTRAL = standing pose, not a universal outward.
   Give the per-leg neutral math-angles and state plainly that FR/BR/BL stand at their
   outward direction but **FL stands at math 75 vs its outward 135** — the one asymmetry —
   and that it's deliberate & self-consistent (cite config.h, NEUTRAL[], convention.json,
   commit e005f5a). Explain *why it's invisible in practice today*.
5. **Defaults vs limits:** config.h `*_DEFAULT_ANGLE` = boot/neutral servo values
   (`spinal_cord.cpp:37-54`); runtime joint *limits* live in `kinematics.cpp`; the URDF
   carries kinematic limits for the sim/exporter. config.h has no range limits.
6. **Robot-invert (`isInverted`) as it is today:** partial & source-inconsistent — gaits
   flip thigh/knee only (math-space, pre-translateToServo); `tickClip` and the T:4 handler
   ignore it; `invertRobot()` is a separate hardcoded `180−servo` pose. State this honestly.
7. **Pose/calibration surface today:** `relax()` = all-90 flat pose but dead code; T:3
   `CMD_POSE` unimplemented; `STATE_REST=4` unreachable; T:4 = single-servo calibrate; T:7 =
   play clip. So there is currently no API command to assume a named pose.
8. **What an exporter must own vs can ignore** (from api-consistency.md): owns NEUTRAL anchor
   + 2/3 scale + the lift convention; can ignore the L/R mirror (translateToServo absorbs it).

Do NOT describe the proposed changes here — this doc is the baseline. (The deltas go in the
next doc.) If something is genuinely broken/inconsistent today, describe it as it is and tag
it `(known gap)` without prescribing the fix.

## Output
Write to `docs/.work/convention-docs/DRAFT-current-reality-conventions.md`. End with a short
"Sources" list of the key `file:line` references. Return a concise summary + any place where
the live code contradicted the evidence docs.
