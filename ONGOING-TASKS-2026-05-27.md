# Ongoing tasks — 2026-05-27

Open threads at wrap-up, each with the prompt that started it. Completed work for
the day is in `CHANGELOG-2026-05-27.md`.

## 1. Standstill servo overload (back-left thigh) — IN PROGRESS, diagnosed, not fixed

> "I think I need to lower friction because at a standing still some links are red, which shouldn't happen"

**It is not friction** (measured: the red is identical at lateral 2.5/1.2/0.6 and
spinning 0.3/0.05/0.02). Root cause: the SIL holds the firmware `NEUTRAL[]` pose,
which renders into the URDF ~20° different at every thigh/knee than the sim's
configured stance (`facehugger_config.yaml`: thigh −40°/knee −60° vs firmware
`NEUTRAL` thigh −60°/knee −37°). In the firmware-neutral pose the **back-left
thigh (`bl_link2`)** sustains stall torque (2.94 N·m → red) at standstill, while
the Python `run_stand` (configured stance) is green (0.24 N·m). So the firmware
neutral and the sim stance are two different poses that should be one.

**Decision needed:** which is canonical (firmware `NEUTRAL` is what the real robot
holds), then reconcile — likely update the sim stance / `body_height` to the
firmware neutral, and/or check body mass (0.8 kg) and the `bl` asymmetry. Not done.

## 2. Final friction value — OPEN decision

> "bump the friction yes, and also for resisting the foot in place. indeed we have rubber at the base."  → later → "I think I need to lower friction…"

Currently `FOOT_LATERAL_FRICTION=2.5`, `FOOT_SPINNING_FRICTION=0.3` (commit
`47e4736`). Lowering it does **not** fix the standstill red (that's task 1). Pick a
final value once the standstill pose is sorted.

## 3. Analog / diagonal joystick — PROPOSED, not started

From the sideways-trot work: `tickTrot` now handles X and Y independently, but the
move protocol still sends discrete directions (FW/R/L…) one axis at a time.
Sending analog `x`/`y` from the joystick would unlock true diagonal trot.

## 4. Clip-based walk — PLANNED, deferred

> "how hard would it be for me to make a new walk based on clips? maybe a simple extra pannel in the blender where I map a clip to forward/back, left/right and diagonals, with a toggle to mirror them. just plan"

Plan delivered; noted in the wiki roadmap (`reference/roadmap.md`, branch
`feat/wiki-setup`) as proposed/not-started. The one genuinely new piece is a
sagittal (L↔R) clip-mirror function. Dropped for now in favour of procedural
trot/crab.

## 5. Push + update the PR — PENDING (needs you; hook blocks the agent)

> "Make sure to document a changelog for today, so we can update the PR also for everything"

`CHANGELOG-2026-05-27.md` is ready for the PR body. Unpushed: `feat/animation-flow-integration` (the day's commits) and one commit on `feat/wiki-setup`.

## 6. Hardware tuning pass — PENDING real robot

`LATERAL_STEP` (35° sideways amplitude), the smoothing default (0.75), and the
friction values are all guesses to dial in during bring-up.

## Note

`.claude/settings.json` is intentionally left uncommitted (local config).
