# Session handoff — servo/control convention documentation

Shared context for the subagents writing the convention docs. **Do not duplicate** the
ground truth + change ledger — those live in `docs/.work/convention-docs/00-orchestration.md`
(read it first). This file is the narrative + pointers.

## How we got here
Multi-step read-only investigation of how a joint angle travels URDF → Blender rig →
exporter → firmware → WebSocket. All evidence is in `docs/.work/convention-chain/`:
- `step-a-urdf.md` … `step-f-roundtrip.md` — per-layer evidence.
- `summary.md` — first pass; its headline verdict "servo 90 = outward DENIED" was later
  **retracted** (see followup). Read it knowing that.
- `followup.md` — corrected verdict with working gaits as ground truth; FL=75 is deliberate.
- `api-consistency.md` — what each layer abstracts vs exposes (the L/R mirror).
- `directions-table.md` — per-leg per-joint d(servo)/d(math_angle) signs; isInverted; FL git.

Then two code fixes already landed on `feat/animation-flow-integration` (separate commits,
do not redo): `b9d2d0d` T:4 bounds guard (G7) + native test; `260476f` exporter servo
clamp+warning (G4/G5) + bake test. Both verified (`pio test -e native` 32/32, parity GREEN,
firmware builds).

## Key decisions reached with Marcus (the project owner)
- Conventions today are real and self-consistent; document the reality first, change minimally.
- Target convention: **servo 90 = flat-spread (outward) for every joint** — already true for
  11/12; only **FL shoulder** is the exception (`servo=sh`, neutral 75 vs outward 135).
- Marcus wants: (1) a wiki doc of current reality; (2) a delta doc + changelog-style plan of
  the agreed changes; (3) final wiki docs after review.
- Agreed changes are enumerated as the **A–E change ledger** in `00-orchestration.md`. Honour
  it exactly, especially the "DO NOT change" list and the FL migration requirements.
- Marcus also wants (design, later) a keyframe-able invert toggle in the Blender exporter.

## What the next agents do
Run the prompts in order, with review gates between:
1. `prompt-1-current-reality-doc.md` → `DRAFT-current-reality-conventions.md`
2. `prompt-2-delta-and-plan.md` → `DRAFT-delta-conventions.md` + `DRAFT-planned-changelog.md`
   + `docs/superpowers/plans/2026-05-23-servo-convention-changes.md`
3. `prompt-3-final-wiki-docs.md` → polished wiki doc (only after approval)

Steps 1–2 are **docs only — no source edits**. Code changes (C/D and B's code parts) happen
later from the plan, via `superpowers:subagent-driven-development` + `test-driven-development`,
each as its own reviewed change. B additionally needs a physical FL recalibration (manual).

## Suggested skills for downstream sessions
- Doc agents: none required; follow project doc style (prose-first).
- Plan execution later: `superpowers:executing-plans` or `subagent-driven-development`, with
  `test-driven-development` and `verification-before-completion`.

## Environment notes
- Native firmware tests: `cd code/firmware && pio test -e native` (32 currently pass).
- Parity: `uv run animation/scripts/test_servo_parity.py` (pure Python, stubs bpy).
- Bake test needs Blender 5.1: `/Applications/Blender-5.1.app/Contents/MacOS/Blender
  --background --factory-startup animation/fh_rigged_latest.blend --python
  animation/scripts/test_bake_independence.py` (use the full path; the `$BLENDER_BIN` inline
  form fails under zsh).
