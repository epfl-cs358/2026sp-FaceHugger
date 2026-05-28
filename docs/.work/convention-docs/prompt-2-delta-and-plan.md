# Agent prompt 2 — Delta doc + planned changelog + implementation plan (DOCS ONLY)

You are writing documentation/plan. **Do NOT modify any source code.** You may read code,
read the reality doc from step 1, and write the markdown files named below. No commits.

## Inputs
- `docs/.work/convention-docs/00-orchestration.md` — the **change ledger (A–E)** is the
  source of truth for what changes and what must not. Follow it exactly.
- `docs/.work/convention-docs/DRAFT-current-reality-conventions.md` — the reality baseline
  (from step 1). Your delta doc is a copy of this with the agreed changes made explicit.
- `docs/.work/convention-docs/handoff.md`, evidence in `docs/.work/convention-chain/`.
- `CHANGELOG-2026-05-21-1821.md` — match this plain-language changelog style.

## Deliverable 1 — Delta conventions doc
`docs/.work/convention-docs/DRAFT-delta-conventions.md`. Take the reality doc and produce the
**target** conventions after changes B (FL regularization), C (CMD_POSE wiring), D (centralized
invert). Show it as the *would-be* reality, with each changed passage clearly marked
`CHANGED:` / `NEW:` and a one-line rationale inline. Crucially, every change must preserve the
"do not break the running robot" constraint — call out exactly which existing behaviours are
unaffected.

## Deliverable 2 — Planned changelog
`docs/.work/convention-docs/DRAFT-planned-changelog.md`. Plain-language, grouped like
`CHANGELOG-2026-05-21-1821.md` (sections, short bullets a non-engineer can follow). For each
of B, C, D: what changes, why, and — critically — what it does NOT change. Include a
**migration / safety** sub-section per item. For B, the migration MUST state the physical FL
recalibration + re-bake + parity re-run, and that the formula change must not ship without
the recalibration. Mark each item DOC / ADD / MIGRATE per the ledger.

## Deliverable 3 — Implementation plan (TDD, bite-sized)
`docs/superpowers/plans/2026-05-23-servo-convention-changes.md`, following the
`superpowers:writing-plans` format (header with the agentic-worker note + REQUIRED SUB-SKILL;
exact files; `- [ ]` steps; failing test → run → minimal impl → run → commit). Cover ONLY the
code changes C and D, and B's code+config parts (the physical recalibration is a manual step —
represent it as an explicit checklist item, not a code step). Concretely:

- **Task — C (CMD_POSE):** add `case CMD_POSE` in `network.cpp` parsing `{"T":3,"p":<id>}`,
  bounds-checked; add `SpinalCord::stand()` (drive legs to `NEUTRAL[]`) and reuse `relax()`
  for all-90; document T:3 in `code/API_SPEC.md`. Tests: a native/host test for the pose-id
  bounds (mirror the `test_servo_index_bounds` pattern) and that an out-of-range `p` is a
  no-op. Keep gaits/clips untouched.
- **Task — B (FL formula+config):** change FL shoulder branch in `motion_math.cpp` AND
  `_frame_to_servo` to `90 + (sh − 135)`; update `convention.json` fl shoulder neutral →135,
  `NEUTRAL[FL].sh`→135, `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE`→90. Tests: extend
  `test_servo_parity.py` so FL at outward 135 → servo 90 (and stays byte-identical to firmware);
  add a check that all four shoulders map outward→90. Explicit manual checklist item:
  recalibrate FL horn, re-bake FL clips, re-flash, verify on hardware. Note re-bake will change
  exported clip files.
- **Task — D (centralize invert):** introduce a single invert transform `servo'=180−servo` at
  the PCA-write choke point (`Servo::setServoAngle` or wrapper), gated by the firmware invert
  flag; remove the per-gait `th=-th; kn=-kn`; reconcile/retire hardcoded `invertRobot()`. Add a
  decision note for the flip-axis semantics (per-channel mirror vs L↔R role swap) — if undecided,
  the plan's first step is an `AskUserQuestion`/decision checkpoint, NOT code. Tests: host test
  that with invert on, a known servo command reflects about 90 for every joint; that gait output
  with invert matches the OLD behaviour for thigh/knee (no regression); that clips + a simulated
  T:4 now also invert. Prereq note: depends on B.

Each task must list which existing tests must still pass (`pio test -e native`, the parity test,
`test_bake_independence.py`) as a regression gate.

## Output
Write the three files above. Run the writing-plans self-review (spec coverage, no placeholders,
type/name consistency) before finishing. Return a concise summary: the change list with
DOC/ADD/MIGRATE tags, and the single most important risk for each of B/C/D.
