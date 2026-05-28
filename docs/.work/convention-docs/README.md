# Convention docs — folder index

This folder holds the servo-convention documentation work for
`feat/animation-flow-integration` (changes C, 3a, 3b, B, D). The code for all five is
committed; B's hardware flash is held pending the FL horn remount.

## Wiki-ready

These two are clean, reviewed-style outputs meant for readers, not process artifacts.

- **`CONVENTIONS.md`** — the servo & control conventions as they are now (post C/3a/3b/B/D).
  The candidate to promote into the real repo docs (e.g. `doc/conventions/`) once Marcus has
  reviewed it. Prose-first, with the per-leg direction table and the T:2 state table.
- **`CHANGELOG-convention-work.md`** — per-commit, plain-language changelog of the five
  changes, with flash status for each.

## Working / process

Kept for provenance; not intended for the wiki as-is.

- `00-orchestration.md` — change ledger (A–E), verified ground truth, and the 2026-05-23
  decisions update (D = pitch-only, flashing policy, etc.).
- `DRAFT-current-reality-conventions.md` — the pre-change baseline prose (structural starting
  point for `CONVENTIONS.md`).
- `DRAFT-delta-conventions.md` — the agreed deltas (the would-be reality after B/C/D),
  annotated `CHANGED:`/`NEW:`.
- `DRAFT-planned-changelog.md` — pre-implementation planned changelog draft.
- `prompt-1-current-reality-doc.md` — prompt that produced the reality draft.
- `prompt-2-delta-and-plan.md` — prompt that produced the delta draft + TDD plan.
- `prompt-3-final-wiki-docs.md` — prompt for the final wiki pass.
- `servo-smoothing-prompt.md` — prompt that introduced changes 3a/3b/3c.
- `handoff.md` — session handoff context.

## Evidence

Read-only research the above is built on lives one level up in
`../convention-chain/` (`step-a`..`step-f`, `summary.md`, `followup.md`,
`api-consistency.md`, `directions-table.md`).
