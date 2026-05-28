# Agent prompt 3 — Promote drafts to final wiki docs (DOCS ONLY)

RUN ONLY AFTER Marcus has reviewed and approved the step-1 and step-2 drafts.
**Do NOT modify source code.** You may read code + the approved drafts and write the final
doc(s). No commits unless explicitly told.

## Inputs
- Approved: `docs/.work/convention-docs/DRAFT-current-reality-conventions.md`,
  `DRAFT-delta-conventions.md`, `DRAFT-planned-changelog.md`, and any review notes Marcus
  left (he will point you to them or paste edits).
- `00-orchestration.md` for the ledger; `handoff.md` for context.

## Task
Produce the polished, wiki-ready conventions document(s) from the approved drafts. Default
home (confirm with Marcus if unsure): a single `doc/conventions/servo-and-control-conventions.md`
(create the folder), cross-linked from `CLAUDE.md`'s "Where the architecture decisions live"
list and from `code/simulation/docs/`. Keep it prose-first per project doc style.

Structure:
- The **current reality** as the main body (the authoritative convention).
- A clearly separated **"Planned changes"** appendix summarising the deltas (B/C/D) with
  status tags, linking the implementation plan in `docs/superpowers/plans/`.
- Embed/reference the two convention images; add a one-line correction that their filenames
  are swapped vs content (or, if Marcus renamed them, drop the note).

Do not invent new conventions — only what the approved drafts contain, verified against code.

## Output
The final doc(s) at the agreed path. Return a summary listing every file created/moved and the
exact cross-links added, so Marcus can review before anything is committed.
