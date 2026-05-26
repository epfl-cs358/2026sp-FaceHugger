# PyBullet simulation docs — folder index

A technical report of the FaceHugger PyBullet simulation as it stands on
`feat/pybullet-sim-interpreter`: the files involved, the exact PyBullet API used
under the hood, the gait/kinematics and clip-interpreter drivers, the
servo/joint conventions, and the sim↔firmware parity chain.

Companion to `../convention-docs/` — that folder is the servo-*convention* work;
this one is the *simulation* as a whole (engine setup, gaits, the clip
interpreter, parity). The convention material here cites the same canon.

## Wiki-ready

Reader-facing outputs, candidates to promote into the real repo docs / wiki.

- **`REPORT.md`** — the full technical report. Prose-first, with file map,
  per-aspect PyBullet API walkthrough (quoted `file:line` snippets), conventions
  reference table, and the parity guarantee. The §7 "doc-vs-code drift" list is
  the actionable bit for adapting the firmware and updating older docs.
- **`report.html`** — the same report rendered as a self-contained styled page
  (sticky TOC, syntax-highlighted code). Open directly in a browser; no build.

## Evidence

Read-only research the report is built on. One focused study per subsystem,
deeper `file:line` detail than the synthesized report.

- `research/01-engine-setup.md` — connect/load/physics/step loop, the monitor.
- `research/02-gaits-kinematics.md` — IK/FK math, the `GAITS` table, foot trajectories.
- `research/03-interpreter.md` — `pybullet_interpreter/`, the servo-convention engine.
- `research/04-conventions-parity.md` — conventions, joint axes, the parity chain.

## Status

Generated 2026-05-26. The report flags seven places where prose docs lag the
live code/URDF (joint names `link1/2/3` not `shoulder/hip/knee`, asymmetric
shoulder ROM, per-leg shoulder axis sign, the 2026-05-25 BR un-mirror, etc.) —
see `REPORT.md §7`.
