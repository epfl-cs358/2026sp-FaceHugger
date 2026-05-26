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
- **`REORG-PLAN.md`** — executable migration plan to split `code/simulation/`
  into `pybullet_sim/` (runtime) + `urdf_gen/` (build). Proposal, not yet run.

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

### Changes since the report snapshot (2026-05-26, same day)

`REPORT.md` is a snapshot; the code moved slightly after it was written:

- **Servo corrected to QYRC DSS-230MG** (30 kg·cm → 2.94 N·m, ~2 A). The
  monitor stall model and `facehugger_config.yaml` `effort_nm` now both use
  2.94, matching the URDF's `effort="2.94"` (no drift). `mass_kg: 0.300` is an
  unverified placeholder.
- **`SimLogger` + `--log` flag** added to `sim_monitor.py` / `simulate.py` —
  per-step torque/current capture → summary + `sim_log.csv` + `sim_log.png`.
  Additive; `--monitor` unchanged.
- **Dead files removed**: `teleop.py`, `terrain.py` (stubs) and `view_urdf.py`
  (the `view` subcommand is gone — use `sim`). The report's §1 file-map still
  lists these; treat them as deleted.
- **`clip_player.py` docstring fixed** — it now correctly states link1 gets the
  per-leg axis sign too (REPORT.md §7 item 4 is resolved).
