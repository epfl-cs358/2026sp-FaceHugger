# TODO — landing the three branches onto `feat/animation-flow-integration`

Actionable checklist. Detail + rationale: `docs/.work/convention-docs/MERGE-PLAN.md`
and `SIMULATION-PARITY.md`. Status as of 2026-05-25.

## Branch state

- **`feat/pybullet-sim-interpreter`** (current) — DONE, not yet merged. Carries:
  the PyBullet interpreter, the servo-convention fixes (link1 delta→absolute,
  uniform yaw, **BR shoulder un-mirror**), exporter + add-on UX fixes, the
  `--float`/`--loop`/`--monitor` sim flags, physics realism, the verification
  tooling — and **`feat/clip-discoverability` is already merged in**.
- **`feat/clip-discoverability`** — fully contained in the branch above. Nothing
  separate to do.
- **`feat/invert-flag`** — NOT merged anywhere yet. = `animation-flow` + 2
  firmware commits: `T:9 CMD_SET_INVERT` (flag-only) and `T:6` no-repose-on-toggle.

## Before merging (gates — all currently green except hardware)

- [ ] `cd code/simulation && uv run --with pytest python -m pytest` (interpreter +
      sim_monitor) and `uv run python verify_export_parity.py` → 6/6.
- [ ] `cd animation/scripts && uv run --with pytest python -m pytest test_servo_parity.py test_check_export_consistency.py` and `python check_export_consistency.py` → all pass.
- [ ] `pio test -e native` (firmware: `test_motion_math`, `test_clip_parity`, gaits).
- [ ] Settle `code/simulation/facehugger_config.yaml` `servo.mass_kg` (mid-experiment
      at 0.300; was 0.060 "spec") — decide the value, fix the comment.
- [ ] Commit the rig: `animation/fh_rigged_latest.blend` (LFS) — currently
      uncommitted; the exported clips are committed, the source .blend is not.

## Merge steps

- [ ] **1. Merge this branch → `feat/animation-flow-integration`.** Brings the
      interpreter + convention fixes + clip-discoverability. Expect few conflicts
      (mostly new files / files animation-flow hasn't touched since the fork).
- [ ] **2. Merge `feat/invert-flag` → `feat/animation-flow-integration`.** Resolve
      conflicts (keep BOTH sides — different command arms / enum values / functions):
      - `code/firmware/src/shared/data.h` — state enum.
      - `code/firmware/src/brain/network.cpp` — dispatch (`T:8` + `T:9`).
      - `code/firmware/src/nervous_system/spinal_cord.{cpp,h}` — BR `YAW_COEF`/`fwdDir`
        (this branch) vs `T:6` re-pose change (invert-flag).
      - `code/API_SPEC.md`, `code/firmware/platformio.ini` — additive.
- [ ] **3. Re-run the gates** on the merged `animation-flow`.

## After re-exporting clips (every time — no auto-sync!)

- [ ] `cp animation/exported_clips/clips_all.h code/firmware/src/nervous_system/clips_all.h`
- [ ] `pio test -e native` + `python verify_export_parity.py`.

## Open follow-ups (not merge blockers)

- [ ] **Hardware (when multiplexer fixed):** confirm BR shoulder yaw direction
      (`T:4` jog BR vs FR) and that turn direction is unchanged — the BR un-mirror
      was decided from the hardware but not yet eyeballed.
- [ ] Automate the firmware `clips_all.h` sync from the export (remove the manual `cp`).
- [ ] Consider whether the wave/fallingRobot fast knee jumps (>30°/frame warnings)
      need smoothing for hardware.
- [ ] Wiki: publish `CONVENTIONS.md`, `DRAFT-delta-conventions.md` §6,
      `SIMULATION-PARITY.md`, and the two changelogs.
