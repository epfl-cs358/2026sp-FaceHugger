# Changelog — PyBullet sim interpreter, servo-convention fixes & verification

Plain-language summary of what changed on the `feat/pybullet-sim-interpreter`
branch, in the same style as `CHANGELOG-2026-05-21-1821.md`. Organised by group
of work, newest understanding first. This branch builds on
`feat/animation-flow-integration` and has `feat/clip-discoverability` merged into
it; `feat/invert-flag` is separate (see "Branch & merge state" at the end).

## PyBullet simulation interpreter (new)

A new `code/simulation/pybullet_interpreter/` package replays the exact same
animation clips the firmware plays, in PyBullet, so clips can be validated on a
laptop before flashing.

- Ported the firmware's `translateToServo` math to Python (`servo_convention.py`)
  as the single source of truth for the math-space → servo → PyBullet-radian
  pipeline, with tests pinning it to the firmware formulas.
- Added a parser for the bundled `clips_all.h` and a clip player that
  interpolates frames and drives the PyBullet joints, mirroring the firmware
  `tickClip()` loop (EMA smoothing deliberately omitted — the sim validates
  geometry, not smoothing).
- Wired `python facehugger.py sim --clip NAME` to play a named clip.
- Fixed the per-leg URDF joint-axis sign for the hip/knee (link2/3) so the
  diagonal legs articulate in the correct direction in the sim.
- Added `--loop` to replay a clip continuously (watch cumulative drift/slip), and
  `--float` to play with no gravity, no floor, and the body pinned — so you see a
  clip's pure joint geometry with no falling, slipping, or collapse.
- Added a live "Servo Angles" read-out and `--clip` plumbing through the CLI.

## Servo convention — shoulder/yaw corrections

Three linked bugs in how the Blender export reached the servos, all fixed so the
exported clips finally match the authored animation:

- **Shoulder anchored wrong (robot collapsed).** The rig's shoulder (link1) is
  driven as a delta from rest (0° at standing), but the export and firmware
  expect an absolute angle equal to NEUTRAL at standing. Reading it as a delta
  compressed every shoulder toward a third of neutral and drove the ±135° legs
  (FL, BL) past the servo clamp, so the robot collapsed lopsided on playback in
  both sim and firmware. Fixed by converting link1 from delta to absolute in the
  exporter (adding the per-leg NEUTRAL from `convention.json`).
- **Yaw direction wasn't uniform.** The canon is "math-space `+sh` = CCW yaw, the
  same direction for all four legs." The rig writes the bone-local Z rotation,
  which carries the per-leg URDF axis sign and made the exported yaw non-uniform.
  Fixed by cancelling the bone-axis sign so the exported math-space is uniform CCW.
- **BR shoulder was mirrored.** BR's shoulder servo was the lone mirrored one
  (slope −1). Since all four shoulder motors are identical with their shafts on
  the same vertical axis, yaw must not flip per leg (only pitch flips, by
  mounting orientation). Un-mirrored BR's shoulder to +1 across firmware,
  exporter, and sim, flipping BR's compensating signs in the two gaits
  (`tickYawRotation` `YAW_COEF`, `tickGait` `fwdDir`) in tandem so the robot's
  motion stays byte-identical (proven: BR servo = `90 − dev·scale` before and
  after). Standing/REST poses unchanged. Firmware native tests
  (`test_motion_math`, `test_clip_parity`) pass.

## Clip export & exporter robustness

- Repaired the headless re-export path (`export_all_clips.py` pointed at the old
  `scripts/` location of the panel) and added the missing `Path` import that
  silently disabled `clips_all.h`'s post-export consistency gate.
- "Export Active/Selected" now also regenerates the bundled `clips_all.h` from
  every clip, so a single-clip export never leaves the firmware bundle stale.
- The bake, scene-range sync, and panel now use a clip's **full span** (the union
  of all five actions), not body_ctrl alone — a clip whose body holds a pose
  while the feet animate (fallingRobot) no longer truncates to one frame.
- "Custom range" is now an honest per-clip section selector: on → export exactly
  that range (trim); off → export the whole clip.
- Foot friction in the sim was dead code (the dynamics loop matched "knee", but
  joints are named `*_link3`); fixed so feet grip (1.5, no bounce) instead of
  sliding during planted-feet moves.

## Blender add-on (N-panel) usability

- Per-clip frame-range sync: switching a clip syncs the scene timeline to that
  clip's range; "Custom range" toggle + Start/End fields + "Apply to Scene", with
  live sync via msgbus.
- Live "Servo Angles" panel showing the 12 exported servo degrees for the current
  pose, red when a joint hits its safe limit.
- Fixed the Export-Selected checkbox not visually refreshing (the toggle updated
  the data but the icon stayed stale) by redrawing every window.
- Made the embedded startup script resilient (no traceback if the panel can't be
  imported at load).

## Clip discoverability (merged from `feat/clip-discoverability`)

- Firmware `T:8 CMD_LIST_CLIPS` returns the clip manifest over WebSocket.
- Mobile app clip-discovery UI: requests the list, renders it, stores it (Zustand
  + Jest tests).
- `check_export_consistency.py`: verifies each clip's `.h` round-trips to its
  `.js`, plus (added here) machine-checks the convention itself — `translateToServo`
  at NEUTRAL puts every shoulder at servo 90 (standing) and all twelve at 90
  (flat) — so the exporter and firmware can never silently drift.

## Independent verification & consistency

- `code/simulation/verify_export_parity.py`: derives each clip frame's servos two
  independent ways — the sim's `translate_to_servo` reading `clips_all.h`, and the
  browser `.js` written by the exporter's separate `_frame_to_servo` — and asserts
  they agree (6/6 clips). Together with `test_clip_parity` (firmware C++ ↔ exporter)
  this closes the loop: **sim == exporter == firmware** on the actual exported data.
- Synced the firmware's `clips_all.h` with the export (it had drifted to 5 stale
  clips) — there is no auto-sync; the export writes `animation/exported_clips/`
  only, so the firmware copy must be updated by hand on each re-export.

## Branch & merge state

- This branch = `feat/animation-flow-integration` + the above, with
  `feat/clip-discoverability` already merged in.
- `feat/invert-flag` is **not** here: it is `animation-flow` + two firmware-only
  commits (`T:9 CMD_SET_INVERT` flag-only; `T:6` no longer re-poses on toggle).
- See `docs/.work/convention-docs/` for the convention spec and the merge plan.
