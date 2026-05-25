# Merge plan — consolidating onto `feat/animation-flow-integration`

How the open branches relate and a concrete order for landing them on
`feat/animation-flow-integration` (the integration branch).

## Branch topology (fork point `bdddcbf`)

| branch | = animation-flow + … | in animation-flow already? |
|--------|----------------------|----------------------------|
| `feat/pybullet-sim-interpreter` (this) | 30 commits: pybullet interpreter, servo-convention fixes (link1, FL, uniform yaw, **BR un-mirror**), exporter/UX fixes, **clip-discoverability merged in** | no |
| `feat/clip-discoverability` | 4 commits: `T:8` firmware, clip-list app UI, `check_export_consistency`, flash docs | **already inside this branch** (merge `ef48679`) |
| `feat/invert-flag` | 2 commits: `T:9 CMD_SET_INVERT` (flag-only), `T:6` no-repose-on-toggle | no |

So only **two** things still need to reach `animation-flow`: this branch (which
carries clip-discoverability) and `invert-flag`.

## Recommended order

1. **Merge `feat/pybullet-sim-interpreter` → `feat/animation-flow-integration`.**
   This brings the interpreter, all the convention fixes, the exporter/UX work,
   and clip-discoverability in one go. Expect few/no conflicts: most of this
   branch is new files (`pybullet_interpreter/`, verifier, docs) or edits to
   files animation-flow hasn't touched since the fork.

2. **Merge `feat/invert-flag` → `feat/animation-flow-integration`.**
   `invert-flag` edits the same firmware files this branch did, so resolve
   conflicts in:
   - `code/firmware/src/shared/data.h` — state enum (this branch's clip work vs
     invert's additions): keep both.
   - `code/firmware/src/brain/network.cpp` — command dispatch: keep both `T:8`
     and `T:9` case arms.
   - `code/firmware/src/nervous_system/spinal_cord.{cpp,h}` — different functions
     (BR `YAW_COEF`/`fwdDir` here vs invert's `T:6` re-pose change); keep both.
   - `code/API_SPEC.md`, `code/firmware/platformio.ini` — additive; keep both.

## Pre-merge / post-merge gates

Run before declaring the integration branch good:

- `pio test -e native` — firmware unit + parity tests (`test_motion_math`,
  `test_clip_parity`, gait checks).
- `cd code/simulation && uv run --with pytest python -m pytest pybullet_interpreter/tests/`
  and `python verify_export_parity.py` — sim ↔ export parity, 6/6.
- `cd animation/scripts && python -m pytest test_servo_parity.py test_check_export_consistency.py`
  and `python check_export_consistency.py` — exporter ↔ firmware convention + clip consistency.
- After any re-export: **manually sync** `code/firmware/src/nervous_system/clips_all.h`
  from `animation/exported_clips/clips_all.h` (no auto-sync exists).

## Open follow-ups (not blockers)

- Hardware: confirm BR shoulder turn direction once the multiplexer is repaired
  (the un-mirror was decided from the hardware, not yet eyeballed).
- `facehugger_config.yaml` `servo.mass_kg` is mid-experiment (0.060 → 0.300) —
  settle it before merge; the comment still says "spec".
- Consider auto-syncing the firmware `clips_all.h` from the export to remove the
  manual copy step.
