# Changelog — servo convention & smoothing work

Running log of the work landing on `feat/animation-flow-integration`, newest first.
Plain-language, in the style of `CHANGELOG-2026-05-21-1821.md`. Each entry maps to one
commit. Flash status reflects Marcus's policy: C / 3a / 3b are safe to flash now; B is
held until the FL horn is remounted; D comes after B.

## Change E — BR shoulder yaw un-mirrored (commit `1645cb6`) — VERIFY-ON-HARDWARE ⚠️

- The shoulder/yaw convention is now uniform: `+sh` (CCW, right-hand about +Z) → `+servo`
  for **all four** legs. BR's shoulder was the lone mirrored servo (`translateToServo`
  slope −1, `90 − (sh+45)`); it's now `+1` (`90 + (sh+45)`), matching FR/FL/BL.
- **Rationale:** the four shoulder servos are identical motors with output shafts all on
  the same vertical axis — mounting orientation only offsets the zero, it doesn't reverse
  rotation. So yaw must not flip per leg. (Pitch still flips on the {FL,BR}↔{FR,BL}
  diagonal because those servos face opposite ways.)
- **Robot motion is unchanged.** The flip is applied in the three parity-locked
  `translateToServo` twins (firmware / exporter / sim), and BR's compensating math-space
  sign is flipped in tandem in both gaits — `tickYawRotation` `YAW_COEF` `{-1,-1,+1,-1}` →
  uniform `{-1,-1,-1,-1}`, `tickGait` `fwdDir` adds BR to the `+1` group. Proof: BR servo
  `= 90 − dev·scale` before and after. Standing/REST poses unchanged (BR sh −45 → 90).
- **No client change:** webapp/JS send high-level commands or raw `T:4` servo (below the
  convention); the browser `.js` clip player is corrected by re-export.
- **Spec:** DRAFT-delta-conventions.md §6 + the §3 slope table (BR shoulder now +1).
- **Pending:** `pio test -e native` (test_motion_math, test_clip_parity, gait spot-check),
  re-export clips, and a hardware check that turn direction is unchanged and BR shoulder
  jogs the same way as FR via `T:4`.

## Change C — reachable flat & stand reference poses (commit `4c3741a`) — FLASH-NOW ✅

- Made the robot's two reference poses commandable over WebSocket, on the existing
  state command `T:2`:
  - `{"T":2,"s":4}` → **flat / all-servos-90** pose. This is the pose to use when
    physically calibrating: every servo goes to mid-travel, you mount the links to match.
    It also clears the "inverted" flag, since calibration is done right-side-up.
  - `{"T":2,"s":5}` → **standing / neutral** pose (the pose gaits start from and return to).
- The flat pose already existed in the firmware (`relax()`) but was unreachable dead code —
  it was the *original* calibration pose, orphaned when the team switched to standing-neutral
  during the gait-engine rewrite. This restores it and adds a standing counterpart (`stand()`).
- **Backwards compatible:** existing states `s:0–3` behave exactly as before; out-of-range
  values are still ignored; gaits and clip playback are untouched.
- Documented both states in `code/API_SPEC.md`. Added a host test proving the accepted
  state range is `0–5` and that `6 / -1 / 99` are rejected. Firmware builds clean; 35/35
  native tests pass.

## Change 3a — smoothstep easing (commit `9bc1e1d`) — FLASH-NOW ✅

- Timed servo moves now ease in and out (smoothstep) instead of ramping linearly, so the
  robot accelerates and decelerates smoothly rather than jerking at the start and stop of a
  move. It still lands exactly on the target.
- This affects the gentle glide back to the standing pose at the end of a clip, and any
  "move to angle over N ms" command. **Gaits are not affected at all** — they don't use the
  timed-move path — so walking/trotting/crab are unchanged.
- Added a host test covering the curve's shape (slow start, fast finish, exact endpoints).
  Firmware builds clean; 39/39 native tests pass.

## Change 3b — clip-playback smoothing / EMA (commit `5f6ee28`) — FLASH-NOW ✅

- Clip animations are now smoothed with a small exponential moving average so the motion
  between keyframes is less jerky. It's seeded from the clip's first frame so playback
  starts cleanly on the right pose. The smoothing strength is a single tunable constant
  (`CLIP_EMA_ALPHA = 0.75`).
- **Strictly clip-only.** Gaits, calibration, and the manual servo command never touch the
  smoothing state, so walking/trotting and every existing WebSocket command are bit-for-bit
  unchanged. The smoothing happens before the (future) invert, so the two compose cleanly.
- Added a host test (single-step blend, convergence, no overshoot, alpha=0 pass-through).
  Firmware builds clean; 43/43 native tests pass.

## Change B — front-left shoulder regularization (commit `8936b04`) — ⚠️ FLASH HELD

- The front-left shoulder was the one leg whose servo 90 didn't point outward (it stood at 75
  while the other three stand at their outward direction). Regularized it to match the others,
  so the all-90 flat pose is now a clean symmetric "X" with every leg pointing outward.
- Changed firmware and exporter together so they stay byte-identical (firmware FL formula +
  `NEUTRAL[FL]` + FL boot default; exporter `_frame_to_servo` FL branch; `convention.json` FL
  neutral). Also fixed two test/generator scripts that still pointed at the pre-move
  `animation/scripts/` path (the panel moved to `animation/addons/` in `2f627a1`) and
  regenerated the clip-parity reference. Builds clean; 44/44 native tests + servo parity pass.
- **⚠️ DO NOT FLASH until the FL shoulder horn is physically remounted** so servo 90 points
  outward. The code and the hardware step are inseparable. The exported clip `.js`/`.h` files
  are still baked with the old FL mapping and must be re-exported before clips are played.
- What it does NOT change: the other three legs, all pitch joints, the gait math, the 2/3 scale.

## Change D — unified invert, pitch-only (commit `1b847e5`) — FLASH-NOW ✅ (logically after B)

- "Inverted" (robot upside-down) now flips hip and knee for **every** motion source, not just
  gaits. Clips and animations now play identically whether the robot is flipped or not — which
  is what lets you keyframe an invert toggle in the exporter later. The flip is applied in one
  place (the servo write point), as a pitch-only mirror; the shoulder is never flipped.
- **Gaits are bit-for-bit unchanged** — the new servo-space mirror is mathematically identical
  to the old per-gait negation (proven for all four legs by a host test). The old hardcoded
  "inverted pose" table (which still held FL's pre-B value) is gone; flipping now re-assumes the
  neutral pose through the same path. Per-leg flip-axis semantics were intentionally left out of
  scope — invert is exactly "hip and knee upside down."
- The clip smoothing (3b) sits before the invert, so the two compose cleanly. Native test
  test_invert_mirror covers it. Builds clean; 47/47 native tests pass.

---

### Flash order for tomorrow
1. **Now-safe (no hardware change):** C, 3a, 3b, D — additive or behaviour-identical for
   existing commands and gaits.
2. **B:** flash only after the FL horn remount + re-exporting the clips. B is committed but must
   not reach hardware before those two manual steps.

