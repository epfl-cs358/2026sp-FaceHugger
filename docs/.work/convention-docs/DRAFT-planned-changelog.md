# Planned changelog — servo & control convention changes (B, C, D)

Plain-language summary of three agreed convention changes, written in the same style as
`CHANGELOG-2026-05-21-1821.md`. **This is a plan, not landed work** — it describes what each
change will do, why, and (most importantly) what it will *not* change. Each item is tagged:

- **DOC** — documentation only, zero risk.
- **ADD** — additive, low risk; existing behaviour unaffected.
- **MIGRATE** — changes how the robot moves; needs migration steps + (for B) hardware work.

The overriding rule across all three: **the way the robot works today — gaits, clips,
calibration — must not break.**

---

## C — Reachable "flat" and "stand" reference poses  (ADD)

### What changes
- Added two on-demand poses you can command over the existing state channel:
  - `{"T":2,"s":4}` → **flat / all-90** pose: drives all 12 servos to 90°. This is the
    "safe to power off" / calibration pose. (The firmware already had this as `relax()`, but
    nothing could reach it — now it has a command.)
  - `{"T":2,"s":5}` → **stand / neutral** pose: drives the legs to their standing NEUTRAL
    angles on demand (today the robot only reaches that pose by easing back at the end of a
    gait or clip).
- Documented both new state values in the API spec's state table.

### Why
- Gives the animation exporter and the mobile app an explicit "go flat for calibration" and
  "return to standing" command, instead of having to nudge a gait and wait for it to ease
  back. Useful before/after running clips and during servo calibration.

### What it does NOT change
- No existing command or state changes: IDLE/WALK/ACTION and `T:1`/`T:4`/`T:5`/`T:6`/`T:7`
  behave exactly as before.
- Nothing auto-calls the new poses — not on boot — so a robot that never sends `s=4`/`s=5`
  is bit-for-bit unchanged.
- `T:3` (the documented-but-unimplemented "body pose / static IK" command) is **left
  untouched**. We deliberately did not repurpose it; it stays a known gap for future IK work.
- The clip player still auto-eases to the standing pose at the end of a clip — that path is
  unchanged; we just made the same pose reachable directly.

### Migration / safety
- Purely additive. Out-of-range state selectors stay a no-op (the bounds check is widened to
  cover the two new values, not removed). No hardware change, no re-bake.

---

## B — FL shoulder regularization: servo 90 = "outward" for all four legs  (MIGRATE)

### What changes
- Today, "servo 90 = legs spread flat/outward" is true for three of the four legs. The
  front-left (FL) shoulder is the lone exception: it stands at an off-90 value (math 75).
- This change rewrites FL's shoulder conversion to match the other three legs
  (`90 + (sh − 135)`), and updates FL's neutral/standing numbers to suit, in both the
  firmware and the Blender exporter (kept byte-identical so a clip looks the same on the
  robot as in the browser).
- Result: an "all servos at 90" command finally produces a *symmetric* flat-spread pose, and
  FL/BL stop running into the edge of the shoulder's travel range at extremes (the silently
  clamped back-left shoulder noted in the 2026-05-23 verification).

### Why
- Removes the one asymmetry that traps anyone assuming "90 = flat" for every leg. It is also
  a prerequisite for the invert change (D): a clean `180 − servo` mirror only works once 90
  means the same thing on every leg.

### What it does NOT change
- The three other legs (FR, BR, BL) are completely untouched — same neutrals, same
  conversion.
- FL's *thigh* and *knee* are untouched; only the FL *shoulder* offset changes.
- The rotation-direction convention is unchanged — FL's outward direction is still +135°.
  This only changes which math angle the servo's 90 lines up with.
- Gait math, the 2/3 amplitude scale, clip interpolation, and the absolute-angle model are
  all unchanged.

### Migration / safety — REQUIRED, in order (B must not ship half-done)
1. **Do not ship the formula + config change alone.** On the bench today the FL horn is
   physically mounted so servo 75 = FL's standing pose. After this change the firmware
   commands servo 90 for that same pose. Shipping the code without the hardware step leaves
   FL ~15° off and biases every FL motion.
2. **Physically re-calibrate the FL shoulder horn** so that servo 90 = FL outward (the same
   flat-spread the other three legs show at 90).
3. **Re-bake / re-export every clip that keys FL**, because FL keyframes authored in the old
   75-based frame must be regenerated in the new 135-based frame. (This will change the
   exported clip files in git — expect a diff in `animation/exported_clips/`.)
4. **Re-flash** the firmware and **verify FL on hardware** against the other three legs at
   the flat pose and through a known clip.
5. **Re-run the parity test** (`test_servo_parity.py`) so the firmware and exporter agree on
   the new FL formula before anything else proceeds.
- Until all of the above are done, the convention docs continue to state "FL is the
  exception."

---

## D — One unified robot-flip behaviour for every motion source  (MIGRATE)

### What changes
- Today the robot's "inverted" (flipped-over) state behaves three different ways depending on
  what's driving: gaits flip thigh/knee only; the invert toggle writes a separate hardcoded
  pose; clips and direct calibration ignore the flip entirely.
- This change collapses all of that into **one** mirror — `servo' = 180 − servo` — applied at
  the single point where any angle is written to a servo, gated by the robot's flip flag. So
  gaits, on-board clips, and the exporter's live stream all flip the same way automatically.
- The per-gait thigh/knee flip and the hardcoded one-shot invert pose are removed so the
  robot can't double-flip.

### Why
- A flipped robot should run *every* motion identically — including clips and the exporter's
  WebSocket stream, which the flip state ignores today. This is also the groundwork for the
  planned keyframe-able invert toggle in Blender (change E, design-only for now): an
  animation could flip the robot mid-clip and have the following frames reinterpreted
  automatically.

### What it does NOT change
- **Upright motion is completely unchanged.** With the flip flag off, the new mirror is a
  no-op, so gaits, clips, calibration, and the deadman switch are bit-for-bit identical to
  today.
- The flip toggle's wire command (`{"T":6,"a":0}`) is unchanged — only its internal effect
  is unified.
- It must reproduce the *old* flipped gait result for thigh/knee (no regression for users who
  already flip the robot during walking).

### Migration / safety
- **Depends on B.** `180 − servo` is only a clean mirror once 90 = flat on every leg;
  shipping D before B would mirror FL's old off-90 values to the wrong place.
- **Decision required before any code:** `180 − servo` mirrors each channel in place. A true
  physical roll-over may also need a left↔right leg-role swap. This flip-axis question
  (per-channel mirror vs. also swapping leg roles) must be answered first — the
  implementation plan's first D step is a decision checkpoint, not code.
- Gated behind tests proving: (a) upright output is unchanged; (b) flipped gait thigh/knee
  matches the old behaviour; (c) clips and a simulated T:4 now also flip when inverted.

---

## Cross-cutting safety gate

Before any of B/C/D is considered done, these existing checks must still pass as a
regression gate:
- the firmware native test suite (`pio test -e native`),
- the servo parity test (`test_servo_parity.py`),
- the clip bake-independence test (`test_bake_independence.py`).

Change **E** (keyframe-able invert toggle in the Blender exporter) is **design-only** this
round and depends on D landing first; it is not part of this changelog's shipped work.
