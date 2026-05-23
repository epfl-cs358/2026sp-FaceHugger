# Convention documentation — orchestration & change ledger

Owner: Marcus. Created 2026-05-23. Read-only research already lives in
`docs/.work/convention-chain/` (step-a..f, summary, followup, api-consistency,
directions-table). This folder drives turning that evidence into:

1. **Current-reality conventions doc** (for the wiki) — what the robot + exporter
   actually do today, with the direction conventions stated clearly.
2. **Delta doc + planned changelog** — a copy of the reality doc annotated with the
   agreed changes, serving as the implementation plan. Changelog phrased like
   `CHANGELOG-2026-05-21-1821.md`.
3. **Final wiki docs** — produced only after 1 & 2 are reviewed and the changes land.

The hard constraint: **the way the robot works today (gaits, clips, calibration)
must not break.** Only the FL fix and invert-centralization change runtime
behaviour, and both are gated, migration-checklisted, and justified below.

---

## Verified ground truth (from the evidence + this session)

- **Control model:** every motion source sends *absolute servo degrees* per frame
  (gaits via `tickGait`→`translateToServo`→`setServoAngle`; clips via `tickClip`;
  exporter via T:4 `{a:…}`). Confirmed with teammate (Antoine): "setJointAngles, full
  hardcodé" for all gaits. The exporter is already in the right model.
- **Control convention is uniform across all 4 legs:** +shoulder = CCW-from-above,
  +thigh/+knee = up. `translateToServo` (`motion_math.cpp:4-31`) is the single place
  that hides every hardware mirror (BR shoulder sign-flip; FR/BL vs FL/BR thigh/knee
  L/R mirror). Callers never branch per leg. (directions-table.md)
- **servo 90 = each leg's NEUTRAL (standing) pose**, NOT a universal "outward". For
  FR/BR/BL the standing math-angle equals the image outward direction (45/−45/−135);
  **FL stands at math 75 while its outward is +135** — the lone asymmetry. Deliberate
  & internally consistent (config.h `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE=75`, firmware
  `NEUTRAL[FL].sh=75`, exporter `neutral_joint_deg.fl=75`, FL branch `servo=sh`);
  introduced in commit e005f5a as the "standing crouched pose". (followup.md, directions-table.md)
- **Convention images** (`code/simulation/docs/img/servo-1-…`, `servos-2-3-…`) define
  *rotation direction + global zero only*, not the tuned ranges/neutrals. (The two
  PNG filenames are swapped vs their content: servo-1 shows pitch, servos-2-3 shows yaw.)
- **config.h holds neutral *defaults*, not limits.** `*_DEFAULT_ANGLE` init each Servo
  at boot (`spinal_cord.cpp:37-54`). Runtime joint *limits* live in `kinematics.cpp`
  (`SHOULDER/HIP/KNEE_LIMIT_LO/HI`, radians, IK clamp). The URDF carries the kinematic
  limits for the sim/exporter side.
- **`relax()`** (`spinal_cord.cpp:93-99`) sets all 12 servos to 90 ("safe to power
  off") = the flat pose — **but is dead code, called nowhere.**
- **`CMD_POSE` (T:3)** is in the enum (`data.h:8`) but has **no handler** in
  `network.cpp` — silently dropped. `STATE_REST=4` is unreachable (CMD_STATE caps at 3).
- **`invertRobot()`** (`spinal_cord.cpp:431-438`) is a hardcoded one-shot pose,
  `servo'=180−servo` for thigh/knee, hip unchanged. The gait tickers separately apply
  `th=-th; kn=-kn` (math-space, before translateToServo) when `isInverted`; `tickClip`
  and the T:4 handler **ignore** `isInverted` entirely. So invert is partial &
  source-inconsistent today.

---

## Change ledger — what to change, what NOT to, and why

Legend: **DOC** = no code, zero risk · **ADD** = additive, low risk to existing behaviour
· **MIGRATE** = changes runtime behaviour, needs migration steps + hardware.

### A. Document current reality (DOC) — DO NOW
Write the conventions exactly as the code behaves today. No behaviour change. This is
deliverable 1 (wiki). Highest value, zero risk.

### B. FL shoulder `sh → sh−45` regularization (MIGRATE) — PLAN, do deliberately
**What:** change FL shoulder branch from `out.hip = sh` to `out.hip = 90 + (sh − 135)`
(= `sh − 45`) in BOTH `motion_math.cpp` and `_frame_to_servo`; set `convention.json`
`neutral_joint_deg.fl[0]` and firmware `NEUTRAL[FL].sh` to 135; set
`FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE` to 90.
**Why:** makes "servo 90 = outward/flat-spread" true for *all* legs (today 11/12 — FL
is the only exception), so the all-90 calibration pose is symmetric, and frees FL/BL
shoulder range (kills the BL-rails-at-180 clamp seen in `wave`).
**Why it hasn't bitten yet:** nothing in the live pipeline requires FL's servo-90 to be
outward — the standing pose, gaits, and all baked animations were authored in the same
75-based frame, so it's self-consistent. It only shows up (a) cosmetically, if you ever
run `relax()` (all-90) you'd see FL splayed ~45° off-symmetric — and `relax()` is dead,
so nobody has — and (b) as the out-of-range shoulder clamp at extremes.
**Migration (REQUIRED, or FL breaks):** physically re-calibrate the FL shoulder horn so
servo 90 = outward; re-derive FL's standing servo values; re-bake/re-export any clip
that keys FL; re-run `test_servo_parity.py`. **Do not ship the formula change without
the recalibration** — half-done leaves FL 45° off on hardware.
**If Marcus wants minimal/no-hardware now:** keep FL=75, document it as a known offset,
defer B. The reality doc must then state FL is the exception.

### C. Reference-pose API command (ADD) — DO, approach pending Marcus
**Correction (verified):** T:3 / `CMD_POSE` is **already documented** in `API_SPEC.md:54-61`
as **Body Pose / Static IK** (chassis `h`/`p`/`r` with feet planted) — it just has no
handler. So we must NOT repurpose T:3 for "assume a reference pose"; that would clash with
its documented meaning.
**What's needed:** an explicit way to command two reference poses — **flat / all-90**
(calibration; `relax()` already does exactly this and `STATE_REST=4` already means "all
servos 90, safe to power off") and **standing/neutral** (a new `stand()` driving legs to
`NEUTRAL[]`, today only reached by easing at gait/clip end).
**Approach — DECIDED (Marcus, 2026-05-23): option A.** Make `STATE_REST` reachable via
`CMD_STATE` (T:2): raise the handler's upper bound so `s=4` → `relax()` (flat / all-servos-90,
the calibration pose), and add a new state value `s=5` → `stand()` driving legs to `NEUTRAL[]`
(standing/neutral). Update the `RobotState` enum + `API_SPEC.md` §2 state table accordingly,
and bounds-check `s`. Smallest change, reuses existing semantics, no new command type. Do NOT
touch T:3 (leave it as documented-but-unimplemented body-pose IK; note it as a known gap).
**Why:** gives the exporter/app a flat-calibration command AND a return-to-neutral. Additive;
existing gaits/clips unaffected. Don't auto-call on boot. Bounds-check the selector.
**Risk:** low.

### D. Centralize `isInverted` as a live mirror (MIGRATE) — PLAN, separate task
**What:** apply invert at one choke point (`Servo::setServoAngle` or just before the PCA
write) as `servo' = 180 − servo`, so gait + clip + T:4 all inherit it; firmware owns the
invert state. Remove the per-gait `th=-th; kn=-kn` (else double-invert). Reconcile/retire
the hardcoded `invertRobot()` pose. Decide the physical flip semantics (pure per-channel
`180−servo`, vs also a L↔R leg-role swap for a true roll-over).
**Why:** makes a flipped robot run *every* motion source identically — clips and the
exporter's WebSocket stream included. Lets Marcus keyframe an invert toggle in Blender
and have the robot flip itself mid-animation with the post-flip frames reinterpreted
automatically. **Prereq: B** (so 90=flat uniformly, making `180−servo` a clean mirror).
**Risk:** medium — touches gait invert path + network + clip; must avoid double-apply and
must not regress the existing flip behaviour. Gated behind a flip-axis decision + tests.

### E. Exporter: keyframe-able invert state (DESIGN) — depends on D
Blender N-panel toggle (keyframe-able) that emits an "invert" state change in the clip /
JS stream / `clips_all.h`, so an animation can flip the robot and continue with the robot
inverted. Design only until D lands. Note in delta doc; not in this round's code.

### Explicitly DO NOT change (protect current behaviour)
- The other per-leg `translateToServo` signs (BR shoulder flip; FR/BL vs FL/BR thigh/knee
  mirror) — they are correct and load-bearing.
- The 2/3 amplitude scale, the absolute-angle model, the standing NEUTRAL values for
  FR/BR/BL.
- Gait math, clip interp, the deadman switch. None of A–E should touch these.

---

## Update — decisions from draft review (Marcus, 2026-05-23)

**History confirmed (answers "did the initial firmware calibrate at all-90?"):** YES. Commit
`623e30f` "set default leg angles to 90" set every `*_DEFAULT_ANGLE` to 90; the angle-space
gait refactor `3899ebc` then replaced all-90 with the per-leg standing `NEUTRAL[]` table and
left `relax()` (all-90) behind as the now-dead vestige. So change C **restores** the original
calibration pose rather than inventing one. (Antoine's "avant oui" = this.)

**Calibration philosophy (agreed):** calibration = command a known pose, the servos move to
those angles, then you physically place/mount the links to match. The pose itself doesn't
matter for correctness as long as it's a known reference; all-90 is best because every servo
sits at mechanical mid-travel. Calibrating from the flat/all-90 pose instead of standing is
fine — motors just turn to 90; a roughly-correct robot stays correct.

**C additions (still ADD, flash-now):** the flat/calibration command (`T:2 s=4` → `relax()`)
must **reset `isInverted=false`** and assumes the robot is right-side-up. Document that.

**B is the "make all-90 a true diagonal-X" change.** Marcus's question "should I adapt the
shoulders so servo 90 = outward?" → yes, that's exactly B, and only FL needs it (FR/BR/BL
already map servo 90 → outward). After B + the FL horn remount, the all-90 calibration pose is
the symmetric outward X. Until then all-90 is a near-X with FL ~45° off (still fine for
calibrating the other 11 joints + FL pitch).

**D simplified (decided): invert = pitch-only.** No flip-axis semantics, no shoulder flip, no
L↔R leg-role swap. Invert negates **thigh + knee only** about flat (`180 − servo` on those two
channels), shoulder untouched — identical to today's gait `th=-th; kn=-kn`, just applied at one
choke point so clips + T:4 inherit it too. This removes D's only open decision. Still MIGRATE,
still after B.

**New items from `servo-smoothing-prompt.md` (flash-now, additive):**
- **3a — smoothstep in `tickEase`** (`servo.cpp:45`): change the lerp fraction to
  `frac = frac*frac*(3−2*frac)`. Reaches target exactly at `t=easeDurMs` (smoothstep(1)=1).
  Affects return-to-neutral / `setServoAngleTimed` only; **gaits never call tickEase** (verified —
  they call `setJointAngles` at `spinal_cord.cpp:221/314/372`). Safe.
- **3b — EMA in clip path only** (`tickClip` `CLIP_ACT_APPLY_POSE`, between `clipPoseAt` at
  `:405` and `setJointAngles` at `:409`): per-channel `smoothed[4][3] = α·smoothed + (1−α)·target`,
  `α = CLIP_EMA_ALPHA = 0.75`, reset to frame 0 on `playClip`. **Must not touch `setServoAngle`,
  gaits, calibration, or T:4.** The gait input filter (`activeX/Y/Yaw += (target−active)*0.1f`,
  `:118-120`) is separate and unrelated. Marcus's requirement (EMA strictly inside tickClip,
  gaits bit-for-bit) is satisfiable as specified.
- **3c — rig shoulder limits: DEFERRED** until B lands (FL neutral 75→135 changes FL limits).

**Flashing / risk policy (Marcus):**
- **Flash now** (additive, behaviour-identical for existing commands): **C, 3a, 3b**.
- **B (FL regularization):** implement code, **HOLD the flash** until the FL horn is physically
  remounted. Code + hardware are inseparable.
- **D (unified invert):** after B.

**Backward-compatibility gates (hard):** the WebSocket API (existing `T:1/2/4/5/6/7` wire shape
+ semantics) and the gait patterns must stay **bit-for-bit unchanged**. C only *adds* state
values `s=4/5` (existing `s=0..3` untouched). 3a/3b never touch the gait path. D must net to
identical servo output for gaits (and identity when not inverted).

---

## Pipeline & agent dispatch

Each agent gets: this file + `handoff.md` (from `/handoff`) + its prompt + pointer to
`docs/.work/convention-chain/` evidence. Read-only on source unless its prompt says
otherwise; docs are written under the paths each prompt specifies.

1. `prompt-1-current-reality-doc.md` → writes the reality conventions doc. **Review gate.**
2. `prompt-2-delta-and-plan.md` → writes the delta doc + planned changelog + TDD task
   plan for B/C/D. **Review gate.**
3. `prompt-3-final-wiki-docs.md` → only after review: produce the polished wiki docs.

Subagents do NOT edit firmware/exporter code in steps 1–2 (docs only). Code changes
happen later, from the plan, as their own reviewed PRs.
