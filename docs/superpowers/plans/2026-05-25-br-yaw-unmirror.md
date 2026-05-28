# BR Shoulder Yaw Un-Mirror Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the shoulder/yaw convention uniform — `+sh` (CCW) → `+servo` for **all four** legs, including BR — across firmware, exporter, sim, and baked clips, while keeping every existing firmware gait's servo output byte-identical.

**Architecture:** BR's shoulder servo is currently mirrored (`translateToServo` slope −1: `90 − (sh+45)`). The motors are identical and all yaw shafts share the vertical axis, so BR should be `+1` like the others. The flip happens in the three parity-locked `translateToServo` twins (firmware `motion_math.cpp`, exporter `_frame_to_servo`, sim `translate_to_servo`). To preserve physical motion, BR's compensating *math-space* sign is flipped in tandem in the two firmware gaits (`tickYawRotation` `YAW_COEF[BR]`, `tickGait` `fwdDir[BR]`) — proven servo-output-preserving below. Clients (webapp, browser JS) need **no code change**: they send high-level commands (firmware-handled) or raw servo (`T:4`, below the convention). Clips are re-exported; verification spans every control path.

**Servo-preservation proof (BR, neutral sh = −45, deviation `d`):**
```
old: servo = 90 − (−45 + d + 45) = 90 − d
new: servo = 90 + (−45 − d + 45) = 90 − d   ← identical, for any d
```
So flipping `translateToServo[BR]` slope **and** negating BR's math-space deviation in each gait leaves every servo command unchanged. Only the math-space representation and the (re-exported) clip data change.

**Tech Stack:** C++ (firmware, PlatformIO `native` env + Unity tests), Python (exporter `fh_clip_panel.py`, sim `servo_convention.py`, pytest), Blender headless export, React Native webapp (verification only).

**Standing pose is unaffected:** at BR sh = −45, both formulas give servo 90.

---

### Task 1: Gait characterization guardrail (firmware, native)

Pin the current SERVO output of both gaits so later tasks prove "motion unchanged". This test must stay GREEN through Tasks 2–3.

**Files:**
- Create: `code/firmware/test/test_gait_servo_chars/test_gait_servo_chars.cpp`
- Reference: `code/firmware/src/nervous_system/spinal_cord.cpp:185-330` (tickGait), `:338-387` (tickYawRotation)

- [ ] **Step 1: Write the characterization test**

Drive `translateToServo` with the exact math-space `sh,th,kn` each gait produces for a sweep of `activeX/Y/Yaw`, and assert the resulting servo triples equal hard-coded golden values. (Compute the goldens once from the CURRENT build — see Step 2.) Cover, per leg: `tickGait` walk `{X:0,Y:1,Yaw:0}`, turn-in-walk `{X:0,Y:1,Yaw:0.5}`, crab `{X:1,Y:0,Yaw:0}`; `tickYawRotation` `{Yaw:+0.5}` and `{Yaw:-0.5}` at gait phase 0.25 and 0.75.

```cpp
#include <unity.h>
#include "motion_math.h"
// Golden servo values captured from the pre-change build (Step 2 prints them).
// Format: gait, leg, phase, {hip,thigh,knee}
// ... TEST_ASSERT_INT_WITHIN(1, GOLD[...], translateToServo(leg, sh, th, kn).hip) ...
```

- [ ] **Step 2: Capture goldens from the current build**

Temporarily `Serial.printf`/print the servo triples (or add a throwaway print test), run `pio test -e native -f test_gait_servo_chars`, copy the values into the `GOLD[]` table, remove the prints.

Run: `pio test -e native -f test_gait_servo_chars`
Expected: PASS (goldens match current behavior).

- [ ] **Step 3: Commit**

```bash
git add code/firmware/test/test_gait_servo_chars/
git commit -m "test(firmware): characterize gait servo output (BR un-mirror guardrail)"
```

---

### Task 2: Flip BR shoulder in firmware translateToServo + compensate gaits

**Files:**
- Modify: `code/firmware/src/nervous_system/motion_math.cpp:47`
- Modify: `code/firmware/src/nervous_system/spinal_cord.cpp:351` (YAW_COEF), `:218` (fwdDir)
- Test: `code/firmware/test/test_motion_math/test_motion_math.cpp:28`

- [ ] **Step 1: Update the translateToServo unit test (RED)**

In `test_motion_math.cpp:28`, change BR's expected hip from `90.0 - (sh + 45.0)` to `90.0 + (sh + 45.0)`. (Line 44, BR neutral == 90, stays.)

Run: `pio test -e native -f test_motion_math`
Expected: FAIL on the BR hip slope assertion.

- [ ] **Step 2: Flip BR shoulder slope**

`motion_math.cpp:47`:
```cpp
// BR shoulder un-mirrored: identical motor, yaw shaft on the same vertical
// axis as the others, so +sh = CCW = +servo for every leg. Gait YAW_COEF[BR]
// and tickGait fwdDir[BR] are flipped in tandem to keep servo output identical.
out.hip   = 90.0 + (sh + 45.0);   // was 90.0 - (sh + 45.0)
```

- [ ] **Step 3: Compensate the gaits (keep servo output identical)**

`spinal_cord.cpp:351`:
```cpp
// All legs now have +1 shoulder slope, so YAW_COEF is uniform.
static const float YAW_COEF[LEG_COUNT] = { -1.0f, -1.0f, -1.0f, -1.0f };
```
`spinal_cord.cpp:218` — BR's shoulder deviation must negate (was grouped with "rear" −1):
```cpp
// BR (LEG_RR) shoulder un-mirrored -> its fwd/yaw shoulder sign now matches the
// front legs (the old "front/rear" grouping encoded BR's servo mirror).
const float fwdDir = (i == LEG_FR || i == LEG_FL || i == LEG_RR) ? 1.0f : -1.0f;
```

- [ ] **Step 4: Run both tests**

Run: `pio test -e native -f test_motion_math -f test_gait_servo_chars`
Expected: BOTH PASS — `test_motion_math` matches the new BR formula; `test_gait_servo_chars` still matches the goldens (servo output unchanged → gaits not broken).

- [ ] **Step 5: Commit**

```bash
git add code/firmware/src/nervous_system/motion_math.cpp code/firmware/src/nervous_system/spinal_cord.cpp code/firmware/test/test_motion_math/
git commit -m "fix(firmware): un-mirror BR shoulder yaw; compensate gaits to preserve motion"
```

---

### Task 3: Mirror the flip in the exporter (parity twin)

**Files:**
- Modify: `code/simulation/../animation/addons/fh_clip_panel.py` (`_frame_to_servo`, the `br` branch, currently `90 - (sh + 45)`)
- Test: `code/firmware/test/gen_clip_parity_reference.py` (regenerates `clip_parity_reference.h`)

- [ ] **Step 1: Flip BR in `_frame_to_servo`**

In `_frame_to_servo` the `else: # br` branch, change `90 - (sh + 45)` → `90 + (sh + 45)`:
```python
else:  # br — un-mirrored: +sh = +servo, matching fr/fl/bl and firmware
    servo = [90 + (sh + 45), 90 + th, 90 - kn]
```

- [ ] **Step 2: Regenerate the clip-parity reference**

Run: `uv run python code/firmware/test/gen_clip_parity_reference.py`
Expected: `clip_parity_reference.h` updated with new BR shoulder values.

- [ ] **Step 3: Verify parity (firmware ≡ exporter)**

Run: `pio test -e native -f test_clip_parity` and `cd animation/scripts && uv run --with pytest python -m pytest test_servo_parity.py -q`
Expected: PASS (both use the new BR formula).

- [ ] **Step 4: Commit**

```bash
git add animation/addons/fh_clip_panel.py code/firmware/test/test_clip_parity/clip_parity_reference.h
git commit -m "fix(animation): un-mirror BR shoulder in exporter (parity with firmware)"
```

---

### Task 4: Mirror the flip in the sim + fix link1 axis handling

**Files:**
- Modify: `code/simulation/pybullet_interpreter/servo_convention.py` (`translate_to_servo`, BR branch)
- Modify: `code/simulation/pybullet_interpreter/clip_player.py` (`frame_to_joint_targets` — add URDF link1 axis correction)
- Test: `code/simulation/pybullet_interpreter/tests/test_servo_convention.py`, `tests/test_clip_player.py`

- [ ] **Step 1: Update sim translate_to_servo test (RED) and flip BR**

In `test_servo_convention.py`, update the BR (`LEG_RR`) shoulder expectation to `90 + (sh + 45)`. Then in `servo_convention.py` `translate_to_servo`, BR branch: `out.hip = 90.0 + (sh + 45.0)`.

Run: `cd code/simulation && uv run --with pytest python -m pytest pybullet_interpreter/tests/test_servo_convention.py -q`
Expected: RED then GREEN after the flip.

- [ ] **Step 2: Add link1 URDF-axis correction in the sim**

`frame_to_joint_targets` drives link1 with no axis factor though URDF link1 axes are diagonal (fr −Z, fl +Z, br +Z, bl −Z). Add a link1 axis-sign map and apply it so the sim renders yaw correctly:
```python
# URDF link1 <axis z> per leg (diagonal, like link2). Without this the sim
# drives shoulder yaw with the wrong sign for the -Z legs.
LEG_ID_TO_URDF_LINK1_SIGN = {LEG_FR: -1, LEG_FL: +1, LEG_RR: +1, LEG_RL: -1}
...
link1_axis = LEG_ID_TO_URDF_LINK1_SIGN[leg_id]
targets[f"{urdf_name}_link1_joint"] = link1_axis * servo_to_radians(servo.hip)
```

- [ ] **Step 3: Update the yaw uniformity test**

`test_body_rotation_uniform_mathspace_yaw` (animation/scripts) already asserts uniform math-space; update its servo-consequence assertion — after un-mirroring BR, all four shoulder servos move the SAME direction (BR no longer opposite).

Run: `cd code/simulation && uv run --with pytest python -m pytest pybullet_interpreter/tests/ -q` and `cd animation/scripts && uv run --with pytest python -m pytest test_check_export_consistency.py -q`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add code/simulation/pybullet_interpreter/servo_convention.py code/simulation/pybullet_interpreter/clip_player.py code/simulation/pybullet_interpreter/tests/ animation/scripts/test_check_export_consistency.py
git commit -m "fix(sim): un-mirror BR shoulder + add link1 URDF-axis correction"
```

---

### Task 5: Re-export clips + update fixtures

**Files:** Blender rig (`animation/fh_rigged_latest.blend`), `animation/exported_clips/*`, `code/simulation/pybullet_interpreter/tests/test_clip_loader.py`

- [ ] **Step 1: Re-export ALL clips (needs Blender)**

```bash
BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
  "$BLENDER_BIN" --background --factory-startup animation/fh_rigged_latest.blend \
  --python animation/scripts/export_all_clips.py
```
Expected: zero shoulder range-warnings; `check_export_consistency` PASS (convention + all clips).

- [ ] **Step 2: Update the on-disk clip fixtures**

Re-run `cd code/simulation && uv run --with pytest python -m pytest pybullet_interpreter/tests/test_clip_loader.py -q`; if `test_first_frame_lie_down_fr_shoulder` (and any BR fixture) drift, update the pinned values to the new export.

- [ ] **Step 3: Commit**

```bash
git add animation/exported_clips/ code/firmware/src/nervous_system/clips_all.h code/simulation/pybullet_interpreter/tests/test_clip_loader.py
git commit -m "chore(animation): re-export clips under un-mirrored BR yaw convention"
```

---

### Task 6: Docs + convention update

- [ ] Update `docs/CLIP_SHOULDER_CONVENTION.md` and `docs/.work/convention-docs/DRAFT-delta-conventions.md` §3 slope table: BR shoulder is now **+1** (was −1); `YAW_COEF` uniform; note the identical-motor / same-yaw-axis rationale and the gait `fwdDir[BR]` tandem flip. Commit `docs: BR shoulder un-mirrored — uniform yaw convention`.

---

### Task 7: Cross-path verification matrix (hardware + sim)

No code — confirm BR yaw is consistent everywhere. For a CCW body/turn input, BR's shoulder should now move the **same** servo direction as FR/FL/BL (before: opposite).

- [ ] **Ground truth (raw servo, `T:4`):** `{"T":4,"id":2,"servo_id":0,"a":120}` — confirm BR shoulder physically yaws the same way FR does for `{"T":4,"id":0,"servo_id":0,"a":120}`. (This path is convention-agnostic; it's the physical reference.)
- [ ] **Clip (`T:7` / sim `--clip` / browser `.js`):** play a yaw clip; BR shoulder moves with the others. Sim Servo-Angles panel shows all four shoulders same direction.
- [ ] **Turn (`T:1` direction / `T:5` gait):** command a turn; robot turns the same direction as before (gait motion preserved — Task 1 guardrail), BR shoulder consistent.
- [ ] **Webapp:** no code change; `LegControl` `T:4` jog and movement/clip screens all reflect the consistent direction.

---

## Notes / risks

- **Run `pio test -e native` after Tasks 1–3** — the gait characterization guardrail (Task 1) is what proves the firmware change didn't alter robot motion. This requires PlatformIO (run locally; not runnable from the planning environment).
- **Parity is all-or-nothing:** Tasks 2–4 change the three `translateToServo` twins together; do not leave the tree between Task 2 and Task 4 (parity tests red).
- **`T:4` calibration is unchanged** (raw servo). If any saved per-servo calibration offsets assumed BR's mirror, re-check them on hardware — but the command semantics don't change.
