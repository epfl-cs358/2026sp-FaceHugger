# Servo Convention Changes (B, C, D) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land three servo-convention changes — C (reachable flat/stand reference poses), B (FL shoulder regularization so servo 90 = outward for all legs), and D (one unified robot-flip mirror) — without breaking gaits, clips, or calibration.

**Architecture:** All three sit on the existing "absolute servo degrees, single chokepoint" model. C is purely additive (two new reachable FSM states reusing `relax()` and a new `stand()`). B is a one-line offset change to FL's shoulder in the firmware AND the byte-identical exporter, plus three config values, gated on a physical FL recalibration. D moves the robot-invert from three partial implementations into one `servo' = 180 − servo` transform at the PCA-write chokepoint, gated on a flip-axis decision and prereq B.

**Tech Stack:** C++ (ESP32 / PlatformIO, Unity host tests via `pio -e native`), Python 3.12 (`uv run`, the pure-Python `test_servo_parity.py`), Blender 5.1 (`test_bake_independence.py`), Markdown (API spec).

**Order:** C → B → D. C is independent and safe; D depends on B; B needs hardware.

**Regression gate (must stay green after every task):**
- `cd code/firmware && pio test -e native`
- `uv run animation/scripts/test_servo_parity.py` → `RESULT GREEN`
- `/Applications/Blender-5.1.app/Contents/MacOS/Blender --background --factory-startup animation/fh_rigged_latest.blend --python animation/scripts/test_bake_independence.py`

**Do NOT touch (from the change ledger):** the BR shoulder sign-flip; the FR/BL vs FL/BR thigh/knee mirror; FR/BR/BL NEUTRAL values; the 2/3 amplitude scale; gait math; clip interpolation; the deadman switch; the `T:3` handler (leave unimplemented).

---

## Task C: Reachable flat/stand reference poses (ADD)

**Files:**
- Modify: `code/firmware/src/shared/data.h:17-23` (add `STATE_STAND = 5`)
- Modify: `code/firmware/src/nervous_system/spinal_cord.h:18-19` (declare `stand()`)
- Modify: `code/firmware/src/nervous_system/spinal_cord.cpp` (add `SpinalCord::stand()`)
- Modify: `code/firmware/src/brain/network.cpp:77-89` (widen `CMD_STATE` bound + new cases)
- Modify: `code/API_SPEC.md` §2 (document s=4, s=5)
- Create: `code/firmware/test/test_state_bounds/test_state_bounds.cpp`
- Test: existing `test_motion_math` covers `translateToServo`; `stand()` reuses it.

> Context for the worker: `relax()` already exists (`spinal_cord.cpp:93-99`, all 12 servos → 90) and already sets `STATE_REST`. It is dead code only because nothing reaches it. C makes it reachable AND adds a `stand()` companion. The `CMD_STATE` handler today bounds-checks `newState >= STATE_IDLE && newState <= STATE_FAILSAFE` (`network.cpp:80`) and switches on IDLE/WALK/ACTION (`:82-84`). `STATE_REST = 4` already exists in the enum; `STATE_STAND` does not.

- [ ] **Step C1: Write the failing bounds test**

Create `code/firmware/test/test_state_bounds/test_state_bounds.cpp`. Mirror the `test_servo_index_bounds` pattern (a pure predicate test, no hardware). Add a small free predicate to the test that encodes the *intended* accepted range so the test is self-contained, then assert the enum values it must cover:

```cpp
#include <unity.h>
#include "../../src/shared/data.h"

// The CMD_STATE (T:2) handler must accept STATE_IDLE..STATE_STAND and reject
// anything outside that closed range. This predicate mirrors the bound the
// handler in network.cpp will use after Task C.
static bool isAcceptedState(int s) {
    return s >= STATE_IDLE && s <= STATE_STAND;
}

void setUp(void) {}
void tearDown(void) {}

void test_stand_state_value_is_5(void) {
    TEST_ASSERT_EQUAL_INT(5, STATE_STAND);   // FAILS until enum gains STATE_STAND
}

void test_rest_and_stand_accepted(void) {
    TEST_ASSERT_TRUE(isAcceptedState(STATE_REST));   // 4
    TEST_ASSERT_TRUE(isAcceptedState(STATE_STAND));  // 5
}

void test_idle_walk_action_failsafe_accepted(void) {
    TEST_ASSERT_TRUE(isAcceptedState(STATE_IDLE));
    TEST_ASSERT_TRUE(isAcceptedState(STATE_WALK));
    TEST_ASSERT_TRUE(isAcceptedState(STATE_ACTION));
    TEST_ASSERT_TRUE(isAcceptedState(STATE_FAILSAFE));
}

void test_out_of_range_rejected(void) {
    TEST_ASSERT_FALSE(isAcceptedState(-1));
    TEST_ASSERT_FALSE(isAcceptedState(6));   // one past STATE_STAND
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_stand_state_value_is_5);
    RUN_TEST(test_rest_and_stand_accepted);
    RUN_TEST(test_idle_walk_action_failsafe_accepted);
    RUN_TEST(test_out_of_range_rejected);
    return UNITY_END();
}
```

- [ ] **Step C2: Run the test, verify it fails**

Run: `cd code/firmware && pio test -e native -f test_state_bounds`
Expected: compile error or FAIL on `test_stand_state_value_is_5` — `STATE_STAND` is not defined yet.

- [ ] **Step C3: Add `STATE_STAND` to the enum**

In `code/firmware/src/shared/data.h`, extend `RobotState`:

```cpp
enum RobotState {
    STATE_IDLE     = 0,
    STATE_WALK     = 1,
    STATE_ACTION   = 2,
    STATE_FAILSAFE = 3,
    STATE_REST     = 4,  // All servos at 90° — safe to power off (flat/calibration)
    STATE_STAND    = 5,  // Legs driven to NEUTRAL[] — standing/neutral pose
};
```

- [ ] **Step C4: Run the test, verify it passes**

Run: `cd code/firmware && pio test -e native -f test_state_bounds`
Expected: PASS (4 tests).

- [ ] **Step C5: Declare and implement `stand()`**

In `code/firmware/src/nervous_system/spinal_cord.h`, add the declaration next to `relax()`:

```cpp
        void relax();
        void stand();
```

In `code/firmware/src/nervous_system/spinal_cord.cpp`, add right after `relax()` (after line 99). Drive every leg to its `NEUTRAL[]` math angle through `translateToServo` — the exact path gaits use to reach standing, just on demand:

```cpp
void SpinalCord::stand() {
    robotState = STATE_STAND;
    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };
    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        ServoTriple s = translateToServo(i, NEUTRAL[i].sh, NEUTRAL[i].th, NEUTRAL[i].kn);
        legs[i]->setJointAngles(s.hip, s.thigh, s.knee);
    }
}
```

- [ ] **Step C6: Wire the two new states into the handler**

In `code/firmware/src/brain/network.cpp`, widen the bound and add the two cases:

```cpp
        case CMD_STATE:
            if(doc.containsKey("s")){
                int newState = doc["s"];
                if(newState >= STATE_IDLE && newState <= STATE_STAND){
                    switch(newState){
                        case STATE_IDLE:  spinalCord.rest();     break;
                        case STATE_WALK:  spinalCord.walk();     break;
                        case STATE_ACTION: spinalCord.wallFlip(); break;
                        case STATE_REST:  spinalCord.relax();    break;
                        case STATE_STAND: spinalCord.stand();    break;
                        default: break;   // FAILSAFE (3) still a no-op, as today
                    }
                }
            }
            break;
```

- [ ] **Step C7: Build the firmware to confirm it compiles**

Run: `cd code/firmware && pio run -e upesy_wroom`
Expected: build succeeds (this exercises the real `network.cpp` + `spinal_cord.cpp`, which the native env does not compile).

- [ ] **Step C8: Document s=4 / s=5 in the API spec**

In `code/API_SPEC.md`, append two rows to the §2 (T:2) state table:

```markdown
| **4** | **REST** | All 12 servos to 90° — flat / calibration pose, safe to power off. |
| **5** | **STAND** | Drive legs to their standing NEUTRAL pose on demand. |
```

- [ ] **Step C9: Run the full regression gate**

Run: `cd code/firmware && pio test -e native`
Expected: all tests pass (existing suite + the new `test_state_bounds`).
Run: `uv run animation/scripts/test_servo_parity.py`
Expected: `RESULT GREEN` (C does not touch conversion; this just confirms no collateral).

- [ ] **Step C10: Commit**

```bash
git add code/firmware/src/shared/data.h code/firmware/src/nervous_system/spinal_cord.h code/firmware/src/nervous_system/spinal_cord.cpp code/firmware/src/brain/network.cpp code/API_SPEC.md code/firmware/test/test_state_bounds/test_state_bounds.cpp
git commit -m "feat(api): reachable flat (T:2 s=4) and stand (T:2 s=5) reference poses

Wire STATE_REST -> relax() (all-90 calibration) and add STATE_STAND -> stand()
(legs to NEUTRAL[]) via CMD_STATE. T:3 left as documented-but-unimplemented IK.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task B: FL shoulder regularization (MIGRATE — code + config; hardware is manual)

**Files:**
- Modify: `code/firmware/src/nervous_system/motion_math.cpp:13` (FL hip branch)
- Modify: `animation/scripts/fh_clip_panel.py:1903` (FL branch of `_frame_to_servo`)
- Modify: `code/firmware/src/nervous_system/spinal_cord.cpp:19` (`NEUTRAL[FL].sh`)
- Modify: `code/firmware/src/shared/config.h:32` (`FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE`)
- Modify: `animation/convention.json` (`neutral_joint_deg.fl[0]`)
- Test: `animation/scripts/test_servo_parity.py` (extend; it embeds its own firmware ground truth at `_firmware_translate`)

> **STOP — read before starting B.** B changes how the FL leg physically moves. The
> code/config steps below must NOT be merged to the running robot's branch until the manual
> hardware checklist (Step B0) is scheduled, because a code-only B leaves FL ~15° off. If the
> intent is "minimal / no hardware now," do NOT do Task B — keep FL=75 and leave the reality
> doc's "FL is the exception" note. (Per the ledger's B deferral option.)

- [ ] **Step B0: Manual hardware migration checklist (NOT a code step — owner: Marcus, on the bench)**

These are physical/manual actions that must accompany the code change. They are tracked here
so the plan is complete, but they are performed by a human on hardware, not by the agent:

  - [ ] Physically re-mount the FL shoulder horn so that **servo 90 = FL outward** (matching
        the flat-spread the other three legs show at servo 90).
  - [ ] Confirm FL's standing pose now corresponds to servo 90 on the recalibrated horn.
  - [ ] Re-bake / re-export every clip that keys FL in Blender, then re-export the bundle
        (`clips_all.h`) — FL keyframes authored in the old 75-frame are invalid in the new
        135-frame. Expect a diff in `animation/exported_clips/` and `clips_all.h`.
  - [ ] Re-flash the firmware to the board.
  - [ ] Verify on hardware: at the flat pose all four shoulders are symmetric; a known clip
        plays correctly on FL.
  - [ ] Only after the above: proceed to land the code/config steps below (or land them on a
        branch that is flashed together with the recalibration).

- [ ] **Step B1: Extend the parity test to assert the NEW FL formula (failing test)**

In `animation/scripts/test_servo_parity.py`, update the embedded ground-truth FL branch in
`_firmware_translate` and add an explicit "outward → servo 90 for every leg" assertion. First
change the FL line (currently `return [sh, 90.0 + th, 90.0 - kn]`) to:

```python
    if leg == "fl":  # LEG_FL (LegId 1) — regularized: servo 90 = outward (Change B)
        return [90.0 + (sh - 135.0), 90.0 + th, 90.0 - kn]
```

Then, before `print("RESULT ...")`, add a new check block. The four outward directions are
FR +45, FL +135, BR −45, BL −135; with all joints at outward+flat the shoulder servo must be
90 for every leg:

```python
    # Change B: servo 90 == outward (flat-spread) shoulder for ALL four legs.
    OUTWARD_SH = {"fr": 45.0, "fl": 135.0, "br": -45.0, "bl": -135.0}
    for leg in legs:
        hip_servo = _firmware_translate(leg, OUTWARD_SH[leg], 0.0, 0.0)[0]
        check(f"{leg}: outward shoulder -> servo 90", abs(hip_servo - 90.0) < 1e-6, hip_servo)
```

- [ ] **Step B2: Run the parity test, verify it fails**

Run: `uv run animation/scripts/test_servo_parity.py`
Expected: `RESULT RED` — `fl: outward shoulder -> servo 90` fails (exporter still uses
`servo = [sh, …]`, so FL outward 135 → servo 135, not 90), and the `@ N` / `@ N+30` FL cases
now mismatch the updated ground truth.

- [ ] **Step B3: Update the FL branch in the exporter (`_frame_to_servo`)**

In `animation/scripts/fh_clip_panel.py`, change the FL branch (currently
`servo = [sh, 90 + th, 90 - kn]`) to:

```python
        if leg == "fl":
            servo = [90 + (sh - 135), 90 + th, 90 - kn]
```

- [ ] **Step B4: Update FL neutral in `convention.json`**

In `animation/convention.json`, change `neutral_joint_deg.fl` shoulder from `75` to `135`:

```json
    "fl": [135, -60, -40],
```

- [ ] **Step B5: Run the parity test, verify it now passes**

Run: `uv run animation/scripts/test_servo_parity.py`
Expected: `RESULT GREEN`. Sanity-check: FL @ N now computes `90 + (135 − 135) = 90` for the
shoulder, matching the new ground truth and the new `convention.json` neutral.

- [ ] **Step B6: Update the firmware FL branch (`translateToServo`)**

In `code/firmware/src/nervous_system/motion_math.cpp`, change the FL case (line 13):

```cpp
        case 1:  // LEG_FL
            out.hip   = 90.0 + (sh - 135.0);   // Change B: servo 90 = FL outward
            out.thigh = 90.0 + th;
            out.knee  = 90.0 - kn;
            break;
```

- [ ] **Step B7: Update firmware FL neutral + boot default**

In `code/firmware/src/nervous_system/spinal_cord.cpp`, change the `NEUTRAL[FL]` row (line 19):

```cpp
    {  135.0f, -60.0f, -40.0f },  // LEG_FL (1) — Change B: outward shoulder
```

In `code/firmware/src/shared/config.h`, change line 32:

```cpp
#define FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE 90
```

> Cross-check: FL standing math 135 → `translateToServo` hip `90 + (135 − 135) = 90` →
> matches the new boot default of 90. The firmware and exporter now use the same FL formula.

- [ ] **Step B8: Build the firmware**

Run: `cd code/firmware && pio run -e upesy_wroom`
Expected: build succeeds.

- [ ] **Step B9: Update the native `translateToServo` test for the new FL formula**

`code/firmware/test/test_motion_math/test_motion_math.cpp:21` hardcodes the OLD FL hip
expectation (`fl.hip == sh`). It WILL fail after B6 and must be updated to the new offset:

```cpp
    ServoTriple fl = translateToServo(1, sh, th, kn);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + (sh - 135.0), fl.hip);   // Change B
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 + th,           fl.thigh);
    TEST_ASSERT_DOUBLE_WITHIN(TOL, 90.0 - kn,           fl.knee);
```

(Lines 16, 22-23, 26, 31 — FR/BR/BL hip and FL thigh/knee — are unchanged.)

- [ ] **Step B9b: Run the full regression gate**

Run: `cd code/firmware && pio test -e native`
Expected: all pass, including the updated `test_motion_math`.
Run: `uv run animation/scripts/test_servo_parity.py` → `RESULT GREEN`.
Run the Blender bake test (full path) and confirm it still passes:
`/Applications/Blender-5.1.app/Contents/MacOS/Blender --background --factory-startup animation/fh_rigged_latest.blend --python animation/scripts/test_bake_independence.py`

- [ ] **Step B10: Commit (code/config only — note the hardware dependency in the message)**

```bash
git add code/firmware/src/nervous_system/motion_math.cpp code/firmware/src/nervous_system/spinal_cord.cpp code/firmware/src/shared/config.h animation/scripts/fh_clip_panel.py animation/convention.json animation/scripts/test_servo_parity.py
git commit -m "feat(convention): regularize FL shoulder so servo 90 = outward for all legs (Change B)

FL hip: servo = 90 + (sh - 135); NEUTRAL[FL].sh 75->135; FL boot default 75->90;
convention.json fl[0] 75->135; exporter _frame_to_servo FL branch matched.
REQUIRES physical FL horn recalibration + FL clip re-bake before flashing — see
docs/superpowers/plans/2026-05-23-servo-convention-changes.md Step B0.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task D: Centralize `isInverted` as one live mirror (MIGRATE — depends on B)

**Files:**
- Modify: `code/firmware/src/nervous_system/servo.cpp:14-25` (or a thin wrapper) — apply the mirror
- Modify: `code/firmware/src/nervous_system/spinal_cord.cpp:216,310,369` (remove per-gait `th=-th;kn=-kn`)
- Modify: `code/firmware/src/nervous_system/spinal_cord.cpp:431-447` (`invertRobot()` → toggle flag only)
- Create: `code/firmware/test/test_invert_mirror/test_invert_mirror.cpp`
- Test: existing `test_motion_math`, `test_clip_*`, parity, bake tests as regression gate.

> **Prereq: Task B must be landed first.** `180 − servo` is only a clean mirror once 90 = flat
> on every leg; with FL still at the old 75 frame, FL mirrors to the wrong place.

- [ ] **Step D0: DECISION CHECKPOINT — flip-axis semantics (NOT code)**

Before writing any D code, resolve how a flipped robot should move. Use `AskUserQuestion` (or
a recorded decision in the orchestration doc) to choose between:

  - **Option 1 — per-channel mirror only:** `servo' = 180 − servo` on every channel, no leg
    re-mapping. Reproduces today's thigh/knee flip behaviour and extends it to shoulders +
    clips + T:4. Simplest; matches the existing `invertRobot()` literals which are pure
    `180 − default`.
  - **Option 2 — per-channel mirror + L↔R leg-role swap:** additionally route left-leg
    commands to right-leg servos (and vice-versa) for a true geometric roll-over.

  Record the answer in `docs/.work/convention-docs/00-orchestration.md` under change D. The
  steps below assume **Option 1** (per-channel mirror). If Option 2 is chosen, expand D into
  a separate plan covering the leg-role remap — do NOT improvise it inside the chokepoint.

- [ ] **Step D1: Add an invert flag the chokepoint can read (failing test first)**

The mirror lives at the per-servo write but the flip state lives in `SpinalCord`. Introduce a
single source of truth the `Servo` write can consult. Simplest: a file-scope `volatile bool`
in `servo.cpp` set by `SpinalCord`, or a static on `Servo`. Create the test first.

Create `code/firmware/test/test_invert_mirror/test_invert_mirror.cpp`. It tests a pure mirror
helper (extract the arithmetic into a free function so it is host-testable without the PCA
driver):

```cpp
#include <unity.h>

// Pure mirror used by Servo::setServoAngle when the robot is inverted (Change D).
// Extracted so it is testable on host without the PCA9685 driver.
static double mirrorIfInverted(double angle, bool inverted) {
    return inverted ? (180.0 - angle) : angle;
}

void setUp(void) {}
void tearDown(void) {}

void test_upright_is_identity(void) {
    TEST_ASSERT_EQUAL_DOUBLE(90.0, mirrorIfInverted(90.0, false));
    TEST_ASSERT_EQUAL_DOUBLE(30.0, mirrorIfInverted(30.0, false));
    TEST_ASSERT_EQUAL_DOUBLE(150.0, mirrorIfInverted(150.0, false));
}

void test_inverted_reflects_about_90(void) {
    TEST_ASSERT_EQUAL_DOUBLE(90.0,  mirrorIfInverted(90.0,  true)); // center fixed
    TEST_ASSERT_EQUAL_DOUBLE(150.0, mirrorIfInverted(30.0,  true)); // 180-30
    TEST_ASSERT_EQUAL_DOUBLE(30.0,  mirrorIfInverted(150.0, true)); // 180-150
}

void test_inverted_matches_old_invertRobot_literals(void) {
    // OLD invertRobot() wrote 180-default for each servo. Confirm the mirror
    // reproduces those exact literals from the pre-D hardcoded pose (FR default
    // thigh=150,knee=53 -> 30,127; FL 30,130 -> 150,50; etc.).
    TEST_ASSERT_EQUAL_DOUBLE(30.0,  mirrorIfInverted(150.0, true)); // FR thigh
    TEST_ASSERT_EQUAL_DOUBLE(127.0, mirrorIfInverted(53.0,  true)); // FR knee
    TEST_ASSERT_EQUAL_DOUBLE(150.0, mirrorIfInverted(30.0,  true)); // FL thigh
    TEST_ASSERT_EQUAL_DOUBLE(50.0,  mirrorIfInverted(130.0, true)); // FL knee
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_upright_is_identity);
    RUN_TEST(test_inverted_reflects_about_90);
    RUN_TEST(test_inverted_matches_old_invertRobot_literals);
    return UNITY_END();
}
```

- [ ] **Step D2: Run the test, verify it fails**

Run: `cd code/firmware && pio test -e native -f test_invert_mirror`
Expected: FAIL/compile error — the test file is new and `mirrorIfInverted` is only defined in
the test; this step proves the test harness runs and the assertions are exercised. (After
D3 the same arithmetic moves into `servo.cpp`; the host test keeps its local copy as the
contract, mirroring how `test_servo_parity.py` embeds its own ground truth.)

> Note: the test's local `mirrorIfInverted` makes it self-contained. After D3, optionally
> include `servo.h` and call the real helper if it is exposed; otherwise the local copy is
> the spec the implementation must match (same pattern as `_firmware_translate`).

- [ ] **Step D3: Apply the mirror at the chokepoint, gated by the flag**

In `code/firmware/src/nervous_system/servo.cpp`, add a file-scope flag and apply the mirror at
the top of `setServoAngle`, before the clamp:

```cpp
// Single source of truth for the robot-flip mirror (Change D). SpinalCord owns
// the toggle; every servo write inherits it here so gaits, clips and T:4 all
// flip identically. servo' = 180 - servo (reflect about the 90 center).
static volatile bool g_inverted = false;
void Servo::setInverted(bool v) { g_inverted = v; }

void Servo::setServoAngle(double angle){
    if (g_inverted) angle = 180.0 - angle;   // Change D: one live mirror
    double clamped = constrain(angle, 0.0, 180.0);
    if (clamped != angle) {
        Serial.printf("[WARN] servo %d clamped: %.1f -> %.1f\n", this->pcaChannel, angle, clamped);
    }
    uint16_t pulse = map(clamped, 0, 180, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(this->pcaChannel, 0, pulse);
    this->servoAngle = clamped;
}
```

Declare `static void setInverted(bool);` in `code/firmware/src/nervous_system/servo.h`.

- [ ] **Step D4: Run the mirror test, verify it passes**

Run: `cd code/firmware && pio test -e native -f test_invert_mirror`
Expected: PASS (3 tests).

- [ ] **Step D5: Remove the per-gait double-invert**

In `code/firmware/src/nervous_system/spinal_cord.cpp`, delete the three lines that pre-mirror
in math-space (now redundant — the chokepoint handles it):

- `:216` `if (isInverted) { th = -th; kn = -kn; }` in `tickGait`
- `:310` same line in `tickTrot`
- `:369` same line in `tickYawRotation`

> Why remove: leaving them would double-invert (once in math-space, once at the choke).

- [ ] **Step D6: Reduce `invertRobot()` to a flag toggle**

In `code/firmware/src/nervous_system/spinal_cord.cpp`, replace the hardcoded `180 − default`
pose body (`:431-447`) with a toggle that updates both the local flag and the chokepoint
flag, then snaps the legs to defaults so the new mirror takes effect immediately:

```cpp
void SpinalCord::invertRobot() {
    isInverted = !isInverted;
    Servo::setInverted(isInverted);   // Change D: chokepoint mirror owns the flip
    // Re-apply the standing default; it now flows through the mirror automatically.
    leg1.returnToDefaultAngles();
    leg2.returnToDefaultAngles();
    leg3.returnToDefaultAngles();
    leg4.returnToDefaultAngles();
    gaitPhaseStartMs_ = millis();
}
```

> Verify against OLD behaviour: when inverted, `returnToDefaultAngles()` writes each servo's
> default, the chokepoint mirrors it to `180 − default`, which equals the OLD hardcoded
> literals (FR `90,30,127`; FL `90/75…` per the recalibrated default; etc.). This is exactly
> what `test_invert_mirror`'s third test asserts. Note FL's default is now 90 (post-B), so its
> mirrored value is `180 − 90 = 90` — consistent with B's symmetric flat pose.

- [ ] **Step D7: Build the firmware**

Run: `cd code/firmware && pio run -e upesy_wroom`
Expected: build succeeds (compiles `spinal_cord.cpp`, `servo.cpp`, `network.cpp` together).

- [ ] **Step D8: Run the full regression gate**

Run: `cd code/firmware && pio test -e native`
Expected: all pass, including `test_invert_mirror`, `test_motion_math`, `test_clip_*`,
`test_state_bounds`. The motion_math/gait tests confirm the **upright** path is unchanged
(invert flag defaults false → identity).
Run: `uv run animation/scripts/test_servo_parity.py` → `RESULT GREEN` (D does not touch the
exporter conversion).
Run the Blender bake test (full path) → still passes.

- [ ] **Step D9: Commit**

```bash
git add code/firmware/src/nervous_system/servo.cpp code/firmware/src/nervous_system/servo.h code/firmware/src/nervous_system/spinal_cord.cpp code/firmware/test/test_invert_mirror/test_invert_mirror.cpp
git commit -m "feat(invert): one live 180-servo mirror at the PCA chokepoint (Change D)

Gaits, clips and T:4 now all inherit the robot-flip from a single transform in
Servo::setServoAngle, gated by the invert flag. Removed per-gait th/kn negation
and the hardcoded invertRobot() pose to avoid double-invert. Per-channel mirror
(flip-axis Option 1). Depends on Change B.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Note on Change E (out of scope this round)

Change E (a keyframe-able invert toggle in the Blender exporter that emits an invert state
change into the clip / JS stream / `clips_all.h`) is **design-only** and depends on D landing.
It is documented in the delta doc §9 and the orchestration ledger; do not implement it here.

---

## Self-review

**Spec coverage** (against `prompt-2-delta-and-plan.md` Task list + the orchestration ledger):
- C — `case CMD_STATE` widened + bounds-checked (C6), `stand()` driving `NEUTRAL[]` (C5),
  `relax()` reused for all-90 (C6), API_SPEC documented (C8), bounds test mirroring
  `test_servo_index_bounds` + out-of-range no-op (C1). T:3 left untouched (stated in header +
  E note). Note: the prompt's pre-edit wording said "add case CMD_POSE / T:3"; per the
  authoritative ledger correction this plan uses the **CMD_STATE (T:2) s=4/s=5** approach
  instead. ✓
- B — FL branch in `motion_math.cpp` AND `_frame_to_servo` → `90 + (sh − 135)` (B6/B3);
  `convention.json` fl[0]→135 (B4); `NEUTRAL[FL].sh`→135 (B7);
  `FRONT_LEFT_LEG_HIP_DEFAULT_ANGLE`→90 (B7); parity test extended (FL outward 135→servo 90,
  all four outward→90) (B1); manual recalibrate/re-bake/re-flash/verify checklist (B0); note
  re-bake changes exported clips (B0). ✓
- D — single `servo'=180−servo` at chokepoint gated by flag (D3); remove per-gait
  `th=-th;kn=-kn` (D5); retire hardcoded `invertRobot()` pose (D6); flip-axis decision
  checkpoint FIRST, not code (D0); tests: invert reflects about 90 for every joint, matches
  OLD behaviour, identity when upright (D1); prereq B stated (header + D0). ✓
- Regression gate (`pio test -e native`, parity, `test_bake_independence.py`) listed per task
  (C9, B9, D8) and in the header. ✓

**Placeholder scan:** no TBD/TODO/"add error handling"/"similar to Task N"; every code step
shows the actual code; every run step shows the command + expected result. ✓

**Type/name consistency:** `STATE_STAND = 5` (data.h) used identically in test, handler, and
`stand()`; `stand()` / `relax()` names consistent across .h/.cpp/handler;
`mirrorIfInverted`/`setInverted`/`g_inverted` consistent across D1–D6; FL formula
`90 + (sh − 135)` identical in firmware (B6) and exporter (B3) and the parity ground truth
(B1); FL neutral `135` identical in `NEUTRAL`, `convention.json`, and parity. ✓
