# FaceHugger firmware servo/robot control architecture

Read-only study for porting firmware control into the PyBullet sim for parity.
All paths relative to `code/firmware/src/`. Line refs are at time of reading
(2026-05-26, branch `feat/pybullet-sim-interpreter`).

The headline finding for the port: **every motion path converges on a single,
pure, host-tested function `translateToServo()` (math-space → servo degrees) and
a single physical sink `Servo::setServoAngle()` (servo degrees → PCA9685 PWM).
The cleanest parity seam is the 12 servo angles in degrees** — that is exactly
the `Snapshot.servo_angles[12]` already exported for the diagnostics CSV.

---

## 1. THE CONTROL SURFACE — every pathway that drives servos

The state machine in `SpinalCord::update()` (`spinal_cord.cpp:126`) dispatches
the *continuous* paths; the *one-shot* paths are invoked directly from the
network handler (`brain/network.cpp:67`). Enumerated:

| # | Pathway | Entry fn | Inputs | Computes | Final write |
|---|---------|----------|--------|----------|-------------|
| A | **Gait (walk/crab)** | `tickGait()` `spinal_cord.cpp:168` | `millis()`, `activeX/Y/Yaw`, `GAITS[currentGait_]`, `NEUTRAL[]` | phase clock → per-leg sweep/lift → math-space sh/th/kn | `applyServos → translateToServo → Leg::setJointAngles → Servo::setServoAngle` |
| A'| **Trot** (specialised) | `tickTrot()` `spinal_cord.cpp:250` | same + hardcoded constants | 4-phase front / continuous-sweep rear, 2/3 scale | same |
| A''| **In-place yaw rotation** | `tickYawRotation()` `spinal_cord.cpp:342` | `millis()`, `activeYaw` | diagonal trot, all hips same rotational sense | same |
| B | **Clip playback** | `tickClip()` `spinal_cord.cpp:419` (started by `playClip()` `:395`) | `millis()`, `FH_CLIPS[id]` baked frames, `clipState_`, `clipSmoothed_` | frame interp → per-channel EMA → translate → clip clamp | `applyServos(clampClipServos(translateToServo(...)))` then 500 ms ease to NEUTRAL via `Servo::tickEase` |
| C | **Stand / neutral** | `stand()` `spinal_cord.cpp:110` | `NEUTRAL[]` | translate of NEUTRAL pose, once | `applyServos → translateToServo → setJointAngles` |
| D | **Rest / calibration flat pose** | `relax()` `spinal_cord.cpp:89` | constant 90° | sets all 12 servos to 90, clears invert | `Leg::setJointAngles(90,90,90)` (NO translateToServo) |
| E | **Direct servo calibration** | `applyCalibration(channel,angle)` `spinal_cord.cpp:119` → `Leg::identifyAndMove` `leg.cpp:98` | raw PCA channel + angle from `CMD_CALIBRATE` | none — passes angle straight through | `Servo::setServoAngle` on the matching channel |
| F | **Idle / boot / graceful-stop / failsafe** | `returnToDefaultAngles()` `leg.cpp:76` (and `begin()` `:61`) | `*_DEFAULT_ANGLE` from config.h | none — writes the per-servo default | `Servo::returnToDefaultAngle → setServoAngle` |
| G | **IK pose (legacy / `setPose`)** | `Leg::setPose(x,y,z)` `leg.cpp:20` | foot xyz | `legIK` (kinematics.cpp) → per-leg hardware map | `updateServos → setServoAngle`. **NOT wired to any command** — `CMD_POSE` (T:3) has no handler in network.cpp; dead at runtime. |
| H | **Invert** | `invertRobot()` `spinal_cord.cpp:465` | toggles `isInverted` | re-applies NEUTRAL through `applyServos` (which mirrors pitch) | same as C |

Notes that matter for the port:
- Paths A/A'/A''/B/C/H all go through the **single choke point** `applyServos()`
  (`spinal_cord.cpp:105`), which applies the invert mirror then calls
  `Leg::setJointAngles`. Paths D/E/F bypass `translateToServo` entirely (they
  command servo degrees directly).
- `relax()` (D) writes **servo-space 90** directly, *not* a NEUTRAL math pose —
  it is the "Superman"/T-pose calibration stance, distinct from `stand()`.
- Only A/A'/A''/B are *ticked* every loop; C/D/E/F/H are one-shot writes that
  then sit in a no-op state (`STATE_IDLE/REST/STAND` are empty cases in
  `update()`).

---

## 2. tickGait — the port target (full algorithm)

`tickGait()` handles WALK and CRAB. TROT and pure-yaw are split off at the top:

```cpp
void SpinalCord::tickGait() {
    // Yaw input -> coordinated in-place rotation, regardless of selected gait.
    if (fabsf(activeYaw) > 0.05f) { tickYawRotation(); return; }
    if (currentGait_ == GAIT_TROT) { tickTrot(); return; }

    const GaitParams& cfg = GAITS[currentGait_];
    const float t           = (millis() - gaitPhaseStartMs_) / 1000.0f;
    const float globalPhase = fmodf(t / cfg.period_s, 1.0f);

    // Graceful stop: clean phase boundary -> returnToDefaultAngles, STATE_IDLE
    if (!isMovingRequested && fabsf(activeX) < 0.01f && fabsf(activeY) < 0.01f
        && fabsf(activeYaw) < 0.01f && globalPhase < 0.05f) {
        leg1.returnToDefaultAngles(); ... ; robotState = STATE_IDLE; return;
    }

    Leg* legs[LEG_COUNT] = { &leg1, &leg2, &leg3, &leg4 };
    const bool isCrab = (currentGait_ == GAIT_CRAB);

    for (uint8_t i = 0; i < LEG_COUNT; ++i) {
        const float legPhase = fmodf(globalPhase - cfg.offsets[i] + 1.0f, 1.0f);

        float sweep, lift;
        if (legPhase < cfg.duty) {                 // STANCE
            const float p = legPhase / cfg.duty;
            sweep = cfg.step_length_deg * (0.5f - p);
            lift  = 0.0f;
        } else {                                   // SWING
            const float p = (legPhase - cfg.duty) / (1.0f - cfg.duty);
            sweep = cfg.step_length_deg * (-0.5f + p);
            lift  = sinf(p * (float)M_PI) * cfg.step_height_deg;
        }

        float sh = NEUTRAL[i].sh;
        float th = NEUTRAL[i].th;
        float kn = NEUTRAL[i].kn;

        // Shoulder sweep: fwd/back (Y) + yaw. Crab suppresses fwd, keeps yaw.
        {
            const float fwdDir = (i==LEG_FR || i==LEG_FL || i==LEG_RR) ? 1.0f : -1.0f;
            const float yawDir = (i==LEG_FR || i==LEG_RR) ? 1.0f : -1.0f;
            const float fwdContrib = isCrab ? 0.0f : activeY;
            sh += fwdDir * sweep * (fwdContrib + yawDir * activeYaw);
        }

        // Thigh sweep: lateral (X). Left legs (FL,RL) subtract; right (FR,RR) add.
        if (i == LEG_FL || i == LEG_RL) th -= sweep * activeX;
        else                             th += sweep * activeX;

        // Lift: thigh +, knee - (JS convention)
        th += lift;
        kn -= lift;

        applyServos(legs[i], translateToServo((uint8_t)i, sh, th, kn));
    }
}
```

### Phase clock
- `t = (millis() - gaitPhaseStartMs_) / 1000` seconds since `setGait()`/invert.
- `globalPhase = fmod(t / period_s, 1)` ∈ [0,1).
- Per-leg: `legPhase = fmod(globalPhase - offsets[i] + 1, 1)`.

### Per-leg offsets (order `[FR, FL, RR, RL]`, `spinal_cord.cpp:22`)
```
None  : { 0.0,  0.0,  0.0,  0.0 }
Walk  : { 0.5,  0.0,  0.25, 0.75 }   period 2.0 s, step_len 26.7°, step_ht 26.7°, duty 0.75
Trot  : { 0.5,  0.0,  0.0,  0.5  }   (tickGait params unused; tickTrot hardcodes)
Crab  : { 0.5,  0.0,  0.0,  0.5  }   period 1.5 s, step_len 20.0°, step_ht 33.3°, duty 0.50
```
`GaitParams` field order is `{period_s, step_length_deg, step_height_deg, duty, offsets[4], label}` (`movements.h:26`).

### Foot trajectory / leg-angle generation
- **Stance** (`legPhase < duty`): linear hip sweep `sweep = step_len*(0.5 - p)`, no lift.
- **Swing** (`legPhase >= duty`): reverse sweep `sweep = step_len*(-0.5 + p)`, sinusoidal lift `lift = sin(p·π)*step_ht`.
- Sweep drives the **shoulder** (forward/yaw) and the **thigh** (lateral/X). Lift
  drives **thigh +lift / knee −lift**. Knee never gets sweep — only lift.

### YAW_COEF / fwdDir / direction handling
- `fwdDir` per leg: `+1` for FR, FL, **RR(BR)**; `−1` for RL(BL). The BR +1 grouping
  was changed 2026-05-25 to absorb BR's former servo mirror (see translateToServo BR).
- `yawDir`: `+1` for right side (FR, RR); `−1` for left side (FL, RL).
- Combined shoulder term: `sh += fwdDir * sweep * (fwdContrib + yawDir*activeYaw)`.
- `activeX/Y/Yaw` are smoothed body intents, **not** raw — see §5.
- In **`tickYawRotation`** (`:342`) the relevant constants are:
  `STEP_LENGTH=40, STEP_HEIGHT=50, DUTY=0.5, PERIOD_S=1.5, SCALE=2/3, YAW_GAIN=2`,
  `OFFSETS={0.5,0,0,0.5}`, and `YAW_COEF[LEG_COUNT] = {-1,-1,-1,-1}` (now uniform
  because BR was un-mirrored in `translateToServo`). Shoulder:
  `sh -= sweep * YAW_COEF[i] * activeYaw * YAW_GAIN`. Then a 2/3 scale-from-NEUTRAL.

### tickTrot constants (`spinal_cord.cpp:250`)
`STEP_LENGTH=40, STEP_HEIGHT=50, DUTY=0.50, PERIOD_S=1.5, SCALE=2/3, YAW_GAIN=2`,
`OFFSETS={0.5,0,0,0.5}`, front-leg hip endpoints `HIP_IN={65,75}`, `HIP_OUT={25,110}`
(indices [FR,FL]). `mag = min(|activeY|,1)`; backward reverses the phase
(`globalPhase = 1 - globalPhase`). Front legs use 4 discrete hip positions
keyed on legPhase quartiles; rear legs use the continuous stance/swing sweep.
Both finish with a 2/3 scale-from-NEUTRAL: `x = NEUTRAL + (x - NEUTRAL)*2/3`.

### Flow to hardware
`translateToServo(legId, sh, th, kn)` (`motion_math.cpp:32`) maps math → servo
degrees per leg, then `applyServos` mirrors pitch if inverted, then
`Leg::setJointAngles` clamps `[0,180]` and calls `Servo::setServoAngle` →
`map(angle,0,180,MIN_PULSE,MAX_PULSE)` → `pwm.setPWM`. **Gaits do NOT use
`clampClipServos`** (that is clip-only) — gait output is bit-for-bit
translateToServo + the `[0,180]` constrain in `setJointAngles`/`setServoAngle`.

NEUTRAL pose (`neutral_pose.h:13`, order FR,FL,RR,RL = sh,th,kn):
```
FR {  45, -60, -37 }   FL { 135, -60, -40 }
BR { -45, -50, -50 }   BL {-135, -60, -35 }
```

---

## 3. THE SERVO BOUNDARY — exact hardware output + parity seam

### Final hardware write (`servo.cpp:15`)
```cpp
void Servo::setServoAngle(double angle){
    double clamped = constrain(angle, 0.0, 180.0);   // electrical backstop
    uint16_t pulse = map(clamped, 0, 180, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(this->pcaChannel, 0, pulse);
    this->servoAngle = clamped;
}
```
`map` is Arduino's integer map: `pulse = (clamped-0)*(600-150)/(180-0) + 150`,
integer-truncated. **MIN_PULSE=150, MAX_PULSE=600** (`config.h:13-14`), PCA9685
at **60 Hz** (`driver.setPWMFreq(60)`, `spinal_cord.cpp:63`), I2C addr `0x40`.
So `pulse_counts = 150 + round_down(clamped * 450/180) = 150 + floor(clamped*2.5)`.

### Per-servo "calibration / offset / direction"
There is **no per-servo offset/direction array** (no `SERVO_CONFIG[]`). The
direction/offset live entirely in `translateToServo()` per-leg switch
(`motion_math.cpp:32`):
```cpp
case 0:  // LEG_FR
    out.hip   = 90.0 + (sh - 45.0);   out.thigh = 90.0 - th;  out.knee = 90.0 + kn;
case 1:  // LEG_FL
    out.hip   = 90.0 + (sh - 135.0);  out.thigh = 90.0 + th;  out.knee = 90.0 - kn;
case 2:  // LEG_RR / BR
    out.hip   = 90.0 + (sh + 45.0);   out.thigh = 90.0 + th;  out.knee = 90.0 - kn;
    //          ^ was 90.0 - (sh + 45.0) before 2026-05-25 un-mirror
case 3:  // LEG_RL / BL
    out.hip   = 90.0 + (sh + 135.0);  out.thigh = 90.0 - th;  out.knee = 90.0 + kn;
```
Shoulder global compass offsets: FR +45, FL +135, BR −45, BL −135. Pitch sign
alternates by diagonal: FR/BL use `thigh = 90-th, knee = 90+kn`; FL/BR use
`thigh = 90+th, knee = 90-kn`. The PCA channel mapping is a separate table
`LEG_SERVO_CHANNEL[4][3]` (`config.h:64`) and the per-servo boot defaults are the
`*_DEFAULT_ANGLE` macros (`config.h:23-52`).

### Cleanest parity seam
**Servo angle in degrees (the 12 values), captured at `Snapshot.servo_angles[12]`
(`spinal_cord.h:39`, filled by `snapshot()` `:477` via `Leg::getJointAngles` ←
`Servo::getServoAngle`).** This is post-`translateToServo`, post-invert,
post-`[0,180]` constrain, but **pre-PWM/integer-map**. Sim and firmware can be
compared bit-for-bit here without modelling the integer `map()` truncation or
I2C. (If you want to also verify the electrical layer, compare PWM counts via
`pulse = 150 + floor(deg*2.5)`, but that adds integer-truncation noise.)
Caveat: `getServoAngle()` returns whole-degree precision because `servoAngle` is
stored as `uint16_t` (`servo.h:24`, comment at `servo.cpp:64`) — so even the
degree seam is quantised to whole degrees in firmware.

---

## 4. SHARED MATH (portable, bit-for-bit) vs HARDWARE/TIMING-COUPLED

### (a) Pure math — port to Python, verify exactly
All of `motion_math.cpp` is explicitly "Arduino-free … compiled in the `native`
env so it is unit-tested on the host" (`motion_math.h:5`):
- `translateToServo()` — per-leg math→servo map. **Pure. Port verbatim.**
- `clampClipServos()` — hip [38,142], thigh [30,150], knee [0,180]. Pure.
- `applyInvert()` — pitch mirror `180-x`. Pure.
- `emaStep(prev,target,alpha)` = `alpha*prev + (1-alpha)*target`. Pure *math*,
  but its *effect* is timing-coupled (see below).
- `easeFraction()` — smoothstep `t*t*(3-2t)`. Pure given (elapsed,dur).
- `clipPoseAt()` — linear frame interpolation. Pure given elapsed_ms.
- `clipPlayerStep()` — clip lifecycle FSM. Pure given `now` (millis passed in).
- The gait per-leg geometry inside `tickGait/tickTrot/tickYawRotation` (sweep,
  lift, NEUTRAL offsets, fwdDir/yawDir, 2/3 scale) is pure arithmetic — portable.
- `NEUTRAL[]`, `GAITS[]` tables, config offsets/limits — constants, portable.

### (b) Hardware / timing-coupled — only approximable in sim
- `millis()` clock feeding `t` in every tick fn and `clipStartMs/returnStartMs`.
  Real wall-clock; sim must substitute a deterministic clock (see §5).
- `Servo::setServoAngle` → `map()` integer truncation → `pwm.setPWM` over I2C.
  Hardware-only; replace with direct joint-target in PyBullet.
- `uint16_t` storage of `servoAngle` (whole-degree quantisation). A sim using
  doubles will differ in the fractional bits unless it also truncates.
- **EMA smoothing on the clip path** (`CLIP_EMA_ALPHA = 0.75f`,
  `spinal_cord.cpp:18`): `clipSmoothed = 0.75*prev + 0.25*target` applied **once
  per loop iteration** in `tickClip` (`:435`). Because it is per-*tick* not
  per-*dt*, its lag depends on the loop rate. Seeded from frame 0 on `playClip`
  (`:411`). To match firmware exactly the sim must run the EMA at the same tick
  cadence as firmware, not at 240 Hz — otherwise the smoothing time-constant
  diverges. This is the single biggest exact-parity hazard on the clip path.
- **Input smoothing** in `update()` (`:136`): `activeX += (targetX-activeX)*0.1`
  per loop — same per-tick (not per-dt) lag dependence as the EMA.
- `Servo::tickEase()` (`servo.cpp:38`) uses `millis()` deltas for the 500 ms
  clip-return ease — wall-clock, timing-coupled (but deterministic if you feed a
  synthetic clock).
- **Deadman switch** in `update()` (`:128`): `millis()-lastCommandMs > 500`
  zeroes targets — wall-clock dependent.

### Summary for the port
Port `motion_math.cpp` and the three tick-fn geometry blocks verbatim in pure
Python, drive them from a synthetic millisecond clock, and you get exact gait
servo-angle parity. The clip path can be made exact **only** if you replicate
the firmware loop cadence for the EMA/ease steps; otherwise it is an
approximation.

---

## 5. THE TICK LOOP — rate, dt-dependence, vs sim 240 Hz

- `main.cpp:19` `loop()` calls `updateNetwork()`, `spinalCord.update()`,
  `diagnostics::tick/handleHttp()` — **as fast as the Arduino loop runs, with no
  fixed timestep, no delay** (only a 5 s serial heartbeat). Rate is whatever the
  ESP32 can sustain (network/I2C-bound, typically hundreds of Hz, jittery).
- Gait/clip phase is advanced by **wall-clock `millis()`**, NOT by counting
  ticks: `t = (millis() - gaitPhaseStartMs_)/1000`. So the *pose at a given
  wall-clock time is loop-rate-independent* — good. **But** two per-tick
  recurrences are loop-rate-*dependent*:
  1. input smoothing `active += (target-active)*0.1` (`update():136`),
  2. clip EMA `0.75*prev + 0.25*target` (`tickClip():435`).
  These converge faster the more ticks per second occur.
- **Sim runs a fixed 240 Hz step.** Parity implications:
  - *Gait geometry*: exact if the sim samples `tickGait` at the same wall-clock
    `t` values — feed `millis() = step * (1000/240)`. The phase math is closed-form
    in `t`, so 240 Hz sampling reproduces it exactly at those instants.
  - *Input smoothing & clip EMA*: **will NOT match** unless the sim ticks them at
    the firmware loop rate. At 240 fixed Hz the sim applies far more (or fewer)
    `*0.1` / `*0.75` steps per second than the real jittery ESP32 loop, so the
    transient lag differs. Steady-state (target held constant long enough)
    converges to the same value; transients diverge.
  - The whole-degree `uint16_t` quantisation also breaks exact equality unless
    mirrored.
- **Flag**: the dt-dependence is entirely in the two EMA-style smoothers and the
  `tickEase` 500 ms return. Everything keyed on `millis()` directly is
  rate-independent and safe to sample at 240 Hz.

---

## 6. STATE & MODES — the mode state machine

`RobotState` enum (`shared/data.h:18`):
```cpp
enum RobotState {
    STATE_IDLE=0, STATE_WALK=1, STATE_ACTION=2, STATE_FAILSAFE=3,
    STATE_REST=4,   // all servos 90, calibration flat pose
    STATE_STAND=5,  // per-leg NEUTRAL[] standing pose
};
```
Dispatch switch in `SpinalCord::update()` (`spinal_cord.cpp:140`):
```cpp
switch (robotState) {
    case STATE_WALK:     if (currentGait_ != GAIT_NONE) tickGait(); break;
    case STATE_ACTION:   tickClip(); break;
    case STATE_IDLE:
    case STATE_REST:
    case STATE_STAND:    break;   // static holds, no per-tick work
    case STATE_FAILSAFE: leg1..4.returnToDefaultAngles(); break;
}
```
So only WALK (gait) and ACTION (clip) are actively ticked; IDLE/REST/STAND are
inert holds; FAILSAFE continuously re-asserts the default pose.

### Transitions (driven by WebSocket commands, `brain/network.cpp:67`)
- `CMD_STATE` (T:2) `s` field → `rest()`→IDLE, `walk()`→WALK, `wallFlip()`→ACTION,
  `relax()`→REST, `stand()`→STAND (`network.cpp:79-92`). Out-of-range `s` ignored
  via `isValidStateCommand`.
- `CMD_MOVE` (T:1) → calls `walk()` (forces STATE_WALK) **then** `processCommand(dir)`
  which sets `targetX/Y/Yaw` and `isMovingRequested` (`network.cpp:115`,
  `spinal_cord.cpp:70`). Dir strings: FW/BW/L/R/FW_R/FW_L/BW_R/BW_L/STOP.
- `CMD_GAIT_MODE` (T:5) `g` → `setGait(g)` if changed; resets `gaitPhaseStartMs_`
  and `currentGait_` (`network.cpp:128`, `spinal_cord.cpp:160`). `g` validated
  GAIT_NONE..GAIT_CRAB.
- `CMD_PLAY_CLIP` (T:7) `c` → `playClip(c)`, which **sets robotState=STATE_ACTION,
  pre-empting any running gait** (`spinal_cord.cpp:416`, "single motion owner").
- `CMD_ACTION_SELECTION` (T:6) `a==INVERT_ROBOT(0)` → `invertRobot()` toggles
  `isInverted` and re-asserts NEUTRAL (`network.cpp:140`).
- `CMD_CALIBRATE` (T:4) → `applyCalibration(channel,angle)` — direct write, does
  NOT change robotState (`network.cpp:94`).
- `CMD_LIST_CLIPS` (T:8), `CMD_TELEMETRY` (T:10) — query/telemetry, no motion.
- **`CMD_POSE` (T:3) has NO handler** in network.cpp — declared in `data.h:8` and
  documented in API_SPEC §3 but unimplemented; `Leg::setPose` IK path is unreached.
- **Auto-transitions back to IDLE**: gait graceful-stop (`tickGait`/`tickTrot`
  when input released on a clean phase boundary, `spinal_cord.cpp:181`,`:274`)
  and clip completion (`CLIP_ACT_FINISH` → `robotState=STATE_IDLE`,
  `spinal_cord.cpp:457`).
- **Deadman**: `update()` zeroes targets if no command for 500 ms
  (`spinal_cord.cpp:128`) — note firmware uses 500 ms here, while API_SPEC §Safety
  documents a 2 s WALK→IDLE timeout; the code's 500 ms is the live value.

### Clip data format (for the port)
`FhClipFrame { uint16_t t_ms; float a[12]; }` (`clips_all.h:11`). `a[12]` is
**PRE-SCALED math-space joint degrees** (the 2/3 scale-from-NEUTRAL already
baked in), order FR,FL,RR,RL × (shoulder,thigh,knee). Firmware applies **only
translateToServo()** at runtime to clip frames (plus EMA + clampClipServos) — it
never re-scales. 6 clips compiled in (`FH_CLIP_COUNT 6`).
