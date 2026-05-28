# Conventions

Single reference for FaceHugger's coordinate systems, angle spaces, leg naming, and servo numbering. Authoritative for animation pipeline, firmware, and simulation work.

## Body Coordinate Frame

The body frame origin is the geometric center of the chassis (`base_link` in URDF), anchored to a named construction point in Fusion 360. The axes follow the Fusion 360 / Blender Z-up convention: **+X** points right, **+Y** points forward, **+Z** points up. Units are metres in simulation and millimetres in Blender scene and clip storage.

This frame governs URDF joint placement and Blender rig alignment. It is an authoring and URDF concept rather than a runtime one in current firmware, and it does not directly affect clip playback (clips store math-space joint angles, not foot positions).

```
        +Y (forward)
         ^
         |
    FL   |   FR
     \   |   /
      \  |  /
    ---\|/---  <- +X (right) --------->
      /  |  \
     /   |   \
    BL   |   BR
         |
    body center (base_link origin)
```

## Leg Naming and IDs

Naming is context-dependent. Use `fl/fr/bl/br` everywhere except inside firmware. The firmware boundary translates to `FR/FL/RR/RL` internally. Never propagate firmware leg names into Python or Blender code; translation happens only at the boundary (e.g., `spinal_cord.cpp` `NEUTRAL[]` array indexing).

| Position | Blender / URDF / Python | Firmware (`LegId` enum) | CAD / Old firmware |
|----------|--------------------------|-------------------------|-------------------|
| Front-left  | `fl` | `LEG_FL` = 1 | `fl` |
| Front-right | `fr` | `LEG_FR` = 0 | `fr` |
| Back-left   | `bl` | `LEG_RL` = 3 | `rl` (rear-left) |
| Back-right  | `br` | `LEG_RR` = 2 | `rr` (rear-right) |

Each leg has three joints, in order: **shoulder** (`_link1_joint`, yaw about Z, connects body to leg), **thigh** (`_link2_joint`, pitch about Y, upper leg), and **knee** (`_link3_joint`, pitch about Y, lower leg). Firmware servo arrays index them as `[shoulder=0, thigh=1, knee=2]` per leg.

## Angle Spaces

FaceHugger uses two distinct angle spaces. Confusing them is the most common source of bugs.

**Math-space** is the abstract joint-angle space shared by all four legs. Its zero is the URDF/CAD calibration pose: every URDF joint angle is zero and the pitch servos sit at their 90 deg mid-scale (legs extended, the flat "spread out" calibration position). It is **not** centered on the `NEUTRAL[]` standing pose. The same math-space value produces the same physical motion on all four legs when passed through `translateToServo()`. This is the space used by gaits, clips, URDF limits, and all animation tooling.

`NEUTRAL[]` is one specific pose expressed in this space, not its origin. Its values are nonzero (e.g., FR thigh = -60 deg), so a thigh angle of -60 deg is measured from the calibration zero, not "below NEUTRAL". Gaits and clips treat `NEUTRAL[]` as a reference to add deltas onto and to scale around (`value = NEUTRAL + (raw - NEUTRAL) * SCALE`); this only works because the raw angles are absolute math-space measured from the calibration zero.

**Servo-space** is the physical 0-180 deg angle written to each servo via the PCA9685 I2C driver. It is always positive, hardware-only, and never stored in clips or gaits. The firmware clamps it via `constrain(angle, 0, 180)` inside `Servo::setServoAngle()`, then maps to pulse width with `map(angle, 0, 180, MIN_PULSE, MAX_PULSE)` where `MIN_PULSE=150` and `MAX_PULSE=600` microseconds (`config.h`).

### Quick Reference: Which Space Am I In?

| Question | Answer | Space |
|----------|--------|-------|
| Is this value stored in a clip file? | Yes | Math-space joint angles in **`clips_all.h`** |
| Does this come from a gait parameter? | Yes | **Math-space** angle (degrees) |
| Does this get written to PCA9685? | Yes | **Servo-space** 0-180 deg |
| Is this from URDF `<limit>`? | Yes | **Math-space** relative to rest (radians) |
| Is this in Fusion CAD? | Yes | **CAD frame** (specific to Fusion origin) |
| Does this describe servo mounting? | Yes | **Firmware hardware** (PCA channel, calibration) |

### NEUTRAL Standing Pose

This is the default standing posture, defined in math-space and indexed by firmware `LegId`. It is distinct from the calibration zero (math-space origin) described above: the values below are the math-space coordinates of the standing pose, not zeros. Source of truth: `code/firmware/src/nervous_system/spinal_cord.cpp`, `NEUTRAL[]` array (lines 21-26).

| Leg | LegId | Name | Shoulder (deg) | Thigh (deg) | Knee (deg) | Physical pose |
|---|:---:|---|---|---|---|---|
| 0 | FR | front-right | +45 | -60 | -37 | leg splayed right-forward, knee bent |
| 1 | FL | front-left | +135 | -60 | -40 | leg splayed left-forward, knee bent |
| 2 | RR/BR | back-right | -45 | -50 | -50 | leg splayed right-back, knee bent |
| 3 | RL/BL | back-left | -135 | -60 | -35 | leg splayed left-back, knee bent |

Each leg's NEUTRAL shoulder equals that leg's **outward (flat-spread) direction** on the yaw circle: **FR +45°, FL +135°, BR −45°, BL −135°**. Since change B regularized FL (75° → 135°), *servo 90 now means "outward" for all four legs*. Every leg's standing shoulder math-angle maps to servo 90, and the all-servos-90 pose is the symmetric outward "X" (the flat calibration pose). The thigh/knee values are per-leg physical-calibration choices, tuned on hardware.

!!! warning "Change B hardware step pending"
    FL's 75° → 135° move is committed in code, but its **hardware step (re-mounting the FL shoulder horn so servo 90 points outward) is not yet done**, and the FL clips have not been re-exported. Until both happen, the *running* robot still expects the old servo-75 FL standing pose, so the symmetric "X" holds in code only. FR/BR/BL are unaffected.

Leg positions at NEUTRAL (top view, front at top):

```
   +135 deg (FL)      +45 deg (FR)
        \\               /
            FaceHugger
        /               \\
   -135 deg (BL)      -45 deg (BR)
```

## The Math-to-Servo Transform: `translateToServo()`

`translateToServo()` converts math-space joint angles to servo-space 0-180 deg. It is not IK and not kinematics; it is purely a mounting remap that encodes which way each servo horn faces. The firmware runs this transform every tick in both gait and clip players.

Source of truth: `code/firmware/src/nervous_system/motion_math.cpp`, `translateToServo()` (lines 22-50). The Blender exporter carries a byte-identical twin, `_frame_to_servo` (`fh_clip_panel.py`), kept in lockstep by `test_servo_parity.py`.

Input: math-space angles `sh` (shoulder), `th` (thigh), `kn` (knee).
Output: servo-space angles for hip, thigh, and knee servos.

| Leg | hip_servo | thigh_servo | knee_servo |
|-----|-----------|-------------|------------|
| FR (LegId=0) | `90 + (sh - 45)` | `90 - th` | `90 + kn` |
| FL (LegId=1) | `90 + (sh - 135)` | `90 + th` | `90 - kn` |
| RR/BR (LegId=2) | `90 + (sh + 45)` | `90 + th` | `90 - kn` |
| RL/BL (LegId=3) | `90 + (sh + 135)` | `90 - th` | `90 + kn` |

The shoulder offsets (−45, −135, +45, +135) just *centre* each leg's outward direction on servo 90. All four are `90 + (sh ± offset)` with a **+1 slope**: a positive `sh` (CCW yaw) drives every shoulder servo up. FL's historic `hip = sh` special case is gone (change B), and BR's old `90 − (sh + 45)` mirror was removed when its shoulder was un-mirrored, 2026-05-25, since all four shoulder shafts share one vertical axis. The **thigh/knee** signs, by contrast, *do* mirror on the {FL,BR} ↔ {FR,BL} diagonal because those servo horns face opposite ways:

| leg | shoulder | thigh | knee |
|-----|:--------:|:-----:|:----:|
| FL  | +1 | +1 | −1 |
| FR  | +1 | −1 | +1 |
| BL  | +1 | −1 | +1 |
| BR  | +1 | +1 | −1 |

All transformations are deterministic and invertible.

Example (FR at NEUTRAL): math `sh=45, th=-60, kn=-37` -> servo `hip=90+(45-45)=90, thigh=90-(-60)=150, knee=90+(-37)=53`.
Example (FL at NEUTRAL, post-B): math `sh=135, th=-60, kn=-40` -> servo `hip=90+(135-135)=90, thigh=90+(-60)=30, knee=90-(-40)=130`.

The table above is written with `90`s for readability. In the live firmware, the thigh and knee `90` is replaced by the per-joint `CALIB_*_THIGH` / `CALIB_*_KNEE` from `config.h`. See [Invert mirror and CALIB](#invert-mirror-and-calib).

## Invert mirror and CALIB

Why per-joint, not constant 90: a spline tooth is a few degrees wide, so when a thigh or knee horn is pressed on at calibration time, "mechanical flat" rarely lands exactly on servo 90. It lands on servo 84, or 95, or 103 (the current per-leg values, measured on real hardware on 2026-05-28). The firmware records those numbers in `CALIB_FR_THIGH`, `CALIB_FR_KNEE`, ... `CALIB_BL_KNEE` (`code/firmware/src/shared/config.h`) and uses them as the per-joint zero of math-space. `translateToServo` adds and subtracts math-space deltas from `CALIB`, not from `90`, so a single horn that's two teeth off no longer biases every gait and clip on that leg.

The upside-down pitch mirror has to follow the same rule. When `isInverted` is set, `applyInvert` (`motion_math.cpp`) mirrors each pitch joint **about its CALIB**:

```cpp
s.thigh = 2.0 * CALIB_THIGH_BY_LEG[legId] - s.thigh;
s.knee  = 2.0 * CALIB_KNEE_BY_LEG[legId]  - s.knee;
```

This is exactly math-space negation in servo-space: `translateToServo` emits `CALIB ± delta`, and `2*CALIB - (CALIB ± delta) = CALIB ∓ delta`, the negated-delta pose. Pre-calibration the mirror was `180 - angle` because flat was assumed to be servo 90 uniformly; the new form reduces to `180 - angle` exactly when `CALIB == 90`, so a fully-uncalibrated robot still mirrors as before.

The shoulder is not in this mirror. Inverting flips pitch, not yaw, and the shoulder stays at whatever servo angle the gait or clip is currently asking for.

## Joint Axes and URDF Conventions

Joint origins are placed at the physical rotation axis (servo shaft), following the URDF convention (each joint frame sits on its rotation axis, with the child link defined relative to it). URDF is used purely as a file format here, for PyBullet and Blender; the project does not run ROS.

The **shoulder (yaw) joint** uses `+Z` (vertical up) as its axis uniformly across all four legs. At rest (theta=0 in URDF), each leg's shoulder points in its mechanical zero direction. Positive rotation is CCW viewed from above (right-hand rule along +Z).

The **thigh and knee (pitch) joints** use an axis along the leg's longitudinal direction at rest. L-side legs (FL, BL) use `+Y` in the body frame; R-side legs (FR, BR) use `-Y` (mirrored mounting). This axis flip means the same positive theta lifts the foot toward the chassis on every leg. URDF limits are expressed as signed bounds in radians relative to rest; R-side limits are negated and swapped to account for the axis flip.

## Servo Channels and Hardware

The PCA9685 driver controls 16 PWM channels at 60 Hz. Twelve are used (3 joints x 4 legs).

Channel assignments are the live hardware configuration. Source of truth: `code/firmware/src/shared/config.h`.

```
LEG_SERVO_CHANNEL[4][3] = {
  {8,  9,  10},  // LEG_FR (0): hip=8, thigh=9, knee=10
  {12, 13, 14},  // LEG_FL (1): hip=12, thigh=13, knee=14
  {4,  5,  6},   // LEG_RR/BR (2): hip=4, thigh=5, knee=6
  {0,  1,  2},   // LEG_RL/BL (3): hip=0, thigh=1, knee=2
}
```

A proposed servo ID scheme (`animation/SERVO_ID_CONVENTION.md`) uses `servo_id = leg_idx * 3 + joint_idx` with legs in alphabetical order (`fl:0, fr:1, bl:2, br:3`). **STATUS: PROPOSAL** pending firmware sign-off. Until `SERVO_CONFIG[]` is finalized, the channel table above is authoritative, and the animation exporter's `servo_mapping.yaml` must match it exactly.

## SCALE Factor

The SCALE factor (2/3 = 0.6667) is applied to clip amplitudes at bake time in the Blender exporter when `clips_all.h` is generated. Firmware never re-scales clip data; the scale is already baked into stored keyframe values.

```
scaled_value = NEUTRAL + (raw_value - NEUTRAL) * SCALE
```

SCALE applies to clips only. Gait engine parameters (`step_length_deg`, `step_height_deg`, offsets) are firmware-side live parameters and are unaffected. The purpose is to preserve hardware headroom (avoid torque spikes) and produce natural movement on the leg mechanism.

## Clip and Gait Terminology

A **clip** is a canned, one-shot authored gesture (e.g., "wiggle", "bow", "jump"). It runs once on its own timeline via `tickClip()` on `STATE_ACTION`, then returns to NEUTRAL over 500 ms before transitioning to `STATE_IDLE`. Clips are stored in `clips_all.h` as flat arrays of 12 pre-scaled math-space angles per frame, generated by the Blender exporter. Do not call clips "actions" (overloaded with FSM terminology) or "animations" (ambiguous).

A **gait** is a looping locomotion pattern (walk, trot, crab, crawl). It runs continuously via `tickGait()` on `STATE_WALK` and is parameterized by step_length, step_height, period, duty cycle, and phase offsets. Gaits are firmware-driven live engines in `spinal_cord.cpp`, not pre-baked data.

## When Conventions Change

The following coordinate contracts invalidate all stored clips if changed:

1. `base_link` origin position in Fusion (must stay anchored to named construction point)
2. Per-leg shoulder mount positions (URDF `leg_*_yaw` `<origin>`)
3. Joint axis directions (negating an axis flips all stored angles)
4. Link lengths (changes reachable workspace)

When any of these must change, regenerate `clips_all.h` from source `.blend` files with the updated URDF, then rebuild and reflash firmware. Changes that are safe without re-baking include visual mesh origins, collision geometry, inertial values, and joint limits (old clips within the old range remain reachable).

## Related Documents

- `code/firmware/src/nervous_system/motion_math.cpp`: `translateToServo()` source
- `code/firmware/src/nervous_system/spinal_cord.cpp`: `NEUTRAL[]` array
- `code/firmware/src/shared/config.h`: PCA9685 channel assignments and pulse range
- `code/simulation/docs/MERGE_AND_CONVENTION.md`: the rest-pose definition and per-leg shoulder derivation
- `doc/animation-pipeline/urdf-conventions.md`: joint origins, axis vectors, axis flip rationale
- `animation/SERVO_ID_CONVENTION.md`: servo ID proposal
- `code/API_SPEC.md`: WebSocket protocol (gait selection, clip playback, body pose)
