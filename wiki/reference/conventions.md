# Conventions

Single reference for FaceHugger's coordinate systems, angle spaces, leg naming, and servo numbering. Authoritative for animation pipeline, firmware, and simulation work.

---

## Body Coordinate Frame

The body frame origin is the geometric center of the chassis (`base_link` in URDF), anchored to a named construction point in Fusion 360. The axes follow the Fusion 360 / Blender Z-up convention: **+X** points right, **+Y** points forward, **+Z** points up. Units are metres in simulation and millimetres in Blender scene and clip storage.

This frame governs URDF joint placement and Blender rig alignment. It is an authoring/URDF concept - not a runtime concept in current firmware - and does not directly affect clip playback (clips store math-space joint angles, not foot positions).

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

---

## Leg Naming and IDs

Naming is context-dependent. Use `fl/fr/bl/br` everywhere except inside firmware. The firmware boundary translates to `FR/FL/RR/RL` internally; never propagate firmware leg names into Python or Blender code - translation happens only at the boundary (e.g., `spinal_cord.cpp` `NEUTRAL[]` array indexing).

| Position | Blender / URDF / Python | Firmware (`LegId` enum) | CAD / Old firmware |
|----------|--------------------------|-------------------------|-------------------|
| Front-left  | `fl` | `LEG_FL` = 1 | `fl` |
| Front-right | `fr` | `LEG_FR` = 0 | `fr` |
| Back-left   | `bl` | `LEG_RL` = 3 | `rl` (rear-left) |
| Back-right  | `br` | `LEG_RR` = 2 | `rr` (rear-right) |

Each leg has three joints, in order: **shoulder** (`_link1_joint`, yaw about Z, connects body to leg), **thigh** (`_link2_joint`, pitch about Y, upper leg), and **knee** (`_link3_joint`, pitch about Y, lower leg). Firmware servo arrays index them as `[shoulder=0, thigh=1, knee=2]` per leg.

---

## Angle Spaces

FaceHugger uses two distinct angle spaces. Confusing them is the most common source of bugs.

**Math-space** is the abstract joint angle centered on the `NEUTRAL[]` rest pose. Values can be negative (e.g., thigh = -60 deg means the leg drops below rest). The same math-space value produces the same physical motion on all four legs when passed through `translateToServo()`. This is the space used by gaits, clips, URDF limits, and all animation tooling.

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

### NEUTRAL Rest Pose

The rest pose is defined in math-space and indexed by firmware `LegId`. Source of truth: `code/firmware/src/nervous_system/spinal_cord.cpp`, `NEUTRAL[]` array (lines 17-22), physically tested on hardware.

| Leg | LegId | Name | Shoulder (deg) | Thigh (deg) | Knee (deg) | Physical pose |
|---|:---:|---|---|---|---|---|
| 0 | FR | front-right | +45 | -60 | -37 | leg splayed right-forward, knee bent |
| 1 | FL | front-left | +75 | -60 | -40 | leg splayed left-forward, knee bent |
| 2 | RR/BR | back-right | -45 | -50 | -50 | leg splayed right-back, knee bent |
| 3 | RL/BL | back-left | -135 | -60 | -35 | leg splayed left-back, knee bent |

Each leg's shoulder rest is derived from the FL value:

```
FR_shoulder = -FL_shoulder
BL_shoulder = -wrap_pi(FL_shoulder + pi)
BR_shoulder = +wrap_pi(FL_shoulder + pi)
```

Leg positions at NEUTRAL (top view):

```
    45 deg (FR)     75 deg (FL)
      ->              <-
        FaceHugger
      <-              ->
   -45 deg (BR)   -135 deg (BL)
```

---

## The Math-to-Servo Transform: `translateToServo()`

`translateToServo()` converts math-space joint angles to servo-space 0-180 deg. It is not IK and not kinematics - it is purely a mounting remap that encodes which way each servo horn faces. The firmware runs this transform every tick in both gait and clip players.

Source of truth: `code/firmware/src/nervous_system/motion_math.cpp`, lines 4-31.

Input: math-space angles `sh` (shoulder), `th` (thigh), `kn` (knee).
Output: servo-space angles for hip, thigh, and knee servos.

| Leg | hip_servo | thigh_servo | knee_servo |
|-----|-----------|-------------|------------|
| FR (LegId=0) | `90 + (sh - 45)` | `90 - th` | `90 + kn` |
| FL (LegId=1) | `sh` | `90 + th` | `90 - kn` |
| RR/BR (LegId=2) | `90 - (sh + 45)` | `90 + th` | `90 - kn` |
| RL/BL (LegId=3) | `90 + (sh + 135)` | `90 - th` | `90 + kn` |

The shoulder offsets (+45, -45, etc.) encode each leg's mounting orientation. Hip and knee signs differ between L-side and R-side legs because servo horns face opposite directions. All transformations are deterministic and invertible.

Example (FR at NEUTRAL): math `sh=45, th=-60, kn=-37` -> servo `hip=90+(45-45)=90, thigh=90-(-60)=150, knee=90+(-37)=53`.

---

## Joint Axes and URDF Conventions

Joint origins are placed at the physical rotation axis (servo shaft), following ROS convention.

The **shoulder (yaw) joint** uses `+Z` (vertical up) as its axis uniformly across all four legs (Convention A). At rest (theta=0 in URDF), each leg's shoulder points in its mechanical zero direction. Positive rotation is CCW viewed from above (right-hand rule along +Z).

The **thigh and knee (pitch) joints** use an axis along the leg's longitudinal direction at rest. L-side legs (FL, BL) use `+Y` in the body frame; R-side legs (FR, BR) use `-Y` (mirrored mounting). This axis flip means the same positive theta lifts the foot toward the chassis on every leg. URDF limits are expressed as signed bounds in radians relative to rest; R-side limits are negated and swapped to account for the axis flip.

---

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

---

## SCALE Factor

The SCALE factor (2/3 = 0.6667) is applied to clip amplitudes at bake time in the Blender exporter when `clips_all.h` is generated. Firmware never re-scales clip data; the scale is already baked into stored keyframe values.

```
scaled_value = NEUTRAL + (raw_value - NEUTRAL) * SCALE
```

SCALE applies to clips only. Gait engine parameters (`step_length_deg`, `step_height_deg`, offsets) are firmware-side live parameters and are unaffected. The purpose is to preserve hardware headroom (avoid torque spikes) and produce natural movement on the leg mechanism.

---

## Clip and Gait Terminology

A **clip** is a canned, one-shot authored gesture (e.g., "wiggle", "bow", "jump"). It runs once on its own timeline via `tickClip()` on `STATE_ACTION`, then returns to NEUTRAL over 500 ms before transitioning to `STATE_IDLE`. Clips are stored in `clips_all.h` as flat arrays of 12 pre-scaled math-space angles per frame, generated by the Blender exporter. Do not call clips "actions" (overloaded with FSM terminology) or "animations" (ambiguous).

A **gait** is a looping locomotion pattern (walk, trot, crab, crawl). It runs continuously via `tickGait()` on `STATE_WALK` and is parameterized by step_length, step_height, period, duty cycle, and phase offsets. Gaits are firmware-driven live engines in `spinal_cord.cpp`, not pre-baked data.

---

## When Conventions Change

The following coordinate contracts invalidate all stored clips if changed:

1. `base_link` origin position in Fusion (must stay anchored to named construction point)
2. Per-leg shoulder mount positions (URDF `leg_*_yaw` `<origin>`)
3. Joint axis directions (negating an axis flips all stored angles)
4. Link lengths (changes reachable workspace)

When any of these must change, regenerate `clips_all.h` from source `.blend` files with the updated URDF, then rebuild and reflash firmware. Changes that are safe without re-baking include visual mesh origins, collision geometry, inertial values, and joint limits (old clips within the old range remain reachable).

---

## Related Documents

- `code/firmware/src/nervous_system/motion_math.cpp` - `translateToServo()` source
- `code/firmware/src/nervous_system/spinal_cord.cpp` - `NEUTRAL[]` array
- `code/firmware/src/shared/config.h` - PCA9685 channel assignments and pulse range
- `code/simulation/docs/MERGE_AND_CONVENTION.md` - Convention A rationale and per-leg shoulder derivation
- `doc/animation-pipeline/urdf-conventions.md` - joint origins, axis vectors, axis flip rationale
- `animation/SERVO_ID_CONVENTION.md` - servo ID proposal
- `code/API_SPEC.md` - WebSocket protocol (gait selection, clip playback, body pose)
