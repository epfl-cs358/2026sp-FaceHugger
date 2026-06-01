# Servo conventions

This page documents the servo numbering, the URDF↔firmware translation, and the status of the long-standing alignment proposal between them. There are twelve servos (three per leg, four legs), and the names used to refer to them differ between Blender/Python, the URDF, and the firmware. Picking up the wrong table when authoring or baking a clip means every leg moves wrong on hardware, so the conventions are worth nailing down before anything is exported.

## What the firmware actually uses today

The firmware does **not** carry a flat `SERVO_CONFIG[]` array indexed by `servo_id 0..11`. It carries a `LEG_SERVO_CHANNEL[leg][joint]` 2D table in [`code/firmware/src/shared/config.h`](https://github.com/EPFL-FaceHugger/2026sp-FaceHugger/blob/main/code/firmware/src/shared/config.h), keyed by firmware `LegId` and a per-leg joint index. The actual channel assignments are:

```
LEG_SERVO_CHANNEL[4][3] = {
  {8,  9,  10},  // LEG_FR (0): hip=8, thigh=9, knee=10
  {12, 13, 14},  // LEG_FL (1): hip=12, thigh=13, knee=14
  {4,  5,  6},   // LEG_RR/BR (2): hip=4, thigh=5, knee=6
  {0,  1,  2},   // LEG_RL/BL (3): hip=0, thigh=1, knee=2
}
```

This is the canonical channel map; it is also reproduced in the [Servo Channels and Hardware](../conventions.md#servo-channels-and-hardware) section of the main conventions page.

The firmware leg order is `FR=0, FL=1, RR=2, RL=3`, and `RR/RL` ("rear") are the same physical legs the URDF and Python code call `BR/BL` ("back"). The translation table lives in [`kinematics.md`](kinematics.md):

| Firmware leg id | Firmware name | URDF id | Full name |
|:---:|:---:|:---:|---|
| 0 | `fr` | `fr` | front-right |
| 1 | `fl` | `fl` | front-left |
| 2 | `rr` | **`br`** | rear-right / back-right |
| 3 | `rl` | **`bl`** | rear-left / back-left |

Within a leg, the firmware indexes joints as `hip=0, thigh=1, knee=2` (`SERVO_HIP`, `SERVO_THIGH`, `SERVO_KNEE` in `config.h`). In URDF/Python terms that is `link1=shoulder/hip, link2=thigh, link3=knee` — same physical joints, slightly different vocabulary.

## The flat `servo_id` proposal (not yet adopted)

A separate convention was proposed for the Blender→firmware export path: a flat numbering of `servo_id = leg_index * 3 + joint_index`, with legs in URDF alphabetical order (`fl=0, fr=1, bl=2, br=3`). That gives twelve servos numbered linearly:

| URDF link | servo_id | Physical role |
|---|:---:|---|
| `fl_link1` | 0 | FL shoulder (yaw) |
| `fl_link2` | 1 | FL thigh (pitch) |
| `fl_link3` | 2 | FL knee (pitch) |
| `fr_link1` | 3 | FR shoulder |
| `fr_link2` | 4 | FR thigh |
| `fr_link3` | 5 | FR knee |
| `bl_link1` | 6 | BL shoulder |
| `bl_link2` | 7 | BL thigh |
| `bl_link3` | 8 | BL knee |
| `br_link1` | 9 | BR shoulder |
| `br_link2` | 10 | BR thigh |
| `br_link3` | 11 | BR knee |

The proposal prioritises URDF/Python readability — legs in alphabetic-cardinal order matching the convention documents and the URDF generator's leg list. It is **not** what the firmware currently uses: the firmware's `LEG_SERVO_CHANNEL` orders legs `FR, FL, RR(=BR), RL(=BL)`, and the channel numbers themselves are non-contiguous (`{8,9,10}`, `{12,13,14}`, `{4,5,6}`, `{0,1,2}`) because they reflect physical PCA9685 wiring rather than any nice formula.

This is exactly the kind of disagreement that has to be resolved before any `.gait`-style files are baked against a `servo_id` field. Until it is, treat `LEG_SERVO_CHANNEL` as authoritative.

## The bridge file that does not yet exist

Earlier design docs (and some lingering references in the code-side `docs/`) describe a `servo_mapping.yaml` file as the single point of agreement between Python and firmware: URDF link names on the left, firmware-side `servo_id` and `direction` on the right. As of writing **no such file exists in the repository** — neither does the `export_gait.py` script that would consume it. The whole "bridge role" pattern is design intent, not current state.

What does exist is the clip exporter in [`animation/addons/`](https://github.com/EPFL-FaceHugger/2026sp-FaceHugger/tree/main/animation/addons) (the FH Clip Panel), which writes `clips_all.h` in firmware `LegId` order (FR, FL, RR, RL × shoulder, thigh, knee). That is the current Blender→firmware channel — it bypasses the proposed `servo_id` indirection entirely by emitting joint angles in the order the firmware already expects.

## How `direction` would work (post-rig, design-intent)

When the rig-build pipeline lands, kinematic L/R asymmetry (URDF `<axis>0 -1 0</axis>` on FR/BL hip and knee versus `0 1 0` on FL/BR) will be absorbed into Blender bone roll via `EditBone.align_roll(joint.axis)`. A positive Blender `rotation_euler[2]` will then rotate the leg "up" in the same physical sense on both sides, with no per-bone `direction: -1` needed for kinematic mirroring.

`direction` is therefore intended as **hardware-calibration-only**: flip to `-1` only after the robot is assembled, when a specific servo is observed driving its joint in the wrong direction (typically because its horn was pressed on facing the opposite way from another unit). Default `direction: 1` everywhere; calibrate after assembly.

## How to align (when the time comes)

When the `.gait` / `.fhc` pipeline is built out far enough to need a flat `servo_id`, the alignment step is:

1. Decide whether the proposal flips to follow firmware ordering, or whether firmware adopts the proposal's ordering. The firmware side is the one with running code, so usually it wins.
2. Update the proposal table above (or replace it with the firmware-confirmed table) and any new bridge file (`servo_mapping.yaml` or equivalent) to match.
3. Regenerate any baked files (`.gait`, `.fhc`) against the finalized numbering.
4. Strip the "proposal" qualifier wherever it appears.

See [Roadmap: Servo numbering alignment](../roadmap.md#servo-numbering-alignment) for the open checklist.

## Related pages

- [Conventions](../conventions.md) — leg naming, joint axes, channel table, calibration model.
- [Kinematics conventions](kinematics.md) — leg naming convention authority, URDF source-of-truth rules.
- [Motion engine](motion-engine.md) — `translateToServo`, `applyInvert`, `clampClipServos`.
- [Roadmap](../roadmap.md) — servo numbering alignment as an open thread.
