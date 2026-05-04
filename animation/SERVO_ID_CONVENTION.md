# Servo ID Convention

> **STATUS: PROPOSAL** — pending firmware confirmation against
> `SERVO_CONFIG[]` ordering in `code/firmware/...`. Once the firmware
> author confirms (or specifies a different ordering), the `.gait`
> exporter's `servo_mapping.yaml` must match whatever the firmware
> uses — there is exactly one source of truth for servo numbering, and
> it lives on the firmware side.

This doc exists so teammates can align on the per-servo numbering
**before** firmware `SERVO_CONFIG[]` is finalized and before any
`.gait` files are baked. Bake the gait file against the wrong numbering
and every leg moves wrong on the physical robot — easy to misdiagnose,
easy to avoid by agreeing on numbering up front.

## Proposed numbering

Twelve servos, ordered by leg (FL → FR → BL → BR) then by joint
(link1 → link2 → link3):

| Bone (URDF link name) | servo_id | Physical role |
|---|:---:|---|
| `fl_link1` | 0 | FL shoulder (yaw) |
| `fl_link2` | 1 | FL hip (pitch) |
| `fl_link3` | 2 | FL knee (pitch) |
| `fr_link1` | 3 | FR shoulder |
| `fr_link2` | 4 | FR hip |
| `fr_link3` | 5 | FR knee |
| `bl_link1` | 6 | BL shoulder |
| `bl_link2` | 7 | BL hip |
| `bl_link3` | 8 | BL knee |
| `br_link1` | 9 | BR shoulder |
| `br_link2` | 10 | BR hip |
| `br_link3` | 11 | BR knee |

Mnemonic: `servo_id = leg_index * 3 + joint_index`, where
`leg_index ∈ {fl: 0, fr: 1, bl: 2, br: 3}` and
`joint_index ∈ {link1: 0, link2: 1, link3: 2}`.

## Firmware leg ID translation

The firmware on `main` numbers legs `0..3` and uses `rl/rr` ("rear")
where the URDF/Python code uses `bl/br` ("back"). Translation table
(authoritative copy in
[`code/simulation/docs/MERGE_AND_CONVENTION.md` §5](../code/simulation/docs/MERGE_AND_CONVENTION.md)):

| Firmware leg id | Firmware name | URDF id | Full name |
|:---:|:---:|:---:|---|
| 0 | `fr` | `fr` | front-right |
| 1 | `fl` | `fl` | front-left |
| 2 | `rr` | **`br`** | rear-right / back-right |
| 3 | `rl` | **`bl`** | rear-left / back-left |

Note this means the **firmware's leg ordering disagrees with the
proposed servo_id ordering above**. The proposed numbering uses
`fl→fr→bl→br`; the firmware uses `fr→fl→rr→rl` (firmware's per-leg
servo IDs would be e.g. `fr_link1=0, fl_link1=3, rr_link1=6,
rl_link1=9`). This is *exactly* the kind of disagreement that has to
be resolved before any gait files are baked.

**The proposal above prioritizes URDF/Python readability** ("legs in
alphabetic-cardinal order, which matches the convention documents and
the URDF generator's leg list"). If the firmware's `SERVO_CONFIG[]`
already commits to its own ordering, the proposal flips to match the
firmware — see "How to align" below.

## Bridge role: `servo_mapping.yaml`

`servo_mapping.yaml` is the URDF↔firmware translator:

- **Keys** (left side, e.g. `fl_link1`) are URDF link names. Lowercase,
  numbered. Don't propagate firmware numbering into Python/Blender code.
- **Values** (right side, including `servo_id`) are the firmware's
  per-PWM-channel index. Whatever the firmware's `SERVO_CONFIG[]` array
  uses, the yaml mirrors it.

The export script (`export_gait.py`) reads each bone's pose-mode
`rotation_euler[2]`, looks up its `servo_id` via the yaml, and writes
that ID into the `.gait` track. The firmware then reads the `.gait`
track ID, indexes into `SERVO_CONFIG[]`, and drives the right PWM
channel.

If the URDF link names ever change, the yaml keys change. If the
firmware reorders its servos, only the yaml values change. The Python
side and the C side never need to know each other's numbering — the
yaml is the only place they meet.

## `direction` field is hardware-calibration-only

`servo_mapping.yaml` also carries a `direction` field per servo
(`+1` or `-1`). With the URDF pipeline's `EditBone.align_roll(joint.axis)`
approach, the **kinematic L/R asymmetry** (URDF `<axis>0 -1 0</axis>`
for FR/BL hip and knee joints, vs `0 1 0` for FL/BR) is **already
absorbed into the bone roll at rig-build time**. So a positive Blender
`rotation_euler[2]` rotates the leg "up" in the same physical sense
on both sides — no per-bone `direction: -1` needed for kinematic
mirroring.

`direction` is therefore **hardware-calibration-only**: flip to `-1`
only after assembly, when a specific servo is observed driving its
joint in the wrong direction (typically because the servo horn on that
unit is mounted facing the opposite way from another unit).

**Default `direction: 1` everywhere.** Calibrate after assembly.

## How to align with firmware

1. Firmware author confirms or specifies `SERVO_CONFIG[]` ordering
   (which `servo_id` corresponds to which physical joint).
2. If different from the proposal, update both:
   - This document's proposal table → matches firmware
   - `servo_mapping.yaml` `servo_id:` values → matches firmware
3. Strip the **STATUS: PROPOSAL** banner above.
4. Mark this document as the source of truth for any future re-export
   or new `.gait` baking.

## Cross-references

- [`code/simulation/docs/MERGE_AND_CONVENTION.md`](../code/simulation/docs/MERGE_AND_CONVENTION.md)
  — leg naming convention (§0, §5), URDF source-of-truth rules
- [`doc/gait-design/specs/blender-rig-and-export.md`](../doc/gait-design/specs/blender-rig-and-export.md)
  — `servo_mapping.yaml` schema and the rig-build process that uses it
- [`doc/gait-design/specs/gait-file-format.md`](../doc/gait-design/specs/gait-file-format.md)
  — the `.gait` JSON format, where `servo_id` lands
- [`doc/gait-design/specs/esp32-playback-engine.md`](../doc/gait-design/specs/esp32-playback-engine.md)
  — firmware-side consumer of `servo_id`
