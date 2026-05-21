<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/gait-design/specs/blender-rig-and-export.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/animation/blender-rig.md
-->

> **Reference material.** Verbatim copy of `doc/gait-design/specs/blender-rig-and-export.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/gait-design/specs/blender-rig-and-export.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
# Spec: Blender Rig & Export Workflow

> Covers: Fusion 360 import, armature rigging, IK setup, animation, baking, and the `export_gait.py` script.
> See also: `gait-file-format.md` (what gets exported), `esp32-playback-engine.md` (what consumes it).

---

## 1. Fusion 360 → Blender Import

> **In the automated pipeline this is handled by
> [`animation/scripts/urdf_to_blender_rigged.py`](../../../animation/scripts/urdf_to_blender_rigged.py)**:
> the URDF generator emits one STL per rigid body
> (`code/simulation/generated/exported_meshes/`), the rigged importer
> walks the URDF chain to position each mesh in Blender at the link
> frame, and the rest of this section is automatic. The manual workflow
> below is only relevant when working outside the pipeline.

### Export from Fusion 360 (manual, fallback only)

- Export each **rigid body part** as a separate STL (not one monolithic mesh)
  - Each link segment (upper leg, lower leg, foot)
  - Each servo body
  - The body/chassis
- Use millimeters as the unit in Fusion 360 (Blender will import at 1 unit = 1mm, scale if needed)

### Import into Blender (manual, fallback only)

- File → Import → STL
- Import each part individually
- Position each mesh at its correct location (should match Fusion 360 assembly)
- Group related parts (e.g. FL leg parts) into collections for organization

---

## 2. Armature Setup

### Bone placement

Create an Armature object with **12 leg bones + 1 body bone** — one per
URDF link, named to match the URDF link names exactly (lowercase,
`fl_link1` etc.):

```
Armature
├── base_link              (root, fixed — represents the chassis)
├── fl_link1               (FL shoulder; head = BodyToLink1Point, tail = Link1ToLink2Point)
│   └── fl_link2           (FL hip;      head = Link1ToLink2Point, tail = Link2ToLink3Point)
│       └── fl_link3       (FL knee;     head = Link2ToLink3Point, tail = FootTip)
├── fr_link1
│   └── fr_link2
│       └── fr_link3
├── bl_link1
│   └── bl_link2
│       └── bl_link3
└── br_link1
    └── br_link2
        └── br_link3
```

The bone names match the URDF link names exactly so the export script can
look up `pose.bones["fl_link1"]` without a translation table. Conventional
labels like "shoulder/hip/knee" appear only in comments / debug output.

Each bone's **head** sits at its parent joint's pivot (= the URDF link
frame origin, computed by walking the joint chain at rest pose). The
**tail** sits at the next joint's pivot (or the foot tip for `*_link3`
bones, taken from the URDF comment-block landmark `FootTip` in link3 frame).

### Rotation axes

After `EditBone.align_roll(joint.axis)` is applied at rig-build time,
**bone-local Z is the joint axis for all 12 joints uniformly** —
regardless of whether the URDF axis is `+Z` (shoulder) or `±Y` (hip/knee)
in world frame. The per-side ±Y sign flip on hip/knee (URDF
`<axis>0 -1 0</axis>` for FR/BL, `0 1 0` for FL/BR) is absorbed into the
bone roll automatically.

This claim only holds because
[`_bone_endpoints_world_mm`](../../../animation/scripts/urdf_to_blender_rigged.py)
projects each bone's tail onto the plane perpendicular to that bone's
joint axis at rig-build time. `align_roll(target)` is exact only when
`target ⊥ bone-Y`; otherwise it projects internally and bone-local Z
drifts from the URDF axis (≈13° on `*_link1`, where the joint origin
has a Z lift; ≈23° on `*_link3`, where the foot tip has a Y component).
The tail projection forces bone-Y ⊥ joint axis on every bone, making
`align_roll` exact and the `rotation_euler[2] == joint angle` contract
literal. The bone becomes a rotation control whose Y is not along the
limb on `*_link1` / `*_link3`; mesh geometry is set independently via
`matrix_world` in `attach_visuals` and is unaffected.

| Joint type | URDF `<axis>` | Bone-local axis after `align_roll` | Blender channel |
|---|---|---|---|
| All 12 (shoulder + hip + knee) | per URDF | local Z | `rotation_euler[2]` |

Set all bones to use **XYZ Euler** rotation mode (not quaternion — single-axis
readability still matters even though only one channel is used). The export
script reads `rotation_euler[2]` for every servo, no per-joint branching.

### Joint limits

Joint limits are read from the URDF `<limit lower upper>` field per joint
**at rig-build time**, not hardcoded in this spec or in `servo_mapping.yaml`.
The URDF is regenerated whenever the Fusion joint motion is retuned, so any
hardcoded numeric limits drift the moment the CAD changes.

The `LIMIT_ROTATION` constraint on each bone uses `use_limit_z = True` and
`min_z` / `max_z` populated from URDF `<limit>` (radians). Per-axis branching
is unnecessary because of the uniform `align_roll` convention above.

Current values (informational only — may drift):

| Joint | Range (relative to per-leg rest) |
|---|---|
| Shoulder (link1) | [-90°, +90°] — see [`MERGE_AND_CONVENTION.md` §0](../../../code/simulation/docs/MERGE_AND_CONVENTION.md) |
| Hip (link2) | [-90°, +90°] |
| Knee (link3) | [-90°, +90°] |

The `min_deg` / `max_deg` fields in `servo_mapping.yaml` are a **separate**,
post-conversion clamp — they're servo-side PWM safety bounds, not kinematic
limits. The Blender rig itself is bound by the URDF limits; the yaml clamps
catch numerical excursions during export → PWM conversion.

### Mesh parenting

- Select each mesh part → select the armature → Ctrl+P → Bone (relative)
- Each rigid body follows its parent bone
- No automatic weights — these are rigid parts, not deformable mesh

---

## 3. IK Setup (Recommended for Animation)

### Per-leg IK chain

For each leg, add an IK constraint to the knee bone:

1. Select knee bone in Pose Mode
2. Add Constraint → Inverse Kinematics
3. Set **Target** to an Empty object placed at the foot position
4. Set **Chain Length** to 2 (knee + hip, shoulder is independent)
5. Create one Empty per leg: `fl_foot_target`, `fr_foot_target`, `bl_foot_target`, `br_foot_target`.

### Shoulder stays FK

The shoulder (yaw) is animated directly with Forward Kinematics — you rotate it manually. IK only controls the hip-knee chain (the 2-DOF pitch plane).

### Why IK for animation

Animating foot positions is more intuitive than animating joint angles:
- Place foot targets on the ground → IK solves hip/knee angles
- Easier to ensure feet don't clip through the floor
- Walk cycles are naturally defined as foot trajectories

### Critical: Bake before export

IK-solved angles are NOT stored in FCurves. Before running the export script:

```
Pose → Animation → Bake Action
  ☑ Visual Keying (evaluates IK solve)
  ☑ Clear Constraints (optional — removes IK after baking)
  ☑ Overwrite Current Action (or create new)
  Frame range: match your animation
```

Or via Python:
```python
bpy.ops.nla.bake(
    frame_start=bpy.context.scene.frame_start,
    frame_end=bpy.context.scene.frame_end,
    only_selected=False,
    bake_types={'POSE'},
    visual_keying=True,
    clear_constraints=False,
    use_current_action=True
)
```

After baking, every bone has explicit keyframes at every frame. The export script reads these.

---

## 4. Neutral Pose

The neutral pose lives across **three coupled layers** — they describe
the SAME physical state in three vocabularies. They MUST all agree, and
the URDF is the single source of truth that ties them together.

| Layer | Value at neutral |
|---|---|
| Blender bones | every bone's `rotation_euler = (0, 0, 0)` |
| URDF joint angles | every joint at θ=0 (= the Fusion rest pose) |
| Servo PWM | every servo at `offset_deg` (typically 135° = mid of 0–270°) |

### Geometric appearance

The neutral pose is **not** "all bones zeroed and the robot standing
straight along the body axes." It's the splayed standing stance from
Convention A:

- FL shoulder yaw = -45° (world frame)
- FR shoulder yaw = +45°
- BL shoulder yaw = -135°
- BR shoulder yaw = +135°
- All hips and knees at 0°

The per-leg shoulder offset is **baked into the URDF** as
`<joint><origin rpy="0 0 {rest_rad}"/>` per Convention A — so it does
NOT appear as a non-zero rotation value in Blender. Blender's rest pose
**is** the URDF θ=0 **is** the splayed stance — three views of the same
configuration.

If a reader is tempted to set Blender bones to 135° to "match the servo
neutral," that's a misreading of the layered relationship. The 135° is
the servo PWM angle, not the Blender bone angle. The conversion is
`servo_deg = offset_deg + direction * degrees(blender_rad)`; with
Blender at 0 rad and `offset_deg = 135`, the servo PWM lands at 135°.

### Calibration

1. Power on ESP32, command all servos to `offset_deg` (typically 135°).
2. Physically assemble the robot while servos hold that position. The
   assembly geometry now defines the splayed stance.
3. The Blender rig's rest pose (built at `rotation_euler = 0` everywhere)
   must match this assembly. If the URDF and the assembly disagree,
   regenerate the URDF from Fusion — that's the source of truth.
4. Per-servo direction/offset tweaks happen via `direction` and
   `offset_deg` in `servo_mapping.yaml`. **The kinematic L/R asymmetry
   is NOT calibrated here** — it's already absorbed at rig-build time
   by `EditBone.align_roll(joint.axis)` reading the URDF `<axis>`.
   `direction: -1` should only ever be needed for physical servo-horn
   handedness mismatches discovered after assembly.

---

## 5. Animation Workflow

### One Action per gait

- Create a new Action for each gait: `walk_forward`, `turn_left`, etc.
- Use the Action Editor (Dope Sheet → Action Editor) to manage multiple gaits
- Each Action is a self-contained, loopable clip

### Animating a walk cycle

1. Set frame range (e.g. 1–24 at 30fps = 800ms cycle)
2. Frame 1: neutral pose (keyframe all bones)
3. Animate foot targets through the gait pattern
4. Last frame: return to neutral pose (must match frame 1 exactly)
5. Preview the loop: set playback to "repeat" in the Timeline

### Tips

- Use the Graph Editor to check F-Curve smoothness
- Shoulder yaw is animated separately from hip/knee IK
- Keep cycles short: 0.5–2 seconds typically
- A trot gait has diagonal legs in phase: FL+BR move together, FR+BL move together, 50% phase offset

---

## 6. Servo Mapping Config

File: `servo_mapping.yaml` — the **URDF↔firmware bridge**. Each entry
maps a URDF link name (= Blender bone name, lowercase: `fl_link1`, etc.)
to firmware-side per-servo configuration.

See [`animation/SERVO_ID_CONVENTION.md`](../../../animation/SERVO_ID_CONVENTION.md)
for the proposed servo numbering and how it aligns with the firmware's
`SERVO_CONFIG[]` ordering.

Field semantics:

- **`channel`** is uniform `2` after `align_roll` makes bone-local Z the
  joint axis for every bone. Kept in the file for clarity but the
  exporter may hardcode `2` instead.
- **`direction`** is **HARDWARE-CALIBRATION-ONLY**. The kinematic L/R
  asymmetry (URDF `<axis>0 -1 0</axis>` for FR/BL hip and knee joints)
  is already absorbed into the bone roll at rig-build time, so a
  positive Blender `rotation_euler[2]` rotates the leg "up" the same
  way on both sides — no per-bone `direction: -1` needed for kinematic
  mirroring. Default `1` everywhere; flip to `-1` only after the robot
  is assembled and a specific servo is observed driving its joint
  backward (typically because that unit's horn was mounted facing the
  wrong way).
- **`offset_deg`** is the SERVO PWM angle when the Blender bone is at
  0 rad (the URDF rest pose). For DSS-M15S 0–270° servos, mid-range = 135°.
- **`min_deg` / `max_deg`** are POST-CONVERSION PWM SAFETY CLAMPS — they
  protect the servo and are independent of the URDF kinematic limits
  (which the rig already enforces via `LIMIT_ROTATION` in §2). Keep them
  tighter than 0/270.

```yaml
# servo_mapping.yaml — URDF↔firmware bridge.
# Keys: Blender bone names (= URDF link names, lowercase).
# servo_id values follow the convention in animation/SERVO_ID_CONVENTION.md
# (PROPOSAL — pending firmware confirmation against SERVO_CONFIG[]).

servos:
  fl_link1: { servo_id: 0,  channel: 2, direction: 1, offset_deg: 135, min_deg: 10, max_deg: 260 }
  fl_link2: { servo_id: 1,  channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  fl_link3: { servo_id: 2,  channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  fr_link1: { servo_id: 3,  channel: 2, direction: 1, offset_deg: 135, min_deg: 10, max_deg: 260 }
  fr_link2: { servo_id: 4,  channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  fr_link3: { servo_id: 5,  channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  bl_link1: { servo_id: 6,  channel: 2, direction: 1, offset_deg: 135, min_deg: 10, max_deg: 260 }
  bl_link2: { servo_id: 7,  channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  bl_link3: { servo_id: 8,  channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  br_link1: { servo_id: 9,  channel: 2, direction: 1, offset_deg: 135, min_deg: 10, max_deg: 260 }
  br_link2: { servo_id: 10, channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
  br_link3: { servo_id: 11, channel: 2, direction: 1, offset_deg: 135, min_deg:  0, max_deg: 270 }
```

**Note:** the `offset_deg` values above are placeholders. They must be
calibrated once the robot is assembled. The `direction: 1` defaults are
correct under Convention A — only flip after observing a servo running
backward on the assembled robot.

### Angle conversion formula

```
servo_angle = offset_deg + direction * degrees(blender_euler[channel])
servo_angle = clamp(servo_angle, min_deg, max_deg)
```

---

## 7. Export Script: `export_gait.py`

Runs inside Blender (Scripting workspace or via command line).

### Input / Output

```
Input:  Baked Action on the active armature
Config: servo_mapping.yaml (path set in script or via UI)
Output: <action_name>.gait (JSON file)
```

### Algorithm

```
1. Load servo_mapping.yaml
2. Get active armature object
3. Get active Action (or named Action via argument)
4. fps = bpy.context.scene.render.fps

5. For each servo in mapping:
   a. bone_name = servo config key (e.g. "fl_link1")
   b. data_path = f'pose.bones["{bone_name}"].rotation_euler'
   c. fcurve = action.fcurves.find(data_path, index=2)   # always Z after align_roll
   d. If fcurve is None: ERROR — bone not animated or not baked

   e. For each keyframe_point in fcurve.keyframe_points:
      - frame = kp.co[0]
      - value_rad = kp.co[1]
      - time_ms = round((frame / fps) * 1000)
      - servo_deg = offset_deg + direction * degrees(value_rad)
      - servo_deg = clamp(servo_deg, min_deg, max_deg)
      - interp_type = map_blender_interp(kp.interpolation)
        (BEZIER → "cubic", LINEAR → "linear", CONSTANT → "constant")

   f. Append to track keyframes list

6. neutral_pose = [track[0].angle for each track]  # frame 0 values
7. duration_ms = max time_ms across all tracks

8. Validate:
   - All tracks start at t=0 and end at duration_ms
   - First angle == last angle == neutral_pose[servo_id]
   - All angles within min/max

9. Write .gait JSON
```

### Key Blender API calls

```python
import bpy, yaml, math, json

# Find the Z-axis FCurve for a bone (every joint rotates on local Z after
# align_roll, so index=2 uniformly).
def find_fcurve(action, bone_name):
    data_path = f'pose.bones["{bone_name}"].rotation_euler'
    return action.fcurves.find(data_path, index=2)

# Convert frame to milliseconds
fps = bpy.context.scene.render.fps
time_ms = round((frame / fps) * 1000)

# Read keyframes
for kp in fcurve.keyframe_points:
    frame, value_rad = kp.co[0], kp.co[1]
    interp = kp.interpolation  # 'BEZIER', 'LINEAR', 'CONSTANT'

# Map Blender interpolation to gait format
INTERP_MAP = {'BEZIER': 'cubic', 'LINEAR': 'linear', 'CONSTANT': 'constant'}
```

### Optional: re-sparsification

After baking, every frame has a keyframe. For compact output, remove redundant keyframes:

1. Start with all baked keyframes
2. For each keyframe, check: can it be removed without exceeding an angle error threshold (e.g. 0.5°)?
3. Remove keyframes that don't significantly affect the curve shape
4. This is a simplification pass — keeps only inflection points

This is optional for v1. Baked keyframes are already small enough for firmware.

---

## 8. Reference: Blender-Servo-Animation Addon

Repository: `timhendriks93/blender-servo-animation`

This existing addon does a similar job. Worth reading before writing `export_gait.py`:
- How it reads evaluated bone angles per frame
- How it handles neutral/offset per bone
- How it converts rotation to servo degrees
- It evaluates per-frame (dense sampling), not per-keyframe (sparse)

We don't use it directly because it doesn't support our state machine, gait format, or DSS-M15S servos. But its angle extraction code is a good reference for avoiding matrix pitfalls.

---

## 9. Blender API Quick Reference

```python
import bpy, math

# Get armature and action
arm = bpy.data.objects["QuadArmature"]
action = arm.animation_data.action

# List all FCurves in action
for fc in action.fcurves:
    print(fc.data_path, fc.array_index, len(fc.keyframe_points))

# Find specific FCurve (every joint uses Z = index 2 after align_roll)
fc = action.fcurves.find('pose.bones["fl_link2"].rotation_euler', index=2)

# Read keyframes
for kp in fc.keyframe_points:
    frame, val = kp.co[0], kp.co[1]
    print(f"frame {frame:.0f}: {math.degrees(val):.2f}°  interp={kp.interpolation}")

# Evaluate at specific frame (alternative to reading FCurve)
bpy.context.scene.frame_set(12)
angle = arm.pose.bones["fl_link2"].rotation_euler[2]

# Bake IK to FK
bpy.ops.nla.bake(
    frame_start=bpy.context.scene.frame_start,
    frame_end=bpy.context.scene.frame_end,
    only_selected=False,
    bake_types={'POSE'},
    visual_keying=True,
    clear_constraints=False,
    use_current_action=True
)
```
