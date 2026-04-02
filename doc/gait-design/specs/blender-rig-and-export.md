# Spec: Blender Rig & Export Workflow

> Covers: Fusion 360 import, armature rigging, IK setup, animation, baking, and the `export_gait.py` script.
> See also: `gait-file-format.md` (what gets exported), `esp32-playback-engine.md` (what consumes it).

---

## 1. Fusion 360 → Blender Import

### Export from Fusion 360

- Export each **rigid body part** as a separate STL (not one monolithic mesh)
  - Each link segment (upper leg, lower leg, foot)
  - Each servo body
  - The body/chassis
- Use millimeters as the unit in Fusion 360 (Blender will import at 1 unit = 1mm, scale if needed)

### Import into Blender

- File → Import → STL
- Import each part individually
- Position each mesh at its correct location (should match Fusion 360 assembly)
- Group related parts (e.g. FL leg parts) into collections for organization

---

## 2. Armature Setup

### Bone placement

Create an Armature object with **12 bones** — one per servo shaft:

```
Armature
├── FL_shoulder    (yaw,   Z rotation)
│   └── FL_hip     (pitch, X rotation)
│       └── FL_knee (pitch, X rotation)
├── FR_shoulder
│   └── FR_hip
│       └── FR_knee
├── BL_shoulder
│   └── BL_hip
│       └── BL_knee
└── BR_shoulder
    └── BR_hip
        └── BR_knee
```

Each bone's **head** (root) is placed exactly at the corresponding servo shaft center. The bone's **tail** points toward the next joint in the chain (or toward the foot for knee bones).

### Rotation axes

| Joint type | Servo axis | Blender rotation channel | Euler order |
|------------|-----------|-------------------------|-------------|
| Shoulder (yaw) | Z | `rotation_euler[2]` | XYZ |
| Hip (pitch) | X | `rotation_euler[0]` | XYZ |
| Knee (pitch) | X | `rotation_euler[0]` | XYZ |

Set all bones to use **XYZ Euler** rotation mode (not quaternion — we need single-axis readability).

### Joint limits

In Bone Properties → Inverse Kinematics (or via bone constraints):

| Joint | Axis | Min | Max | Notes |
|-------|------|-----|-----|-------|
| Shoulder | Z | -125° | +125° | ~250° usable of 270° range |
| Hip | X | -135° | +135° | Full 270° range |
| Knee | X | -135° | +135° | Full 270° range |

These limits map to servo degrees via: `servo_angle = offset_deg + direction * blender_degrees`

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
5. Create one Empty per leg: `FL_foot_target`, `FR_foot_target`, etc.

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

### Definition

The neutral pose is the stance the robot assumes when:
- All servos are at their mechanical zero (typically 135° = middle of 270° range)
- The robot is standing stable
- No gait is active

### In Blender

- The **rest pose** (Edit Mode bone positions) must match the physical neutral stance
- Frame 1 and the last frame of every gait animation must be this pose
- Apply as rest pose: Pose Mode → Pose → Apply Pose as Rest Pose

### Calibration

1. Power on ESP32, command all servos to 135° (or chosen neutral angle)
2. Physically assemble the robot while servos hold position
3. The Blender rig's rest pose must match this assembly
4. If there's a mismatch, adjust the `offset_deg` in `servo_mapping.yaml`

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

File: `servo_mapping.yaml`

```yaml
# Maps Blender bone names/axes to physical servo IDs
# direction: 1 = Blender positive rotation = servo angle increases
#           -1 = inverted
# offset_deg: Blender 0 rad → this servo angle (mechanical zero)
# channel: rotation_euler index (0=X, 1=Y, 2=Z)

servos:
  FL_shoulder:
    servo_id: 0
    channel: 2          # Z = yaw
    direction: 1
    offset_deg: 135     # middle of 270° range
    min_deg: 10
    max_deg: 260

  FL_hip:
    servo_id: 1
    channel: 0          # X = pitch
    direction: -1
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  FL_knee:
    servo_id: 2
    channel: 0
    direction: -1
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  FR_shoulder:
    servo_id: 3
    channel: 2
    direction: -1       # mirrored from FL
    offset_deg: 135
    min_deg: 10
    max_deg: 260

  FR_hip:
    servo_id: 4
    channel: 0
    direction: 1        # mirrored from FL
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  FR_knee:
    servo_id: 5
    channel: 0
    direction: 1
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  BL_shoulder:
    servo_id: 6
    channel: 2
    direction: 1
    offset_deg: 135
    min_deg: 10
    max_deg: 260

  BL_hip:
    servo_id: 7
    channel: 0
    direction: -1
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  BL_knee:
    servo_id: 8
    channel: 0
    direction: -1
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  BR_shoulder:
    servo_id: 9
    channel: 2
    direction: -1
    offset_deg: 135
    min_deg: 10
    max_deg: 260

  BR_hip:
    servo_id: 10
    channel: 0
    direction: 1
    offset_deg: 135
    min_deg: 0
    max_deg: 270

  BR_knee:
    servo_id: 11
    channel: 0
    direction: 1
    offset_deg: 135
    min_deg: 0
    max_deg: 270
```

**Note:** The `direction` and `offset_deg` values above are placeholders. They must be calibrated per servo once the robot is assembled. Left/right mirroring is assumed but must be verified.

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
   a. bone_name = servo config key (e.g. "FL_shoulder")
   b. channel = servo config channel index
   c. data_path = f'pose.bones["{bone_name}"].rotation_euler'
   d. fcurve = action.fcurves.find(data_path, index=channel)
   e. If fcurve is None: ERROR — bone not animated or not baked

   f. For each keyframe_point in fcurve.keyframe_points:
      - frame = kp.co[0]
      - value_rad = kp.co[1]
      - time_ms = round((frame / fps) * 1000)
      - servo_deg = offset_deg + direction * degrees(value_rad)
      - servo_deg = clamp(servo_deg, min_deg, max_deg)
      - interp_type = map_blender_interp(kp.interpolation)
        (BEZIER → "cubic", LINEAR → "linear", CONSTANT → "constant")

   g. Append to track keyframes list

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

# Find an FCurve for a specific bone + channel
def find_fcurve(action, bone_name, channel_index):
    data_path = f'pose.bones["{bone_name}"].rotation_euler'
    return action.fcurves.find(data_path, index=channel_index)

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

# Find specific FCurve
fc = action.fcurves.find('pose.bones["FL_hip"].rotation_euler', index=0)

# Read keyframes
for kp in fc.keyframe_points:
    frame, val = kp.co[0], kp.co[1]
    print(f"frame {frame:.0f}: {math.degrees(val):.2f}°  interp={kp.interpolation}")

# Evaluate at specific frame (alternative to reading FCurve)
bpy.context.scene.frame_set(12)
angle = arm.pose.bones["FL_hip"].rotation_euler[0]

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
