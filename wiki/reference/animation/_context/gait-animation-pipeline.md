<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/gait-design/gait-animation-pipeline.md
Original kind:  design
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/animation/gait-design.md
-->

> **Reference material.** Verbatim copy of `doc/gait-design/gait-animation-pipeline.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/gait-design/gait-animation-pipeline.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [stale] Whole .gait pipeline (export_gait.py, gait_to_c.py, gait_validator.py, *.gait, gait_player.c) never implemented — none exist in repo. This entire doc/gait-design/ tree is superseded by doc/animation-pipeline/ per CLAUDE.md.
- [stale] §5/§8 servo_mapping.yaml at code/blender/servo_mapping.yaml does not exist; bone names FL_hip/FL_knee/FL_shoulder and per-channel index (0=X/2=Z) are obsolete — current rig uses fl_link1/2/3 with uniform bone-local Z (align_roll), conversion via fh_clip_panel.py _frame_to_servo + animation/convention.json.
- [stale] §8 project file structure (code/blender/, code/tools/, code/gaits/, gait_*.h) does not match the repo (animation/scripts/, animation/exported_gaits/, code/firmware/src/).
- [stale] 270° range / 135° flat neutral / `270 - angle` inversion contradict config.h (180° servos, PCA9685, MIN_PULSE 150/MAX_PULSE 600); neutral is per-leg per-joint in convention.json.
- [drift] §16 REST API (/gait POST) is superseded by the WebSocket T-command protocol in code/API_SPEC.md (T:1/T:2/T:5 etc.), not REST endpoints.
- [todo]  Self-labelled "pre-implementation design document"; treat all of it as historical design, replaced by the .js/clips_all.h route (onboard-clip-player-design.md) and the .fhc design (leg-coordinates.md).
-->
# FaceHugger — Blender → ESP32 Gait Animation Pipeline

> Reference notes for the `blender-gait-export` issue.  
> Integrates research from Perplexity + initial Claude Code design.  
> Status: pre-implementation design document.
>
> **Detailed specs:** See `specs/` for per-topic breakdowns:
> - `specs/gait-file-format.md` — full `.gait` JSON + C format spec with validation rules
> - `specs/esp32-playback-engine.md` — interpolation, state machine, command queue, inversion, speed
> - `specs/blender-rig-and-export.md` — rig setup, IK, baking, export script, servo mapping
> - `specs/runtime-api-and-control.md` — REST endpoints, command patterns, error handling

---

## 1. Mental model before anything else

The pipeline has three conceptually separate problems. Keep them separate in your head:

```
Blender (visual design)
    ↓  bake IK → FK
    ↓  export_gait.py
.gait JSON (intermediate format, human-readable)
    ↓  gait_to_c.py
C headers (compiled into ESP32 firmware)
    ↓  playback engine at 50 Hz
servo PWM signals
```

You are **not** trying to use the raw F-curves at runtime. The F-curves are Blender's internal animation representation. Your job is to sample them (or the baked result) and produce a compact servo angle table.

---

## 2. How Blender stores animation (what you're reading)

### The data hierarchy

```
bpy.data.actions["walk_forward"]     ← one animation clip
  └── action.fcurves                  ← one FCurve per (bone, axis)
        FCurve  data_path = 'pose.bones["FL_hip"].rotation_euler'
                array_index = 0        ← X axis
                keyframe_points = [(frame=1, val=0.0), (frame=12, val=1.57), ...]
                interpolation = BEZIER ← what fills the gaps
```

Each `keyframe_point.co` is a `(frame_number, value_in_radians)` pair. Blender fills the frames in between using Bezier interpolation (the curves you see in the Graph Editor). The keyframe data is **sparse** — only at frames where you explicitly pressed I.

### The IK problem

When you use IK in Blender to animate foot positions, the FCurves on the leg bones are **not populated**. The IK solver computes those angles at evaluation time from the IK target position. So you cannot read `FL_hip`'s FCurve directly — it might be empty or only have the rest-pose value.

**The fix: always bake before export.**

`Pose → Animation → Bake Action` (or `bpy.ops.nla.bake`) evaluates the full IK solve at every frame and writes explicit keyframes onto every bone. After baking, `FL_hip`'s FCurve is fully populated with the IK-solved angles. You can now read them directly.

### Two reading strategies

| Strategy | When to use | How |
|---|---|---|
| Read FCurve keyframe_points directly | After baking; want sparse data | `fcurve.keyframe_points` — gives you only keyframed frames |
| Sample frame-by-frame | Any rig; dense output | `scene.frame_set(N)` → `pose_bone.matrix_channel` per frame |

For a gait cycle on this robot, **bake first, then read FCurve keyframe_points**. This gives you the sparse, meaningful frames (inflection points in the motion) and keeps the `.gait` file small. You can always re-densify on the ESP32 with interpolation.

### Reading the actual angle correctly

After baking, for a hinge joint:

```python
# In Blender Python (runs inside Blender)
pose_bone = arm_obj.pose.bones["FL_hip"]
# rotation_euler gives you the evaluated Euler angle in radians
# index 0=X, 1=Y, 2=Z — depends on which axis is your servo axis
angle_rad = pose_bone.rotation_euler[0]
```

**Important:** `rotation_euler` gives values relative to the bone's rest pose orientation, **not** global/world space. That's what you want — the delta from the neutral stance. You do not need to invert parent matrices for single-axis hinge joints.

For the full local matrix approach (if you have multi-axis joints or need to be more precise):

```python
mat = pose_bone.matrix
if pose_bone.parent:
    local_mat = pose_bone.parent.matrix.inverted() @ mat
else:
    local_mat = mat
eul = local_mat.to_euler("XYZ")
angle_rad = eul.x  # or y or z depending on servo axis
```

---

## 3. What the blender-servo-animation addon does

**Repo:** `github.com/timhendriks93/blender-servo-animation`

It is a Blender add-on that:
- Adds a "Servo Settings" panel in the Bone Properties tab
- Lets you tag each bone as a servo with: neutral angle, min/max range, rotation axis
- Exports the animation as servo position values (degrees or microseconds) via File → Export
- Supports IK rigs (it evaluates the rig per-frame, same as the bake approach)
- Outputs C header arrays directly (compatible with its paired Arduino library)

**Should you use it?** Probably not as-is — it targets Arduino + PWM servos generically and doesn't know about your DSS-M15S servos, your ESP32, or your gait state machine. But **read its source** before writing your export script. Specifically look at how it:
- Reads evaluated bone angles per frame
- Converts rotation to servo degrees
- Handles the neutral/offset per bone

The pattern it uses avoids several common pitfalls that trip people up. It's in `addon/` in the repo.

---

## 4. The file format question

### What you need to store

For each gait cycle, per servo, per keyframe: `(time_ms, angle_degrees)` plus an interpolation type. That's it.

### Why not raw CSV / flat array

A flat `frame, servo0, servo1, ..., servo11` CSV table works but:
- Dense: 800ms at 50Hz = 40 rows × 12 columns = fine, but scales poorly
- No interpolation metadata — firmware must guess (linear? step?)
- No named fields, easy to mess up servo ordering

### The `.gait` JSON format (intermediate, human-readable)

```json
{
  "name": "walk_forward",
  "duration_ms": 800,
  "fps_source": 30,
  "neutral_pose": [135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135],
  "interpolation_default": "cubic",
  "tracks": [
    {
      "servo_id": 0,
      "name": "FL_shoulder",
      "keyframes": [
        {"t": 0,   "angle": 135.0},
        {"t": 200, "angle": 150.0},
        {"t": 400, "angle": 135.0},
        {"t": 800, "angle": 135.0}
      ]
    },
    {
      "servo_id": 1,
      "name": "FL_hip",
      "keyframes": [
        {"t": 0,   "angle": 135.0},
        {"t": 200, "angle": 90.0, "interp": "linear"},
        {"t": 600, "angle": 180.0},
        {"t": 800, "angle": 135.0}
      ]
    }
  ]
}
```

**Key design decisions:**
- `t` is milliseconds from cycle start (not frame numbers — decoupled from Blender FPS)
- `angle` is servo degrees 0–270 (not radians, not raw Blender values)
- `interp` is per-keyframe, meaning "how to interpolate from this keyframe to the next one"
- First and last keyframe must be the neutral pose on every track — this enables clean loop transitions
- Per-servo `name` field is for debugging only; `servo_id` is the canonical reference

### Compiled C format (firmware)

A Python script (`gait_to_c.py`) converts `.gait` → C headers compiled into firmware:

```c
// auto-generated by gait_to_c.py — do not edit
#define GAIT_WALK_FORWARD_DURATION_MS 800

static const keyframe_t gait_walk_forward_track0[] = {
    {0, 1350}, {200, 1500}, {400, 1350}, {800, 1350}
    // angles stored × 10 for 0.1° precision in uint16_t
};
static const gait_track_t gait_walk_forward_tracks[12] = {
    {.servo_id = 0, .num_kf = 4, .keyframes = gait_walk_forward_track0},
    // ...
};
static const gait_t GAIT_WALK_FORWARD = {
    .name = "walk_forward",
    .duration_ms = 800,
    .num_tracks = 12,
    .tracks = gait_walk_forward_tracks,
    .interp = INTERP_CUBIC,
};
```

**Memory estimate:** ~10 keyframes × 12 servos × 4 bytes (t: uint16 + angle×10: uint16) = 480 bytes per gait. 20 gaits ≈ 10 KB. ESP32 has 4 MB flash — no concern.

---

## 5. The servo mapping config

This is the critical bridge between Blender bone names/axes and physical servo IDs/directions. Lives in `code/blender/servo_mapping.yaml`.

```yaml
# Bone name → servo physical mapping
# channel: which rotation_euler index (0=X, 1=Y, 2=Z) is this servo's axis?
# direction: 1 = Blender positive = servo angle increases; -1 = inverted
# offset_deg: what Blender 0 radians maps to in servo degrees
#             (typically 135 = middle of 270° range = mechanical zero)

servos:
  FL_shoulder:
    servo_id: 0
    channel: 2          # Z = yaw
    direction: 1
    offset_deg: 135
    min_deg: 10
    max_deg: 260

  FL_hip:
    servo_id: 1
    channel: 0          # X = pitch
    direction: -1       # inverted: Blender + = servo decreases
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

  # ... FR_shoulder (3), FR_hip (4), FR_knee (5)
  # ... BL_shoulder (6), BL_hip (7), BL_knee (8)
  # ... BR_shoulder (9), BR_hip (10), BR_knee (11)
```

**The offset and direction values are calibration constants.** You will not know them exactly until you've assembled the robot and measured the mechanical zero of each joint. Leave them as placeholders and plan one calibration session per servo axis.

The angle conversion formula is:

```
servo_angle = offset_deg + direction * degrees(blender_euler[channel])
servo_angle = clamp(servo_angle, min_deg, max_deg)
```

---

## 6. Export script structure (`export_gait.py`)

Runs inside Blender (Scripting workspace or as an add-on operator).

```
Input:  active Action in Blender (must be baked)
Config: servo_mapping.yaml
Output: <action_name>.gait JSON

Steps:
1. Load servo_mapping.yaml
2. Get armature object and active Action
3. For each servo in mapping:
   a. Find the FCurve matching (bone_name, channel_index)
   b. Read keyframe_points: [(frame, value_radians), ...]
   c. Convert frame → time_ms using scene FPS
   d. Convert radians → servo degrees using mapping config
   e. Validate: within min/max range?
4. Assemble tracks + neutral_pose from frame 0 values
5. Write .gait JSON
```

**Key Blender API call to find an FCurve:**

```python
import bpy, yaml, math, json

def find_fcurve(action, bone_name, channel_index):
    data_path = f'pose.bones["{bone_name}"].rotation_euler'
    return action.fcurves.find(data_path, index=channel_index)
```

**Converting frame to ms:**

```python
fps = bpy.context.scene.render.fps
time_ms = round((frame / fps) * 1000)
```

**Reading keyframes from a curve:**

```python
for kp in fcurve.keyframe_points:
    frame = kp.co[0]
    value_rad = kp.co[1]
```

---

## 7. ESP32 playback engine

### Core structures

```c
typedef struct {
    uint16_t time_ms;
    uint16_t angle_x10;   // degrees × 10, fits 0–2700 in uint16_t
} keyframe_t;

typedef struct {
    uint8_t  servo_id;
    uint8_t  num_keyframes;
    const keyframe_t* keyframes;
} gait_track_t;

typedef struct {
    const char*       name;
    uint16_t          duration_ms;
    uint8_t           num_tracks;
    const gait_track_t* tracks;
    uint8_t           interp;    // INTERP_LINEAR, INTERP_CUBIC, INTERP_CONSTANT
} gait_t;
```

### Interpolation at 50 Hz

```
every 20ms tick:
  cycle_time_ms = (millis() - cycle_start_ms) % current_gait.duration_ms
  for each track:
    find kf_a, kf_b such that kf_a.t <= cycle_time_ms < kf_b.t
    t_norm = (cycle_time_ms - kf_a.t) / (kf_b.t - kf_a.t)  // 0.0 to 1.0
    angle = interpolate(kf_a.angle, kf_b.angle, t_norm, interp_type)
    set_servo_angle(track.servo_id, angle)
```

Interpolation types:
- **Linear:** `a + (b - a) * t`
- **Cubic (ease-in/out):** `a + (b - a) * smoothstep(t)` where `smoothstep(t) = 3t² - 2t³`
- **Constant:** `a` (step at keyframe boundary — useful for instant repositioning)

### Gait state machine

```
States:    IDLE → PLAYING ⇄ TRANSITIONING
Trigger:   command queue (circular buffer, capacity 4)

PLAYING:
  - on each cycle boundary (time wraps): check queue
  - if queue non-empty: pop next gait, update current_gait, reset cycle_start_ms
  - since all gaits share neutral_pose at t=0 and t=duration: no transition blending needed

STOP command:
  - finish current cycle, land at neutral_pose, → IDLE
```

### Inversion and speed

```c
// Inversion: for walking backwards
if (inverted) angle = 270.0f - angle;

// Speed scaling (fixed-point, stored as uint8 × 100)
uint32_t elapsed = millis() - cycle_start_ms;
uint32_t scaled_time = (elapsed * speed_x100) / 100;
uint32_t cycle_time_ms = scaled_time % gait.duration_ms;
```

---

## 8. Project file structure

```
2026sp-FaceHugger/
├── code/
│   ├── blender/
│   │   ├── export_gait.py          # Blender script: Action → .gait JSON
│   │   ├── servo_mapping.yaml      # Bone → servo ID/axis/direction/offset
│   │   └── README.md               # Blender workflow instructions
│   ├── tools/
│   │   ├── gait_to_c.py            # .gait JSON → C header converter
│   │   └── gait_validator.py       # Validates .gait files before compile
│   ├── gaits/
│   │   ├── walk_forward.gait
│   │   ├── turn_left.gait
│   │   ├── turn_right.gait
│   │   └── stand.gait
│   └── firmware/
│       ├── gait_player.h/.c        # Interpolation engine + state machine
│       ├── gait_registry.h          # Auto-generated: all gaits registered
│       ├── gait_walk_forward.h      # Auto-generated per gait
│       └── servo_config.h           # GPIO, PWM range, servo limits
```

---

## 9. Implementation order

These are ordered to give you the fastest feedback loops.

### Phase 1: Understand your data
1. Open Blender Python console on your current rig
2. Run: `obj = bpy.context.active_object; [print(f.data_path, f.array_index) for f in obj.animation_data.action.fcurves]`
3. Confirm which bones have FCurves and what paths they use
4. If using IK: run `bpy.ops.nla.bake(frame_start=1, frame_end=60, bake_types={'POSE'}, visual_keying=True)` and re-inspect FCurves

### Phase 2: One servo, one export
5. Hard-code one bone in a test script, print its keyframe angles to console
6. Verify the numbers match what Blender's properties panel shows (N key → Item → Rotation)
7. Add the angle conversion (radians → servo degrees using mapping config)

### Phase 3: Full export
8. Generalize to all 12 servos using servo_mapping.yaml
9. Write .gait JSON output
10. Write gait_validator.py (checks neutral pose alignment, angle bounds, monotonic timing)

### Phase 4: Firmware
11. Write gait_to_c.py converter
12. Implement gait_player.c with linear interpolation first (cubic later)
13. Test single servo, then single leg, then full quad

### Phase 5: Polish
14. Add Blender UI panel (operator button, armature/action picker)
15. Add cubic interpolation to playback engine
16. Add gait state machine + REST API endpoints
17. Calibrate servo_mapping offsets on physical hardware

---

## 10. Open questions to answer before coding

1. **Bone naming convention** — are the 12 bones already named consistently in your rig, or do you need to rename them to match the mapping config?
2. **Single rotation axis per servo** — for shoulder yaw, is it a pure Z rotation, or does the bone also have X/Y components due to the mesh orientation?
3. **Rest pose alignment** — does your Blender rest pose match the physical neutral stance of the assembled robot? If not, every exported angle will be offset.
4. **Control loop frequency** — 50 Hz assumed. Does the ESP32's servo PWM refresh rate match this, or is it driven by the servo protocol itself?
5. **Calibration plan** — how will you measure the mechanical zero of each joint once assembled? Manual measurement, or write a calibration mode that sweeps each servo and lets you mark zero?
6. **Gait file versioning** — if you change the rig or the mechanical linkage, how do you know which gaits are stale and need re-export?

---

## 11. Reference: useful Blender API calls

```python
import bpy, math

# Get armature and pose bones
arm = bpy.data.objects["QuadArmature"]
pose_bones = arm.pose.bones

# Get active action
action = arm.animation_data.action

# Inspect all FCurves
for fc in action.fcurves:
    print(fc.data_path, fc.array_index, len(fc.keyframe_points))

# Find a specific FCurve
fc = action.fcurves.find('pose.bones["FL_hip"].rotation_euler', index=0)

# Read keyframes from it
for kp in fc.keyframe_points:
    frame, value_rad = kp.co[0], kp.co[1]
    print(f"frame {frame:.1f}: {math.degrees(value_rad):.2f}°")

# Evaluate rig at a specific frame (for dense sampling or validation)
bpy.context.scene.frame_set(12)
angle_rad = pose_bones["FL_hip"].rotation_euler[0]

# Bake IK to FK (run from Blender Python console)
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

---

## 12. Related reference

- `timhendriks93/blender-servo-animation` — read `addon/` source for angle extraction pattern
- `pollen-robotics/reachy2-blender` — example of Blender → robot angle streaming
- Blender API docs: `bpy.types.FCurve`, `bpy.types.PoseBone`, `bpy.ops.nla.bake`
- Blender manual: Animation → Keyframes → Introduction

---

## 13. Gait state machine & command queue

> Full spec: `specs/esp32-playback-engine.md` §4–5

### States

The ESP32 has two states: **IDLE** (neutral pose, no movement) and **PLAYING** (looping a gait cycle).

### The key constraint: all gaits share a neutral start/end pose

Every gait cycle starts and ends at the same neutral stance (all servos at their mechanical zero, typically 135° = middle of 270° range). This eliminates the need for transition interpolation between gaits.

### Command queue

- Circular buffer, capacity 4 commands
- Commands are **enqueued** when received (from REST API)
- Commands are **dequeued only at cycle boundaries** (when `cycle_time` wraps past `duration_ms`)
- Mid-cycle commands are never applied immediately — this prevents jerky motion
- A STOP command clears the rest of the queue

### Flow

```
IDLE + PLAY(gait) → start immediately → PLAYING
PLAYING + new command → push to queue
PLAYING + cycle ends + queue non-empty → pop queue, switch gait
PLAYING + cycle ends + queue has STOP → finish cycle → IDLE (at neutral)
PLAYING + cycle ends + queue empty → loop same gait
```

---

## 14. Inverted mode

For running the robot upside-down without creating duplicate gait files.

### Runtime transform

```c
if (inverted) angle = 270.0f - angle;
```

Applied after interpolation, before PWM write. One boolean flag, no extra gait data.

This works because flipping the robot reverses gravity's direction relative to the joints. Mirroring the angle through the midpoint of the 270° range achieves the equivalent physical motion.

### When NOT to use

If the inverted gait needs different timing or foot placement (not just mirrored angles), create a separate gait cycle.

---

## 15. Speed multiplier

### How it works

```c
uint32_t scaled_time = (elapsed * speed_x100) / 100;
uint32_t cycle_time = scaled_time % gait.duration_ms;
```

- `speed_x100 = 100` → normal (1.0×)
- `speed_x100 = 120` → 20% faster
- `speed_x100 = 80` → 20% slower

### Limitations

Useful range: ±20%. Beyond that, the motion looks unnatural — faster risks exceeding servo acceleration limits, slower looks floaty. For significantly different speeds, create a new gait with appropriate timing.

---

## 16. REST API

> Full spec: `specs/runtime-api-and-control.md`

The ESP32 already runs a REST API. Gait control adds these endpoints:

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/gait` | POST | Queue a gait: `{"name": "walk_forward", "speed": 1.0, "inverted": false}` |
| `/gait` | POST | Stop: `{"name": "stop"}` — finishes cycle, returns to IDLE |
| `/gait/status` | GET | Current state, gait name, cycle progress, queue contents |
| `/gait/list` | GET | Available gaits with durations |
| `/gait/clear` | POST | Empty the command queue without stopping current gait |

### Watchdog

If no API request is received within a timeout (e.g. 5 seconds), automatically queue STOP. Prevents the robot from walking indefinitely if the controller disconnects.

---

## 17. Assembly calibration

Standard servo calibration approach:

1. **Set zero in code:** Power on ESP32, command all servos to 135° (center of 270° range)
2. **Assemble in position:** Physically attach legs while servos hold that angle
3. **Mechanical zero = electrical zero:** The assembly defines what 135° looks like physically
4. **Match in Blender:** The rig's rest pose must match this stance exactly
5. **Fine-tune offsets:** If there's a mismatch, adjust `offset_deg` in `servo_mapping.yaml` per servo

The `offset_deg` and `direction` values in the servo mapping are calibration constants — expect to adjust them once during assembly and whenever a servo is replaced.
