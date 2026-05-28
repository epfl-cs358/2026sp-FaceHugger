# PyBullet Engine Setup & Runtime — FaceHugger Quadruped

How the FaceHugger simulation talks to the PyBullet API under the hood. Every aspect below is quoted from the live source.

A note on file layout: `simulate.py` is only an argparse front-end — it does **no** PyBullet work itself. It hands off to `run_stand` / `run_gait` / `run_clip` in `gaits.py`, where the actual physics-server wiring lives. `view_urdf.py` is a separate, self-contained no-physics viewer. `helpers.py` holds the joint-map / motor-command plumbing shared by all entry points, and `sim_monitor.py` reads back torque/current. So the canonical "engine setup" path is `gaits._connect_and_setup`.

---

## 1. Connecting to the physics server

The main sim connects GUI-or-DIRECT depending on the `gui` flag, then registers the `pybullet_data` search path so `plane.urdf` can be found, and (in non-float mode) loads the ground plane.

```python
# gaits.py:242
p.connect(p.GUI if gui else p.DIRECT)
```

`p.connect(p.GUI)` opens an OpenGL window with the built-in debug visualizer; `p.DIRECT` runs headless in-process with no rendering (used for `--headless` / CI smoke checks). `gui` is computed in `simulate.py` as `gui = not args.headless`.

```python
# gaits.py:249
p.setAdditionalSearchPath(pybullet_data.getDataPath())
```

`pybullet_data.getDataPath()` returns the install directory of PyBullet's bundled assets (which contains `plane.urdf`). `p.setAdditionalSearchPath` appends it to the resource search path so a bare filename like `"plane.urdf"` resolves.

```python
# gaits.py:255
if not float_mode:
    p.loadURDF("plane.urdf")
```

The ground plane is only loaded when **not** in `--float` mode. In float mode there is no floor (and no gravity — see §3), so the robot is pinned mid-air to inspect pure joint geometry.

The standalone viewer does the same dance independently:

```python
# view_urdf.py:28
p.connect(p.GUI)
...
# view_urdf.py:35
p.setAdditionalSearchPath(pybullet_data.getDataPath())
...
# view_urdf.py:39
p.loadURDF("plane.urdf")
```

`view_urdf.py` always connects GUI and always loads the plane (it has no headless/float modes).

---

## 2. Loading the robot

URDF path resolution: the path is **not** hardcoded in `gaits.py`; it comes from the config object (`cfg.urdf_path`, ultimately `constants.URDF_PATH = generated/facehugger.urdf`). The base spawn Z is `cfg.body_height + 0.02` — a 20 mm clearance margin above the computed standing height so the feet don't start inter-penetrating the floor.

```python
# gaits.py:258
robot_id = p.loadURDF(
    cfg.urdf_path,
    basePosition=[0, 0, cfg.body_height + 0.02],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=float_mode,
)
```

- `basePosition` — world XYZ of the base link at spawn (metres).
- `baseOrientation` — quaternion; `getQuaternionFromEuler([0,0,0])` = identity (no initial tilt).
- `useFixedBase=float_mode` — when `True` (float mode) the base is welded to the world so the body hangs in the air; when `False` (normal walk/stand/clip) the base is a free-floating 6-DOF body subject to gravity and contact, which is what lets you observe balance/collapse.

No special `flags=` are passed to `loadURDF` (no self-collision flag, no inertia-from-geometry flag), so PyBullet uses the URDF's declared inertials and default collision filtering.

`run_gait` may raise the spawn height before this call: `_body_height_for_gait` (gaits.py:181) samples the worst foot depth over a full gait period and, if deeper than `cfg.body_height`, overwrites `cfg.body_height` (gaits.py:417-422) so the body won't sink into the floor when a gait's stance-phase Z differs from the neutral pose.

The viewer's load is simpler — fixed base, fixed 0.10 m spawn, no config:

```python
# view_urdf.py:40
robot = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0.10], useFixedBase=True)
```

---

## 3. Physics configuration

```python
# gaits.py:250
p.setGravity(0, 0, 0 if float_mode else -9.81)
# gaits.py:251
p.setTimeStep(TIMESTEP)
# gaits.py:254
p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERATIONS)
```

- `setGravity` — Z gravity is `-9.81 m/s²` in normal mode, `0` in float mode (the body is pinned, so nothing should fall).
- `setTimeStep(TIMESTEP)` with `TIMESTEP = 1.0 / 240.0` (constants.py:12) — the fixed 240 Hz integration step, PyBullet's recommended default for stable contact.
- `setPhysicsEngineParameter(numSolverIterations=150)` — `SOLVER_ITERATIONS = 150` (gaits.py:25). More constraint-solver iterations per step give stiffer, less-jittery contact resolution, which matters for planted-feet moves where multiple feet press the floor simultaneously. Comment (gaits.py:252-253) notes it is harmless in float mode (no contacts to solve).

### `changeDynamics` — body mass

```python
# gaits.py:274
p.changeDynamics(robot_id, -1, mass=BODY_MASS_KG)
```

`-1` is the **base link** index. The URDF base is just the bare frame; `BODY_MASS_KG = 0.8` (gaits.py:22) overrides it to a realistic ~0.8 kg total including ESP32, PCA9685 servo driver and battery. This changes how hard the legs must work to support/balance the chassis — directly feeding the torque readouts in §7.

### `changeDynamics` — foot contact friction

```python
# gaits.py:281
for name, idx in joint_map.items():
    if "link3" in name:
        p.changeDynamics(
            robot_id,
            idx,
            lateralFriction=FOOT_LATERAL_FRICTION,
            spinningFriction=FOOT_SPINNING_FRICTION,
            restitution=0.0,
        )
```

Values: `FOOT_LATERAL_FRICTION = 1.2`, `FOOT_SPINNING_FRICTION = 0.05` (gaits.py:23-24).

- `lateralFriction=1.2` — well above PyBullet's default 0.5 so feet grip the floor and don't slide during stance.
- `spinningFriction=0.05` — small torsional friction so a planted foot resists pivoting in place.
- `restitution=0.0` — no bounce on foot contact.

Realism note baked into the comment (gaits.py:276-280): the link is matched on the substring `"link3"` (the knee joint's child = lower leg/foot) because the URDF joint names are `*_link3_joint`, **not** `*_knee`. An earlier `"knee"` match never fired, leaving feet at the default 0.5 friction and slipping — this is a fixed bug, documented in place.

---

## 4. Joint discovery & mapping

The joint-name → index dictionary is built once per load by iterating all joints and decoding the URDF joint name (field index `1` of `getJointInfo`, a bytes object):

```python
# helpers.py:175
def build_joint_map(robot_id):
    out = {}
    n = p.getNumJoints(robot_id)
    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        out[info[1].decode()] = i
    return out
```

So the dict shape is `{"fl_link1_joint": 0, "fl_link2_joint": 1, "fl_link3_joint": 2, ...}` — URDF joint name string → integer PyBullet joint index. `getJointInfo(robot_id, i)` returns a tuple; index `[1]` is the joint name (bytes), index `[2]` is the joint **type**.

Fixed joints are filtered out at command/reset time by comparing the type field against `p.JOINT_FIXED`:

```python
# helpers.py:188 (reset_to_stance)
info = p.getJointInfo(robot_id, idx)
if info[2] == p.JOINT_FIXED:
    continue
```

The same `info[2] == p.JOINT_FIXED` guard appears in `apply_leg_pose` (helpers.py:200). Semantic role (shoulder/hip/knee) is recovered from the name, not the index, via `_joint_type_from_name` (helpers.py:162): `"link1"→shoulder`, `"link2"→hip`, `"link3"→knee`. The 2-char leg prefix is sliced as `name[:2]` (e.g. `"fl"`).

> Design note (simulate.py:6-11): joint **geometry** (origins, axes, limits, inertials) is deliberately parsed from the raw URDF XML via `_load_urdf_joints` / `xml.etree` (helpers.py:65), **not** from `getJointInfo`, because PyBullet silently shifts link frames to the COM when inertial `<origin>` is non-zero, which would skew parent-frame origins. PyBullet is used for the index mapping and simulation; the XML is the geometric source of truth.

---

## 5. Actuation

All actuation goes through `setJointMotorControl2` in `POSITION_CONTROL` mode. Two helpers wrap it.

Per-leg stance (used for reset/settle/stand):

```python
# helpers.py:206
p.setJointMotorControl2(
    robot_id,
    idx,
    p.POSITION_CONTROL,
    targetPosition=per_leg_stance[leg][jtype],
    force=force,
    maxVelocity=velocity,
)
```

Arbitrary per-joint targets (used by gaits and clip playback):

```python
# helpers.py:222
p.setJointMotorControl2(
    robot_id,
    idx,
    p.POSITION_CONTROL,
    targetPosition=angle,
    force=force,
    maxVelocity=velocity,
)
```

- `p.POSITION_CONTROL` — PyBullet's built-in PD position servo: it drives the joint toward `targetPosition` (radians).
- `targetPosition` — commanded joint angle in radians (gait IK output or clip angle).
- `force` — the **maxForce / torque ceiling** the motor may apply to reach the target; passed as `cfg.servo_force` (servo effort, N·m). This is the saturation limit that produces the "[STALL]" readings in §7.
- `maxVelocity` — caps joint angular speed (rad/s), passed as `cfg.servo_velocity`, modelling the servo's slew limit.

Note: these calls do **not** set `positionGain`/`velocityGain` explicitly, so PyBullet uses its internal defaults for the position-control PD constants; the only tuned knobs are `force` and `maxVelocity`.

Initial pose is set two ways at load (gaits.py:266-269): `reset_to_stance` calls `p.resetJointState(robot_id, idx, stance[...])` (helpers.py:194) to **teleport** joints to stance instantly (no physics), then `apply_leg_pose` arms the position motors to **hold** that stance against gravity.

---

## 6. The simulation step loop

All three entry points share the same pattern: command targets → `p.stepSimulation()` → optional monitor → real-time sleep. The stand loop is the minimal form:

```python
# gaits.py:346 (run_stand)
try:
    while p.isConnected():
        p.stepSimulation()
        if on_step is not None:
            on_step(step)
        if gui:
            time.sleep(TIMESTEP)
        step += 1
except (KeyboardInterrupt, p.error):
    pass
finally:
    if p.isConnected():
        p.disconnect()
```

- `p.stepSimulation()` advances physics by exactly one `TIMESTEP` (no internal sub-stepping configured).
- `time.sleep(TIMESTEP)` — real-time pacing, only when `gui` is true. Headless runs free-wheel as fast as the CPU allows (no wall-clock throttle). This is wall-clock pacing, **not** `setRealTimeSimulation`; stepping stays manual.
- `p.isConnected()` is the loop guard so closing the GUI window cleanly ends the loop; `KeyboardInterrupt` / `p.error` are swallowed and `p.disconnect()` is always called in `finally`.

The gait loop (gaits.py:444) adds, before each step: compute IK targets (`gait_joint_targets`), `apply_joint_targets(...)`, and every 4th step redraw the trajectory overlay; `t += TIMESTEP` advances gait phase. The settle phase (gaits.py:205) is a bare pre-loop that holds stance and steps `int(duration_s / TIMESTEP)` times to let gravity resolve spawn overlap before the main loop.

`getBasePositionAndOrientation` is used only by the debug overlay to transform body-frame foot-trajectory points into world space for drawing:

```python
# gaits.py:139 (_draw_overlay)
base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
```

It is fed to `p.multiplyTransforms(base_pos, base_orn, pos_body, [0,0,0,1])` in `_body_to_world` (gaits.py:114) to compose the body pose with each local point.

Clip playback (`run_clip`) delegates the loop to `ClipPlayer.play_blocking(...)` in `pybullet_interpreter/clip_player.py` (not a focus file), passing the same `on_step` monitor callback (gaits.py:398-400).

---

## 7. The monitor — torque readback & current estimate

`sim_monitor.py` reads the torque the position controller actually applied at each joint from `getJointState()` field `[3]`:

```python
# sim_monitor.py:68
def read_joint_torques(robot_id, joint_map) -> dict:
    """{joint_name: applied torque (N·m)} from getJointState()[3]."""
    import pybullet as p
    return {name: p.getJointState(robot_id, idx)[3] for name, idx in joint_map.items()}
```

`getJointState(robot_id, idx)` returns `(position, velocity, reactionForces, appliedJointMotorTorque)`; index `[3]` is the **applied motor torque** (N·m) — meaningful precisely because the motor runs in `POSITION_CONTROL` (§5). Current joint angle for the status line comes from field `[0]`:

```python
# sim_monitor.py:75
math.degrees(p.getJointState(robot_id, idx)[0])
```

### Torque → current math

```python
# sim_monitor.py:24
def estimate_current_a(torque_nm: float) -> float:
    """...linear from the stall point: |torque| / (stall_torque / stall_current)."""
    return abs(torque_nm) * (STALL_CURRENT_A / STALL_TORQUE_NM)
```

A simple linear model: current is proportional to |torque|, scaled so that at the servo's stall torque it draws the stall current. Constants (sim_monitor.py:16-19):

```python
STALL_TORQUE_NM = 2.94   # 30 kg·cm servo stall torque (matches servo.effort_nm)
STALL_CURRENT_A = 2.5    # approximate stall current at 6 V
CURRENT_LIMIT_A = 10.0   # supply / multiplexer budget — warn above this total
STALL_WARN_NM  = 2.5     # flag a joint whose applied torque exceeds this
```

So the scale factor is `2.5 / 2.94 ≈ 0.85 A per N·m`. `total_current_a` (sim_monitor.py:30) sums `estimate_current_a` across all joints. `format_status` (sim_monitor.py:35) builds the one-line readout: per-leg angles, peak torque + joint name, total estimated current, appends `" [WARN >10A]"` when total current exceeds `CURRENT_LIMIT_A`, and lists `[STALL]` joints whose applied torque exceeds `STALL_WARN_NM` (2.5 N·m).

The monitor is driven by `_make_step_monitor` (gaits.py:217): an `on_step(i)` callback that fires every 30 steps (~8 Hz at 240 Hz) and prints `sim_monitor.format_status(i * TIMESTEP, torques, pos)`. Enabled by `--monitor`.

The three pure helpers (`estimate_current_a`, `total_current_a`, `format_status`) take plain numbers/dicts and are unit-testable without PyBullet; only the two `read_*` functions touch the engine (they `import pybullet` lazily inside the function body — sim_monitor.py:70,77).

---

## 8. Debug visualizer configuration

GUI runs strip the default visualizer chrome and enable shadows immediately after connect:

```python
# gaits.py:243 (and identical in view_urdf.py:29)
if gui:
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
```

- `COV_ENABLE_GUI, 0` — hides the side parameter/explorer panels for a clean view.
- `COV_ENABLE_SHADOWS, 1` — turns shadow rendering on.
- The three `*_PREVIEW, 0` flags — disable the RGB/depth/segmentation preview thumbnails (small perf win, less clutter).

Camera framing:

```python
# gaits.py:292
p.resetDebugVisualizerCamera(
    cameraDistance=0.55,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.1],
)
```

Positions the orbit camera 0.55 m out, yawed 45°, pitched −25° (looking slightly down), aimed at `[0,0,0.1]` (≈ body height). `view_urdf.py:48` uses near-identical values (`cameraDistance=0.6`).

### `addUserDebugLine` — gait trajectory overlay

`_draw_overlay` (gaits.py:138) draws each leg's foot-trajectory loop and a small 3-axis marker at the live target, reusing line IDs across frames so lines are updated rather than re-created:

```python
# gaits.py:147 (trajectory polyline segment)
_LINE_IDS[key] = p.addUserDebugLine(
    prev,
    cur,
    lineColorRGB=color,
    lineWidth=1.5,
    replaceItemUniqueId=lid if lid is not None else -1,
)
```

```python
# gaits.py:167 (per-target axis-cross marker, lineWidth=3.0)
_MARK_IDS[mkey] = p.addUserDebugLine(
    p0, p1, lineColorRGB=color, lineWidth=3.0,
    replaceItemUniqueId=mid if mid is not None else -1,
)
```

`replaceItemUniqueId` is the key trick: passing a previously returned line id updates that line in place (avoiding unbounded debug-item growth); `-1` means "create new" on the first frame. Per-leg colours come from `_TRAJ_COLORS` (gaits.py:51). The overlay only runs in GUI gait mode, redrawn every 4th step (gaits.py:439,450). `view_urdf.py` uses no debug lines.

---

## Stubs / not-implemented

None in the engine-setup path. The only stubs in the simulation tree are `teleop.py` and `terrain.py` (every entry point raises `NotImplementedError` per CLAUDE.md) — neither is on the PyBullet connect/step path and neither is a focus file here. `view_urdf.py` itself is slated for removal (redundant with `facehugger.py sim`) but is fully functional today.
