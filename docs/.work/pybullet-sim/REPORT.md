# FaceHugger PyBullet Simulation — Technical Report

*A walkthrough of how the FaceHugger quadruped is simulated in PyBullet: the files
involved, the physics setup, the gait and animation drivers, and — most subtly —
how the simulation understands the project's servo/joint conventions and proves
it agrees with the firmware.*

Generated 2026-05-26 on branch `feat/pybullet-sim-interpreter`. Every code
reference is `file:line` against this worktree, quoted from live source.

---

## 0. Executive summary

The FaceHugger simulation is a thin, faithful PyBullet front-end over a
**URDF-derived** kinematic model. Nothing about the robot's geometry is
hand-typed: link lengths, mount points, joint axes and zero poses all flow out of
the Fusion 360 CAD export into `generated/facehugger.urdf`, and every Python
consumer reads them back from there. PyBullet itself is used for three things
only — building the joint-name→index map, running the physics integrator, and
applying position-controlled motor commands. All the interesting engineering sits
*around* PyBullet: the gait trajectory generator, the inverse kinematics, and the
clip interpreter that replays baked animation clips through the exact same
servo-convention math the firmware uses.

There are two ways to drive the robot:

1. **Procedural gaits** (`gaits.py` + `kinematics.py`) — a phase-clock generates
   foot trajectories, IK turns them into joint angles, and those are pushed to
   PyBullet's position controllers.
2. **Clip playback** (`pybullet_interpreter/`) — baked animation clips
   (`clips_all.h`, math-space degrees) are interpolated per frame, converted
   through `translate_to_servo → clamp → servo_to_radians`, and pushed to the same
   controllers. This path is byte-for-byte identical to the ESP32 firmware's clip
   pipeline, and a parity test chain proves it.

---

## 1. File map

The simulation lives in `code/simulation/`. Files are grouped by role.

### Engine + entry points

| File | Lines | Role |
|---|---:|---|
| `facehugger.py` | 290 | CLI dispatcher: `urdf` / `sim` / `view` / `blender` / `all` subcommands. |
| `simulate.py` | 107 | `argparse` front-end for `sim`. Does **no** PyBullet work — hands off to `gaits.py`. |
| `gaits.py` | 477 | The real PyBullet wiring: connect, load, configure physics, the step loop, and the gait engine. |
| `helpers.py` | 229 | Shared plumbing: URDF XML parsing, joint-name→index map, the `setJointMotorControl2` wrappers. |
| `constants.py` | 20 | `TIMESTEP = 1/240`, `URDF_PATH`, stance angles. |
| `sim_monitor.py` | 82 | `--monitor` readout: applied torque from `getJointState`, linear torque→current model. |
| `view_urdf.py` | 65 | Standalone no-physics URDF viewer. Functional but slated for removal. |

### Kinematics + config

| File | Lines | Role |
|---|---:|---|
| `kinematics.py` | 362 | `build_config()` (reads geometry from the URDF), forward kinematics `fk_v2`, inverse kinematics `ik_v2`. |
| `facehugger_config.yaml` | — | Deliberately thin: only data CAD can't know (servo effort/velocity, leg ids/mounts). |
| `generate_urdf.py` | 1188 | CAD-export → URDF generator. Owns the shoulder-rest derivation. |
| `generated/facehugger.urdf` | — | The kinematic source of truth. Never hand-edited. |

### Clip interpreter (`pybullet_interpreter/`)

| File | Lines | Role |
|---|---:|---|
| `servo_convention.py` | 158 | **The convention engine.** Math-space → servo → radians, ported line-for-line from firmware. |
| `clip_loader.py` | 140 | Parses `clips_all.h` (C arrays) into `ClipData`/`ClipFrame` dataclasses. |
| `clip_player.py` | 215 | Interpolates frames, maps joints, and drives PyBullet. Lazy `import pybullet`. |
| `gait_interpreter.py` | 15 | Stub — `NotImplementedError`. Reserved for future gait playback. |
| `tests/` | — | Pure-math tests (no PyBullet) that lock down the convention contract. |

### Verification

| File | Lines | Role |
|---|---:|---|
| `verify_export_parity.py` | 141 | Proves the sim interprets exported clips identically to the firmware/browser. |
| `test_sim_monitor.py` | 50 | Unit tests for the torque/current math. |

### Stubs (look usable, aren't)

`teleop.py` and `terrain.py` import cleanly but every entry point raises
`NotImplementedError` — survivors from a pre-merge branch. Neither sits on the
PyBullet path.

---

## 2. How the simulation talks to PyBullet

`simulate.py` is only an argument parser; the canonical engine-setup path is
`gaits._connect_and_setup`. The same eight-step pattern is repeated (more simply)
by the standalone `view_urdf.py`.

### 2.1 Connecting to the physics server

```python
# gaits.py:242
p.connect(p.GUI if gui else p.DIRECT)
```

`p.GUI` opens an OpenGL window with the debug visualizer; `p.DIRECT` runs headless
in-process (used by `--headless` / CI). The ground plane is loaded from PyBullet's
bundled assets, but **not** in `--float` mode (where the body is pinned mid-air
with no gravity, purely to inspect joint geometry):

```python
# gaits.py:249
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# gaits.py:255
if not float_mode:
    p.loadURDF("plane.urdf")
```

`getDataPath()` returns PyBullet's install dir (which contains `plane.urdf`);
`setAdditionalSearchPath` lets a bare filename resolve.

### 2.2 Loading the robot

```python
# gaits.py:258
robot_id = p.loadURDF(
    cfg.urdf_path,
    basePosition=[0, 0, cfg.body_height + 0.02],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=float_mode,
)
```

- `basePosition` Z is `body_height + 0.02` — a 20 mm clearance so feet don't start
  intersecting the floor.
- `useFixedBase=float_mode` — when `True`, the base is welded to the world (float
  inspection mode); when `False` (normal stand/walk/clip) the base is a free
  6-DOF body subject to gravity and contact, which is what lets you watch the
  robot balance or collapse.
- No `flags=` are passed, so PyBullet uses the URDF's declared inertials and
  default collision filtering.

The URDF path is **not** hardcoded; it comes from `cfg.urdf_path`
(`constants.URDF_PATH = generated/facehugger.urdf`).

### 2.3 Physics configuration — and what each tweak buys

```python
# gaits.py:250-254
p.setGravity(0, 0, 0 if float_mode else -9.81)
p.setTimeStep(TIMESTEP)                                 # 1/240 s
p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERATIONS)  # 150
```

| Call | Value | Why |
|---|---|---|
| `setGravity` | −9.81 m/s² (0 in float) | Standard gravity; zeroed when the body is pinned. |
| `setTimeStep` | 1/240 s (`constants.py:12`) | PyBullet's recommended stable contact step (240 Hz). |
| `setPhysicsEngineParameter(numSolverIterations=150)` | 150 (`gaits.py:25`) | Stiffer, less-jittery contact resolution for multi-foot stance. |

Two `changeDynamics` calls add realism:

```python
# gaits.py:274  — override the bare URDF base with a realistic chassis mass
p.changeDynamics(robot_id, -1, mass=BODY_MASS_KG)       # 0.8 kg, link index -1 = base

# gaits.py:281  — make the feet grip
for name, idx in joint_map.items():
    if "link3" in name:                                 # link3 child = lower leg / foot
        p.changeDynamics(robot_id, idx,
            lateralFriction=FOOT_LATERAL_FRICTION,       # 1.2 (vs PyBullet default 0.5)
            spinningFriction=FOOT_SPINNING_FRICTION,     # 0.05 — resists pivoting in place
            restitution=0.0)                             # no bounce
```

A fixed bug is documented in place (`gaits.py:276-280`): the link is matched on
the substring `"link3"`, **not** `"knee"` — an earlier `"knee"` match never fired
(URDF joints are named `*_link3_joint`), leaving feet at the default 0.5 friction
and slipping.

### 2.4 Joint discovery & mapping

The joint-name→index dictionary is built once per load:

```python
# helpers.py:175
def build_joint_map(robot_id):
    out = {}
    n = p.getNumJoints(robot_id)
    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        out[info[1].decode()] = i      # info[1] = joint name (bytes); info[2] = type
    return out
```

Shape: `{"fl_link1_joint": 0, "fl_link2_joint": 1, ...}`. Fixed joints are skipped
at command/reset time by comparing the type field against `p.JOINT_FIXED`:

```python
# helpers.py:188
info = p.getJointInfo(robot_id, idx)
if info[2] == p.JOINT_FIXED:
    continue
```

The semantic role (shoulder/hip/knee) is recovered from the *name*, not the index
(`link1→shoulder`, `link2→hip`, `link3→knee`; leg prefix = `name[:2]`).

> **Design note** (`helpers.py:65`, `simulate.py:6-11`): joint *geometry* (origins,
> axes, limits) is parsed from the raw URDF XML via `xml.etree`, **not** from
> `getJointInfo` — because PyBullet silently shifts link frames to the COM when an
> inertial `<origin>` is non-zero, which would corrupt parent-frame origins.
> PyBullet supplies the index map and the physics; the XML is the geometric truth.

### 2.5 Actuation — POSITION_CONTROL

Every motor command goes through `setJointMotorControl2` in position-control mode:

```python
# helpers.py:222
p.setJointMotorControl2(
    robot_id, idx, p.POSITION_CONTROL,
    targetPosition=angle,    # radians (IK output or clip angle)
    force=force,             # cfg.servo_force = 2.94 N·m — the torque ceiling
    maxVelocity=velocity,    # cfg.servo_velocity = 5.0 rad/s — the slew limit
)
```

`POSITION_CONTROL` is PyBullet's built-in PD servo. Only `force` (the saturation
torque, which produces the `[STALL]` readings in §2.7) and `maxVelocity` are
tuned; `positionGain`/`velocityGain` use PyBullet defaults. At load, joints are
first **teleported** to stance with `p.resetJointState` (no physics), then the
position motors are armed to *hold* that stance against gravity.

### 2.6 The step loop

All three entry points share the pattern *command → step → monitor → pace*:

```python
# gaits.py:346 (run_stand, minimal form)
while p.isConnected():
    p.stepSimulation()                 # advance physics one TIMESTEP
    if on_step is not None: on_step(step)
    if gui: time.sleep(TIMESTEP)        # wall-clock pacing — GUI only; headless free-wheels
    step += 1
```

Stepping is manual (no `setRealTimeSimulation`). The gait loop adds, before each
step, an IK target computation, `apply_joint_targets(...)`, and every 4th step a
trajectory overlay redraw. `p.isConnected()` is the loop guard so closing the GUI
ends the loop cleanly; `p.disconnect()` always runs in `finally`.

### 2.7 The `--monitor` readout: torque & current

`sim_monitor.py` reads the torque the position controller actually applied:

```python
# sim_monitor.py:68
return {name: p.getJointState(robot_id, idx)[3] for name, idx in joint_map.items()}
#                                              ^ field [3] = appliedJointMotorTorque (N·m)
```

`getJointState` returns `(position, velocity, reactionForces,
appliedJointMotorTorque)`. Field `[3]` is meaningful precisely because the motor
runs in `POSITION_CONTROL`. A simple linear stall-point model maps torque→current:

```python
# sim_monitor.py:24
def estimate_current_a(torque_nm):
    return abs(torque_nm) * (STALL_CURRENT_A / STALL_TORQUE_NM)   # 2.5 / 2.94 ≈ 0.85 A per N·m
```

Constants: `STALL_TORQUE_NM = 2.94` (30 kg·cm servo), `STALL_CURRENT_A = 2.5`,
`CURRENT_LIMIT_A = 10.0` (supply budget → `[WARN >10A]`), `STALL_WARN_NM = 2.5`
(flags a `[STALL]` joint). The readout fires every 30 steps (~8 Hz).

### 2.8 Debug visualizer

GUI runs strip the default chrome, enable shadows, frame the camera, and draw the
gait trajectory as updatable debug lines:

```python
# gaits.py:243
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)            # hide side panels
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)   # + DEPTH, SEGMENTATION = 0
# gaits.py:292
p.resetDebugVisualizerCamera(cameraDistance=0.55, cameraYaw=45,
                             cameraPitch=-25, cameraTargetPosition=[0, 0, 0.1])
# gaits.py:147 — trajectory polyline; replaceItemUniqueId reuses the line each frame
_LINE_IDS[key] = p.addUserDebugLine(prev, cur, lineColorRGB=color, lineWidth=1.5,
                                    replaceItemUniqueId=lid if lid is not None else -1)
```

`replaceItemUniqueId` is the trick that keeps the overlay from growing unbounded:
passing back a previous line id updates it in place; `-1` means "create new".

---

## 3. Procedural gaits & kinematics

Data flow:

```
GAITS table ──▶ gait_joint_targets(t) ──▶ foot_target(phase)  (body-frame XYZ)
                                       └─▶ cfg.leg_ik (= ik_v2) ──▶ (θ_s, θ_h, θ_k)
                                                                  └─▶ apply_joint_targets
                                                                       └─▶ p.setJointMotorControl2
```

### 3.1 The kinematic model (`kinematics.py`)

The per-leg chain (`kinematics.py:3-11`):

```
T = Rz(yaw_offset + θ_s) · Ttrans(L1) · Ry(s_h) · Ttrans(L2) · Ry(s_k) · Ttrans(foot_L3)
    s_h = leg.hip_axis_sign  · θ_h
    s_k = leg.knee_axis_sign · θ_k
```

- **Shoulder** `θ_s` — yaw about body **+Z**, plus the URDF's baked `yaw_offset`.
- **Hip** `θ_h` / **knee** `θ_k` — pitch about **±Y** (L pair +Y, R pair −Y).
- **User-facing angles are uniform**: the same numeric `θ_h`/`θ_k` puts every leg
  in the same *physical* pose; the per-leg `axis_sign` carries the mirroring so the
  FK matches what PyBullet applies.

**All geometry is URDF-derived** — `build_config()` reads offsets straight from the
parsed URDF joints, never hardcoding them:

```python
# kinematics.py:324
mount      = sh_joint["xyz"]
yaw_offset = sh_joint["rpy"][2]
L1_vec     = hip_joint["xyz"]
L2_vec     = knee_joint["xyz"]
```

**Inverse kinematics** (`ik_v2`) decouples shoulder yaw with `atan2` (the Y
component in the link1 frame is invariant under both Ry rotations, so it's a per-leg
constant), then solves a planar 2-link triangle with the law of cosines:

```python
# kinematics.py:155 — decouple yaw
w_y = leg.L1_vec[1] + leg.L2_vec[1] + leg.foot_L3[1]
w_x = chain_x_sign * math.sqrt(max(dx*dx + dy*dy - w_y*w_y, 0.0))
theta_s = _wrap_pi(math.atan2(dy, dx) - math.atan2(w_y, w_x) - leg.yaw_offset)

# kinematics.py:179 — 2-link law-of-cosines, clamped to a reachable triangle
d = min(max(math.hypot(u, v), abs(l2 - l3) + 1e-6), l2 + l3 - 1e-6)
cos_kprime = (d*d - l2*l2 - l3*l3) / (2.0*l2*l3)
kprime = math.acos(_clamp(cos_kprime, -1.0, 1.0))
```

An unreachable foot target yields the nearest reachable pose, not an error. A
self-check at startup (`gaits.py:319`) round-trips the neutral foot through `leg_ik`
and asserts the recovered angles match the stance within 2°.

### 3.2 Config — deliberately thin

`facehugger_config.yaml` supplies only what CAD can't know. Servo drive
properties feed every motor command:

```yaml
# facehugger_config.yaml:95
servo:
  mass_kg: 0.300
  effort_nm: 2.94        # 30 kg·cm → N·m  (→ cfg.servo_force)
  velocity_rad_s: 5.0    # (→ cfg.servo_velocity)
```

Legs carry only id / mount-point / mesh-side. The shoulder-rest fields were
**removed** after "Convention A" landed (they're derived in code now), and **gait
params are not here** — they live in the `GAITS` dict in `gaits.py`. Stance bend
lives in `constants.py` (`STANCE_DEG = {"hip": -40, "knee": -60}`), shoulder
stance is 0.

### 3.3 The gait engine

```python
# gaits.py:32
GAITS = {
  "walk": {"period": 2.4, "step_length": 0.04, "step_height": 0.02, "duty": 0.25,
           "offsets": {"fl": 0.00, "br": 0.25, "fr": 0.50, "bl": 0.75}},   # one leg at a time
  "trot": {"period": 0.8, "step_length": 0.05, "step_height": 0.025, "duty": 0.5,
           "offsets": {"fl": 0.0, "br": 0.0, "fr": 0.5, "bl": 0.5}},        # diagonal pairs
}
```

- **Walk** — slow, `duty=0.25` so 3 feet are always planted (statically stable);
  legs lift one at a time FL→BR→FR→BL.
- **Trot** — fast, `duty=0.5`, diagonal pairs (fl+br, fr+bl) move together.

A normalized phase clock drives each leg with its offset; the foot target is a
sine-arc swing followed by a flat ground-push stance, relative to each leg's
FK-computed neutral foot:

```python
# gaits.py:64 (foot_target, condensed)
if phase < duty:                                   # SWING (in the air)
    s = phase / duty
    d  = -step_length*0.5 + s*step_length          # back → front, linear
    dz = step_height * math.sin(math.pi * s)       # half-sine lift
else:                                              # STANCE (on the ground)
    s = (phase - duty) / (1.0 - duty)
    d  = step_length*0.5 - s*step_length           # front → back push
    dz = 0.0
return (nx, ny + d, nz + dz)                        # body +Y = forward
```

Each frame, `gait_joint_targets(cfg, gait, t)` computes the three joint angles per
leg via `leg_ik` and maps them to `{leg}_link{1,2,3}_joint`, which
`apply_joint_targets` pushes to PyBullet. `t += TIMESTEP` advances the phase so the
gait tracks sim time exactly.

### 3.4 The shoulder-rest convention

Implemented in `generate_urdf.py`, not the sim (it bakes into the URDF origins):

```python
# generate_urdf.py:182
if leg_id == "fl": return fl_rest_rad                       # −45°
if leg_id == "fr": return -fl_rest_rad                      # +45°
if leg_id == "bl": return -_wrap_pi(fl_rest_rad + math.pi)  # −135°
if leg_id == "br": return  _wrap_pi(fl_rest_rad + math.pi)  # +135°
```

`_wrap_pi` normalizes any angle into `(−π, π]`. The four legs splay outward into
their quadrants. **Note:** the signs here (`BL = −wrap_pi(FL+π)`, `BR =
+wrap_pi(FL+π)`) are the *corrected* version — a docstring at `generate_urdf.py:177`
records that an earlier revision had BL/BR swapped, sending the back legs into the
front quadrants.

---

## 4. The clip interpreter — how conventions are understood

The `pybullet_interpreter` package is the headline feature of this branch. Its
mission (README): **mirror the firmware clip pipeline exactly** so a clip can be
validated visually in sim before flashing to the ESP32.

### 4.1 Data flow

```
clips_all.h  (C arrays, pre-scaled math-space degrees)
   │  load_clips_all_h  (regex parse)
   ▼
list[ClipData] → frames: ClipFrame{t_ms, a[12]}
   │  _interpolate_frame(elapsed_ms)   (linear, clamped at ends)
   ▼
a[12] math-space degrees
   │  frame_to_joint_targets — per leg (FR, FL, RR, RL):
   │     translate_to_servo  →  clamp_clip_servos  →  servo_to_radians
   │     × LEG_ID_TO_URDF_AXIS_SIGN   (URDF <axis> correction, sim-only)
   ▼
dict[urdf_joint_name → radians]
   │  ClipPlayer.step
   ▼
pybullet.setJointMotorControl2(POSITION_CONTROL)  →  stepSimulation @ 240 Hz
```

### 4.2 Clip format & loader

The loader reads `animation/exported_clips/clips_all.h` (a **C header**, not `.fhc`
or JSON), generated by the Blender add-on. The crucial semantic: the `a[12]` values
are **already pre-scaled math-space degrees** — the exporter has baked in the
2/3-scale-from-NEUTRAL, and only `translate_to_servo` runs at playback
(`clip_loader.py:7-14`). The layout is firmware leg order:
`a[0..2]=FR(sh,th,kn)`, `a[3..5]=FL`, `a[6..8]=RR`, `a[9..11]=RL`. Two plain
dataclasses (`ClipFrame{t_ms, a}`, `ClipData{name, frames, frame_count,
duration_ms}`) hold the parsed data; a two-pass regex extracts the frame arrays and
the `FH_CLIPS[]` metadata table.

### 4.3 `servo_convention.py` — the crown jewel

This file is the single Python source of truth for `math-space → servo → radian`,
ported line-for-line from firmware `motion_math.cpp` / `neutral_pose.h`. It is how
the simulation "understands" the robot's conventions.

**The NEUTRAL standing pose** (math-space degrees per leg):

```python
# servo_convention.py:61
NEUTRAL = [
    NeutralPose(sh=45.0,   th=-60.0, kn=-37.0),  # FR
    NeutralPose(sh=135.0,  th=-60.0, kn=-40.0),  # FL  (# Change B: 135 so "outward" matches)
    NeutralPose(sh=-45.0,  th=-50.0, kn=-50.0),  # RR / BR
    NeutralPose(sh=-135.0, th=-60.0, kn=-35.0),  # RL / BL
]
```

**`translate_to_servo`** — the core. Each leg has its own formula encoding (a) the
physical servo mounting direction as a *sign*, and (b) the zero-offset (the
±45/±135 constant = that leg's shoulder rest yaw), so that **servo 90° = the
joint's math-space neutral**:

```python
# servo_convention.py:98
if leg_id == LEG_FR:
    out.hip = 90 + (sh - 45);   out.thigh = 90 - th;  out.knee = 90 + kn
elif leg_id == LEG_FL:
    out.hip = 90 + (sh - 135);  out.thigh = 90 + th;  out.knee = 90 - kn
elif leg_id == LEG_RR:   # BR un-mirrored (2026-05-25): +sh = +servo like the others
    out.hip = 90 + (sh + 45);   out.thigh = 90 + th;  out.knee = 90 - kn
elif leg_id == LEG_RL:
    out.hip = 90 + (sh + 135);  out.thigh = 90 - th;  out.knee = 90 + kn
```

How the conventions are encoded:

- **Shoulder/yaw** — `90 + (sh ± offset)`, the offset being that leg's rest yaw, so
  feeding the NEUTRAL `sh` yields exactly servo 90. The `+sh` sign is **uniform
  across all four legs** — there is no per-leg sign flip on the shoulder.
- **Thigh** — sign alternates per diagonal pair: FR/RL use `90 − th`, FL/RR use
  `90 + th`. This encodes the mirrored servo mounting of the body's left/right sides.
- **Knee** — the opposite alternation.

**The BR un-mirror (2026-05-25) — load-bearing detail.** The original plan and an
earlier firmware revision had `RR hip = 90 − (sh + 45)`. The shipped code flips
this to `90 + (sh + 45)`: the BR motor is an identical part with its yaw shaft on
the same vertical axis, so `+sh = +servo` like every other leg
(`servo_convention.py:108-110`). The firmware compensates elsewhere (`YAW_COEF[BR]`,
gait `fwdDir[BR]`) so gait output is unchanged. The test suite carries a verbatim
copy of the *new* formula as ground truth, so reverting BR would fail loudly.

**`clamp_clip_servos`** keeps joints off mechanical stops (hip [38,142], thigh
[30,150], knee [0,180]). **`servo_to_radians`** is a pure offset+scale:

```python
# servo_convention.py:158
return math.radians(servo_deg - 90.0)    # 90→0, 0→−π/2, 180→+π/2
```

This is clean precisely because the URDF joint zero is *defined* to coincide with
servo-90 neutral.

### 4.4 The URDF axis-sign correction (sim-only)

A subtlety **absent from the firmware** — physical motors don't care about URDF
`<axis>` orientation, but PyBullet does. The URDF emits `<axis xyz="0 ±1 0">` with
the diagonal pairs flipped, so the clip player multiplies by a per-leg sign:

```python
# servo_convention.py:38
LEG_ID_TO_URDF_AXIS_SIGN = {LEG_FR: -1, LEG_FL: +1, LEG_RR: +1, LEG_RL: -1}

# clip_player.py:54 (frame_to_joint_targets)
targets[f"{urdf_name}_link1_joint"] =  axis * servo_to_radians(servo.hip)
targets[f"{urdf_name}_link2_joint"] =  axis * servo_to_radians(servo.thigh)
targets[f"{urdf_name}_link3_joint"] = -axis * servo_to_radians(servo.knee)   # knee = −axis
```

This factor is applied **here**, not in the servo math — keeping the
firmware-parity math byte-identical while still rendering correctly in sim. Note
link1 (shoulder) *does* get the axis factor: without it, FR/BL shoulder yaw
rendered backwards. (A stale docstring at `clip_player.py:48-50` wrongly claims
link1 needs no correction; the lower comment at `:62-65` and the code are correct.)

### 4.5 Playback

`ClipPlayer.step` interpolates the current frame, converts it, resolves each named
joint to a PyBullet index via a pre-built `joint_map`, and issues position control
— importing `pybullet` **lazily inside the method**:

```python
# clip_player.py:145
import pybullet as p     # lazy: lets clip_loader & servo_convention be tested
                         # without a PyBullet connection (no macOS PyPI wheel)
...
p.setJointMotorControl2(self._robot_id, joint_idx, p.POSITION_CONTROL,
    targetPosition=rad, force=self._force, maxVelocity=self._velocity)
```

`play_blocking` runs a real-time wall-clock loop at a fixed 240 Hz step.
Behavioral modes: **headless** plays once and returns (CI smoke); **GUI** holds the
final pose after the last frame; **`loop=True`** restarts without resetting physics
state (so drift/foot-slip accumulate across loops, which is realistic). An optional
`on_step(step_index)` callback drives the torque/current monitor.

`gait_interpreter.tick_gait` is a `NotImplementedError` stub — gait *playback* (as
opposed to procedural gaits) is future work.

---

## 5. Conventions reference

The authoritative values (from code + URDF; the prose docs lag in several places —
project rule is to trust live code):

| Aspect | Value | Source |
|---|---|---|
| World frame | +Y forward, +X right, +Z up | `MERGE_AND_CONVENTION.md:46` |
| Leg naming (Py/URDF/Blender) | `fl`, `fr`, `bl`, `br` | `MERGE_AND_CONVENTION.md:43` |
| Leg naming (firmware) | `fr=0`, `fl=1`, `rr=2 (=br)`, `rl=3 (=bl)` | `servo_convention.py:18-21` |
| URDF joint names | `{leg}_link1/2/3_joint` | `facehugger.urdf:68,75,82` |
| link1 = shoulder/yaw | axis Z: `+Z` FL/BR, `−Z` FR/BL | `facehugger.urdf:72,256` |
| link2 = hip, link3 = knee | axis Y, same per-leg sign | `facehugger.urdf:79,86` |
| URDF θ=0 | = Fusion rest pose (baked into `<origin rpy>`) | `MERGE_AND_CONVENTION.md:106` |
| Shoulder rest FL/FR/BL/BR | −45° / +45° / −135° / +135° | `MERGE_AND_CONVENTION.md:151` |
| Shoulder rest derivation | `FR=−FL`, `BL=−wrap_pi(FL+π)`, `BR=+wrap_pi(FL+π)` | `generate_urdf.py:182` |
| servo→radian | `radians(servo_deg − 90)`; servo 90 = 0 rad | `servo_convention.py:158` |
| Clip-clamp ranges | hip [38,142], thigh [30,150], knee [0,180] | `servo_convention.py:135` |
| Servo numbering | `servo_id = leg*3 + joint` — **PROPOSAL** | `SERVO_ID_CONVENTION.md` |

### Why the URDF is the single source of truth

A hand-written `kinematics.py` from another branch was rejected: its hardcoded link
lengths (L1=0.080, L2=0.075, L3=0.077) were built against a *different* CAD
revision than the live export (≈0.058 / 0.095 / 0.097), and would deliver IK
solutions for the wrong robot. The design generates the URDF from CAD so that
"moving any construction point in CAD propagates to the URDF without human
intervention" (`URDF_PIPELINE.md:383`), and every consumer reads geometry from the
URDF via `helpers._load_urdf_joints` / `kinematics.build_config`.

### A representative URDF joint

```xml
<!-- LEG: FL  (side=L, shoulder_rest=-45.0°, limits=[-52.0°, +90.0°]) -->
<!-- facehugger.urdf:67 -->
<joint name="fl_link1_joint" type="revolute">
  <parent link="base_link"/>
  <child link="fl_link1"/>
  <origin xyz="-0.050401 0.043222 0.036844" rpy="0 0 -0.785398"/>  <!-- −45° rest baked in -->
  <axis xyz="0.0 0.0 1.0"/>
  <limit lower="-0.907571" upper="1.570796" effort="2.94" velocity="5.0"/>
</joint>
```

---

## 6. Sim ↔ firmware parity — the guarantee

`verify_export_parity.py` proves the sim interprets exported clips identically to
the firmware/browser, by deriving each servo command from *independent* paths and
asserting agreement:

- **Path 1** — `clips_all.h` (math-space) → the sim's own `translate_to_servo`.
- **Path 2** — the per-clip `<clip>.js` (servo degrees the browser streams).

```python
# verify_export_parity.py:101
for i, (frame, js) in enumerate(zip(clip.frames, js_frames)):
    sim = _sim_servo_js_equiv(list(frame.a))      # runs translate_to_servo per leg
    for leg in _JS_ORDER:
        if sim[leg] != js[leg]:                    # EXACT integer equality after shared [0,180] clamp
            errors.append(...)
```

The two paths are produced by **separate implementations** of the
`translateToServo` contract (the sim's `servo_convention.py` vs the exporter's
`fh_clip_panel._frame_to_servo`). The firmware C++ is locked to the exporter
*separately* by the Unity test `code/firmware/test/test_clip_parity` (1° tolerance,
references generated from the exporter). So the full chain closes the loop:

```
clips_all.h → sim servo_convention   ≡   .js (exporter _frame_to_servo)     [exact int]
exporter _frame_to_servo             ≡   firmware translateToServo          [1° tolerance]
────────────────────────────────────────────────────────────────────────────────────────
∴  sim's PyBullet servo output       ≡   firmware's physical servo command   (every clip frame)
```

The sim's `translate_to_servo` is byte-identical to the firmware
`motion_math.cpp:translateToServo` — same per-leg formulas including the 2026-05-25
BR un-mirror. The pure-math test suites (no PyBullet) carry a verbatim firmware
formula copy as independent ground truth and validate the loader against a JSON
manifest, so the convention contract is regression-locked from both ends.

---

## 7. Known doc-vs-code drift (flag list)

For maintainers — places where prose docs lag the live code/URDF:

1. **Joint names** — docs say `*_shoulder/hip/knee_joint`; URDF + code use
   `*_link1/2/3_joint` (renamed in commit `80b8e98`).
2. **Shoulder ROM** — docs claim symmetric `±90°`; the URDF emits `[−52°, +90°]`
   for FL.
3. **Shoulder axis "uniform +Z"** — the URDF actually has `+Z` for FL/BR, `−Z` for
   FR/BL; the sim corrects this with the per-leg `LEG_ID_TO_URDF_AXIS_SIGN`.
4. **`clip_player.py:48-50` docstring** — wrongly says link1 needs no axis
   correction; the code applies it (and is right to).
5. **README/plan actuation path** — describe `helpers.apply_joint_targets`; the
   shipped `ClipPlayer.step` calls `setJointMotorControl2` directly.
6. **Plan's BR formula** (`90 − (sh+45)`) — superseded by the shipped
   `90 + (sh+45)`; plan not updated.
7. **`servo_mapping.yaml`** — documented but not present in this worktree; servo
   numbering is an explicit PROPOSAL pending firmware confirmation.

---

*Sources: `code/simulation/` live source as of branch `feat/pybullet-sim-interpreter`,
2026-05-26. Per-aspect research notes in `.work/research/01–04`.*
