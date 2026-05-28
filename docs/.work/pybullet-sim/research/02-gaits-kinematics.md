# Gait Generation & Kinematics — FaceHugger PyBullet Sim

How leg motion is produced and turned into PyBullet joint commands. The data
flow is:

```
GAITS table  ──▶ gait_joint_targets(t)  ──▶ foot_target(phase)  (body-frame XYZ)
                                          └─▶ cfg.leg_ik (= ik_v2)  ──▶ (θ_s, θ_h, θ_k)
                                                                       └─▶ apply_joint_targets
                                                                            └─▶ p.setJointMotorControl2
```

Everything kinematic is **URDF-derived**: link lengths come from the URDF joint
origins parsed at `build_config()` time, not hand-typed.

---

## 1. Leg naming convention

Python/URDF use **`fl`, `fr`, `bl`, `br`** (front-left, front-right, back-left,
back-right). The gait `offsets` and the config `legs[]` list are keyed on these.
Joint names in the URDF/`joint_map` are `{leg}_link1_joint` (shoulder),
`{leg}_link2_joint` (hip), `{leg}_link3_joint` (knee). `apply_leg_pose` recovers
the leg id with `leg = name[:2]`. The firmware's alternate naming
(`fr/fl/rr/rl`) is *not* used in this layer.

---

## 2. Kinematics model (`kinematics.py`)

### Coordinate frame & joint definitions

The per-leg chain (docstring, `kinematics.py:3-11`):

```python
# kinematics.py:3
#   T = Rz(yaw_offset + theta_s) *
#       Ttrans(L1) *
#       Ry(s_h) * Ttrans(L2) *
#       Ry(s_k) * Ttrans(foot_L3)
#   s_h = leg.hip_axis_sign  * theta_h
#   s_k = leg.knee_axis_sign * theta_k
```

- **shoulder** (`theta_s`) rotates about body **+Z** (yaw), offset by the joint's
  fixed `yaw_offset` (the rpy-Z baked into the URDF shoulder joint).
- **hip** (`theta_h`) and **knee** (`theta_k`) rotate about **±Y** (the L pair has
  axis +Y, the R pair -Y — "Phase H sign flip").
- **User-facing angles are uniform**: the same numeric `theta_h`/`theta_k` puts
  every leg in the same *physical* pose. The internal `s_h`/`s_k` carry the
  per-leg axis sign so the FK matches what PyBullet actually applies
  (`kinematics.py:13-16`).

`LegGeom` holds the per-leg data (`kinematics.py:48-60`): `mount` (shoulder origin
in body frame), `yaw_offset`, `L1_vec`/`L2_vec`/`foot_L3` (the three link offset
vectors), `hip_axis_sign`/`knee_axis_sign`, and per-leg `joint_limits`.

### Where link lengths come from (URDF-derived)

In `build_config()` (`kinematics.py:320-328`), the offsets are read straight from
the parsed URDF joints — **not** hand-coded:

```python
# kinematics.py:324
mount = sh_joint["xyz"]
yaw_offset = sh_joint["rpy"][2]
L1_vec = hip_joint["xyz"]
L2_vec = knee_joint["xyz"]
foot_L3 = foot_L3_R if side == "R" else foot_L3_L
```

The foot tip (`foot_L3`) comes from the URDF's LEG-ASSEMBLY metadata comment
(`FootTip`, in mm, link3-mesh frame), converted to metres. The R-pair tip is the
L tip rotated by Ry(π) because the R link3 visual carries `rpy=(0,π,0)`
(`kinematics.py:285-291`):

```python
# kinematics.py:290
foot_L3_L = tuple(v / 1000.0 for v in foot_tip_link3_mm)
foot_L3_R = (-foot_L3_L[0], foot_L3_L[1], -foot_L3_L[2])
```

Axis signs are read from the URDF joint axis vectors, tolerating float dust
(`kinematics.py:248-251`):

```python
# kinematics.py:248
def _axis_sign_y(axis_xyz):
    return 1 if axis_xyz[1] >= 0.0 else -1
```

### Rotation helpers

```python
# kinematics.py:87
def _ry(v, c, s):
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)

# kinematics.py:95
def _rz(v, c, s):
    x, y, z = v
    return (x * c - y * s, x * s + y * c, z)
```

`_ry` applies a right-hand-rule rotation about +Y given precomputed
`c=cos`, `s=sin` (note: it rotates the (X,Z) plane *by -θ* in the usual screen
convention, which the IK comments rely on).

### Forward kinematics — `fk_v2` (`leg_fk`)

Composes the chain inside-out: rotate foot into link2 frame, add L2; rotate into
link1 frame, add L1; rotate by shoulder yaw and add the mount
(`kinematics.py:111-127`):

```python
# kinematics.py:112
s_h = leg.hip_axis_sign * theta_h
s_k = leg.knee_axis_sign * theta_k
ch, sh = math.cos(s_h), math.sin(s_h)
ck, sk = math.cos(s_k), math.sin(s_k)

# foot in link2 frame: Ry(s_k) * foot_L3 + L2
f_l2 = _ry(leg.foot_L3, ck, sk)
v_l2 = (leg.L2_vec[0] + f_l2[0], leg.L2_vec[1] + f_l2[1], leg.L2_vec[2] + f_l2[2])
# foot in link1 frame: Ry(s_h) * v_l2 + L1
v_l1 = _ry(v_l2, ch, sh)
w = (leg.L1_vec[0] + v_l1[0], leg.L1_vec[1] + v_l1[1], leg.L1_vec[2] + v_l1[2])
# foot in body frame: mount + Rz(yaw_offset + theta_s) * w
q = leg.yaw_offset + theta_s
cq, sq = math.cos(q), math.sin(q)
foot = _rz(w, cq, sq)
return (leg.mount[0] + foot[0], leg.mount[1] + foot[1], leg.mount[2] + foot[2])
```

### Inverse kinematics — `ik_v2` (`leg_ik`)

Foot (body frame) → user-facing `(θ_s, θ_h, θ_k)`. Five-step strategy in the
docstring (`kinematics.py:138-147`). Key math:

**1. Decouple shoulder yaw.** The Y component in the link1 frame is invariant
under both Ry rotations, so it is a per-leg constant; the XY-norm is preserved by
the Rz yaw (`kinematics.py:155-176`):

```python
# kinematics.py:155
w_y = leg.L1_vec[1] + leg.L2_vec[1] + leg.foot_L3[1]
rxy2 = dx * dx + dy * dy
w_x2 = rxy2 - w_y * w_y
if w_x2 < 0.0:
    w_x2 = 0.0  # target laterally closer than w_y alone can reach
chain_x_sign = (
    1.0 if (leg.L1_vec[0] + leg.L2_vec[0] + leg.foot_L3[0]) >= 0 else -1.0
)
w_x = chain_x_sign * math.sqrt(w_x2)
w_z = dz
q = math.atan2(dy, dx) - math.atan2(w_y, w_x)
theta_s = _wrap_pi(q - leg.yaw_offset)
```

`chain_x_sign` picks which branch the chain extends along X (-X for L, +X for R)
so FK/IK round-trips.

**2. Planar 2-link solve in (X,Z) relative to L1.** Law of cosines on the
L2/foot_L3 triangle, clamped to a reachable range (`kinematics.py:179-206`):

```python
# kinematics.py:179
u = w_x - leg.L1_vec[0]
v = w_z - leg.L1_vec[2]
l2 = math.hypot(leg.L2_vec[0], leg.L2_vec[2])
l3 = math.hypot(leg.foot_L3[0], leg.foot_L3[2])
d = math.hypot(u, v)
d = min(d, l2 + l3 - 1e-6)            # keep triangle reachable
d = max(d, abs(l2 - l3) + 1e-6)
cos_kprime = (d * d - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
kprime = math.acos(_clamp(cos_kprime, -1.0, 1.0))  # [0, pi]
alpha_rel = _wrap_pi(
    math.atan2(leg.foot_L3[2], leg.foot_L3[0])
    - math.atan2(leg.L2_vec[2], leg.L2_vec[0])
)
s_k = chain_x_sign * kprime - alpha_rel
```

`alpha_rel` is the angle of `foot_L3` measured *from L2's direction* in the X-Z
plane (the links are not axis-aligned, so this offset is needed). The knee branch
(`chain_x_sign * kprime`) folds the lower leg so the foot supports the body.

**3. Recover the hip.** The bent chain sits at angle `phi_V`; `(u,v)` at `phi_uv`;
since Ry(s_h) rotates by -s_h, `s_h = phi_V - phi_uv` (`kinematics.py:211-214`):

```python
# kinematics.py:212
V_x = leg.L2_vec[0] + leg.foot_L3[0] * ck + leg.foot_L3[2] * sk
V_z = leg.L2_vec[2] - leg.foot_L3[0] * sk + leg.foot_L3[2] * ck
s_h = _wrap_pi(math.atan2(V_z, V_x) - math.atan2(v, u))
```

**4. Convert internal → user-facing, then per-leg clamp** (`kinematics.py:217-227`):

```python
# kinematics.py:217
theta_h = leg.hip_axis_sign * s_h
theta_k = leg.knee_axis_sign * s_k
...
return (
    _clamp(theta_s, lo_s, hi_s),
    _clamp(theta_h, lo_h, hi_h),
    _clamp(theta_k, lo_k, hi_k),
)
```

A self-check lives in `_print_banner` (`gaits.py:319-332`): it round-trips
`neutral_foot` through `leg_ik` and asserts the recovered angles match the stance
within 2°.

---

## 3. Config — `facehugger_config.yaml`

This yaml is deliberately thin: link lengths/limits/axes are **not** here — they
live in `fusion_export.json`/URDF. The yaml only supplies what CAD can't know.

**Servo properties** (drive force/velocity used in every motor command,
`facehugger_config.yaml:95-98`):

```yaml
# facehugger_config.yaml:95
servo:
  mass_kg: 0.300        # DFRobot SER0038 spec
  effort_nm: 2.94       # 30 kg·cm to N·m
  velocity_rad_s: 5.0
```

Read in `kinematics.py:293-295` into `servo_force` / `servo_velocity`.

**Legs** — only id, mount point, mesh side (`facehugger_config.yaml:72-87`):

```yaml
# facehugger_config.yaml:72
legs:
  - id: fl
    mount_point: LegMountPointFL
    side: L
  - id: br
    mount_point: LegMountPointBR
    side: L
  - id: fr
    mount_point: LegMountPointFR
    side: R
  - id: bl
    mount_point: LegMountPointBL
    side: R
```

**Shoulder rest convention is documented here but derived in code** — three
former yaml fields were dropped after "Convention A" landed
(`facehugger_config.yaml:57-68`):

```yaml
# facehugger_config.yaml:59
#   - rpy_z_deg            -> derived from the FL Fusion rest in JSON
#                             (FR=-FL, BL=wrap_pi(FL+π), BR=-wrap_pi(FL+π)).
#   - shoulder_limits_deg  -> URDF limits = Fusion limits shifted relative to FL's rest
#   - shoulder_neutral_deg -> identical to URDF θ=0 under Convention A
#                             Stance code reads stance_rad["shoulder"] = 0.
```

**Stance angles** are *not* in the yaml. Hip/knee bend is shared in `constants.py`,
shoulder stance is 0 (`constants.py:17-20`):

```python
# constants.py:17
STANCE_DEG = {
    "hip": -40.0,
    "knee": -60.0,
}
```

`_stance_for` (`kinematics.py:235-245`) builds the per-leg stance dict: shoulder
from `shoulder_neutral_deg` (defaults to 0 since the field was removed), hip/knee
from `STANCE_DEG`.

**Gait params are NOT in the yaml** — they live in the `GAITS` dict in
`gaits.py` (see §4). The yaml has no step-height / frequency fields.

---

## 4. Gait engine (`gaits.py`)

### Gait registry — phase offsets, periods, step size

```python
# gaits.py:32
GAITS = {
    "walk": {
        "period": 2.4,
        "step_length": 0.04,
        "step_height": 0.02,
        "duty": 0.25,
        "offsets": {"fl": 0.00, "br": 0.25, "fr": 0.50, "bl": 0.75},
        "label": "Static walk",
    },
    "trot": {
        "period": 0.8,
        "step_length": 0.05,
        "step_height": 0.025,
        "duty": 0.5,
        "offsets": {"fl": 0.0, "br": 0.0, "fr": 0.5, "bl": 0.5},
        "label": "Trot (diagonal pairs)",
    },
}
```

- **walk**: slow (2.4 s period), `duty=0.25` (each leg swings only 25% of the
  cycle → 3 feet always planted = statically stable). Offsets are evenly spread at
  0/0.25/0.5/0.75 so legs lift one at a time, in the order FL→BR→FR→BL.
- **trot**: fast (0.8 s), `duty=0.5`, **diagonal pairs move together** — `fl`+`br`
  at offset 0, `fr`+`bl` at offset 0.5. This is the classic dynamic trot.

`step_length`/`step_height` are in metres; `period` in seconds. Frequency =
1/period (≈0.42 Hz walk, 1.25 Hz trot).

### Cycle / phase computation

```python
# gaits.py:83
def gait_joint_targets(cfg, gait, t):
    offsets = gait["offsets"]
    global_phase = (t / gait["period"]) % 1.0
    targets = {}
    for leg_id, off in offsets.items():
        phase = (global_phase - off) % 1.0
        foot = foot_target(
            cfg.neutral_foot, leg_id, phase,
            gait["step_length"], gait["step_height"], gait["duty"],
        )
        s, h, k = cfg.leg_ik(cfg, foot, leg_id)
        targets[f"{leg_id}_link1_joint"] = s
        targets[f"{leg_id}_link2_joint"] = h
        targets[f"{leg_id}_link3_joint"] = k
    return targets
```

`global_phase` is the normalized cycle position in `[0,1)`; each leg's local
`phase` subtracts its offset. The foot target is fed straight into `leg_ik` and
mapped to the three joint names.

### Foot trajectory (swing vs stance shape)

```python
# gaits.py:64
def foot_target(neutral_foot, leg_id, phase, step_length, step_height, duty, swing_axis="y"):
    nx, ny, nz = neutral_foot[leg_id]
    if phase < duty:                       # SWING (foot in the air)
        s = phase / duty
        d = -step_length * 0.5 + s * step_length      # back -> front, linear
        dz = step_height * math.sin(math.pi * s)      # sine arc lift
    else:                                  # STANCE (foot on ground)
        s = (phase - duty) / (1.0 - duty)
        d = step_length * 0.5 - s * step_length       # front -> back, linear push
        dz = 0.0                                       # stays on ground
    if swing_axis == "y":
        return (nx, ny + d, nz + dz)       # body +Y = forward (default)
    return (nx + d, ny, nz + dz)
```

The trajectory is relative to each leg's `neutral_foot` (computed once via FK at
build time, `kinematics.py:350-359`). During **swing** (`phase < duty`) the foot
sweeps from `-step_length/2` to `+step_length/2` along the forward (+Y) axis while
lifting on a half-sine arc of amplitude `step_height`. During **stance**
(`phase ≥ duty`) it stays on the ground (dz=0) and pushes back from `+step_length/2`
to `-step_length/2`, propelling the body. `duty` is the swing fraction.

`_precompute_cycle` / `_draw_overlay` (`gaits.py:118-173`) sample 40 points of the
same trajectory to draw the GUI debug overlay (per-leg colored loop + a target
cross-hair) — purely visual.

### Spawn-height safety

`_body_height_for_gait` (`gaits.py:181-202`) samples the worst foot depth over a
full period and lifts `cfg.body_height` if a gait dips lower than the neutral
stance, so the body doesn't spawn intersecting the floor.

---

## 5. Applying joint angles to PyBullet + loop pacing

### The motor command

Targets go to `apply_joint_targets` (`helpers.py:216-229`):

```python
# helpers.py:216
def apply_joint_targets(robot_id, joint_map, targets, force, velocity):
    """targets = {joint_name: angle}"""
    for name, angle in targets.items():
        idx = joint_map.get(name)
        if idx is None:
            continue
        p.setJointMotorControl2(
            robot_id,
            idx,
            p.POSITION_CONTROL,
            targetPosition=angle,
            force=force,            # cfg.servo_force = 2.94 N·m
            maxVelocity=velocity,   # cfg.servo_velocity = 5.0 rad/s
        )
```

It uses **POSITION_CONTROL** — PyBullet runs an internal PD controller driving
each joint to `targetPosition`, capped by the servo's torque (`force`) and speed
(`maxVelocity`). The standing pose uses `apply_leg_pose` (`helpers.py:197-213`),
the same call keyed per-leg on the stance dict.

### The sim loop (paced at 240 Hz)

`run_gait` main loop (`gaits.py:442-477`):

```python
# gaits.py:442
t = 0.0
step = 0
try:
    while p.isConnected():
        targets = gait_joint_targets(cfg, gait, t)
        apply_joint_targets(
            robot_id, joint_map, targets, cfg.servo_force, cfg.servo_velocity
        )
        if draw_overlay and step % draw_every == 0:
            ...
            _draw_overlay(robot_id, cycles, cur_targets)

        p.stepSimulation()
        if on_step is not None:
            on_step(step)
        if gui:
            time.sleep(TIMESTEP)
        t += TIMESTEP
        step += 1
```

Each iteration: recompute targets from the current time `t`, push them to the
motors, step physics one tick, optionally redraw overlay (every 4 steps) and
print the `--monitor` torque/current readout. `TIMESTEP = 1.0 / 240.0`
(`constants.py:12`), set via `p.setTimeStep` in `_connect_and_setup`
(`gaits.py:251`). In GUI mode `time.sleep(TIMESTEP)` paces it to real time; headless
runs free. `t` advances by `TIMESTEP` per tick, so gait phase tracks sim time
exactly. `run_stand` (`gaits.py:335-358`) is the same loop minus the gait/IK step.

Before any loop, `_settle` (`gaits.py:205-214`) steps the sim holding stance for
~0.5 s so gravity resolves spawn overlap. `_connect_and_setup` also adds physics
realism: 0.8 kg body mass, foot lateral/spinning friction (1.2 / 0.05),
restitution 0, and 150 solver iterations (`gaits.py:22-25, 274-289`).

---

## 6. Per-leg shoulder rest convention

Documented in the yaml (§3) but **implemented in `generate_urdf.py`**
(`_shoulder_rest_for`, `generate_urdf.py:166-190`):

```python
# generate_urdf.py:182
if leg_id == "fl":
    return fl_rest_rad
if leg_id == "fr":
    return -fl_rest_rad
if leg_id == "bl":
    return -_wrap_pi(fl_rest_rad + math.pi)
if leg_id == "br":
    return _wrap_pi(fl_rest_rad + math.pi)
```

`fl_rest_rad` is read from the Fusion export (currently -π/4 = -45°). The four
legs splay outward into their quadrants: FL=-45°, FR=+45°, BL=-135°, BR=+135°.

> Note a subtlety: the task brief states `BL = wrap_pi(FL+π)` and
> `BR = -wrap_pi(FL+π)`, but the **current code has BL and BR swapped** relative
> to that — `BL = -wrap_pi(FL+π)`, `BR = +wrap_pi(FL+π)`. The docstring at
> `generate_urdf.py:177-180` explicitly calls out that an earlier version had them
> the other way and sent the back legs into the front quadrants; the present
> signs are the corrected ones.

### `wrap_pi`

```python
# helpers.py:21
def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi
```

Normalizes any angle into `(-π, π]`. Used here so that `FL + π` (a 180° rotation
of the front-left rest) lands back in the principal range — e.g. with FL=-45°,
`FL+π = +135°` which is already in range, giving BR=+135° and BL=-135°. The IK
also uses `_wrap_pi` throughout to keep `theta_s`, `s_h`, `s_k`, and `alpha_rel`
in the principal branch. (A duplicate copy exists in `generate_urdf.py:161-163`.)

---

## Stubs / caveats

- **No gait params in the yaml** — they are hardcoded in the `GAITS` dict in
  `gaits.py`. Tuning a gait means editing that dict, not config.
- **Shoulder stance = 0** is implicit (the `shoulder_neutral_deg` field was
  removed; `_stance_for` defaults to 0.0). Hip/knee stance lives in `constants.py`,
  not the yaml.
- IK clamps silently to per-leg joint limits and to a reachable triangle — an
  unreachable foot target yields the nearest reachable pose, not an error.
- The shoulder-rest signs differ from the brief's stated convention (see §6); the
  code's version is the deliberately-corrected one.
