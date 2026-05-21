<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/animation-pipeline/leg-coordinates.md
Original kind:  design
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/animation/fhc-format.md
-->

> **Reference material.** Verbatim copy of `doc/animation-pipeline/leg-coordinates.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/animation-pipeline/leg-coordinates.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
# Leg Coordinates, IK Frame, and Animation Pipeline — Design Synthesis

Distilled from a Claude conversation
([`llm-answers/leg-coordinates/claude-conversation.md`](../llm-answers/leg-coordinates/claude-conversation.md))
plus a follow-up grill-me session that resolved the open branches.
Cross-cuts the architectural conclusions in
[`results/firmware-research.md`](firmware-research.md) and the existing
sim/firmware kinematics implementations.

This document is the **load-bearing** spec for the new animation
pipeline: anyone writing the Blender exporter, the binary clip format,
the on-board IK, or the runtime gait engine should treat the decisions
here as canonical. The current `.gait` baked-angle format and
`code/firmware/src/nervous_system/kinematics.cpp` are now **legacy** —
this design replaces both.

---

## TL;DR

1. **Storage = foot XYZ in body frame**, not servo angles. Body frame
   is X-right / Y-forward / Z-up (matches Fusion + Blender).
2. **Per-keyframe cubic Bezier with value-axis handles only.** Linear
   is the degenerate case (zero handle deltas). No per-segment interp
   types, no time-axis handles. 8 bytes per keyframe.
3. **IK on the ESP32** is a port of [`code/simulation/kinematics.py`](../../code/simulation/kinematics.py)'s
   `ik_v2` — cylindrical decomposition with a per-leg lateral
   constant `w_y` that absorbs the URDF's L1 Y offset. Firmware's
   current `kinematics.cpp` is wrong for this rig and gets retired.
4. **Pole sign is keyframable per leg** as a sparse step-function
   track. Default = clip-level constant (+1, "knees support body").
   Animator-driven flips validated to occur near full extension only.
5. **Track layout is per-clip metadata**: `SHARED_WITH_OFFSETS_MIRROR`
   for locomotion (one trajectory + `phase_offset_ms[4]` +
   `mirror_mask`) or `PER_LEG_INDEPENDENT` for free-form expressive
   clips. The format supports both.
6. **IMU body-pose correction is a runtime layer**, not stored. It
   perturbs the next interpolated foot target before IK. The clip
   stays clean. Owned by a separate teammate.
7. **Wall-flip / inversion = separate clip set with a `clip_class`
   byte**, not an "up vector" in the format. The runtime FSM picks
   rightside-up vs inverted clips based on the IMU.
8. **Foot empties live in world space (unparented).** The exporter
   transforms each frame into body frame via
   `body_ctrl.matrix_world.inverted()`. Body movement in Blender is a
   legitimate authoring affordance that bakes into foot-body
   coordinates at export.

**Sister docs in this directory:**
- [`animation-pipeline-roadmap.md`](animation-pipeline-roadmap.md) —
  task breakdown, ownership suggestions, sequencing for the team.
- [`api-surface.md`](api-surface.md) — runtime engine C API + how the
  existing WebSocket protocol in `code/API_SPEC.md` maps onto it.

---

## 1. Coordinate System

### Body frame conventions

| Axis | Direction | Source of truth |
|------|-----------|-----------------|
| +X | Body **right** | Fusion CAD origin |
| +Y | Body **forward** | Fusion CAD origin |
| +Z | Body **up** | Fusion CAD origin |
| Units | metres internally, **mm in storage and exporter** | Blender scene is mm-scaled (`unit_settings.scale_length = 0.001`) |

Blender's default Z-up matches. STL imports use `global_scale=1.0`
because both meshes and scene are in mm — see
[`animation/scripts/README.md`](../../animation/scripts/README.md).

### What is body frame *anchored* to?

- The **`base_link`** in the URDF — the chassis. Foot positions in
  storage are `(fx, fy, fz)` relative to `base_link`'s origin and
  axes.

> **Defining the origin in Fusion 360:**
> Place a named construction point on the Fusion model at the intended
> `base_link` origin (typically the geometric center of the chassis at
> the mounting plane) and anchor the URDF exporter to it. This point
> must never be moved between model iterations — it is the coordinate
> contract between Fusion, the URDF, and every stored `.fhc` clip.
> Minor mesh changes that don't move this point are safe; any change
> that moves it invalidates all stored clips.

- This is **not world frame.** A "walk forward" clip stores the same
  foot trajectories regardless of where in the room the robot is —
  body frame is *control* frame, not odometry frame.
- This is **not leg-local frame.** Leg-local would tie the foot
  positions to each shoulder joint's mounting; making body-pose
  control (tilt, lean, IMU correction) painful because every body
  motion would force re-expressing all four foot targets.

The body-relative choice is the same one every shipping quadruped
makes (OpenCat, SpotMicro, Leika).

### How `body_ctrl` and foot empties relate in Blender

In Blender, `body_ctrl` is the object that represents `base_link`. Its
world transform is the body frame at that instant in the animation.
It may be static (v1 constraint) or animated (future).

Foot empties are **unparented** — they live in world space. The
exporter computes body-frame foot positions frame-by-frame via:

```python
foot_body = body_ctrl.matrix_world.inverted() @ foot_empty.matrix_world.translation
```

This means:

- Animating a foot empty moves only that leg's trajectory.
- Animating `body_ctrl` (translate, rotate) affects all four legs'
  body-frame positions simultaneously — the body moving away from a
  foot is equivalent to that foot moving relative to the body.
- The `.fhc` file stores only the result: body-frame foot XYZ per
  keyframe. Body ctrl animation fully bakes in and is not represented
  separately.

**v1 constraint:** `body_ctrl` must be static (no keyframes) within a
single clip. The exporter must validate this and reject clips where
`body_ctrl` moves. Animated `body_ctrl` is a v2 feature.

See §6 ("Body Pose & IMU Correction") for how this composes with the
runtime IMU-correction layer.

### Why no "up vector" in the file format

Inversion isn't encoded as a vector or a sign-flip. It's encoded as
**clip selection**:

- Rightside-up locomotion: `walk_forward.fhc` with feet at body-Z < 0.
- Inverted locomotion: `walk_inverted.fhc` with feet at body-Z > 0.
- Wall-flip transition: `flip.fhc` with `clip_class = FLIP`.

The runtime FSM consults the IMU to pick which clip family applies;
the IK math is invariant to body orientation in the world.

This collapses upside-down operation from a **coordinate-system
question** ("rotate the storage frame when inverted") into a
**clip-selection question** ("pick the right file"), which is much
simpler firmware.

---

## 2. Storage Format — foot XYZ + Bezier handles

### Why foot positions over servo angles

| Property | Foot XYZ (this design) | Baked angles (legacy `.gait`) |
|---|---|---|
| Adapts to body-pose at runtime (IMU correction) | ✓ trivial | ✗ requires re-baking |
| Adapts to terrain (per-foot Z offset) | ✓ trivial | ✗ requires re-baking |
| L↔R mirroring at runtime | ✓ flip Y component | ✗ baked-in per leg |
| File size | Comparable (3 axes × i16 vs 3 angles × i16) | smaller for one leg's encoding, but no mirror trick |
| On-board IK risk | Solver runs every tick — has to be correct | Zero IK at runtime |
| Authoring intuition (Blender) | Drag empties — direct | Pose bones — indirect |

Decision: **foot XYZ wins**, because the runtime features it enables
(IMU correction, terrain adaptation, mirroring) are core to your
roadmap per [`code/API_SPEC.md` §3](../../code/API_SPEC.md). The
firmware-research synthesis already concluded this for locomotion
specifically; here we extend it to expressive clips too, on the basis
that one engine is simpler than two.

### What gets stored per keyframe

```c
// One keyframe in one axis (x, y, or z) of one leg's foot trajectory.
// Authoring: Blender FCurve.keyframe_points[i] with Auto Clamped handles.
struct FhcKeyframe {
    uint16_t t_ms;            // since clip start, 0..65535 ms
    int16_t  value_01mm;      // foot position component, 0.1 mm units
    int16_t  h_left_dv;       // left handle, value-axis offset relative to value_01mm
    int16_t  h_right_dv;      // right handle, value-axis offset
};
// = 8 bytes per keyframe per axis
```

> **Bezier evaluation formula (what the ESP32 computes each tick):**
>
> Given two adjacent keyframes A = `(t_a, v_a, h_right_dv_A)` and
> B = `(t_b, v_b, h_left_dv_B)`, the four Bezier control points are:
>
> ```
> P0 = (t_a,           v_a)
> P1 = (t_a + dt/3,    v_a + h_right_dv_A)   // right handle of A
> P2 = (t_b - dt/3,    v_b + h_left_dv_B)    // left handle of B
> P3 = (t_b,           v_b)
> where dt = t_b - t_a
> ```
>
> Because time handles are fixed at ±dt/3, the parameter `u` is directly:
>
> ```
> u = (t_current - t_a) / dt,   u ∈ [0, 1]
> ```
>
> The interpolated value is the standard cubic Bezier:
>
> ```
> v(u) = (1-u)³·v_a
>       + 3(1-u)²u · (v_a + h_right_dv_A)
>       + 3(1-u)u² · (v_b + h_left_dv_B)
>       + u³·v_b
> ```
>
> This is evaluated once per axis per leg per tick (12 evaluations
> total). No root-finding is needed — `u` is a direct division. The IK
> (atan2, sqrt, acos) is the dominant cost, not the Bezier evaluation.
>
> **Speed scaling:** `gait_engine_set_speed_scale(s)` multiplies `dt`
> of the current tick advance: `t_current += tick_ms * s`. This
> stretches or compresses the entire clip uniformly without altering
> stored handle values. There is no per-segment speed modifier in v1;
> the Bezier handle shape (`h_left_dv` / `h_right_dv` magnitude)
> controls ease-in/ease-out within a segment, and global speed scale
> controls overall tempo.

Unit: **0.1 mm** (`int16` covers ±3.27 m, plenty). Servo backlash and
mechanical resolution dominate at that scale.

### Why value-handles only (the simplification decision)

Blender FCurve handles have both a time-axis and a value-axis
component: `(h.x, h.y) = (time_offset, value_offset)`. Two reasons we
only store the value component:

1. **For Auto Clamped handles** (Blender's default), time offsets
   are uniformly placed at `(segment_dt) / 3`. With uniform time
   handles, the 2D Bezier `(t(u), v(u))` collapses to `t(u) = t_a +
   (t_b - t_a) · u` — i.e., `u = (t - t_a) / (t_b - t_a)` exactly.
   Time handles carry no information beyond the segment endpoints.
2. **Time-axis handles only matter for explicit temporal easing**
   (Aligned/Free handles with skewed time extents). For foot
   trajectories, the animator wants smooth value paths with
   uniform pacing — not "foot moves slow at start of swing then
   suddenly fast." Easing at clip-period level (slow-mo whole clip)
   is achieved by `gait_engine_set_speed_scale()` at runtime, not
   per-segment in the file.

**Exporter constraint**: every keyframe handle must be Auto Clamped,
Auto, or Vector (linear). Aligned/Free with skewed time deltas →
reject the export with a clear message pointing the animator to
"Right-click handle → Auto Clamped".

### Why no per-segment interp type

Originally drafted with `BEZIER` / `LINEAR` / `CONSTANT` per outgoing
keyframe. Dropped because:

- Linear is a degenerate Bezier with `h_right_dv = 0` and `h_left_dv
  = 0` on the next keyframe. Same code path; one byte saved.
- Constant ("hold previous value") is not a useful mode for foot
  trajectories — feet always move continuously. If you want a static
  pose, use two identical keyframes.
- Single code path = simpler firmware, simpler exporter, smaller
  files.

### How Blender authoring maps to file output

```
Blender                              File
--------                             ----
foot_target.location keyframes  →    Per-leg (x, y, z) tracks
  on FCurves (cubic Bezier)            with per-keyframe Bezier handles
body_ctrl.matrix_world          →    Folded into foot positions
  (per-frame body pose)                via inverted() at sample time
Action.use_cyclic + frame range →    period_ms in clip header
Custom prop "track_layout"      →    track_layout enum byte
Custom prop "clip_class"        →    clip_class enum byte
Custom prop "phase_offset_ms"   →    phase_offset_ms[4] in header
                                      (only meaningful when SHARED)
```

---

## 3. IK on the ESP32 — port the sim's `ik_v2`

### The lateral-Y constant `w_y` (the answer to the photo question)

The foot-tip photos showed:

- **Top view**: joint1 → joint2 → joint3 collinear along the leg.
- **Side view**: joint1 sits *above* joint2/3 (Z lift visible as the
  green-circled gap).

The URDF says (FL leg, in link1 frame):

```
L1_vec    = (-0.05592, +0.02260, +0.01445) m
                ↑           ↑          ↑
            chain X    lateral Y     Z lift
L2_vec    = (-0.08970,  0.00000,  0.00000) m
foot_L3   = (taken from FootTip in link3 frame)
```

The +22.6 mm Y component is **the lateral Y offset** the conversation
called out. Joints 2 and 3 rotate around link1-Y. **Y-axis rotations
preserve the Y component of any point.** So:

```
w_y = L1.y + L2.y + foot_L3.y  ← per-leg constant, ≈ 22.6 mm for FL
```

is invariant under any `(theta_hip, theta_knee)` rotation. It's the
fixed offset between the foot and the joint-1 yaw axis along link1's Y
direction. **This is the constant you intuited from the photos** — the
"intersection of joint 2's rotation axis with the joint-1-to-tip line"
sits at the foot's projection onto the (chain_x, z) plane, displaced
by `w_y` along link1-Y.

### Hip offset subtraction (precedes cylindrical decomposition)

Each leg has a fixed hip joint position in body frame, stored in
`LegGeom`:

```c
typedef struct {
    float hip_x, hip_y, hip_z;  // hip joint origin in body frame (mm)
    float l2, l3;               // link2, link3 lengths (mm)
    float w_y;                  // lateral offset of link2 from yaw axis (mm)
    int   chain_x_sign;         // +1 or -1 depending on leg side
} LegGeom;
```

The IK input is a foot position in **body frame**. Before cylindrical
decomposition, subtract the hip offset:

```c
foot_hip.x = foot_body.x - leg_geom.hip_x;
foot_hip.y = foot_body.y - leg_geom.hip_y;
foot_hip.z = foot_body.z - leg_geom.hip_z;
```

Now `foot_hip` is the foot position relative to the hip joint.
Cylindrical decomposition proceeds on `foot_hip`:

```c
theta = atan2(foot_hip.y, foot_hip.x);   // yaw — link1 target
r     = sqrt(foot_hip.x*foot_hip.x + foot_hip.y*foot_hip.y);
z     = foot_hip.z;
// then 2-link planar IK on (r, z) for link2 and link3,
// accounting for w_y offset per ik_v2 (see below)
```

`hip_x/y/z` are read from the URDF at build time by
`generate_leg_geom_h.py` (T6) and compiled into `leg_geom.h` as
constants. They are never computed at runtime.

### The cylindrical decomposition (correctly accounting for `w_y`)

```
Inputs:
  foot_body = (fx, fy, fz)        in body frame, mm
  mount     = (mx, my, mz)        per-leg shoulder origin, mm
  yaw_offset                      per-leg, radians (rest direction)
  L1, L2, foot_L3                 per-leg link offsets in their local frames
  hip_axis_sign, knee_axis_sign   per-leg, ±1 (URDF axis convention)

Step 1 — translate to shoulder-origin:
  d = foot_body - mount

Step 2 — solve yaw, accounting for w_y:
  w_y      = L1.y + L2.y + foot_L3.y
  rxy2     = d.x² + d.y²
  chain_x² = rxy2 - w_y²                     ; can be negative if foot too close laterally → clamp to 0
  chain_x  = sign(L1.x + L2.x + foot_L3.x) * sqrt(chain_x²)
  theta_yaw = atan2(d.y, d.x) - atan2(w_y, chain_x) - yaw_offset
  theta_yaw = wrap_pi(theta_yaw)

Step 3 — 2-link planar IK in the leg's (chain_x, z) plane:
  u = chain_x - L1.x
  v = d.z     - L1.z
  l2 = hypot(L2.x, L2.z)               ; planar length of L2 segment
  l3 = hypot(foot_L3.x, foot_L3.z)     ; planar length of L3 segment + foot tip
  D  = hypot(u, v)
  D  = clamp(D, |l2 - l3| + ε, l2 + l3 - ε)   ; keep triangle reachable

  cos_kprime = (D² - l2² - l3²) / (2·l2·l3)
  kprime     = acos(clamp(cos_kprime, -1, 1))
  alpha_rel  = wrap_pi(atan2(foot_L3.z, foot_L3.x)
                       - atan2(L2.z, L2.x))
  s_k        = sign(L1.x + L2.x + foot_L3.x) * kprime - alpha_rel

Step 4 — recover hip:
  ck, sk = cos(s_k), sin(s_k)
  V_x = L2.x + foot_L3.x·ck + foot_L3.z·sk
  V_z = L2.z - foot_L3.x·sk + foot_L3.z·ck
  s_h = wrap_pi(atan2(V_z, V_x) - atan2(v, u))

Step 5 — convert URDF-internal to user-facing (Phase H convention):
  theta_hip  = hip_axis_sign  * s_h
  theta_knee = knee_axis_sign * s_k

Step 6 — clamp to per-leg URDF limits.
```

### Why `kinematics.cpp` is wrong for this rig

[`code/firmware/src/nervous_system/kinematics.cpp`](../../code/firmware/src/nervous_system/kinematics.cpp)
does `a = r_xy - L1` with scalar `L1`, treating L1 as a horizontal
coxa. This works for textbook "coxa-femur-tibia" robots where joint 2
sits coplanar with joint 1's yaw axis. For your URDF — where L1 has a
nonzero Y component — it produces a constant ~8.6° yaw error
(`atan2(22.6, ~150) ≈ 8.6°`). Retire it; port `ik_v2` from
[`code/simulation/kinematics.py:135`](../../code/simulation/kinematics.py#L135)
verbatim to C.

### Per-leg constants (compile-time)

```c
typedef struct {
    int16_t mount_x_um, mount_y_um, mount_z_um;     // shoulder origin in body frame, µm
    int16_t L1_x_um, L1_y_um, L1_z_um;
    int16_t L2_x_um, L2_y_um, L2_z_um;
    int16_t foot_L3_x_um, foot_L3_y_um, foot_L3_z_um;
    float   yaw_offset_rad;
    int8_t  hip_axis_sign, knee_axis_sign;          // ±1
    int16_t lim_yaw_lo_mrad,  lim_yaw_hi_mrad;
    int16_t lim_hip_lo_mrad,  lim_hip_hi_mrad;
    int16_t lim_knee_lo_mrad, lim_knee_hi_mrad;
} LegGeom;

extern const LegGeom LEG_GEOM[4];   // FL, FR, BL, BR — generated from the URDF
```

The exporter generates this header from the URDF at build time so the
firmware never re-parses XML.

### Pole sign — keyframable per leg, not static

Originally drafted as a single global `+1` constant. Revised: the
animator wants per-clip control over knee bend direction, including
flips during stunts. Design:

- **Per-leg `pole_track`**: a 4th sparse track per leg (alongside X,
  Y, Z), with `interp = STEP`. Values: `+1`, `−1`, or `0` (= use
  clip-level default).
- **Authoring**: a custom property `pole_sign` on each foot empty,
  keyframable in Blender. Animator places keyframes at moments where
  the leg crosses full extension (swing apex) — that's where the two
  IK solutions converge and a flip is geometrically smooth.
- **Export validation**: every pole-sign change must occur within ε
  of full-extension (e.g. `D > 0.95 · (l2 + l3)` at the change
  frame). Otherwise reject with a diagnostic so the animator can
  retime the change.
- **Default behaviour**: empty `pole_track` → clip-level default
  `+1` is used. Free of cost for normal locomotion.
- **Runtime**: at IK time, sample `pole_sign(t)` from the track
  (constant-between-keyframes), pass to the IK as the sign multiplier
  on the `acos` result.

This gives you the slider UX you wanted without the impossible
"halfway between elbow up and elbow down" interpolation — the value
is always discrete (±1), but the animator decides exactly when it
flips.

For `SHARED` track-layout clips, the pole_track applies to all four
legs uniformly (with mirror_mask still flipping Y, not pole). For
`PER_LEG`, each leg has its own pole_track.

### Cost on ESP32

Per leg per IK call: 1× `acosf`, 2× `atan2f`, 2× `cosf`, 2× `sinf`,
~10 mults/adds. Per `firmware-research.md` §1, that's ~70–80 µs at
240 MHz. Four legs: ~300 µs per tick. At 100 Hz: ~3 % of one core.
Trivial.

---

## 4. Track Layout and Phase Offsets

### Two layouts, one format — selected per clip

```c
typedef enum {
    TRACK_LAYOUT_SHARED       = 0,   // 1 track, applied to all 4 legs
    TRACK_LAYOUT_PER_LEG      = 1,   // 4 independent tracks
} TrackLayout;
```

| Layout | When to use | Storage cost (typical 2 s clip) |
|---|---|---|
| `SHARED` | Locomotion: walk, trot, bound, crab. The four legs do "the same thing" at different phases. | 1 leg's tracks + `phase_offset_ms[4]` + `mirror_mask` ≈ 200–400 B |
| `PER_LEG` | Expressive: wave one leg, asymmetric stance, choreographed motion. | 4 × per-leg tracks ≈ 800–1600 B |

### `SHARED` layout: phase offsets + mirror mask

```c
struct FhcSharedHeader {
    int16_t  phase_offset_ms[4];    // per-leg phase in [0, period_ms)
    uint8_t  mirror_mask;           // bit i: mirror Y axis for leg i
                                    // (turns L↔R within the body frame
                                    // — leg-side handedness)
    uint8_t  pad[3];
};
```

Standard quadruped gaits in `phase_offset_ms[FL, FR, BL, BR]` for
period P:

| Gait | FL | FR | BL | BR |
|------|----|----|----|----|
| Trot (diagonal pairs) | 0 | P/2 | P/2 | 0 |
| Walk (4-beat) | 0 | P/2 | 3P/4 | P/4 |
| Bound (front/back pairs) | 0 | 0 | P/2 | P/2 |
| Pace (lateral pairs) | 0 | P/2 | 0 | P/2 |

The `mirror_mask` handles L↔R asymmetry: e.g. for trot with one
authored track for FL, you need FR and BR to mirror Y so the right
side legs swing on the right side of the body. Typical mask: `0b0110`
(mirror FR and BL) for diagonal pairs.

### `PER_LEG` layout

```c
struct FhcPerLegHeader {
    // No phase offsets, no mirror — each leg's track is authored
    // independently. The animator places keyframes per leg as desired.
};
```

Used for expressive clips where the animator authors each leg
individually in Blender. The exporter writes 4 separate track
sections.

### Authoring UX in Blender

A custom property on the Action declares the layout:

```python
action["track_layout"] = "SHARED"  # or "PER_LEG"
action["mirror_mask"]  = 0b0110    # only meaningful for SHARED
action["phase_offset_ms"] = [0, 1000, 1000, 0]  # only meaningful for SHARED
```

For `SHARED`: the animator can still keyframe all four feet in
Blender so the in-viewport preview shows the trot. The exporter:

1. Reads FL's track only.
2. **Validates** that FR/BL/BR (after un-mirroring per `mirror_mask`)
   are FL phase-shifted by `phase_offset_ms`. If not within tolerance,
   reject the export with a diff so the animator can fix it.
3. Writes only FL's track + the metadata.

For `PER_LEG`: the exporter reads all four leg tracks independently.
No validation, no metadata.

This gives WYSIWYG preview in Blender for both layouts, while keeping
shared-layout file size minimal.

---

## 5. Interpolation Engine

### Cubic Bezier evaluator (per axis, per leg)

```c
static inline float bezier3(float P0, float P1, float P2, float P3, float u) {
    float v  = 1.0f - u;
    float v2 = v * v;
    float u2 = u * u;
    return v2*v*P0 + 3.0f*v2*u*P1 + 3.0f*v*u2*P2 + u2*u*P3;
}

// Sample one axis at clip time t. Single code path for all interpolation —
// linear is the degenerate case where both handles' value offsets are zero.
float sample_axis(const Track* track, uint32_t t_ms) {
    int i = bracket(track, t_ms);    // find segment so kf[i].t <= t < kf[i+1].t
    const FhcKeyframe* a = &track->keyframes[i];
    const FhcKeyframe* b = &track->keyframes[i + 1];
    float u  = (float)(t_ms - a->t_ms) / (float)(b->t_ms - a->t_ms);
    float P0 = a->value_01mm;
    float P3 = b->value_01mm;
    float P1 = P0 + a->h_right_dv;
    float P2 = P3 + b->h_left_dv;
    return bezier3(P0, P1, P2, P3, u);
}

// Sample 3D foot position from three time-aligned tracks (X, Y, Z).
// Bracket once, eval three axes — same u everywhere.
Vec3 sample_foot(const LegTracks* tracks, uint32_t t_ms) {
    int i = bracket(tracks->x, t_ms);
    float u = (float)(t_ms - tracks->x[i].t_ms) /
              (float)(tracks->x[i+1].t_ms - tracks->x[i].t_ms);
    return (Vec3){
        bezier3_kf(&tracks->x[i], &tracks->x[i+1], u),
        bezier3_kf(&tracks->y[i], &tracks->y[i+1], u),
        bezier3_kf(&tracks->z[i], &tracks->z[i+1], u),
    };
}
```

The exporter must enforce **time-aligned keyframes per leg** — every
leg's X/Y/Z tracks have keyframes at the same `t_ms` values. This
makes the bracket-once-eval-three pattern correct and the firmware
trivially simple. Blender does this naturally when the animator
keyframes `Location` (all 3 axes keyed at once with `I` →
`Location`). If the animator keys axes individually at different
frames, the exporter rejects with a clear message.

### Cost

Per Bezier eval: 6 mults, 3 adds. Per leg per tick: 1 bracket + 3
evals = ~30 float ops. At 100 Hz × 4 legs: ~12 K float-ops/sec.
ESP32 Xtensa LX6 does ~150 M float-ops/sec. **<0.01 % of one core
on the Bezier engine.** Genuinely negligible. The IK math (~3 % of
one core for all 4 legs) dominates by orders of magnitude, and the
IK is itself negligible.

### Authoring guidance for animators

- **Default handle type**: set Blender's default to `Auto Clamped`
  (`bpy.context.preferences.edit.keyframe_new_handle_type =
  'AUTO_CLAMPED'`). Prevents handles overshooting and self-intersecting.
- **Per-segment interp**: use `LINEAR` for fast events (foot lift-off
  spike), `BEZIER` for swing arcs (default), `CONSTANT` for poses
  held flat.
- **First/last keyframe**: must equal the neutral foot position so
  the runtime can transition between clips without blending. Validated
  at export.

---

## 6. Body Pose & IMU Correction (runtime)

### What's stored vs what's computed at runtime

| Layer | Source | When applied |
|---|---|---|
| **Foot trajectory** | Clip file (Bezier evaluated) | Every tick |
| **IMU body-pose correction** | IMU + control loop | Every tick, perturbs foot target |
| **Mirror mask + phase offset** | Clip header (SHARED layout) | Every tick, before IK |

### Runtime pipeline per leg per tick

```
foot_clip       = bezier_sample(track, phase_for_leg)            // body frame
foot_mirrored   = apply_mirror_mask(foot_clip, leg)              // body frame
foot_corrected  = imu_correction_matrix * foot_mirrored          // body frame
joint_targets   = leg_ik(foot_corrected, leg)                    // (yaw, hip, knee)
servo_pwm       = angle_to_pwm(joint_targets, servo_calib[leg])
write_pca9685(servo_pwm)
```

`imu_correction_matrix` is a small 3×3 chassis-correction rotation
that levels the body to gravity (or to a target tilt sent via
`T:3 Body Pose` per [`code/API_SPEC.md` §3](../../code/API_SPEC.md#L51)).
Identity when the IMU says level and no command is active.

### Tick pipeline — ordered call sequence

**Per-tick execution order (core 1, 100 Hz):**

1. **Bezier eval:** for each leg (or for the one SHARED track +
   offsets), evaluate `fhc_sample_foot(tracks, t_ms)` →
   `foot_clip[4]` in body frame.
2. **Mirror:** for legs with `mirror_mask` bit set, negate the Y
   component: `foot_clip[i].y = -foot_clip[i].y`. Applied only for
   SHARED layout.
3. **IMU correction:** apply the most-recently-written correction
   matrix: `foot_corrected[i] = R_correction · foot_clip[i] + (0, 0,
   dh_mm)`. If correction has never been set, `R = identity` and
   `dh = 0` (pass-through).
4. **IK:** for each leg, `leg_ik(leg_id, foot_corrected[i],
   pole_sign[i])` → `JointAngles legs[4]` (yaw, hip, knee per leg).
5. **Servo write:** `servo_write_all(legs)` → per-servo pulse via
   PCA9685 I2C.

**Why IMU correction precedes IK:** the correction is a geometric
nudge of the foot target in body frame — it answers "where should
this foot be, given the body is currently tilted?". IK then resolves
what servo angles reach that corrected position. Applying a
correction *after* IK would require modifying servo angles directly,
which don't compose linearly in a kinematic chain and have no clean
geometric interpretation.

### How `body_ctrl` movement in Blender plays into this

Foot empties are unparented (world-space). `body_ctrl` moves the
chassis. At export time:

```python
foot_body[t] = body_ctrl.matrix_world(t).inverted() @ foot_empty.matrix_world(t)
```

So if the animator:
- Moves only the foot empties → standard locomotion or expressive
  motion in body frame.
- Moves only `body_ctrl` (e.g., chassis tilt, lean forward, hop down)
  → the body's relative motion w.r.t. the planted feet bakes into the
  foot-body coordinates as a *negative* body motion. Useful for
  "pose-only" expressive clips where the chassis moves but the feet
  stay in world space.
- Moves both → the per-frame body-frame foot positions reflect
  whatever combined motion is intended.

**Convention**: locomotion clips (`SHARED` layout) should keep
`body_ctrl` at identity throughout — the runtime IMU layer applies
chassis correction on top of clean foot trajectories. Expressive
clips (`PER_LEG` layout) can move `body_ctrl` freely; the result is
baked into foot positions and the IMU layer adds on top of *that*.
Validated at export with a warning (not a hard reject).

---

## 7. Inversion / Wall-flip — clip-class, not coordinate-flip

### `clip_class` enum

```c
typedef enum {
    CLIP_CLASS_LOCOMOTION = 0,    // walk, trot, bound — IMU correction ON
    CLIP_CLASS_EXPRESSIVE = 1,    // sit, wave, idle    — IMU correction ON
    CLIP_CLASS_TRANSITION = 2,    // stance↔gait blends — IMU correction ON
    CLIP_CLASS_FLIP       = 3,    // wall-flip stunt   — IMU correction OFF
    CLIP_CLASS_INVERTED   = 4,    // walking inverted   — IMU correction ON, sign-aware
} ClipClass;
```

### How a wall-flip works in body-relative storage

The clip stores **what the feet do** in body frame. The body's world
orientation during the flip is a *consequence* of physics — the feet
push, the body rotates by reaction, the IMU detects "now inverted."

Authoring a flip in Blender:

1. Foot empties keyframed: windup low (phase 0–20%), thrust high
   (phase 20–60%), inverted-stance positions (phase 60–100%, feet
   above body in body frame).
2. `body_ctrl` keyframed to *show the rotation* in Blender preview
   (so the animator can verify the chassis is going to flip). At
   export, `body_ctrl` motion bakes into foot-body coordinates as it
   does for any other clip.
3. `clip_class = FLIP` declared on the Action.

Runtime:

1. FSM transitions to `FLIP` clip → disables IMU correction layer.
2. Plays the clip end-to-end at fixed period.
3. At end-of-clip, FSM consults IMU:
   - Stable + inverted → transition to `INVERTED` clip family.
   - Stable + rightside-up (flip didn't take) → transition to
     `LOCOMOTION` family.
   - Unstable → `FAILSAFE`.

### Why not "rotate the storage frame when inverted"

You'd have to:
- Negate `z_body` of every foot target.
- Flip `pole_sign` (knees now bend the opposite way).
- Re-validate every limit.

…all to avoid authoring a separate clip. The clip-selection approach
is simpler and gives the animator full creative control over what
inverted locomotion looks like (it's not just "the same gait but
flipped").

---

## 8. File Format Spec — `.fhc` (FaceHugger Clip)

### Container layout

```
+----------------------+   offset 0
| FhcFileHeader        |   16 bytes
+----------------------+
| FhcSharedHeader      |   12 bytes (only if track_layout == SHARED)
+----------------------+
| Track 0 (FL)         |   variable; one per leg (1 if SHARED, 4 if PER_LEG)
|   FhcTrackHeader     |   8 bytes
|   X axis keyframes   |   N × 8 bytes  (N = aligned count)
|   Y axis keyframes   |   N × 8 bytes
|   Z axis keyframes   |   N × 8 bytes
|   pole keyframes     |   M × 4 bytes  (sparse, often 0)
+----------------------+
| Tracks 1..3          |   PER_LEG only
+----------------------+
| (optional: CRC32)    |   4 bytes — boot-time integrity
+----------------------+
```

### Headers

```c
struct FhcFileHeader {
    uint8_t  magic[4];           // 'F','H','C','1'
    uint8_t  clip_class;         // ClipClass enum
    uint8_t  clip_mode;          // 0=ONESHOT, 1=LOOP
    uint8_t  track_layout;       // TrackLayout enum (SHARED | PER_LEG)
    uint8_t  default_pole_sign;  // +1 or -1; per-leg pole_track overrides
    uint16_t period_ms;          // total clip length, 0..65535 ms
    uint16_t flags;              // reserved
    uint16_t track_count;        // 1 (SHARED) or 4 (PER_LEG)
    uint16_t reserved;
};                                // 16 bytes

struct FhcSharedHeader {
    int16_t  phase_offset_ms[4]; // FL, FR, BL, BR
    uint8_t  mirror_mask;        // bit i: mirror Y for leg i
    uint8_t  pad[3];
};                                // 12 bytes; only present if SHARED

struct FhcTrackHeader {
    uint8_t  leg_id;             // 0..3 (FL, FR, BL, BR); ignored if SHARED
    uint8_t  pad;
    uint16_t kf_count;           // keyframes per axis (X, Y, Z each have this many)
    uint16_t pole_kf_count;      // sparse pole-track keyframes; usually 0
    uint16_t reserved;
};                                // 8 bytes

struct FhcPoleKeyframe {
    uint16_t t_ms;
    int8_t   pole_sign;          // +1 or -1; 0 = clear override
    uint8_t  pad;
};                                // 4 bytes

### Storage cost estimates

For a 2-second locomotion clip (SHARED, 4 keyframes per axis, 1 leg's
tracks):
- 16 (file header) + 12 (shared header) + 8 (track header)
  + 3 axes × 4 keyframes × 8 bytes = **132 bytes**.

For a 4-second expressive clip (PER_LEG, 10 keyframes per axis):
- 16 + 4 × (8 + 3 × 10 × 8) = **1 016 bytes**.

Full library of 4 locomotion + 8 expressive ≈ **~8.5 KB**. Well
within your firmware-research budget of ~200 KB free heap, with room
for an order of magnitude more clips.

### Boot loader story (matches firmware-research §3)

1. At boot: enumerate `/clips/*.fhc` on LittleFS.
2. For each: validate magic, parse header, allocate a `Clip` struct
   in pre-allocated SRAM, copy keyframes into it.
3. Close all file handles. Never read flash from the IK loop.
4. Indexed by name (or by enum) for FSM lookup.

---

## 9. What's Open / Future Work

These are the questions the synthesis didn't fully resolve, parked
explicitly so they don't get re-litigated implicitly:

1. **URDF cleanup vs IK port.** Decision: port `ik_v2` verbatim. If
   the URDF is ever regenerated with a cleaner L1 (Y component
   zeroed by re-anchoring the joint origin in Fusion), the firmware
   IK could be simplified to textbook cylindrical decomposition.
   CAD-side task, not firmware. **Reminder**: the joint origin
   location along an axis is a coordinate convention, not physics —
   only the rotation axis is physical. The 22.6 mm `w_y` is real but
   not "wrong"; it's just where the URDF anchored joint 2's origin
   on its rotation axis.
2. **Phase-aligned transitions.** When transitioning between two
   `LOCOMOTION` clips with different periods, blend in joint space
   (per `firmware-research.md` §3) but also choose the start phase
   so all four legs are in stance — avoids "leg moves backward
   through air to new trajectory" artifacts. Spec the blending logic
   when the FSM is built.
3. **Servo angle calibration.** [`config.h`](../../code/firmware/src/shared/config.h)
   says 180° servos with `MIN_PULSE 150 / MAX_PULSE 600`. The
   `servo_mapping.yaml` schema in `API_ANIMATION_SPEC.md` §6 allows
   per-servo `offset_deg / direction / min_deg / max_deg`. Reconcile
   the spec to match `config.h` (drop the 0–270° / DSS-M15S
   references) before the exporter ships.
4. **Pole-flip validation tolerance.** What does "near full
   extension" mean numerically — `D > 0.95 · (l2 + l3)`? `D > 0.9`?
   Determined empirically once the first stunt clip exists; placeholder
   default `0.95`.
5. **First-and-last-keyframe-equals-neutral validation.** Per
   [`API_ANIMATION_SPEC.md` §4](../../code/simulation/docs/API_ANIMATION_SPEC.md#L132),
   gait clips must start and end at neutral pose. Define numerical
   tolerance for "equals" — recommend ≤1 mm in body-frame foot
   position.

---

## 10. Reference Implementations to Borrow From

| Source | What to take |
|---|---|
| [`code/simulation/kinematics.py:135 ik_v2`](../../code/simulation/kinematics.py#L135) | The full IK math, ported to C verbatim. The Y-invariance trick (line 155) is the core insight. |
| [`code/simulation/kinematics.py:105 fk_v2`](../../code/simulation/kinematics.py#L105) | FK for runtime-side validation: every `LOCOMOTION` clip's keyframes should produce reachable joint angles via FK round-trip. Add as an export-time validator. |
| [`runeharlyk/SpotMicroESP32-Leika`](https://github.com/runeharlyk/SpotMicroESP32-Leika) | Bezier-curve trot architecture, FSM motion controller, web UI. Closest reference for runtime engine. |
| [`PingguSoft/esp32_quadruped`](https://github.com/PingguSoft/esp32_quadruped) | 100 Hz update loop pattern, PCA9685 driver, dual-core split. |
| [`PetoiCamp/OpenCatEsp32`](https://github.com/PetoiCamp/OpenCatEsp32) | Mirror-trick (one stored gait → mirrored at runtime). Same pattern as `mirror_mask` here. |

---

## Bottom Line

| Question | Answer |
|---|---|
| What's stored in a clip? | Foot XYZ in body frame, per-keyframe with cubic Bezier handles. |
| Coordinate convention? | X-right, Y-forward, Z-up. Body-relative (anchored to `base_link`). Units: 0.1 mm in storage. |
| How is gait pattern encoded? | Per-clip `track_layout` byte: SHARED (1 track + `phase_offset_ms[4]` + `mirror_mask`) or PER_LEG (4 independent tracks). |
| How is interpolation done? | Cubic Bezier per axis, per-segment `BEZIER`/`LINEAR`/`CONSTANT` from Blender FCurves. ~0.4 µs per tick total. |
| Where does IK run? | On the ESP32, every tick, after Bezier sample + mirror + IMU correction. |
| Which IK math? | Port of [`kinematics.py:135 ik_v2`](../../code/simulation/kinematics.py#L135) — cylindrical with `w_y` constant. Retire `kinematics.cpp`. |
| Pole sign? | Per-leg keyframable sparse track (step interp), default = clip-level constant. Animator-driven flips validated near full extension. |
| Inversion / wall-flip? | Separate clip set with `clip_class` byte. No "up vector" in the format. |
| IMU body-pose correction? | Pure runtime layer. Not stored. Perturbs foot target before IK. |
| Foot empty parenting in Blender? | Unparented (world-space). Exporter transforms world → body via `body_ctrl.matrix_world.inverted()`. |
| Servo range? | 180°, per [`config.h`](../../code/firmware/src/shared/config.h). Spec docs that say 0–270° need updating. |
| File extension? | `.fhc` (FaceHugger Clip). Magic `FHC1`. |
| Boot loader? | Eager preload from LittleFS at boot, file handles closed before IK loop starts. Per `firmware-research.md` §2. |
