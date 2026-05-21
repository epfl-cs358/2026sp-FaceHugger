<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  code/simulation/docs/MERGE_AND_CONVENTION.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/firmware/kinematics.md
-->

> **Reference material.** Verbatim copy of `code/simulation/docs/MERGE_AND_CONVENTION.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `code/simulation/docs/MERGE_AND_CONVENTION.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
# MERGE_AND_CONVENTION

Captures the `feat/urdf-pipeline` ↔ `origin/main` merge decisions and locks
in the joint-angle conventions every consumer (URDF, PyBullet, Blender,
firmware) is expected to honour from here on.

---

## 0. Convention quick-reference (authoritative)

This block is the canonical spec. The detailed sections below explain
*why* and *how to implement* each rule, but if anything contradicts this
block, this block wins.

**SHOULDER (servo 1) — yaw joint, Z axis**
- Viewed from above, 0° = pointing right (+X), CCW = positive.
- Rest angles per leg (baked into URDF as `<joint><origin rpy="0 0 …"/>`):
  - FL: **-45°** &nbsp;&nbsp; FR: **+45°** &nbsp;&nbsp; BL: **-135°** &nbsp;&nbsp; BR: **+135°**
- URDF limits for all legs: `[-90°, +90°]` (relative to rest).
- Derived from the FL Fusion export value via:
  `FR = -FL`, `BL = -wrap_pi(FL + π)`, `BR = wrap_pi(FL + π)`.
- URDF axis: `<axis xyz="0 0 1"/>` uniform across all 4 legs.

> **Note on the FL value.** When Fusion shows 0° on `Link1Revolute`,
> that corresponds to **-45° in this world-space convention** because the
> leg-assembly local frame is offset by +135° from world +X. The Fusion
> mechanical zero (`limits_rad.rest = -π/4` on the current CAD) is what
> the URDF generator reads and uses as the FL rest in the formula above.

**HIP (servo 2) — pitch joint, Y axis**
- Rest: **0°**. Positive = leg swings up, negative = leg drops down.
- Per-leg axis sign flips between L and R sides (so "horn-up =
  positive" comes out the same physical motion on both sides).

**KNEE (servo 3) — pitch joint, Y axis**
- Rest: **0°**. Same sign convention as hip (per-leg ±Y axis flip).

**GENERAL**
- URDF θ=0 = Fusion rest pose for every joint.
- Limits are shifted: `urdf_lower = Fusion.min - Fusion.rest`,
  `urdf_upper = Fusion.max - Fusion.rest`.
- Source of truth: `fusion_export.json` `limits_rad` field.
- Leg naming: `fl/fr/bl/br` in Python / URDF / Blender; firmware uses
  `fr=0, fl=1, rr=2 (=br), rl=3 (=bl)` — translation lives at the
  firmware boundary, not in Python.
- World frame: **+Y = forward**, **+X = right**, +Z = up.

---

## 1. Merge summary (commit `25951ae`)

`origin/main` had developed independently while this branch was building
the URDF pipeline. Six files conflicted; one was resolved by deletion,
five took **OURS** verbatim. **Their** versions were rejected when they
disagreed with the URDF source-of-truth or with this doc's conventions.

| File | Decision | Rationale |
|---|---|---|
| `code/simulation/constants.py` | OURS | Per-leg `STANCE_DEG`; URDF path under `generated/`. Theirs hardcoded a uniform `STANCE` and a stale top-level URDF path. |
| `code/simulation/simulate.py` | OURS | cfg-driven CLI. Theirs' `--bound`/`--crab`/`--terrain`/`--teleop` flags use module-state kinematics; deferred to a later PR. |
| `code/simulation/helpers.py` | OURS | Has the URDF/CAD parsing helpers (`_parse_leg_points_from_urdf`, `_load_urdf_joints`, etc.). Theirs is a strict subset. |
| `code/simulation/gaits.py` | OURS | cfg-driven gait registry. Theirs' `pre_orient_splay` machinery is obsoleted by §3 below (URDF rest = splayed). |
| `code/simulation/kinematics.py` | OURS | `LegGeom` / `RobotConfig` from URDF. Theirs' hardcoded `LEG_INFO` / `L1/L2/L3` / `_JOINT_LIMITS` came from a different CAD revision and would put legs in the wrong physical positions — see §6. |
| `code/simulation/facehugger.urdf` | DELETED | Stale path. Canonical: `code/simulation/generated/facehugger.urdf`. |
| `code/simulation/meshes/` | DELETED (whole dir) | Stale parallel mesh location. Canonical: `code/simulation/generated/exported_meshes/`. |

`code/simulation/teleop.py` and `code/simulation/terrain.py` landed clean
from main but are **not yet integrated** with the URDF pipeline (they
import theirs' module-state kinematics that doesn't exist here). Each
file carries a header comment flagging the status.

Clean adds inherited from main (no action): the firmware tree
(`code/firmware/src/nervous_system/`, plus the `gen_ik_reference.py`
test harness), the React Native remote-control app, `code/API_SPEC.md`.

---

## 2. Shoulder axis convention — **Convention A**

All four shoulder joints use a single uniform axis convention:

```xml
<axis xyz="0 0 1"/>     <!-- global +Z; CCW-from-above = positive angle -->
```

- 0° points along world +X (right of the robot when looking down from +Z),
  measured CCW like a math unit circle.
- "+ shoulder angle" = leg rotates CCW when viewed from above.
- Per-leg rest angle is encoded in `<joint><origin rpy="0 0 {rest_rad}"/>`,
  not in a per-leg axis sign. Hip and knee keep their `±Y` per-side flip
  (still needed because the L and R servo *horns* face opposite directions
  and "horn-up = positive" wants the axis sign to flip).

See [img/servo-1-rotation-convention.png](img/servo-1-rotation-convention.png)
for the shoulder convention; [img/servos-2-3-rotation-convention.png](img/servos-2-3-rotation-convention.png)
for the hip/knee horn-up rule.

**Why Convention A** (vs per-leg shoulder sign): same physical motion across
all legs is achieved by *baking the rest into the URDF rpy*, not by flipping
axes. Animation code keyframes raw URDF angles; "+δ on every leg" rotates
each leg the same way around its own +Z (CCW from above), and the rest pose
already places each leg at the right physical orientation.

---

## 3. URDF θ=0 = Fusion rest pose

The Fusion joint's `restValue` (in `fusion_export.json` as
`limits_rad.rest`) is the URDF's θ=0. The URDF generator emits:

```xml
<joint name="..._shoulder_joint" type="revolute">
  <origin xyz="..." rpy="0 0 {rest_rad}"/>
  <axis xyz="0 0 1"/>
  <limit lower="{min - rest}" upper="{max - rest}" .../>
</joint>
```

Limits in the URDF are **shifted relative to rest**, so the URDF's joint
angle range is `[Fusion.min - Fusion.rest, Fusion.max - Fusion.rest]`. For
Link1 with `rest=-45°`, `min=-135°`, `max=+45°` → URDF range `[-90°, +90°]`
— symmetric ±90° about θ=0. That's the goal.

> **Status (2026-05-04):** convention is **decided, documented, and
> implemented** — see commits `80b8e98` (joint rename to
> `{leg_id}_link{1,2,3}_joint`) and `6e52c45` (Convention A shoulder rest
> derivation in [generate_urdf.py](../generate_urdf.py)). The yaml
> `rpy_z_deg` and `shoulder_limits_deg` per-leg fields have been removed;
> the geometric back-of-pair flip is derived from `leg_id` ("b*" = 180°)
> and the kinematic shoulder rest is derived from the FL Fusion-export
> rest via the formula in §4.

---

## 4. Per-leg shoulder rest derivation from FL

The Fusion JSON only defines one `Link1Revolute` rest value (the source-FL
leg's mechanical zero). The other three corners are derived deterministically:

```
FL =  json.limits_rad.rest         # canonical, e.g. -45° / -π/4
FR = -FL                           # mirror across body X-axis
BL = -wrap_pi(FL + π)              # FL rotated 180° around +Z, then negated
BR =  wrap_pi(FL + π)              # FL rotated 180° around +Z
```

For `FL = -45°` (the current CAD's `Link1Revolute.limits_rad.rest`):
`FR = +45°`, `BL = -135°`, `BR = +135°`. Plotted on the unit circle
(top-down view, 0° = body +X right, CCW positive):

| Corner | Body quadrant | Rest angle (deg) | Rest angle (rad) |
|---|---|---:|---:|
| FL | -X +Y | -45 | -π/4 |
| FR | +X +Y | +45 | +π/4 |
| BL | -X -Y | -135 (= +225) | -3π/4 |
| BR | +X -Y | +135 | +3π/4 |

Earlier versions of `_shoulder_rest_for()` had the BL and BR signs
swapped, which sent the back legs splaying *into the front quadrants*
(visible in Blender as BL pointing toward FL, BR pointing toward FR).
The signs above are post-fix and verified against the visualizer.

See [img/leg-numbering-conventions.png](img/leg-numbering-conventions.png)
for the body-frame layout. The Fusion `Link1Revolute` rest is reported
in the leg-assembly's local frame (which is offset by +135° from world
+X), so a Fusion-side reading of 0° corresponds to **-45° in this
world-space convention**. URDF θ=0 lands at each leg's mechanical zero
under that mapping; symmetric ROM around 0 means equal forward/backward
swing for all four.

`facehugger_config.yaml` should drop its `rpy_z_deg` per-leg field (it's
now derived from the JSON `rest` + the formula above). Side handedness
(`side: L` vs `side: R`, controlling which mesh is used) stays in yaml —
it's a CAD-side concept, not a kinematics one.

---

## 5. Firmware ↔ URDF leg ID translation

The firmware on `main` numbers legs `0..3`. The URDF/Python code uses
two-letter IDs. **The names disagree on the back legs** (firmware says
"rear", URDF says "back"):

| Firmware id | Firmware name | URDF id | URDF/Python name |
|:---:|:---:|:---:|:---:|
| 0 | `fr` | `fr` | front-right |
| 1 | `fl` | `fl` | front-left |
| 2 | `rr` | **`br`** | rear-right / back-right |
| 3 | `rl` | **`bl`** | rear-left / back-left |

The URDF generator emits `bl_*` / `br_*` joint names. PyBullet's
`p.getJointInfo` returns those byte-for-byte. Any code doing
`joint_name[:2]` to extract the leg id receives `bl` / `br` — not `rl` / `rr`.

**Translation lives at the firmware boundary.** Don't propagate `rl`/`rr`
into Python / URDF / Blender code — they don't exist there. The animation
exporter (when written) must emit `bl`/`br` joint names to match URDF;
firmware can re-map locally.

---

## 6. Hardcoded-vs-URDF geometry — why theirs' `kinematics.py` was rejected

Theirs' `kinematics.py` carried these as module constants:

| Quantity | THEIRS hardcoded | URDF (re-export 2026-05-03) | Source-of-truth |
|---|---:|---:|---|
| `L1` | 0.080 m | ≈ 0.058 m | URDF |
| `L2` | 0.075 m | ≈ 0.095 m | URDF |
| `L3` | 0.077 m | ≈ 0.097 m | URDF |
| FL mount | (-0.040, +0.050, +0.025) | (-0.0579, +0.0462, -0.009) | URDF |
| yaw_offset (L) | 0 | per-leg from rpy_z (Convention A) | URDF |
| `_JOINT_LIMITS` | uniform | per-leg, post-rest-shift | URDF |

These don't represent a sign or convention disagreement — theirs was
just built against a different CAD revision than the current one. Using
theirs' values would put the legs at the wrong physical positions and
deliver IK solutions for the wrong robot. The URDF is treated as
the single source of truth; any consumer that needs link lengths or
mount positions reads them from there via `helpers._load_urdf_joints`
or `kinematics.build_config`.

---

## 7. What `facehugger_config.yaml` still controls (post-merge)

| Field | Status | Source-of-truth |
|---|---|---|
| `legs[].id` (`fl`, `fr`, `bl`, `br`) | KEEP | yaml (naming convention) |
| `legs[].mount_point` (`LegMountPointFL` etc.) | KEEP | yaml — wires Fusion landmarks to URDF |
| `legs[].side` (`L` / `R`) | KEEP | yaml — picks `leg_shoulder_{L,R}.stl` mesh |
| `legs[].rpy_z_deg` (0 or 180) | **DROP** (per §3-4) | derive from JSON `rest` + per-leg formula |
| `legs[].shoulder_limits_deg` | **DROP** (per §3) | derive from JSON `limits_rad` shifted |
| `legs[].shoulder_neutral_deg` | KEEP if non-zero | yaml — for FK stance pose distinct from URDF rest |
| `servo.mass_kg`, `effort_nm`, `velocity_rad_s` | KEEP | yaml — physical servo spec |
| `servo.visual_flip_rpy_deg` | KEEP | yaml — uniform visual-only mesh flip |

The "drop" rows are deferred work — they happen in the same Step-3 PR
as the actual URDF generator change.

---

## 8. Outstanding work after this merge

| # | Task | File(s) | Notes |
|---|---|---|---|
| 1 | **Implement Convention A in URDF generator** | [generate_urdf.py](../generate_urdf.py), [facehugger_config.yaml](../facehugger_config.yaml) | Read JSON `limits_rad.rest`, derive 4 corner rests via §4 formula, emit `<origin rpy="0 0 rest_rad">` + shifted `<limit>`. Drop yaml `rpy_z_deg` and `shoulder_limits_deg`. |
| 2 | Re-validate visualizers post-Convention-A | `visualize_urdf.py` joint pivots should overlay `visualize_fusion_export.py` landmark spheres. Robot should appear with all 4 legs splayed at 45° at rest. | |
| 3 | Wire up deferred CLI flags | [simulate.py](../simulate.py), [gaits.py](../gaits.py) | Port theirs' `bound`/`crab` gait definitions + `--terrain`/`--teleop` integration onto the cfg-driven API. |
| 4 | Port theirs' `terrain.py`/`teleop.py` to `cfg`-driven kinematics | Remove the "Not yet integrated" headers when done. | |
| 5 | Body mesh harmonization | Decide between `QuadrupedBody.stl` and the Fusion-`FlexibleSkeletonWithMotorMounts` body the teammate added on main. | |
| 6 | Rigging script (animation pipeline) | New `urdf_to_blender_rigged.py` forking `visualize_urdf.py`. Empties + parenting per joint chain. Regression: all-zeros pose must match `visualize_urdf.py` rest. | |
| 7 | Animation export format | Coordinate with teammate (per `code/API_SPEC.md`); CSV `(t, 12 angles)` is the default safe bet. | |

---

## 9. Convention checklist for new code

When adding any code that touches joint angles, servo channels, IK math,
or coordinate systems, verify:

- [ ] Shoulder axis is `+Z` global, not per-leg sign-flipped.
- [ ] Joint angles are radians end-to-end (degrees only at yaml/UI boundaries).
- [ ] θ=0 corresponds to the URDF rest pose, which equals the Fusion-defined
  rest. Not "leg straight along ±X."
- [ ] Limits are taken from the URDF (already shifted relative to rest), not
  re-imposed in code.
- [ ] Body frame: **+Y forward, +X right, +Z up** (the existing `gaits.py`
  comment is correct and authoritative). Any "ROS standard +X forward"
  assumption from imported code is wrong here — flag it.
- [ ] Leg IDs in URDF/Python: `bl`/`br`. Map to firmware `rl`/`rr` only at
  the firmware boundary.
- [ ] Link lengths and mount positions are read from the URDF, never
  hardcoded.
