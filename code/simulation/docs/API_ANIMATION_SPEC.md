# API_ANIMATION_SPEC — animator-facing reference

How to load the FaceHugger rig in Blender, pose it, and export the
result toward the firmware playback engine. Aimed at the animator
working day-to-day, not at someone re-implementing the import pipeline.

For the import pipeline internals, see
[`animation/scripts/urdf_to_blender_rigged.py`](../../../animation/scripts/urdf_to_blender_rigged.py)
and the surrounding [README](../../../animation/scripts/README.md).
For the joint-angle convention authority, see
[`MERGE_AND_CONVENTION.md`](MERGE_AND_CONVENTION.md). For servo
numbering, see
[`animation/SERVO_ID_CONVENTION.md`](../../../animation/SERVO_ID_CONVENTION.md).

---

## 1. Loading the rig

```bash
python code/simulation/facehugger.py blender --rigged
```

Builds a fresh scene with the FaceHugger model + posable rig in
Blender 5.x. Default Blender version is 5.1; override via
`--blender-version 5.2` or set `BLENDER_BIN` for a non-standard
install path.

Headless smoke-test variant (CI / scripting):

```bash
python code/simulation/facehugger.py blender --rigged \
    --headless --save /tmp/fh_rigged.blend
```

The script reads
[`code/simulation/generated/facehugger.urdf`](../generated/facehugger.urdf)
+
[`code/simulation/generated/fusion_export.json`](../generated/fusion_export.json)
+ STLs from
[`code/simulation/generated/exported_meshes/`](../generated/exported_meshes/).
Re-export from Fusion + regenerate the URDF before re-running if the
CAD has moved. The rig itself is rebuilt from scratch on every run —
nothing persistent in the `.blend`.

---

## 2. Rig structure

13 bones, all named to match URDF link names exactly (lowercase):

```
Armature "FaceHuggerRig"
├── base_link              (root, fixed — represents the chassis)
├── fl_link1               (FL shoulder; URDF axis = world +Z, bone-local Z after align_roll)
│   └── fl_link2           (FL hip;      URDF axis = world +Y, bone-local Z after align_roll)
│       └── fl_link3       (FL knee;     URDF axis = world +Y, bone-local Z after align_roll)
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

After `EditBone.align_roll(joint.axis)`, **bone-local Z is the URDF
joint axis for every joint uniformly** — including the per-side ±Y
sign on hip/knee (URDF `<axis>0 -1 0</axis>` for FR/BL). This means:

- Pose-mode rotation lives entirely on `rotation_euler[2]` for every
  servo. No per-joint axis branching anywhere.
- A positive `rotation_euler.z` rotates the leg "up" the same way on
  both L and R sides.

Constraints (auto-applied at rig-build time):

| Constraint | On which bones | Effect |
|---|---|---|
| `LIMIT_ROTATION` (use_limit_z) | all 12 joint bones | Clamp pose rotation to URDF `<limit lower upper>`. |
| `IK` (chain_count=2, use_stretch=False) | each `*_link3` (knee) | Solve hip+knee to reach `{leg}_foot_target`. Shoulder stays FK. |
| `lock_ik_x = lock_ik_y = True`, `ik_stiffness_x/y = 1.0` | all 12 joint bones | Force IK solver to rotate around bone-local Z only — no lateral bending. |

---

## 3. Animation workflow (the physical 3-servo recipe)

The rig matches how the 3 servos per leg actually work in hardware:

1. **FK shoulder (servo 1, world Z yaw)** — rotate `{leg}_link1` in
   pose mode by hand. Aims the leg laterally. Servo 1 is independent
   of the foot target.
2. **IK hip + knee (servos 2 + 3, both pitch)** — drag the
   `{leg}_foot_target` Empty. The IK constraint solves the hip + knee
   rotations to bring the foot to the empty's position. Shoulder
   doesn't move (chain_count=2 excludes it).
3. **Foot target follows the shoulder.** The foot target is parented
   to `{leg}_link1` via `parent_type='BONE'` — no driver, no manual
   re-positioning when you rotate the shoulder. Animator workflow:
   rotate shoulder → drag foot.

Why not `chain_count=3` (shoulder included in IK)? The combination of
"foot parented to link1" + "link1 in IK chain" creates an unstable
loop (IK rotates link1 → foot moves with link1 → IK iterates against
a moving target → ~180 mm of rest-pose drift). chain=2 is the stable
combination for this rig.

If you ever need full 3-DOF IK without the FK shoulder, run
`facehugger.py blender --rigged --` and pass `--ik-chain 3` — but
that requires un-parenting the foot target manually before the rig
is usable.

---

## 4. Neutral pose

The rest pose has every bone at `rotation_euler = (0, 0, 0)`. This is
the URDF θ=0 pose AND the splayed standing stance:

| Leg | Shoulder yaw (world frame) |
|---|:---:|
| FL | -45° |
| FR | +45° |
| BL | -135° |
| BR | +135° |

All hips and knees at 0°. The per-leg shoulder offset is encoded in
the URDF `<joint><origin rpy>` per Convention A — it does **not**
appear as a non-zero Blender rotation value. See
[`MERGE_AND_CONVENTION.md` §0/§3](MERGE_AND_CONVENTION.md) for the
authoritative spec.

The first and last frame of every gait cycle must return to this
pose so the firmware can transition between gaits without blending.
See `doc/gait-design/specs/blender-rig-and-export.md` §4 and
`gait-file-format.md` §1.

---

## 5. Bake before export

IK-solved angles are NOT stored in FCurves — only the foot target's
keyframes are. Before running the gait export script, **bake the
visual pose into per-bone FCurves**:

**GUI**: Pose Mode → menu Pose → Animation → Bake Action

```
☑ Visual Keying        (evaluates the IK solve at each frame)
☑ Clear Constraints    (optional — removes the IK after baking;
                        recommended once the gait is final)
☑ Overwrite Current Action
Frame range: match your animation
```

**Python** (run inside Blender Scripting workspace, or via a script
launched with `--python`):

```python
import bpy

bpy.ops.nla.bake(
    frame_start=bpy.context.scene.frame_start,
    frame_end=bpy.context.scene.frame_end,
    only_selected=False,
    bake_types={"POSE"},
    visual_keying=True,
    clear_constraints=False,
    use_current_action=True,
)
```

After baking, every joint bone has explicit `rotation_euler[2]`
keyframes at every frame — that's what the export script reads.

---

## 6. Angle conversion (Blender → servo)

The export script reads `pose.bones["{leg}_link{N}"].rotation_euler[2]`
for each frame and converts to servo PWM degrees:

```
servo_deg = offset_deg + direction * degrees(blender_z_rad)
servo_deg = clamp(servo_deg, min_deg, max_deg)
```

Per-bone fields come from
[`servo_mapping.yaml`](../../../doc/gait-design/specs/blender-rig-and-export.md#6-servo-mapping-config)
(authoritative schema in
[`blender-rig-and-export.md` §6](../../../doc/gait-design/specs/blender-rig-and-export.md#6-servo-mapping-config)):

- **`offset_deg`** — servo PWM angle when Blender bone is at 0 rad
  (the URDF rest pose). Default 135° (mid of the DSS-M15S 0–270°
  range). Calibrated per assembled robot.
- **`direction`** — 1 or -1, hardware-calibration-only (kinematic L/R
  asymmetry is already absorbed into bone roll via `align_roll`).
  Default 1 everywhere; flip to -1 only after seeing a specific servo
  drive its joint backward on the assembled robot.
- **`min_deg` / `max_deg`** — post-conversion PWM safety clamps,
  separate from the URDF kinematic limits (which the rig enforces via
  `LIMIT_ROTATION`).
- **`servo_id`** — firmware-side per-PWM-channel index. The default
  numbering uses URDF leg ordering (`fl=0..2, fr=3..5, bl=6..8,
  br=9..11`); see
  [`SERVO_ID_CONVENTION.md`](../../../animation/SERVO_ID_CONVENTION.md)
  for the proposal status and how it aligns (or not) with the
  firmware's `SERVO_CONFIG[]` ordering.

---

## 7. Gait file output

The end product of the export pipeline is a `.gait` JSON file —
sparse keyframes per servo, time in milliseconds, angles in servo
degrees. Format spec:
[`doc/gait-design/specs/gait-file-format.md`](../../../doc/gait-design/specs/gait-file-format.md).
Firmware playback engine spec:
[`doc/gait-design/specs/esp32-playback-engine.md`](../../../doc/gait-design/specs/esp32-playback-engine.md).

Validation: every track's first and last keyframe must equal the
neutral-pose servo angle (typically 135°), tracks must cover all 12
servos, and angles must stay within `min_deg`/`max_deg`. The
validator catches these before the gait file ships to firmware
compilation (`gait_to_c.py`).

---

## 8. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Leg bends laterally when dragging foot target | IK X/Y locks not applied (somehow) | Re-run `facehugger.py blender --rigged` to rebuild rig from scratch. Check pose-bone properties: `lock_ik_x = lock_ik_y = True`. |
| Foot doesn't follow shoulder rotation | Foot target lost its parent (got un-parented in pose mode) | Re-parent: select foot target → Object Properties → Relations → Parent: Armature, Bone: `{leg}_link1`, Type: Bone. Then move it to the rest position. |
| IK rotates the shoulder | `--ik-chain 3` was passed | Re-run with default `--ik-chain 2`, or unparent the foot target. |
| Mesh "drifts" in viewport vs PyBullet | URDF was regenerated but the rig wasn't rebuilt | Re-run `facehugger.py blender --rigged`. The rig is rebuilt from scratch on every invocation. |
| Joint goes past the URDF limit | `LIMIT_ROTATION` constraint disabled | Check pose bone constraints in the Properties panel. |
| Foot target starts in the wrong place | `Link3TipAxis` missing from `fusion_export.json` (CAD not re-exported) | Re-export from Fusion. Falls back to a hardcoded value in the meantime — animator-visible offset of ~50–60 mm in some legs. |

---

## 9. Cross-references

- [`animation/scripts/urdf_to_blender_rigged.py`](../../../animation/scripts/urdf_to_blender_rigged.py) — implementation
- [`animation/scripts/README.md`](../../../animation/scripts/README.md) — script-level overview
- [`animation/SERVO_ID_CONVENTION.md`](../../../animation/SERVO_ID_CONVENTION.md) — servo numbering
- [`code/simulation/docs/MERGE_AND_CONVENTION.md`](MERGE_AND_CONVENTION.md) — joint angle convention
- [`doc/gait-design/specs/blender-rig-and-export.md`](../../../doc/gait-design/specs/blender-rig-and-export.md) — `servo_mapping.yaml` schema, export details
- [`doc/gait-design/specs/gait-file-format.md`](../../../doc/gait-design/specs/gait-file-format.md) — `.gait` JSON format
- [`doc/gait-design/specs/esp32-playback-engine.md`](../../../doc/gait-design/specs/esp32-playback-engine.md) — firmware-side consumer
