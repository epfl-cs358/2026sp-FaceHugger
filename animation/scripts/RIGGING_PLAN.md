# RIGGING_PLAN — animation rig on top of `visualize_urdf.py`

Plan for the next animation-pipeline branch: extend the URDF visualizer
with a Blender rig so the FaceHugger can be keyframed in pose mode and
the timeline exported as servo angles. **Implementation is deferred** —
this document captures the design + open decisions before code lands.

---

## 1. Goal

Today: `visualize_urdf.py` places the 29 STL meshes at their rest-pose
world positions via `obj.matrix_world = link_world @ visual_origin`. No
parenting, no rig — just static geometry. Useful for verifying the URDF
matches the CAD; useless for animation.

Target: a sibling script (working name: `urdf_to_blender_rigged.py`)
that produces the same rest-pose geometry **plus** a rig on top, so:

- An animator can grab a leg in pose mode and rotate it. The mesh
  follows the constraint chain (shoulder → hip → knee → foot).
- Joint limits from the URDF prevent rotating into impossible angles.
- A timeline animation can be exported as `(t, 12 servo angles)` rows
  for the firmware on `main` to consume.

---

## 2. Two architecture options — pick one before coding

### Option A — Empties parented in a chain (the urdf_to_blender.py path)

```
base_link_empty
  ├── fl_link1_empty   (matrix_local = fl shoulder joint origin)
  │     ├── fl_link2_empty  (matrix_local = fl hip joint origin)
  │     │     └── fl_link3_empty  (matrix_local = fl knee joint origin)
  │     └── (×4 legs)
```

Each visual mesh is parented to its link Empty with
`obj.parent = link_empty; obj.matrix_local = visual_origin`.
Animation = keyframing `rotation_euler` on the 12 joint Empties around
their `joint.axis`.

**Pros**:
- 1:1 mapping URDF joint → Blender Empty. Joint angle in URDF = Empty
  rotation around the joint axis, no roll calibration.
- The math in `visualize_urdf.py` extends almost verbatim — just swap
  `obj.matrix_world =` for `obj.parent + matrix_local =`.
- Worked out the parent-inverse pitfalls already (this branch's
  `urdf_to_blender.py` history captures the gotcha — see commit
  `2b5742f` for the proven fix pattern).

**Cons**:
- Not standard Blender rigging — Empties aren't visible as bones in
  pose mode. You can't grab a "leg" and pose it like an armature; you
  rotate one Empty at a time.
- `LIMIT_ROTATION` constraints work on Empties but feel less idiomatic
  than on bones.
- Visual feedback in the viewport is just arrow-shaped Empties, not
  actual bone glyphs.

### Option B — Armature with bones (the LLM's recommendation)

```
Armature
  ├── bone_body          ← root, fixed
  ├── bone_fl_shoulder   ← head at BodyToLink1Point, tail at Link1ToLink2Point
  │     └── bone_fl_hip  ← head at Link1ToLink2Point, tail at Link2ToLink3Point
  │           └── bone_fl_knee  ← head at Link2ToLink3Point, tail at FootTip
  │     (×4 legs)
```

Each visual mesh is parented to its bone with `parent_type='BONE'`
(no skinning needed — the meshes are rigid bodies).

**Pros**:
- Standard Blender rigging pattern. Pose mode shows bones as
  manipulable widgets — animators get the experience they expect.
- `LIMIT_ROTATION` constraints are native to bones.
- Bone head→tail vectors give natural "this is what the leg link
  looks like" hints in the viewport.

**Cons**:
- **Bone roll calibration**. Each bone has a "roll" — rotation around
  its own axis — that determines which local axis maps to which
  physical rotation. Set wrong, the LIMIT_ROTATION constraint clamps
  the wrong axis. Roll has to be derived from the URDF joint axis, and
  Blender's roll convention isn't always what you'd expect from the
  URDF.
- `parent_type='BONE'` attaches the child to the bone's **tail** by
  default, not the head. Have to either set `obj.matrix_parent_inverse`
  carefully or use `parent_type='OBJECT'` and parent to a bone-tracking
  empty.
- The reference `urdf_importer-master/` (see
  `code/simulation/docs/.gitignore`) uses this pattern and is the
  source for the LLM research — but its setup is gnarly enough that
  the original author wrote ~500 lines of bone-construction code
  alone.

### Decision (2026-05-04): Option B — Armature

**Implemented as [`urdf_to_blender_rigged.py`](urdf_to_blender_rigged.py)**.
Option A was the safer first pass, but the animator workflow needs
**inverse kinematics** (drag the foot, hip + knee solve), which is
native to bone armatures and awkward to bolt onto plain Empties.

The bone-roll calibration that this section originally flagged as
"its own debugging week" turned out to be **one line per bone** thanks
to `EditBone.align_roll(joint_axis_world)`: pass the URDF axis vector
in world frame and Blender sets the roll so bone-local Z aligns with
that axis. After `align_roll`, all 12 joints rotate uniformly on
`rotation_euler[2]` — including the per-side ±Y sign flip on hip/knee
(URDF `<axis>0 -1 0</axis>` for FR/BL), which gets absorbed into the
bone roll automatically.

The rest-pose regression test (29 meshes) passes within 0.5 mm of
[`visualize_urdf.py`](visualize_urdf.py)'s placement-only baseline — max
drift 0.0003 mm; the 0.5 mm tolerance is for accumulated floating-point
noise across the 3-joint chain on link3 meshes, not a methodology
loosening.

---

## 3. Algorithm (Option A — write Empties version first)

```
1. Parse URDF (existing visualize_urdf.parse_urdf).

2. Force depsgraph update after each layer is built — see commit
   2b5742f's lessons. Set matrix_parent_inverse = Identity right after
   each obj.parent = ... assignment so the parent-inverse trap can't
   bend the chain.

3. For each link (in URDF document order):
     empty = create Empty named link_name
     if link_name == root:
         empty.matrix_world = Identity
     else:
         empty.parent = empties[joint.parent]
         empty.matrix_parent_inverse = Identity
         empty.matrix_local = joint.origin   # 4×4 mathutils.Matrix
     empty.rotation_mode = "XYZ"   # Euler keyframing

   bpy.context.view_layer.update()   # force matrix_world cascade

4. For each visual on each link:
     import STL with global_scale=1.0  (mm scene)
     obj.parent = empties[link_name]
     obj.matrix_parent_inverse = Identity
     obj.matrix_local = visual_origin   # already in mm via the
                                          # matrix_m_to_mm helper

5. For each joint, add a LIMIT_ROTATION constraint to the joint's
   child Empty:
     constraint = empty.constraints.new('LIMIT_ROTATION')
     constraint.use_limit_x|y|z = True for the joint's axis
     constraint.min_*|max_* = URDF lower|upper (radians)
     constraint.owner_space = 'LOCAL'

6. Drop the joint pivot/axis markers (the red/orange spheres) — or
   keep them as a separate collection toggle. They're useful for
   debugging the rest pose; not needed for animation.
```

**Regression test before merging the rig PR**: at all-zeros pose, the
rigged scene must reproduce `visualize_urdf.py`'s baseline byte-for-byte.
Open both `.blend` files, overlay them in Blender, every mesh should sit
at the same world position. Any mesh that drifts means the rig is doing
something the placement-only version isn't.

---

## 4. Data sources (no new fields needed)

Everything required is already in the URDF + the `fusion_export.json`:

| Datum | Source | How `visualize_urdf.py` reads it |
|---|---|---|
| Joint origin (xyz, rpy) | URDF `<joint><origin>` | `_parse_origin` → 4×4 Matrix |
| Joint axis (unit vector) | URDF `<joint><axis xyz>` | `_parse_axis` → mathutils.Vector |
| Joint limits (lower, upper, rad) | URDF `<joint><limit>` | not yet read; add `_parse_limit` (5 lines) |
| Visual origin (xyz, rpy) | URDF `<visual><origin>` | `_parse_visual` |
| Mesh filename + scale | URDF `<mesh filename scale>` | `_parse_visual` |
| Per-leg shoulder rest | already baked into `<joint><origin rpy>` by Convention A | nothing — chain walk picks it up |

Only addition: `_parse_limit(joint_elem)` returning `(lower_rad, upper_rad)`
or `(None, None)` for fixed joints.

**No need to read fusion_export.json directly** for the rig. The URDF
is the single source of truth (per `MERGE_AND_CONVENTION.md`).

---

## 5. Animation export format — coordinate with firmware

Out-of-scope for the rig PR; called out here for sequencing.

The rig produces 12 keyframable joint angles. `animation_export.py`
walks the timeline, evaluates each joint Empty's `rotation_euler` at
each frame, and writes the trajectory. Three plausible formats:

| Format | Pro | Con |
|---|---|---|
| **CSV `t, fl_link1, fl_link2, … (12 cols)`** at fixed Hz | Trivial firmware parser, no interpolation logic | Larger files |
| **Keyframe + curve type per joint** | Smaller files, smoother | More firmware logic |
| **Compiled bytecode** | Smallest | Opaque; hard to debug |

**Default safe bet: CSV.** Coordinate with whoever owns
`code/firmware/` (per `code/API_SPEC.md`) before locking it in. Pick
the format **before** starting on the export script.

---

## 6. Open decisions (answer before coding)

1. **Architecture**: Option A (Empties) or Option B (Armature)? My
   recommendation is A; if rejected, plan adjusts.
2. **Joint limit application**: as `LIMIT_ROTATION` constraints
   (clamps in pose mode), or just informational (URDF carries them but
   Blender doesn't enforce)? Constraints are nicer UX but add
   complexity for one-axis joints.
3. **Mesh visibility**: keep all 29 visuals (current
   `visualize_urdf.py` behaviour) or drop the chassis-fixed servos
   and bracket meshes for a "skeleton only" view? The animator
   probably wants the full robot; the rigging-debugger wants minimal.
   Could be a CLI flag.
4. **Animation export format** (§5).

---

## 7. Files this work touches

- **New**: `animation/scripts/urdf_to_blender_rigged.py` (forks
  `visualize_urdf.py`).
- **New**: `animation/scripts/animation_export.py` (timeline →
  format-of-choice).
- **Update**: `animation/scripts/README.md` to describe the rig
  script and the export.
- **Update**: `code/simulation/facehugger.py` to add a `rig`
  subcommand wrapping the rigged version (optional — could also be
  a flag on `blender`).
- **Update**: `code/simulation/docs/SIM_PIPELINE.md` to document the
  rig as the canonical animator-facing tool.
- **No URDF / generator changes.** The rig is purely consumer-side.

---

## 8. Why this is its own PR (and its own branch)

Out of `MERGE_AND_CONVENTION.md` §8 the rigging script is
item #6, and the animation exporter is item #7. They're sequenced
after the current `feat/urdf-pipeline` foundation lands so the rig
can rely on Convention A being in place: the rigged pose at all-zero
joint angles equals the URDF rest pose, which is the splayed standing
pose under Convention A. If Convention A weren't in place, the
all-zeros rig would show legs along ±X (not splayed) and an animator
would have to fight that on every keyframe.

Branch: `feat/animation-pipeline`. PR after `feat/urdf-pipeline`
merges to `main`.
