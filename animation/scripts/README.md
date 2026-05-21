# animation/scripts — Blender tooling

Blender 5.x scripts that build the FaceHugger scene from the URDF or
Fusion export, plus animator-facing helpers (clip panel, one-shot
migrations). The scene-building scripts share the same conventions
(1 Blender unit = 1 mm, viewport clip 0.1 → 10000) and the same
red/orange marker palette, so you can open multiple `.blend` outputs
and overlay them for cross-validation.

| File | Kind | Source of truth | What it's for |
|---|---|---|---|
| `urdf_to_blender_rigged.py` | scene builder (`--python`) | `facehugger.urdf` + `fusion_export.json` | **Animator-facing**. Pose-able rig: FK shoulder + IK on hip+knee, foot-target Empties. |
| `visualize_urdf.py` | scene builder (`--python`) | `code/simulation/generated/facehugger.urdf` | Cross-check: does the URDF chain walk reproduce PyBullet's `loadURDF` rest pose? |
| `visualize_fusion_export.py` | scene builder (`--python`) | `code/simulation/generated/fusion_export.json` | Cross-check: does the CAD's raw landmark/world data match the URDF generator's output? |
| `fh_clip_panel.py` | UI add-on (sidebar N-panel) | `bpy.data.actions` + `animation/convention.json` + `animation/poses.json` | Clips (5-Action bundles), a position-based Pose Library, selection sets, export to `exported_gaits/`, activity heatmap — via the layered Action API. |
| `fh_rename_actions.py` | **LEGACY** one-shot CLI (`--background`) | `animation/fh_rigged_latest.blend` | Idempotent migration of legacy `body_ctrlAction` / `foot_target_*Action` names to `base_anim__<target>`. Run once per old rig; not day-to-day. |

## urdf_to_blender_rigged.py — animation rig

Builds a real Blender Armature on top of the URDF: 13 bones (12 leg +
`base_link`), one bone per URDF link, parented in chain. Each bone's
roll is set by `EditBone.align_roll(joint.axis)` so bone-local Z is the
URDF joint axis for every joint uniformly — including the per-side
±Y sign on hip/knee. Visual STLs are parented to their bone via
`parent_type='BONE'`, so the meshes follow pose-mode rotations.

**Animation workflow** — built around the physical 3-servo layout:
1. **FK shoulder**: rotate `{leg}_link1` in pose mode to aim the leg
   laterally (servo 1, world Z yaw).
2. **IK foot reach**: drag the `{leg}_foot_target` Empty. The IK
   constraint solves hip + knee (servos 2 + 3) to bring the leg's
   foot tip to the empty's position. Shoulder stays at its FK pose.
3. The foot target is **parented to the link1 bone**, so when you
   rotate the shoulder in step 1, the target follows automatically —
   no driver / no driver-update step.

**Constraints set up automatically:**
- `LIMIT_ROTATION` on every joint bone, `use_limit_z=True`,
  `min_z` / `max_z` from URDF `<limit lower upper>`. Pose-mode rotation
  clamped to the URDF's per-joint range.
- `IK` constraint on each `{leg}_link3` (knee) bone, `chain_count=2`
  (hip + knee solve), `use_rotation=False`, `use_stretch=False`
  (rigid bones), targeting `{leg}_foot_target`.
- Per-bone IK locks: `lock_ik_x = lock_ik_y = True`,
  `ik_stiffness_x = ik_stiffness_y = 1.0`. After `align_roll`, the
  joint axis is bone-local Z; locks force the IK solver to rotate
  only around Z (= the URDF joint axis), no lateral bending.

**Foot tip source**: read from `Link3TipAxis` in `fusion_export.json`
when present (CAD source-of-truth), with a hardcoded link3-frame
fallback if the export hasn't been re-run with the axis added to
the whitelist.

**Run:**
```bash
python code/simulation/facehugger.py blender --rigged
# or, headless + save:
python code/simulation/facehugger.py blender --rigged --headless --save /tmp/fh_rigged.blend
```

CLI flags after `--`: `--urdf PATH`, `--meshes PATH`, `--json PATH`,
`--save PATH`, `--ik-chain {2,3}` (default 2; 3 is unstable in
combination with the parented foot target — see the script's
docstring for the why).

**Markers**: same red/orange spheres as the placement-only visualizer,
but parented to their bone so they follow pose rotations. Hidden by
default (`hide_viewport=True` on `Joint Origins/` and `Joint Axes/`
collections); toggle the eye in the outliner to see them.

**Object hierarchy** (top-down, post-build):

```
world_origin   (PLAIN_AXES Empty, world (0,0,0) — never animated)
  ├── body_ctrl   (CUBE Empty — animator handle for chassis position + rotation)
  │     └── FaceHuggerRig   (armature, hide_viewport=True; bones drive limbs)
  └── foot_target_{fl,fr,bl,br}   (SPHERE Empties — animator handles)

foot_ik_{fl,fr,bl,br}   (PLAIN_AXES, hidden, UNPARENTED;
                          COPY_LOCATION pulls them to foot_target.world + static offset)
```

`world_origin` is a deterministic anchor for downstream exporters and
is not intended to be animated. It sits at world identity, so each
child's local transform equals its world transform. `foot_ik_*` are
deliberately NOT parented: the COPY_LOCATION offset would double under
world_origin motion if they were (target.world + owner.pre_constraint_world
both shift by the same delta). foot_ik follows foot_target purely via
the constraint, which is correct regardless of anchor motion.

**Regression vs visualize_urdf.py**: at all-zero pose, every visual
mesh's world position matches the placement-only baseline within
0.5 mm (0.001 rad rotation tolerance). Any deviation means the rig is
composing transforms wrong and is worth investigating.

See [code/simulation/docs/API_ANIMATION_SPEC.md](../../code/simulation/docs/API_ANIMATION_SPEC.md)
for the full animator-facing API reference (baking, export to
`.gait`, servo conversion).

---

## visualize_urdf.py — current baseline

Reads the URDF directly and walks the joint chain at rest pose (all joint
angles 0°), the same way PyBullet does on `loadURDF`. One pass:

```
link_world[root]  = Identity
link_world[child] = link_world[parent] @ joint.origin   (rest pose)
mesh.matrix_world = link_world[link]   @ visual.origin
```

Translation is in metres throughout the chain; the script multiplies by
1000 once at the end to land in the mm-scale Blender scene. Rotations
pass through unchanged (they're unitless).

**Markers:**
- **Red spheres** in `Joint Origins/` — one per joint, at the world position
  of that joint's pivot (the child link's frame origin).
- **Orange spheres** in `Joint Axes/` — one per joint, 20 mm out from the
  pivot along the unit axis vector. Direct visualization of `<axis xyz>` —
  spot a bad sign or a misnamed axis instantly.

**Run:**

```bash
/Applications/Blender-5.1.app/Contents/MacOS/Blender \
    --python animation/scripts/visualize_urdf.py
# or, headless + save:
/Applications/Blender-5.1.app/Contents/MacOS/Blender --background \
    --python animation/scripts/visualize_urdf.py \
    -- --save /tmp/fh_urdf.blend
```

CLI flags after `--`: `--urdf PATH`, `--meshes PATH`, `--save PATH`.

### Known minor issues (revisit before rigging if they bite)

- **No "above link1" construction-point markers.** The URDF only carries
  joint origins, not raw CAD landmarks. So the script puts a red sphere at
  each joint pivot (where the joint rotates) but cannot show
  `BodyToLink1Point`, `LegMountFixedPoint`, `ServoMountPoint`, etc. — those
  only exist in `fusion_export.json`. If you need the richer landmark set,
  open the fusion-export visualizer's .blend alongside.
- **Axis tips aren't always visually obvious.** All 12 joints get an orange
  sphere placed at `pivot + 20mm · unit_axis`, but for the shoulder joints
  whose axis is +Z, the tip sits inside / above the body shell where it can
  be hard to see. Increasing `AXIS_LENGTH_MM` or hiding the chassis mesh
  helps. (Eventually we may want oriented arrows / line objects instead of
  spheres so direction is unambiguous.)

## visualize_fusion_export.py — JSON cross-check

Reads `fusion_export.json` directly and places each STL at its
`origin_landmark`'s `pos_world_mm`. The 4-leg replication uses
`LegMountPointXX` plus per-corner `Rz(rpy_z_deg)` to instance the source
FL leg at the other three corners. Pure mesh placement; the `*_root`
Empties were dropped per "no rigging" — there's no rig parent.

Useful for diagnosing CAD-side issues (missing landmarks, wrong
`origin_world_mm` on a construction point, mesh in wrong frame). When the
URDF visualizer disagrees with this one, the disagreement points at the
URDF generator (`code/simulation/generate_urdf.py`).

## fh_clip_panel.py — clip + pose tooling add-on

A Blender add-on (not a CLI script) that adds a **"FaceHugger"** tab to
the 3D viewport sidebar (press `N`). The single old "FH Clips" panel is
now a parent panel with five collapsible sub-panels.

Two core data models, kept deliberately separate:

- **Clips** — named groups of 5 Actions on the rig's control objects:
  ```
  <clip_name>__body_ctrl
  <clip_name>__foot_target_{fl,fr,bl,br}
  ```
  Applied via the Blender 4.4+ **layered Action API** (`obj.animation_data.action`
  + `.action_slot`). No NLA, no drivers — direct per-object assignment.
- **Poses** — a position-based library in committed `animation/poses.json`
  (single-source-of-truth file, same pattern as `convention.json`). A
  *pose* is a snapshot of the 5 controls' **local** transforms
  (`body_ctrl` loc+rot, the 4 `foot_target_*` loc only — the IK reads
  only foot-target position). Poses are **clip-independent**: applying a
  pose sets transforms *only* — no keyframes, no Action/clip side
  effects. Local transforms are stored so a pose round-trips with
  keyframes ("Key into Clip").

**Loading** (two ways):

| Method | When to use |
|---|---|
| `Edit > Preferences > Add-ons > Install...` → pick this file → enable "FH Clip Panel" | Persists across sessions. Use for actual animation work. |
| Open in Text Editor → press `Alt+P` (Run Script) | Dev iteration. The script unregisters before re-registering, so you can edit and re-run without restarting Blender. |

**Panel structure** (parent `FaceHugger` + `bl_parent_id` children):

- **(parent) FaceHugger** — always-visible status: active-clip header,
  and the *not-keyed notice* (decision B). Applying a pose is a live
  viewport change with nothing committed; the parent shows a yellow ⚠
  box `Pose '<name>' applied — not keyed` plus a one-click
  `Key into '<clip>' @ frame N` button that re-applies the pose and
  inserts its 5 keyframes into the **active clip's own Actions** at the
  recorded frame (refuses to key controls not bound to that clip — no
  stray Actions / cross-clip writes). The notice clears on keying, on
  applying another pose, or on applying a clip (now stale).
- **Poses** — one row per library pose: Apply · Rename · Delete. Name
  field + Save Current Pose (re-saving a name updates it in place).
- **Clips** — one button per clip (active = depressed/filled radio)
  applies all 5 Actions; the ⟳ icon next to each **overwrites that
  clip** with the currently-bound animation (confirm dialog;
  destructive to the clip's old content, source left intact). New-clip
  name field + New Clip · Save Current → New Clip · Duplicate · Rename
  (last two need an active clip). Apply auto-completes a clip missing
  targets at neutral so clip switching is exact.

  *Note on "updating" a clip:* a clip **is** its 5 bound Actions, so
  editing/keyframing while a clip is **active** already updates it in
  place (just save the `.blend`). The ⟳ overwrite is for the other
  case — pushing animation that came from elsewhere (another clip, a
  fresh session, drifted names) onto an existing clip by name.
- **Selection** — `All · Body · Legs · Front · Back · FL · FR · BL ·
  BR`. One operator (preset arg); replaces the selection and sets the
  active object. Pure viewport selection — touches no data.
- **Export** — Max-Simultaneous-Servos guard; CSV / `.h` / `.js`
  format toggles; **Export Active Clip**; a **Clips to export**
  checklist (tick any subset) + **Export Selected Clips**. Bakes the
  IK-solved joint angles once per clip (`bake_clip`, Layer 1) then runs
  the enabled converters (Layer 2) into
  `animation/exported_gaits/<clip>/` (git-ignored output) — one folder
  per clip. The `.h` stays raw bone angles; the `.js` applies the full
  hardware conversion (scale-from-neutral + per-leg `translateToServo`)
  from `convention.json`. (No more Set N/Flat buttons here — those poses
  ship in the library now; see below.)
- **Display** (collapsed by default) — Activity Heatmap toggle:
  colours the per-joint **servo** meshes by per-frame angle delta
  (green→orange→red). link2/hip has no `__servo` mesh, so 8 of 12
  joints are shown; viewports switch to Object colour while active and
  are restored losslessly on toggle-off.

**Default poses ship in `animation/poses.json`** (committed, like
`convention.json`). A fresh checkout ships two poses in
`animation/poses.json`:

- `flat` — body on the BodyBottomPoint (`z=-17`), legs splayed flat
  (URDF θ=0 rest).
- `standing` — body raised (`z=+100`), feet tucked under.

A third pose, `neutral` (body at identity, the convention-N hardware
stance), is **not** shipped in `poses.json` — it is seeded on demand by
the console re-seed path below (the convention-N joint angles exceed the
rig's URDF `LIMIT_ROTATION` on some joints, so it isn't kept as a
default library entry).

Just **Apply** the shipped poses from the Poses panel. The old Set N /
Set Flat buttons were removed (they were redundant once the poses ship
as defaults). The seeding machinery still exists as a **console re-seed**
path if you ever delete `flat` from the library or want to (re)create
`neutral`:

```python
bpy.ops.fh.set_rest_pose()   # re-seed + apply 'flat'
bpy.ops.fh.set_n_pose()      # re-seed + apply 'neutral'
```

Seeding reuses the rig's own kinematics (`_pose_foot_targets`):
body_ctrl is set to identity, the foot targets are read at the
requested joint angles, the resulting **local** transforms captured,
and the scene restored. (`convention.json`'s N exceeds the rig's URDF
`LIMIT_ROTATION` on some joints — this position-based library is
exactly why poses are defined by reachable foot positions, not
unreachable angles.)

**Operator IDs** (callable from the Python console):

```python
bpy.ops.fh.apply_clip(clip_name="stand up")
bpy.ops.fh.new_clip()                    # reads scene.fh_new_clip_name
bpy.ops.fh.save_as_clip()                # snapshot live actions → new clip
bpy.ops.fh.duplicate_clip()              # reads scene.fh_new_clip_name
bpy.ops.fh.overwrite_clip(clip_name="walk")  # live anim → overwrite existing
bpy.ops.fh.rename_clip()                 # reads scene.fh_new_clip_name
bpy.ops.fh.export_clip()                 # active clip → exported_gaits/
bpy.ops.fh.export_selected()             # ticked clips (scene.fh_export_clips)

bpy.ops.fh.pose_save()                   # reads scene.fh_pose_name
bpy.ops.fh.pose_apply(pose_name="flat")
bpy.ops.fh.pose_rename(pose_name="old")  # new name in scene.fh_pose_name
bpy.ops.fh.pose_delete(pose_name="flat")
bpy.ops.fh.key_pose_into_clip()          # commit pending pose to active clip
bpy.ops.fh.set_n_pose()                  # console re-seed+apply 'neutral'
bpy.ops.fh.set_rest_pose()               # console re-seed+apply 'flat'
bpy.ops.fh.select_controls(preset="LEGS")
```

**No timer / polling** — the panel rescans `bpy.data.actions` /
`poses.json` on every redraw, and operators call `area.tag_redraw()`
after mutating, so the lists stay current. Items added through other
tools appear on the next mouse-move into the panel.

### Typical workflow

The mental model: a **clip is its 5 bound Actions**; a **pose is a
saved set of control transforms** (independent of clips).

1. **Block out a stance.** In **Poses**, Apply `flat` / `neutral` /
   `standing` (or your own). Apply only sets transforms — nothing is
   keyed yet, so the parent panel shows the yellow *“Pose ‘…’ applied
   — not keyed”* notice.
2. **Pick a clip to work in.** In **Clips**, Apply the clip (or `New
   Clip`). Its 5 Actions are now bound to the controls.
3. **Commit the pose into the clip.** Click *Key into ‘<clip>’ @ frame
   N* in the notice — it inserts the 5 keyframes into that clip at the
   frame the pose was applied. Repeat at other frames to build motion.
4. **Editing a clip = updating it.** While a clip is active, every
   keyframe you set lands directly in its Actions — the clip *is*
   updated in place. Just **Ctrl-S** the `.blend`. There is no
   separate “save clip”.
5. **Branch / overwrite.**
   - `Save Current → New Clip` — snapshot whatever is bound into a
     **new** clip (does not touch the source; Duplicate/Save *before*
     experimenting if you want a fallback).
   - The ⟳ on a clip row — push the currently-bound animation onto
     **that existing** clip (confirm; for animation that came from
     elsewhere — another clip, a fresh session, drifted names).
   - `Save Current Pose` — store the 5 control transforms as a named
     library pose (re-saving a name updates it).
6. **Select fast.** The **Selection** buttons grab control groups
   (`Legs`, `Front`, `FL`, …) so you can grab/key them together.
7. **Export.** In **Export**, tick CSV / `.h` / `.js`, then either
   *Export Active Clip* or tick clips in *Clips to export* and hit
   *Export Selected Clips* → `animation/exported_gaits/<clip>/`.

### Running an exported `.js` clip in a browser

The generated `.js` is the **Phase-1** way to play a clip on the robot
without firmware changes — full design context in
[`doc/animation-pipeline/onboard-clip-player-design.md`](../../doc/animation-pipeline/onboard-clip-player-design.md)
(read §3 for the locked once-shot + hold-at-end semantics this `.js`
mirrors).

1. **Author + export.** Build the clip, tick **Browser JS (.js)**, hit
   *Export Active Clip* → `animation/exported_gaits/<clip>/<clip>.js`.
2. **Optional `.js` toggles** (under the Browser JS tick):
   - *Dry run* — emits a variant that `console.log`s every
     `{T:4,id,a}` message instead of sending it. **Validate the servo
     stream with no robot needed; safe to paste on any page (even
     https).**
   - *Loop* — opt in to loop the clip for diagnostics. Off by default
     so the exported `.js` matches the firmware's one-shot+hold
     semantics (Phase-1 ↔ Phase-2 parity).
3. **Connect.** Join the robot's Wi-Fi (`FaceHugger_Net`, robot at
   `192.168.4.1`). Open a console on a **non-HTTPS** page (http://,
   file://, or about:blank). `ws://` is blocked from `https://` —
   this is the #1 reason "nothing happens".

> **⚠️ Before you run: stop any active gait.** The exported `.js`
> streams via `CMD_CALIBRATE` (`T:4`), which writes servo channels
> directly regardless of FSM state. If a gait is active concurrently,
> `tickGait()` overwrites all 12 servos every `update()` loop on
> `STATE_WALK` and the clip won't show. Put the robot in IDLE with no
> gait first:
>
> ```js
> ws.send(JSON.stringify({"T": 5, "g": 0}));   // CMD_GAIT_MODE → GAIT_NONE
> ws.send(JSON.stringify({"T": 2, "s": 0}));   // CMD_STATE     → STATE_IDLE
> ```
>
> Either one is enough on its own (firmware `update()` skips
> `tickGait` if `currentGait_ == GAIT_NONE` *or* state isn't
> `STATE_WALK`); both is belt-and-suspenders. Verified against
> `origin/main`: `data.h` enums (`CMD_STATE=2`, `CMD_GAIT_MODE=5`,
> `STATE_IDLE=0`, `GAIT_NONE=0`), `network.cpp` handlers, and
> `spinal_cord.cpp` `rest()`/`setGait()`. **Phase-1-only concern:** the
> future on-board clip player (spec [§3.5](../../doc/animation-pipeline/onboard-clip-player-design.md))
> is mutually exclusive with the gait engine by construction.

4. **Run.** Paste the `.js`. It opens `ws://192.168.4.1:81`, plays
   each frame's pre-converted servo angles per joint as
   `{T:4, id:<leg_id 0-3>, servo_id:<0-2>, a:<0-180>}` — the
   firmware's `CMD_CALIBRATE` shape on `origin/main`; the firmware
   maps to a PCA channel via `LEG_SERVO_CHANNEL[id][servo_id]`. At
   the last frame the player **keeps re-sending it every 30 ms** to
   hold the pose — same as firmware `tickClip`.
5. **Stop.** Call `fhStop()` in the console. It clears the interval
   and closes the socket; the servos hold their last commanded
   position. (This is the safe stop. Closing the tab works too.)

**Pre-scale rule.** The `.js` carries already-converted servo angles
(scale-from-N + per-leg `translateToServo` applied at bake by
`_frame_to_servo`); the browser only clamps to `0..180` and sends. The
upcoming bundled `clips_all.h` will instead carry **pre-scaled
math-space joint degrees** and let the firmware do `translateToServo`
at runtime — see the spec §3.2/§5.

**Servo math + channel mapping** are both locked against
`origin/main`:

- **`translateToServo`** in `_frame_to_servo` is byte-identical to the
  firmware `tickGait` switch (verified by
  [`test_servo_parity.py`](test_servo_parity.py)) — this is the
  *math* contract.
- **The wire shape** `{T:4, id, servo_id, a}` matches the
  `CMD_CALIBRATE` handler on `origin/main`; the firmware owns the
  channel mapping via `LEG_SERVO_CHANNEL[id][servo_id]`
  (`config.h:64`). `convention.json`'s `channels` is kept only as a
  reference-only cross-check (also asserted by the parity test against
  the firmware table) — see
  [`animation/SERVO_ID_CONVENTION.md`](../SERVO_ID_CONVENTION.md) for
  the broader convention.

Run `uv run python animation/scripts/test_servo_parity.py` to verify
both contracts in CI.

**Activity Heatmap** (Display, collapsed) colours the servo meshes by
per-frame motion to spot busy/jerky joints. A future *gravity-torque*
mode is sketched in
[torque-heatmap.md](torque-heatmap.md).

## fh_rename_actions.py — one-shot legacy-name migration

> **LEGACY · run once · idempotent.** Only needed for an old rig whose
> Actions still use the pre-convention names. A current rig built by
> `urdf_to_blender_rigged.py` does not need this. Not part of the
> day-to-day workflow — `fh_clip_panel.py` is the active tooling.

One-shot CLI that renames the 5 legacy per-object Actions on the
rig file to the clip convention `fh_clip_panel.py` expects. Idempotent
(skips Actions already at the new name) and **saves the file in place** —
make a backup branch first if you want the legacy names recoverable
without `git checkout`.

| Old name | New name |
|---|---|
| `body_ctrlAction` | `base_anim__body_ctrl` |
| `foot_target_flAction` | `base_anim__foot_target_fl` |
| `foot_target_frAction` | `base_anim__foot_target_fr` |
| `foot_target_blAction` | `base_anim__foot_target_bl` |
| `foot_target_brAction` | `base_anim__foot_target_br` |

After renaming, the script verifies each object's currently-assigned
`AnimData.action_slot.identifier` matches `OB<target>` (Blender's
auto-prefix for OBJECT-type slots) and prints a per-object report.
The panel's slot lookup (`action.slots.get(f"OB{target}")`) only works
if these identifiers are correct — a `MISMATCH` line is worth
investigating before relying on the panel.

**Run:**

```bash
# default: animation/fh_rigged_latest.blend
blender --background --python animation/scripts/fh_rename_actions.py

# override target .blend:
blender --background --python animation/scripts/fh_rename_actions.py -- \
    --blend animation/some-other.blend
```

The script exits non-zero (no save) if the target .blend can't be
found, and skips `wm.save_mainfile()` if no Actions were renamed —
so a no-op re-run leaves the file's mtime untouched.

## Scene conventions (both visualizers)

- `unit_settings.scale_length = 0.001` — 1 BU = 1 mm. Robot reads as
  ~200 BU, comfortable for the default viewport.
- STL imports use `bpy.ops.wm.stl_import(global_scale=1.0)` since both the
  STL vertex coordinates AND the scene are in mm — no conversion.
- `clear_scene` manually wipes data-blocks instead of calling
  `bpy.ops.wm.read_factory_settings(use_empty=True)`. The factory-reset
  call leaves `wm.stl_import.poll()` returning False on Blender 5.x until
  the GUI redraws, which kills imports done from `--python`.
- No Empties, no parenting, no armatures. `obj.matrix_world` is set
  directly per object. This sidesteps every `matrix_parent_inverse` /
  depsgraph-timing trap the rigging-aware versions hit. Rigging belongs in
  a successor script, not these.

## What's next

1. **Optional polish** for the visualizer:
   - Replace orange spheres with oriented arrows / line objects so axis
     direction is unambiguous (currently you have to read the orange-tip
     position relative to the red pivot).
   - Add a CLI flag to apply stance angles (shoulder=0°, hip=-40°,
     knee=-60°) as a static snapshot — useful for comparing against
     PyBullet's settled-stand image without running the simulator.
2. **CAD-side cleanup** if you want symmetric leg meshes:
   currently the URDF generator emits `<visual><origin rpy="0 π 0"/>` on
   `fr_link2`/`fr_link3`/`bl_link2`/`bl_link3` to mirror the L mesh onto R
   legs. Exporting native R-pair STLs from Fusion drops that flip and
   makes any future rigging code one branch simpler.
3. **Binary `.fhc` exporter.** Export Active Clip already bakes the
   IK-solved angles to CSV / `.h` / `.js` in
   `animation/exported_gaits/<clip>/` (two-layer `bake_clip` →
   converters). What's still missing is a converter to the on-board
   binary playback format once it lands (see
   [doc/animation-pipeline/](../../doc/animation-pipeline/)) — it slots
   in as another Layer-2 converter alongside `to_csv` / `to_c_header` /
   `to_js`.
4. **Servo channel/translate convention — converged on `main`.** The
   exported `.js` `T:4` message shape now matches the `origin/main`
   firmware (`code/firmware/src/brain/network.cpp` `CMD_CALIBRATE` →
   `applyCalibration(LEG_SERVO_CHANNEL[id][servo_id], a)`):
   `{T:4, id:<leg_id 0-3>, servo_id:<0-2>, a:<0-180>}`. The exporter,
   the spec ([code/API_SPEC.md](../../code/API_SPEC.md) §4), the mobile
   app, and the firmware all agree on this shape. The math
   (`_frame_to_servo` ↔ firmware `tickGait` `translateToServo`) and the
   channel table (`convention.json` `channels` ↔ firmware
   `LEG_SERVO_CHANNEL`) are both locked by
   [`test_servo_parity.py`](test_servo_parity.py).
5. **Torque heatmap.** Gravity-hold torque mode for the Display
   heatmap — scoped (incl. the ground-contact caveat) in
   [torque-heatmap.md](torque-heatmap.md).
