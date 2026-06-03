# animation/addons — Blender add-ons

| File | Kind | What it's for |
|---|---|---|
| `fh_clip_panel.py` | UI add-on (sidebar N-panel) | Clips (5-Action bundles), a position-based Pose Library, selection sets, export to `exported_gaits/`, activity heatmap — via the layered Action API. |
| `test_servo_parity.py` | CI test (plain Python) | Verifies `_frame_to_servo` math and channel table against the firmware `tickGait` switch. |

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

### Loading

| Method | When to use |
|---|---|
| `Edit > Preferences > Add-ons > Install...` → pick this file → enable "FH Clip Panel" | Persists across sessions. Use for actual animation work. |
| Open in Text Editor → press `Alt+P` (Run Script) | Dev iteration. The script unregisters before re-registering, so you can edit and re-run without restarting Blender. |

### Panel structure

- **(parent) FaceHugger** — always-visible status: active-clip header,
  and the *not-keyed notice*. Applying a pose is a live viewport change
  with nothing committed; the parent shows a yellow `Pose '<name>' applied
  — not keyed` plus a one-click `Key into '<clip>' @ frame N` button that
  inserts the 5 keyframes into the active clip's own Actions at the
  recorded frame.
- **Poses** — one row per library pose: Apply · Rename · Delete.
- **Clips** — one button per clip; the ⟳ icon overwrites that clip with
  the currently-bound animation (confirm dialog). New Clip · Duplicate ·
  Rename · Save Current → New Clip.
- **Selection** — `All · Body · Legs · Front · Back · FL · FR · BL · BR`.
  Pure viewport selection — touches no data.
- **Export** — CSV / `.h` / `.js` toggles; Export Active Clip; Clips to
  export checklist + Export Selected Clips → `animation/exported_gaits/<clip>/`.
- **Display** (collapsed) — Activity Heatmap: colours per-joint servo meshes
  by per-frame angle delta (green→orange→red).

### Operator IDs

```python
bpy.ops.fh.apply_clip(clip_name="stand up")
bpy.ops.fh.new_clip()
bpy.ops.fh.save_as_clip()
bpy.ops.fh.duplicate_clip()
bpy.ops.fh.overwrite_clip(clip_name="walk")
bpy.ops.fh.rename_clip()
bpy.ops.fh.export_clip()
bpy.ops.fh.export_selected()

bpy.ops.fh.pose_save()
bpy.ops.fh.pose_apply(pose_name="flat")
bpy.ops.fh.pose_rename(pose_name="old")
bpy.ops.fh.pose_delete(pose_name="flat")
bpy.ops.fh.key_pose_into_clip()
bpy.ops.fh.set_n_pose()     # console re-seed + apply 'neutral'
bpy.ops.fh.set_rest_pose()  # console re-seed + apply 'flat'
bpy.ops.fh.select_controls(preset="LEGS")
```

### Typical workflow

1. **Block out a stance.** In **Poses**, Apply `flat` / `standing` (or your own).
2. **Pick a clip to work in.** In **Clips**, Apply the clip (or `New Clip`).
3. **Commit the pose.** Click *Key into '<clip>' @ frame N* in the notice.
4. **Edit the clip.** While a clip is active, every keyframe lands directly in its Actions. Ctrl-S the `.blend`.
5. **Export.** Tick CSV / `.h` / `.js`, then Export Active Clip or Export Selected Clips → `animation/exported_gaits/<clip>/`.

### Running an exported `.js` clip in a browser

1. Tick **Browser JS (.js)**, hit *Export Active Clip* → `animation/exported_gaits/<clip>/<clip>.js`.
2. Join the robot's Wi-Fi (`FaceHugger_Net`, `192.168.4.1`). Open a console on a non-HTTPS page (`ws://` is blocked from `https://`).
3. Stop any active gait before playing:
   ```js
   ws.send(JSON.stringify({"T": 5, "g": 0}));  // CMD_GAIT_MODE → GAIT_NONE
   ws.send(JSON.stringify({"T": 2, "s": 0}));  // CMD_STATE → STATE_IDLE
   ```
4. Paste the `.js`. Call `fhStop()` to stop.

See [`doc/animation-pipeline/onboard-clip-player-design.md`](../../doc/animation-pipeline/onboard-clip-player-design.md) for Phase-1 design context.

### Servo math contract

`_frame_to_servo` in the exporter is verified byte-identical to the firmware
`tickGait translateToServo` switch. Run `uv run python animation/addons/test_servo_parity.py`
to assert both the math and the channel table against `origin/main`.

See [`wiki/reference/firmware/servo-conventions.md`](../../wiki/reference/firmware/servo-conventions.md)
for the broader servo ID / channel convention.

### What's next

- **Binary `.fhc` exporter** — a Layer-2 converter to the on-board binary playback format once it lands (see [doc/animation-pipeline/](../../doc/animation-pipeline/)).
- **Torque heatmap** — gravity-hold torque mode for the Display heatmap; scoped in the [Torque heatmap wiki page](../../wiki/reference/simulation/torque-heatmap.md) under "Planned: gravity-hold torque mode".
