# Blender clip authoring and export

FaceHugger clips are one-shot authored gestures baked in Blender and exported as
C headers for firmware playback. The workflow requires Blender 5.0 or later. The
add-on uses the layered Action API introduced in Blender 4.4, and 3.3 LTS is no
longer supported.

## Prerequisites

- Blender 5.0+ installed (5.1 recommended).
- The rig file at `animation/fh_rigged_latest.blend` (git-lfs tracked). If it is
  absent, regenerate it with `python code/facehugger.py blender --rigged` from
  the repo root.
- `animation/convention.json` committed (neutral joint angles, scale factor,
  servo channel map).

## Opening the rig

Open `animation/fh_rigged_latest.blend` in Blender. The scene contains the
`FaceHuggerRig` armature (hidden by default) and five control objects:
`body_ctrl` and the four foot targets `foot_target_fl/fr/bl/br`. You author
clips by posing these controls, not the armature bones directly.

## Using the FH Clip Panel

Press `N` in the 3D Viewport to open the sidebar, then switch to the
**FaceHugger** tab. The panel provides:

- **Clips**: create, rename, duplicate, and overwrite named clip bundles. Each
  clip is a set of five Actions (one per control object). Switch the active clip
  by clicking its entry in the list.
- **Poses**: a commit-tracked library of control-object snapshots. Apply a pose
  to set the viewport without keyframing, then use **Key into Clip** to record
  the pose as a keyframe at the current frame.
- **Export**: bake and export the active clip (or the ticked selection) to CSV,
  C header (`.h`), and browser JavaScript (`.js`). The bundled
  `clips_all.h` + `clips_manifest.json` is the format consumed by the on-board
  clip player, and it is built from exactly the ticked clips, so the per-row
  checkboxes are how you choose which clips end up on the robot.

A **Preview Robot Motion** toggle switches F-curves between Bezier (authoring)
and Linear (exact robot playback) without losing Bezier handles.

## Headless export

To re-export all clips from the command line without opening the GUI, use:

```bash
BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
  "$BLENDER_BIN" --background --factory-startup \
  animation/fh_rigged_latest.blend \
  --python animation/addons/export_all_clips.py
```

The `--factory-startup` flag prevents conflicts from any user add-ons. Output
lands in `animation/exported_clips/`: per-clip folders and the bundled
`clips_all.h` and `clips_manifest.json` at the top level. Exit code 0 means all
clips exported cleanly.

## Getting the clip onto the robot

The bundled `clips_all.h` is a self-contained drop-in: the firmware compiles
`code/firmware/src/nervous_system/clips_all.h`, which is exactly the file the
exporter writes (no per-clip `#include` and no separate `.cpp` to edit). Getting a
clip onto the robot is therefore just keeping that file in sync and reflashing.

By default the panel does the sync for you: with the **Copy clips_all.h to
firmware** toggle on (in the Export sub-panel), every bundle export copies
`clips_all.h` straight into the firmware tree and reports the destination path. The
**Firmware** folder button opens that directory. If you turn the toggle off, or run
outside a repo checkout, copy `animation/exported_clips/clips_all.h` over
`code/firmware/src/nervous_system/clips_all.h` yourself.

Validate the exports before flashing:

```bash
python3 animation/scripts/check_export_consistency.py
```

This checks that every clip's `.h` and `.js` agree with the servo-frame convention
(exit code 0 means all pass). Then reflash with `pio run -t upload`. See
[Flashing the firmware](flashing.md). Because the simulator runs the same firmware,
the clip is then playable in the sim too.

For the full feature reference (pose library, selection sets, activity heatmap,
authoring-time warnings, and integration tests), see
[the Clip Panel reference](../../reference/animation/clip-panel.md).

![Script warning — update your scripts](../../../assets/img/animation-pipeline/script-warning.png)
