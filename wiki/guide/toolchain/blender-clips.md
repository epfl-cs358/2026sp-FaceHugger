# Blender clip authoring and export

FaceHugger clips are one-shot authored gestures baked in Blender and exported as
C headers for firmware playback. The workflow requires Blender 5.0 or later - the
add-on uses the layered Action API introduced in Blender 4.4, and 3.3 LTS is no
longer supported.

## Prerequisites

- Blender 5.0+ installed (5.1 recommended).
- The rig file at `animation/fh_rigged_latest.blend` (git-lfs tracked). If it is
  absent, regenerate it with `python facehugger.py blender --rigged` from
  `code/simulation/`.
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

- **Clips** - create, rename, duplicate, and overwrite named clip bundles. Each
  clip is a set of five Actions (one per control object). Switch the active clip
  by clicking its entry in the list.
- **Poses** - a commit-tracked library of control-object snapshots. Apply a pose
  to set the viewport without keyframing, then use **Key into Clip** to record
  the pose as a keyframe at the current frame.
- **Export** - bake and export the active clip (or a selection of clips) to CSV,
  C header (`.h`), and browser JavaScript (`.js`). The bundled
  `clips_all.h` + `clips_manifest.json` is the format consumed by the on-board
  clip player.

A **Preview Robot Motion** toggle switches F-curves between Bezier (authoring)
and Linear (exact robot playback) without losing Bezier handles.

## Headless export

To re-export all clips from the command line without opening the GUI, use:

```bash
BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
  "$BLENDER_BIN" --background --factory-startup \
  animation/fh_rigged_latest.blend \
  --python animation/scripts/export_all_clips.py
```

The `--factory-startup` flag prevents conflicts from any user add-ons. Output
lands in `animation/exported_clips/`: per-clip folders and the bundled
`clips_all.h` and `clips_manifest.json` at the top level. Exit code 0 means all
clips exported cleanly.

For the full feature reference - pose library, selection sets, activity heatmap,
authoring-time warnings, and integration tests - see
[Reference - Clip Panel](../../reference/animation/clip-panel.md).
