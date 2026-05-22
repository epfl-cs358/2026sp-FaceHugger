# Blender clip panel (FH Clip Panel)

The **FH Clip Panel** add-on (Blender 5.x) is the animator-facing tool for authoring and exporting FaceHugger Clips - one-shot canned gestures played once via `CMD_PLAY_CLIP`. It adds a "FaceHugger" tab to the 3D viewport sidebar (press `N`) and manages clip bundles, pose snapshots, selection presets, and exports to multiple formats. The add-on lives in `animation/scripts/fh_clip_panel.py` and targets the rigged scene at `animation/fh_rigged_latest.blend`.

For a step-by-step authoring walkthrough see [Blender clips how-to](../../guide/toolchain/blender-clips.md). The Phase-2 on-board clip player design is described in [fhc-format.md](fhc-format.md).

---

## Prerequisites

Blender 5.0 or later is required (not 3.3 LTS) because the add-on uses the layered Action API introduced in Blender 4.4 and 5.x frame-change handlers. The rig file `animation/fh_rigged_latest.blend` (git-lfs tracked, built by `urdf_to_blender_rigged.py`) must be open, with the five control objects present in the scene: `body_ctrl` (cube Empty) and `foot_target_fl/fr/bl/br` (sphere Empties); the armature `FaceHuggerRig` is hidden by default. Two JSON files must be committed: `animation/convention.json` (neutral joint angles, scale 2/3, servo channels) and `animation/poses.json` (pose library; auto-created if missing). For headless exports, set `BLENDER_BIN` to your Blender executable and pass `--factory-startup` to avoid user add-on conflicts.

---

## Features

### Clip management

Clips are organized as named 5-Action bundles - one Action per control object (`body_ctrl` plus the four foot targets). A clip is a one-shot gesture, not a loop: after the final frame the robot linearly interpolates back to NEUTRAL over 500 ms, then goes IDLE. The panel lets you create, overwrite, rename, and duplicate clips. The `.blend` file must be saved for clips to persist, because the panel rescans on every viewport redraw.

### Pose library

`animation/poses.json` stores named position snapshots of the five control objects (`body_ctrl` loc+rot, foot targets loc only). Poses are clip-independent - applying a pose is a pure viewport transform with no keyframes or Action side effects. Two default poses ship: `flat` (URDF rest, splayed) and `standing` (raised, tucked); `neutral` is seeded on first use. The **Key into Clip** button commits the currently applied pose into the active clip's Actions at the chosen frame. The panel shows a yellow warning whenever a pose is applied but not yet keyed.

### Selection sets

One-click selection of rig control groups for efficient keyframing: **All, Body, Legs, Front, Back, FL, FR, BL, BR**. Selection is a pure viewport operation - no transforms, keyframes, or Actions are modified.

### Export formats

The **Export** sub-panel bakes the active clip (or a user-selected subset) and writes:

- **CSV per clip** (`<clip>.csv`) - frame, time_ms, 12 joint angles in math-space degrees.
- **C header per clip** (`<clip>.h`) - same data as a C struct array.
- **Browser JavaScript per clip** (`<clip>.js`) - Phase-1 live playback script; applies full hardware conversion (scale + `translateToServo`) and sends frames over WebSocket.
- **Bundled `clips_all.h` + `clips_manifest.json`** - all clips in one pre-scaled math-space header for the Phase-2 on-board clip player.

Output lands under `animation/exported_clips/`. `animation/convention.json` must exist and contain `neutral_joint_deg`, `scale`, and `channels`.

### Activity heatmap

Colours the 8 servo meshes (shoulder and knee; hip/link2 meshes are absent from the CAD) from green (quiet) through orange to red (jerky/high torque) based on frame-to-frame angle delta. Toggling the heatmap on switches the viewport to Object shading; toggling it off restores the prior shading mode. Use this to spot mechanical shock risks before export.

### Bezier/Linear preview toggle

Switches the active clip's F-curves between **BEZIER** (smooth authoring) and **LINEAR** (exact robot playback, no inter-frame easing). The toggle is non-destructive - Bezier handles are preserved and restored when switching back.

### Authoring-time warnings

Three console warnings guard against export errors and mechanical shock:

1. **Frame-delta warning** (~20 deg threshold): fires if any joint moves more than 20 deg between consecutive baked frames.
2. **Simultaneous-servo warning** (5 deg per frame): hints in the export UI when many servos move at once.
3. **FPS warning**: nudges the user to lower the scene fps to 12 or below (fewer WebSocket packets for the same motion).

### Headless export

`animation/scripts/export_all_clips.py` re-exports all clips in a `.blend` file without the GUI, running the bake -> Layer-2 converter pipeline and producing CSV/`.h`/`.js` per clip plus the bundled `clips_all.h` and `clips_manifest.json`.

```bash
BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender
"$BLENDER_BIN" --background --factory-startup \
  animation/fh_rigged_latest.blend \
  --python animation/scripts/export_all_clips.py
```

---

## Terminology

- **Clip** - one-shot canned gesture (not "action", not "animation"). Plays once via `CMD_PLAY_CLIP`.
- **Gait** - looping locomotion pattern (not a clip). Selected by `T:5`.
- **Math-space angle** - joint angle in the rig's convention (symmetric per URDF). Stored in CSV/`.h` exports.
- **Servo-space angle** - physical 0-180 deg PWM angle written to hardware. Applied only in `.js` via per-leg `translateToServo`.
- **SCALE** - 2/3 (0.6667) shrink from NEUTRAL. Applied at bake time; firmware never re-scales clip data.
- **Leg naming** - Blender/URDF uses `fl/fr/bl/br`; firmware LegId uses `FR/FL/RR/RL`. The add-on uses Blender names exclusively.

---

## Limitations & known issues

1. **Link2 (hip) servo meshes absent** - the heatmap colours only 8 of 12 servos (shoulder + knee). Hip servos are not visualized because the servo was merged into the body CAD.
2. **IK chain stability** - foot targets use `chain_count=2` (shoulder FK, hip+knee IK); `chain_count=3` is unstable because the foot target is parented to `link1`.
3. **URDF limit vs neutral** - the hardware-neutral pose in `convention.json` may exceed `LIMIT_ROTATION` on some joints (e.g., hips); `Set N Pose` clamps with best-effort and reports deviations.
4. **`.blend` mtime** - clips are rescanned on every viewport redraw but do not persist until the `.blend` is saved.
5. **Phase-1 only (`.js`)** - the browser `.js` exporter is a temporary Phase-1 solution. Phase-2 uses the bundled `clips_all.h` and the on-board `playClip(id)` player.
