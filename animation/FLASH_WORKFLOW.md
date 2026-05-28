# Flash a clip to the robot

Once you've animated a clip in Blender, getting it onto the physical FaceHugger is mostly automatic: the FH Clip Panel writes a self-contained firmware bundle and copies it straight into the firmware tree for you. There is no per-clip wiring to do — no `#include` to add, no array to edit. Here's the whole flow.

## Step 1 — Author the clip in Blender

Open `fh_rigged_latest.blend`, keyframe your motion as usual, and manage it as a named clip in the **FaceHugger Clip Panel** (N-panel, FaceHugger Clip Panel tab). New clips, renames, and overwrites all live in the **Clips** sub-panel.

## Step 2 — Pick the firmware set and export

The firmware ships exactly the clips you **tick** for export, not every clip in the file. In the **Clips** sub-panel, tick the per-row checkbox on each clip you want on the robot. WIP clips you leave unticked stay off the device.

Then open the **Export** sub-panel:

- Leave **C header (.h)** enabled — this is what builds the firmware bundle.
- Leave **Copy clips_all.h to firmware** enabled (it's on by default; it appears indented under the C-header toggle).
- Click **Export Selected Clips (N)** to bundle the whole ticked set, or **Export Active Clip** to export just the active clip while still regenerating the bundle from the ticked selection.

The export writes, into `animation/exported_clips/`:

- a per-clip `<clip>/` folder with `<clip>.h` (firmware frames) and, if **Browser JS** is enabled, `<clip>.js` (mobile-app preview), and
- the bundled `clips_all.h` + `clips_manifest.json` for the ticked set.

`clips_all.h` is a single self-contained header (header guard, the `FhClipFrame`/`FhClip` typedefs, one `static const FhClipFrame fh_clip_<name>[]` array per clip, `#define FH_CLIP_COUNT N`, and a `static const FhClip FH_CLIPS[FH_CLIP_COUNT]` table). It is exactly the format the firmware expects.

With the copy toggle on, the panel then copies that `clips_all.h` straight over `code/firmware/src/nervous_system/clips_all.h` and reports the destination path in the operator status line. Use the **Firmware** folder button (next to **Exports**) in the Export sub-panel to open that directory and confirm. Watch the system console (Window > Toggle System Console) for any `WARNING` lines; if frames don't match the servo-frame convention, fix the animation and re-export before flashing.

> If you ticked nothing, the bundle is left untouched and the panel warns you — tick at least one clip.
>
> To bundle **every** clip in the file instead of the ticked subset, run the headless exporter: `blender -b fh_rigged_latest.blend --python animation/addons/export_all_clips.py`.

## Step 3 — Validate the export

From the repo root:

```bash
python3 animation/scripts/check_export_consistency.py
```

This checks every exported clip's `.h` against its `.js` for the servo-frame convention. Exit code `0` = all pass; `1` = a clip failed; `2` = file/parse error. Don't flash unless it exits `0`.

## Step 4 — Manual fallback (only if needed)

If the **Copy clips_all.h to firmware** toggle was off, or Blender isn't running from a repo checkout (so the panel couldn't find the firmware dir and warned instead of copying), copy the bundle yourself:

```bash
cp animation/exported_clips/clips_all.h code/firmware/src/nervous_system/clips_all.h
```

That single file is the entire firmware clip set — nothing else needs editing. (There is no `clips_all.cpp`.)

## Step 5 — Flash

```bash
cd code/firmware
pio run -t upload
```

On success you'll see `=== UPLOAD [upesy_wroom] Took X.XX seconds ===`.

## Step 6 — Test on device

The robot is its own Wi-Fi access point. Power it up and connect to:

- **SSID** `FaceHugger_Net`
- **Password** `12345678`
- **WebSocket** `ws://192.168.4.1:81`

In the mobile app's **Actions** tab, the clip list comes from the robot itself (the app sends `T:8` and gets back each clip's id, name, and duration). Your new clips should appear. Tap one to play it — the app sends `T:7` with the clip's id, and the robot plays it. Verify the motion matches your animation.

If a clip doesn't appear, check the serial monitor (`pio device monitor` from `code/firmware/`) for errors at boot, and confirm the clip was ticked when you exported. If motion is wrong, fix the Blender animation, re-export, and reflash.
