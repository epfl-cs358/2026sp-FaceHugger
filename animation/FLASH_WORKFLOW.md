# Flash a clip to the robot

Once you've finished animating a clip in Blender, follow these five steps to test it on the physical FaceHugger.

## Step 1 — Export from Blender

In Blender with `fh_rigged_latest.blend` open, select the **N-panel** (toggle with `N` or View menu) and navigate to the **FaceHugger Clip Panel** tab. Click the **Export Clips** button. The export writes two files per clip to `animation/exported_clips/`:

- `<clip_name>.h` — C++ header with frame data, for the firmware
- `<clip_name>.js` — JavaScript frame array, for the mobile app preview

Watch the Blender console (Window > Toggle System Console) for any `WARNING` lines from the consistency check. If you see warnings, the exported frames don't match the `_frame_to_servo` formula — do not proceed; review the animation and re-export.

## Step 2 — Verify exports

Run the consistency check from the repo root:

```bash
python3 animation/scripts/check_export_consistency.py
```

This tool validates that every exported clip's `.h` and `.js` files agree with the servo-frame convention. Exit code 0 means all clips pass; exit code 1 means failures, exit code 2 means file errors. Do not flash if any clip fails.

## Step 3 — Wire into firmware

The firmware must know about the new clip to compile and serve it. Open two files:

**`code/firmware/src/nervous_system/clips_all.h`**

Add an `#include` directive for your new clip:

```cpp
#include "clips/my_clip_name.h"
```

**`code/firmware/src/nervous_system/clips_all.cpp`**

Add your clip to the `FH_CLIPS[]` array and increment `FH_CLIP_COUNT`:

```cpp
const FhClip FH_CLIPS[] = {
    // ... existing clips ...
    { FH_MY_CLIP_NAME_FRAMES, FH_MY_CLIP_NAME_FRAME_COUNT, "my_clip_name" },
};
const uint16_t FH_CLIP_COUNT = <new_count>;
```

The constant names are derived from your clip name in uppercase with underscores (e.g., `walk_forward` → `FH_WALK_FORWARD_FRAMES`).

## Step 4 — Flash

From `code/firmware/`, build and flash the ESP32:

```bash
cd code/firmware
pio run -t upload
```

Wait for the upload to complete. On success, you'll see `=== UPLOAD [upesy_wroom] Took X.XX seconds ===`.

## Step 5 — Test on device

Power up the robot and connect the mobile app to its IP address (default: `192.168.1.100` or check the serial monitor). Navigate to the **Actions** tab. Your new clip should appear in the clip list. Tap it to trigger playback on the robot — verify the motion looks correct and matches your animation.

If the clip doesn't appear, check the serial monitor (`pio device monitor` from `code/firmware/`) for errors during clip initialization. If motion is wrong, revisit the Blender animation, export again, and reflash.
