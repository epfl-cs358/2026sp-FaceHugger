# Flashing the firmware

The firmware is a PlatformIO project targeting the `upesy_wroom` ESP32 board.
Build, flash, and open the serial monitor with the three commands below. Run them
from the `code/firmware/` directory.

```bash
cd code/firmware
pio run -e upesy_wroom
pio run -t upload
pio device monitor
```

`pio device monitor` opens a 115200-baud serial console with the PlatformIO
exception decoder. Watch for FSM state transitions and any `[WARN] servo <ch>
clamped` lines during testing.

If `pio run -t upload` can't find the board, hold the BOOT button while it
starts flashing, or confirm you're using a data USB cable (charge-only cables
look the same and silently break the upload).

## Connecting to the robot

Once flashed, the ESP32 starts a Wi-Fi access point. Join it from your laptop or
phone before sending commands.

- Network: `FaceHugger_Net`
- Password: `12345678`
- WebSocket endpoint: `ws://192.168.4.1:81`

Commands are JSON objects sent over the WebSocket. For example, open a browser
console on a non-HTTPS page and run:

```js
const ws = new WebSocket("ws://192.168.4.1:81");
ws.send(JSON.stringify({ T: 2, s: 0 }));   // transition to IDLE
```

See the [API reference](../../reference/api.md) for the full command protocol.

## Quick bench test from a browser console

When you're sitting next to the robot and want to sanity-check a clip without
opening the mobile app, the browser console is the fastest path. Open one on
a plain non-HTTPS page (`about:blank` works) — `ws://` connections are blocked
from `https://` origins, which is the usual reason "nothing happens." Paste:

```js
const ws = new WebSocket("ws://192.168.4.1:81");
ws.onopen = () => ws.send(JSON.stringify({ T: 7, c: 4 }));  // play clip id 4
```

`T: 7` is the "play clip" command and `c` is the clip id. Ids come from
`animation/exported_clips/clips_manifest.json` (or `T:8` over the same socket).
The robot plays the clip once and then eases back to the neutral standing pose
over about half a second — that return-to-stand is the normal terminator for
every clip. Sending another `T:7` while a clip is still playing simply restarts
with the new one; sending a movement command (`T:5` to pick a gait, then
`T:1`) hands control back to the gait engine immediately.

What success looks like on the serial monitor: a single `[clip] play <name>
(<N> frames)` line, the motion completes, then idle. A flood of
`[WARN] servo <ch> clamped` lines means a clip is pushing a joint past its
range — re-tune the animation in Blender before flashing again.

To cross-check the on-robot playback against the browser-side ground truth,
paste the standalone `animation/exported_clips/<name>/<name>.js` into the same
console. It streams the exact same servo trace over the same socket without
the firmware's `T:7` decoder in the loop, so a divergence isolates the bug
to the firmware side.

## End-to-end: a Blender clip onto the running robot

A new authored clip reaches the robot in three stages: author and export in Blender, flash the firmware, then verify on device.

Author the clip and export it as described in [Blender clip authoring and export](blender-clips.md). With the **Copy clips_all.h to firmware** toggle on (the default), the FH Clip Panel writes the bundled `clips_all.h` straight into `code/firmware/src/nervous_system/clips_all.h` — there is no per-clip `#include` to add and no array to edit. Validate the bundle before flashing with `python3 animation/scripts/check_export_consistency.py` from the repo root; exit code `0` means every clip's `.h` agrees with its `.js` under the servo-frame convention.

Flash with `pio run -t upload` as above. Then join `FaceHugger_Net` and open the mobile app's **Actions** tab. The clip list is read from the firmware at runtime via `T:8`, so a freshly-flashed clip should appear without changes to the app. Tap it; the app sends `T:7` with the clip id, and the robot plays the motion. If a clip is missing, confirm it was ticked in the FH Clip Panel before export and watch the serial monitor for boot errors. If the motion is wrong, fix the animation, re-export, and reflash — the simulator runs the same firmware, so the same clip plays the same way in the sim.
