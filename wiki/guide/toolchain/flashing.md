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

## End-to-end: a Blender clip onto the running robot

A new authored clip reaches the robot in three stages: author and export in Blender, flash the firmware, then verify on device.

Author the clip and export it as described in [Blender clip authoring and export](blender-clips.md). With the **Copy clips_all.h to firmware** toggle on (the default), the FH Clip Panel writes the bundled `clips_all.h` straight into `code/firmware/src/nervous_system/clips_all.h` — there is no per-clip `#include` to add and no array to edit. Validate the bundle before flashing with `python3 animation/scripts/check_export_consistency.py` from the repo root; exit code `0` means every clip's `.h` agrees with its `.js` under the servo-frame convention.

Flash with `pio run -t upload` as above. Then join `FaceHugger_Net` and open the mobile app's **Actions** tab. The clip list is read from the firmware at runtime via `T:8`, so a freshly-flashed clip should appear without changes to the app. Tap it; the app sends `T:7` with the clip id, and the robot plays the motion. If a clip is missing, confirm it was ticked in the FH Clip Panel before export and watch the serial monitor for boot errors. If the motion is wrong, fix the animation, re-export, and reflash — the simulator runs the same firmware, so the same clip plays the same way in the sim.
