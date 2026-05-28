# Flashing and testing the clip player (with the robot)

A short walkthrough for when you're sitting next to FaceHugger and want to
build the firmware, flash it, connect, and play an animation clip.

## 1. Build and flash

Everything happens from the firmware folder, so start there:

```bash
cd code/firmware
```

First compile, to make sure the code is healthy before you touch the robot:

```bash
pio run -e upesy_wroom
```

You're looking for a `SUCCESS` line at the end. Now plug the ESP32 in over USB
and flash it:

```bash
pio run -t upload
```

If the upload can't find the board, hold the BOOT button while it starts
flashing, or check the USB cable is a data cable (not charge-only). Once it's
on, open the serial monitor so you can watch what the robot is thinking:

```bash
pio device monitor
```

It runs at 115200 baud (already configured). Leave this window open — it's your
window into every command the robot receives.

## 2. Connect to the robot

The robot makes its own Wi-Fi network; you join *it*, not the other way around.
On boot the serial monitor prints the network details. Join:

- **Network:** `FaceHugger_Net`
- **Password:** `12345678`
- **Robot address:** `192.168.4.1` (it's the access point, so this is fixed)

The control channel is a WebSocket on **port 81**.

## 3. Play a clip

The simplest way to send a command is a browser JavaScript console. Open one on
a plain **non-HTTPS** page (an `about:blank` tab, or any `http://` page) —
`ws://` connections are blocked from `https://` pages, which is the usual reason
"nothing happens." Paste:

```js
const ws = new WebSocket("ws://192.168.4.1:81");
ws.onopen = () => ws.send(JSON.stringify({ T: 7, c: 4 }));  // play clip 4 ("wiggle")
```

`T: 7` is the "play clip" command and `c` is the clip id. The ids come from the
generated `animation/exported_clips/clips_manifest.json`; today they are:

| id | clip |
|----|------|
| 0  | lie down and stand up |
| 1  | one leg lift |
| 2  | tiny wiggle |
| 3  | wave |
| 4  | wiggle |

When the command lands you'll see `[clip] play wiggle (73 frames)` in the serial
monitor. The robot plays the clip **once** on its own timeline, then eases back
to the neutral standing pose over about half a second and goes idle. That
return-to-stand at the end is expected — it's how every clip finishes.

To play another clip, just send a new message with a different `c` (you can
reuse the open socket). Sending a clip while one is still playing simply
restarts with the new one.

## 4. What "working" looks like

- The motion runs once and then settles into the standing pose — it does **not**
  loop or freeze mid-pose.
- The serial monitor shows the `[clip] play …` line and, shortly after, the
  robot returning to idle.
- You should **not** see a flood of `[WARN] servo … clamped` lines. A few are
  fine; a constant stream means a clip is pushing a joint past its range and the
  animation should be toned down in Blender.

To confirm the clip player hands control back cleanly, send a movement command
mid-clip (e.g. `{ "T": 5, "g": 1 }` to pick a gait, then `{ "T": 1, "d": 0 }` to
walk): the clip should immediately give way to the gait engine.

## 5. Optional: cross-check against the browser version

Each clip also exports a standalone `.js` you can paste into the same console —
find it at `animation/exported_clips/<clip name>/<clip name>.js`. It streams the
exact same motion over the socket without needing the new firmware command, so
it's a handy way to confirm the on-robot `T:7` playback matches what the browser
sends.
