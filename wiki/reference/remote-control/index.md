# Remote control app

The React Native app (`code/remote-control-app/MyApp/`) connects to the ESP32 over WebSocket (port 81) and drives the robot. It is built with Expo, TypeScript, and Zustand for state.

## What it does

The app provides three control screens, each mapped to a robot FSM state. Switching tabs sends a `T:2` FSM transition command via `Pager.tsx`:

- **GaitControl**: walk control with a joystick and TROT/CRAB selection (`STATE_WALK`; commands `T:1`, `T:5`).
- **LegControl**: per-leg, per-servo calibration buttons (`STATE_IDLE`; command `T:4`).
- **Actions**: one-shot clips, plus the invert toggle that mirrors the robot's current pose (with a confirmation dialog). Invert is command `T:6`; it is a latching toggle, not an FSM state.

## Tech stack

| Layer | Technology |
|---|---|
| Framework | Expo / React Native |
| Language | TypeScript |
| State | Zustand (`store/robotStore.ts`) |
| Transport | WebSocket (`services/socket.ts`) |
| API types | `api/api-types.tsx`, `api-messages.tsx` |

## Connection model

`useRobotConnection.ts` opens the WebSocket on mount and wires the `onmessage` handler to parse incoming telemetry (`T:10`) and update the Zustand store. `socket.ts` queues outgoing messages while the socket is in the CONNECTING state and flushes them on `onopen`. `useSocketStatus.ts` exposes the current connection state (connected, disconnected, error) to UI components.

The Zustand store tracks live robot state (`fsmState`, `gaitMode`, `tofDistances`, `speed`, `gyroscope`, `movementProgress`, `errorMessage`) alongside user-intent fields (`chosenFsmState`, `chosenGaitMode`) so the UI can reflect pending transitions before the next telemetry cycle confirms them.

Preset packet constants (`TrotGaitPacket`, `CrabGaitPacket`, `InvertRobotPacket`, etc.) are exported from `api-messages.tsx`; use these rather than constructing raw JSON.

## Stop-on-blur

Each control screen tears down whatever motion it owns when the user navigates away. Without this, a looping firmware clip launched from Actions would keep running invisibly after the user swiped to GaitControl, leaving the robot dancing with no on-screen control. The same hazard applies to a JS-streamed clip whose timer survives the unmount.

The Pager swaps screens by unmount, so the screen's `useEffect` cleanup is the natural place to hook this. `Actions.tsx` registers `useEffect(() => () => stopMotion(), [])`, which on blur calls `stopMotion()` from `api/api-messages.tsx`. That helper sends `T:2 {"s": 0}` (STATE_IDLE) to preempt any firmware clip (`T:7`) or gait (`T:1`/`T:5`), then calls `stopStream()` on the JS clip streamer in case it was running. Both halves are no-ops when nothing is in flight, so the cleanup is safe to fire unconditionally. The robot holds its last commanded pose after IDLE; tap Neutral stance to reset.

## App-streamed clips (disabled)

Clips are now triggered firmware-side via `T:7` (CMD_PLAY_CLIP): the ESP32 owns the timing and only one packet crosses the wire. The app used to alternatively *stream* bundled clips frame-by-frame as `T:4` calibration packets, but that path saturated the ESP32 WebSocket on real hardware and the connection would drop mid-clip. Flashed clips were unaffected, so the streamer was switched off rather than removed.

The flag lives in `config/config.ts` as `JS_STREAMED_CLIPS_ENABLED = false`. The streamer source (`services/clipStreamer.ts`, `api/streamClips.ts`) is intentionally left intact; `ClipList.tsx` hides the "App" segment and the per-row "App" button when the flag is off, and the streamer cleanup is a no-op when never started. Flipping the constant back to `true` re-enables the path once firmware throughput is fixed.

For the full protocol specification see [../api.md](../api.md). For a no-build laptop debug client that speaks the same protocol, see the [browser control panel](control-panel.md).
