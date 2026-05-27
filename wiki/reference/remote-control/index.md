# Remote control app

The React Native app (`code/remote-control-app/MyApp/`) connects to the ESP32 over WebSocket (port 81) and drives the robot. It is built with Expo, TypeScript, and Zustand for state.

## What it does

The app provides three control screens, each mapped to a robot FSM state. Switching tabs sends a `T:2` FSM transition command via `Pager.tsx`:

- **GaitControl**: walk control with a joystick and TROT/CRAB selection (`STATE_WALK`; commands `T:1`, `T:5`).
- **LegControl**: per-leg, per-servo calibration buttons (`STATE_IDLE`; command `T:4`).
- **Actions**: one-shot maneuvers including the invert-robot flip with a confirmation dialog (`STATE_ACTION`; command `T:6`).

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

For the full protocol specification see [../api.md](../api.md).
