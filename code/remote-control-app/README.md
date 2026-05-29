# FaceHugger Remote Control App

A React Native (Expo) mobile app for controlling the FaceHugger quadruped robot over Wi-Fi. It communicates with the ESP32 firmware via WebSocket on port 81, following the protocol defined in [`code/API_SPEC.md`](../API_SPEC.md).

---

## How to run

```bash
cd code/remote-control-app/MyApp
npm install
npm run start          # opens Expo dev server
npm run android        # or: npm run ios / npm run web
```

The robot's ESP32 acts as a Wi-Fi access point. Connect your phone to that network, then launch the app. The target IP is hardcoded in `config/config.ts`:

```ts
export const webSocketIP = "192.168.4.1"  // ESP32 AP gateway
```

---

## App layout

The app has a persistent header and three swappable pages selected from a bottom navigation bar.

### Persistent header (always visible)

| Section | What it shows |
|---|---|
| Logo | FaceHugger image |
| Connection status | Green "Connected" / red "Not Connected" pill |
| Telemetry bar | Gait mode, speed (m/min), gyroscope X/Y/Z, error message (if any) |

The telemetry bar is populated from the `T:10` status packets the robot broadcasts every ~500 ms. It is hidden when the socket is disconnected.

---

### Page 1 — Remote Control (game-controller icon)

The main driving interface. Switching to this page sends `STATE_WALK` to the robot's FSM.

**Joystick** — drag to steer. The joystick maps angular sectors to the eight direction commands sent over WebSocket (`T:1`):

| Sector | Direction |
|---|---|
| Up | Forward |
| Up-right / Up-left | Forward-right / Forward-left |
| Right / Left | Strafe right / Strafe left |
| Down-right / Down-left | Backward-right / Backward-left |
| Down | Backward |

Releasing the joystick sends a `STOP` command. The firmware enforces a 2-second timeout: if no movement command arrives while in `STATE_WALK`, the robot reverts to idle on its own.

**Gait mode dropdown** (bottom-right corner) — switches between:

| Mode | Description |
|---|---|
| TROT | Diagonal pairs move together — faster, natural walking gait |
| CRAB | Legs move independently for lateral / omnidirectional travel |

Selecting a gait sends a `T:5` packet immediately and is remembered across reconnects.

---

### Page 2 — Actions (fire icon)

One-shot commands that put the robot into `STATE_ACTION`. Three buttons are available:

| Button | What it does |
|---|---|
| Invert robot | Triggers the wall-flip / inversion maneuver (`T:6, a:0`). A confirmation prompt appears before the command is sent. |
| Rest pose (all servos 90°) | Sends 12 `T:4` calibration packets — one per joint — driving every servo to 90°. Useful as a neutral reset before calibration. |
| Neutral stance | Sends 12 `T:4` packets with the firmware's default standing angles (mirroring `returnToDefaultAngles()` in the firmware). Brings the robot to its nominal standing pose. |

Neutral stance angles (hip / thigh / knee, degrees):

| Leg | Hip | Thigh | Knee |
|---|---|---|---|
| Front right (0) | 90 | 150 | 53 |
| Front left (1) | 75 | 30 | 130 |
| Rear right (2) | 90 | 40 | 130 |
| Rear left (3) | 90 | 150 | 50 |

Keep `NEUTRAL_STANCE_ANGLES` in `api/api-messages.tsx` in sync with the firmware `*_DEFAULT_ANGLE` constants in `config.h` if those ever change.

---

### Page 3 — Individual Control (engine icon)

Direct per-servo angle control, primarily for calibration. Switching to this page sends `STATE_IDLE` to the robot's FSM (gait engine off, servos hold position).

**Workflow:**
1. Pick a leg: Front Right, Front Left, Back Right, Back Left.
2. Pick a servo: Hip, Thigh, Knee.
3. Type an angle (0–180°) and tap **Send angle**.

Each send fires a single `T:4` calibration packet. The input field remembers the last-sent angle for each (leg, servo) pair within the session.

---

## State management

Global state is managed with **Zustand** (`store/robotStore.ts`). The store holds:

- `fsmState` — current FSM state reported by the robot
- `chosenFsmState` — the state the app has requested (sent on tab switch)
- `gaitMode` / `chosenGaitMode` — same pattern for gait
- `tofDistances` — ToF sensor readings `[FL, FR, RL, RR, Center]`
- `speed`, `gyroscope` — from the AMU array in `T:10`
- `movementProgress` — gait cycle completion (0.0–1.0)
- `errorMessage` — firmware error string, or `null`

On reconnect, the app immediately re-sends the last chosen FSM state and gait mode so the robot snaps back to the correct configuration without any manual intervention.

---

## WebSocket protocol summary

Full spec: [`code/API_SPEC.md`](../API_SPEC.md). Quick reference:

| T | Direction | Purpose |
|---|---|---|
| 1 | App → Robot | Movement direction command |
| 2 | App → Robot | FSM state transition |
| 3 | App → Robot | Body pose / static IK (pitch, roll, height) |
| 4 | App → Robot | Direct servo angle (calibration) |
| 5 | App → Robot | Gait mode change |
| 6 | App → Robot | Trigger action (e.g. invert) |
| 10 | Robot → App | System status telemetry |

---

## Project structure

```
MyApp/
├── App.tsx                  # root: sets up Pager + WebSocket connection
├── config/config.ts         # robot IP, debug flag
├── api/
│   ├── api-types.tsx        # enums + TypeScript types for all packets
│   └── api-messages.tsx     # pre-built packet objects + helper constructors
├── services/socket.ts       # raw WebSocket wrapper (connect / send / onMessage)
├── hooks/
│   ├── useRobotConnection.ts  # subscribes to T:10, re-syncs state on reconnect
│   └── useSocketStatus.ts     # boolean reactive connection state
├── store/robotStore.ts      # Zustand global state
├── screens/
│   ├── GaitControl.tsx      # Remote Control page
│   ├── Actions.tsx          # Actions page
│   └── LegControl.tsx       # Individual Control page
└── components/
    ├── pager/               # bottom-nav tab container + persistent header
    ├── joystick/            # gesture-based joystick
    ├── connectionStatusComponent/
    ├── robotStatus/         # telemetry display bar
    ├── dropdown/            # gait mode selector
    ├── IndividualSelectionButton/
    └── textInput/ / text/   # shared form primitives
```
