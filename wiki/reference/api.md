# WebSocket API

FaceHugger communicates with control clients over a bidirectional WebSocket connection using minified JSON. The robot hosts its own Wi-Fi network (SSID: `FaceHugger_Net`, password: `12345678`).

- **Protocol:** WebSocket (RFC 6455)
- **Port:** 81
- **Endpoint:** `ws://192.168.4.1:81` (robot is the AP)
- **Framing:** Minified JSON objects, one per message
- **Telemetry interval:** Robot sends a `T:10` status message every ~500 ms (heartbeat / keep-alive)

## Connection

Connect from an HTTP page or browser console. WebSocket connections from HTTPS pages are blocked by browser security policy; use an HTTP page or `about:blank` in the console to test the `ws://` endpoint.

```js
const ws = new WebSocket("ws://192.168.4.1:81");

ws.onopen = () => {
  console.log("Connected to FaceHugger");
};

ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log("Telemetry received:", telemetry);
};

ws.onerror = (error) => {
  console.error("WebSocket error:", error);
};

ws.onclose = () => {
  console.log("Connection closed");
};
```

Once the connection is open, the robot begins sending `T:10` telemetry every ~500 ms. Clients send commands (`T:1` through `T:7`) as needed; the robot executes them and reflects the updated state in the next telemetry cycle.

## Commands (Client to Robot)

All commands carry a `T` field identifying the message type. The robot silently ignores unknown types.

### T:1 CMD_MOVE (Manual Movement)

Payload: `{"T": 1, "dir": "<direction_string>"}`

The field is `dir` (string), not `d` (integer). The `api-types.tsx` `DirectionVector` enum defines the valid values. `API_SPEC.md` incorrectly shows `d: int`, so trust the code.

| String   | Vector   | Description           |
|----------|----------|-----------------------|
| `"FW"`   | (0, 1)   | Forward               |
| `"BW"`   | (0, -1)  | Backward              |
| `"FW_R"` | (1, 1)   | Front-right diagonal  |
| `"FW_L"` | (-1, 1)  | Front-left diagonal   |
| `"BW_R"` | (1, -1)  | Back-right diagonal   |
| `"BW_L"` | (-1, -1) | Back-left diagonal    |
| `"R"`    | (1, 0)   | Rotate right (yaw)    |
| `"L"`    | (-1, 0)  | Rotate left (yaw)     |
| `"STOP"` | (0, 0)   | Stop / finish walk    |

The direction string is consumed by the gait engine while the robot is in `STATE_WALK`, using whichever gait was selected by `T:5`. Directions sent outside `STATE_WALK` are ignored. Sending `"STOP"`, or not sending any `T:1` for more than 500 ms, triggers an automatic revert to `STATE_IDLE`. The next `T:10` includes an updated `pc` field (movement progress, 0.0-1.0).

### T:2 CMD_FSM_STATE (Finite State Machine Transition)

Payload: `{"T": 2, "s": <state_id>}`

| ID | Name     | Effect                                           |
|----|----------|--------------------------------------------------|
| 0  | IDLE     | Static hold / active balancing (default)         |
| 1  | WALK     | Gait engine active; responds to `T:1` direction  |
| 2  | ACTION   | Enables clips (`T:7`) and maneuvers (`T:6`)      |
| 3  | FAILSAFE | Emergency software interrupt; halts all motion   |

`IDLE` is the default safe state. `WALK` activates the gait engine; direction vectors via `T:1` then control movement. `ACTION` is required before clip playback or invert maneuvers. The next `T:10` includes an updated `s` field.

### T:3 CMD_POSE (Body Pose / Static IK)

Payload: `{"T": 3, "h": <height_mm>, "p": <pitch_deg>, "r": <roll_deg>}`

| Field | Type  | Description                | Unit    | Range |
|-------|-------|----------------------------|---------|-------|
| `h`   | int   | Chassis height             | mm      | TBD   |
| `p`   | float | Pitch (tilt forward/back)  | degrees | TBD   |
| `r`   | float | Roll (tilt side-to-side)   | degrees | TBD   |

This command adjusts chassis orientation while all feet stay planted. It runs only in `STATE_ACTION`. The robot holds the new pose until a subsequent command arrives. The next `T:10` includes an error if the requested pose is out of reach.

### T:4 CMD_CALIBRATE (Servo Calibration)

Payload: `{"T": 4, "id": <leg_id>, "servo_id": <servo_id>, "a": <angle_0_180>}`

| Field      | Type | Description          | Range |
|------------|------|----------------------|-------|
| `id`       | int  | Leg ID               | 0-3   |
| `servo_id` | int  | Servo within the leg | 0-2   |
| `a`        | int  | Servo angle          | 0-180 |

**Leg IDs:**

| ID | Leg              |
|----|------------------|
| 0  | Front-right (FR) |
| 1  | Front-left (FL)  |
| 2  | Back-right (BR)  |
| 3  | Back-left (BL)   |

**Servo IDs (per leg):**

| ID | Joint |
|----|-------|
| 0  | Hip   |
| 1  | Thigh |
| 2  | Knee  |

Example: `{"T": 4, "id": 2, "servo_id": 2, "a": 90}` moves the back-right knee to 90 degrees.

This command directly drives a single servo to a specific angle via PCA9685, bypassing IK and the gait engine. It is accepted in any FSM state but is typically used in `STATE_IDLE` or `STATE_ACTION`. The servo holds the commanded angle until a new command arrives. The next `T:10` includes an error if the angle is out of bounds.

### T:5 CMD_GAIT_MODE (Gait Selection)

Payload: `{"T": 5, "g": <gait_id>}`

`API_SPEC.md` lists TROT=0, CRAB=1, CRAWL=2, which is outdated. The firmware `GaitType` enum (`movements.h`) and the frontend `GaitMode` enum (`api-types.tsx`) are authoritative:

| ID | Name      | Description                                 |
|----|-----------|---------------------------------------------|
| 0  | GAIT_NONE | No gait / stationary (no leg motion)        |
| 1  | GAIT_WALK | Classic static walk (slower, smaller steps) |
| 2  | GAIT_TROT | Trot (diagonal pairs: FR+RL, FL+RR)         |
| 3  | GAIT_CRAB | Crab (lateral stepping, side-step + yaw)    |

The frontend app exposes only TROT (g=2) and CRAB (g=3) as user-facing options.

Selecting a gait does not start walking; the robot must be transitioned to `STATE_WALK` with `T:2` to activate the gait engine. Gait changes can be sent mid-walk without stopping. The next `T:10` includes an updated `g` field.

### T:6 CMD_ACTION_SELECTION (Invert Robot / Wall Flip)

Payload: `{"T": 6, "a": <action_id>}`

| ID | Name         | Effect                                         |
|----|--------------|------------------------------------------------|
| 0  | INVERT_ROBOT | Tip the robot over a wall, flip, and right it  |

This command triggers a specialized authored maneuver and runs only in `STATE_ACTION`. After the maneuver completes, the robot returns to `STATE_IDLE`. Note that `T:6` is a high-level action selector (maneuver), not the clip player. Clips are played via `T:7`.

### T:7 CMD_PLAY_CLIP (Play Animation Clip)

Payload: `{"T": 7, "c": <clip_id>}`

| Field | Type | Description                       | Range |
|-------|------|-----------------------------------|-------|
| `c`   | int  | Clip ID (index into FH_CLIPS[])   | 0-4   |

**Available clips:**

| ID | Name                  | Frames | Duration  |
|----|-----------------------|--------|-----------|
| 0  | lie down and stand up | 73     | 3000 ms   |
| 1  | one leg lift          | 50     | 2042 ms   |
| 2  | tiny wiggle           | 73     | 3000 ms   |
| 3  | wave                  | 30     | 1208 ms   |
| 4  | wiggle                | 73     | 3000 ms   |

Clip IDs and names are generated by the animation exporter; the canonical list is in `animation/exported_clips/clips_manifest.json`.

This command plays a one-shot authored animation clip and runs only in `STATE_ACTION`. After the clip finishes, the robot smoothly returns to the neutral standing pose over ~500 ms (via `setServoAngleTimed`), then transitions to `STATE_IDLE`. Sending a new `T:7` while a clip is playing interrupts the current clip and starts the new one. Out-of-range clip IDs are silently ignored. Clip playback does not loop and does not hold at the end.

## Telemetry (Robot to Client)

### T:10 System Status

The robot emits a `T:10` packet every ~500 ms as a heartbeat and connection keep-alive.

Payload: `{"T": 10, "s": <state>, "d": [<tof>...], "a": [<speed>, <gyro_x>, <gyro_y>, <gyro_z>], "g": <gait>, "pc": <progress>, "e": <error_or_null>}`

| Field | Type          | Description                                    | Unit / Range                              |
|-------|---------------|------------------------------------------------|-------------------------------------------|
| `s`   | int           | Current FSM state (0-3)                        | 0=IDLE, 1=WALK, 2=ACTION, 3=FAILSAFE     |
| `d`   | array(int)    | ToF distance readings [FL, FR, RL, RR, Center] | mm, 0-2000                               |
| `a`   | array(float)  | Motion: [speed, gyro_x, gyro_y, gyro_z]        | speed: m/s, gyro: deg/s                  |
| `g`   | int           | Current gait mode (0-3)                        | 0=NONE, 1=WALK, 2=TROT, 3=CRAB           |
| `pc`  | float         | Motion progress (gait cycle or clip frame)      | 0.0-1.0                                  |
| `e`   | string\|null  | Error message, if any                          | e.g., `"battery < 3.0V"`                 |

Example:
```json
{
  "T": 10,
  "s": 0,
  "d": [200, 200, 200, 200, 150],
  "a": [0.6, 50, 90, 15],
  "g": 1,
  "pc": 0.7,
  "e": null
}
```

## Safety and Timeouts

**Movement timeout:** If no `T:1` command arrives for more than 2 seconds while in `STATE_WALK`, the robot automatically reverts to `STATE_IDLE`. Sending a `"STOP"` direction is also safe but not required.

**Angle clamp:** Every servo write, whether from IK, pose, calibration, or a clip, is clamped to 0-180 degrees. If a value exceeds this range, the servo is clamped and a warning `[WARN] servo <ch> clamped: <raw> -> <clamped>` is logged to the serial monitor.

**Frame-delta warning (authoring):** The clip exporter warns during baking if any joint moves more than ~20 degrees between consecutive frames, indicating a torque or shock risk.

## Terminology

**Clip vs. gait:** A clip is a one-shot authored gesture (for example, "wiggle" or "wave"), selected by `T:7`, that runs once and then returns to neutral standing over ~500 ms. A gait is a looping locomotion pattern (trot, crab, or walk), selected by `T:5`, driven by `T:1` direction vectors in `STATE_WALK`. Do not conflate them.

**Clip end behavior:** When a clip finishes, the robot returns to neutral standing over ~500 ms using `setServoAngleTimed`, then transitions to `STATE_IDLE`. Clips do not hold at the end and do not loop.

**T:6 vs. T:7:** `T:6` (`CMD_ACTION_SELECTION`) triggers the invert-robot wall-flip maneuver. `T:7` (`CMD_PLAY_CLIP`) plays a bundled animation clip. Both run in `STATE_ACTION` but serve different purposes.

**Angle spaces:** Math-space angles are abstract joint angles centered on the calibration pose (all URDF joints at 0, all pitch servos at 90 degrees). `NEUTRAL[]` is not the origin of math-space; it is one particular standing pose expressed in math-space, where each shoulder points to its outward rest direction. Servo-space angles are physical 0-180 degree values written to the PCA9685, with per-leg sign conventions applied by `translateToServo`. The angle clamp `constrain(0, 180)` is an electrical backstop applied on every path: IK, pose, calibrate, and clip.
