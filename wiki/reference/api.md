# WebSocket API

FaceHugger communicates with control clients over a bidirectional WebSocket connection using minified JSON. The robot hosts its own Wi-Fi network (SSID: `FaceHugger_Net`, password: `12345678`).

- **Protocol:** WebSocket (RFC 6455)
- **Port:** 81
- **Endpoint:** `ws://192.168.4.1:81` (robot is the AP)
- **Framing:** Minified JSON objects, one per message
- **Telemetry interval:** Robot sends a `T:10` status message every 100 ms (10 Hz heartbeat / keep-alive)

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

Once the connection is open, the robot begins sending `T:10` telemetry every 100 ms (10 Hz). Clients send commands (`T:1` through `T:11`) as needed; the robot executes them and reflects the updated state in the next telemetry cycle.

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

Payload: `{"T": 2, "s": <state_id>}` or `{"T": 2, "s": <state_id>, "dur_ms": <ms>}`

| ID | Name     | Effect                                           |
|----|----------|--------------------------------------------------|
| 0  | IDLE     | Static hold / active balancing (default)         |
| 1  | WALK     | Gait engine active; responds to `T:1` direction  |
| 2  | ACTION   | Runs a clip (`T:7`) or the wall-flip maneuver    |
| 3  | FAILSAFE | Emergency software interrupt; halts all motion   |
| 4  | REST     | All servos to 90 degrees (mechanical flat / calibration pose); also clears the invert flag. Safe to power off. |
| 5  | STAND    | Standing / neutral pose (per-leg `NEUTRAL[]`); the gait launch reference. |

`IDLE` is the default safe state. `WALK` activates the gait engine; direction vectors via `T:1` then control movement. Playing a clip (`T:7`) puts the robot in `ACTION` automatically. The next `T:10` includes an updated `s` field.

**Optional `dur_ms` field:** when present and greater than 0 on REST (4) or STAND (5), the firmware eases each joint to the target pose over that many milliseconds instead of snapping. Clamped to [0, 5000 ms] by the firmware. Ignored on all other states (IDLE, WALK, ACTION, FAILSAFE).

Examples:
- `{"T": 2, "s": 4}` - snap to REST (calibration pose)
- `{"T": 2, "s": 5, "dur_ms": 1000}` - ease to standing neutral over 1 s

### T:3 CMD_POSE (Body Pose / Static IK)

> **Not implemented in current firmware.** `network.cpp` has no T:3 handler; sending this command is a silent no-op. Reserved for a future static-IK body-tilt feature.

Intended payload: `{"T": 3, "h": <height_mm>, "p": <pitch_deg>, "r": <roll_deg>}`

| Field | Type  | Description                | Unit    |
|-------|-------|----------------------------|---------|
| `h`   | int   | Chassis height             | mm      |
| `p`   | float | Pitch (tilt forward/back)  | degrees |
| `r`   | float | Roll (tilt side-to-side)   | degrees |

### T:4 CMD_CALIBRATE (Servo Calibration)

Direct angle control over a specific PCA9685 channel — a raw servo write that
bypasses `translateToServo` and CALIB. Reserved for hardware calibration
sessions (the `LegControl` per-joint sweep); clip streaming uses T:12
(`CMD_STREAM_FRAME`) so the firmware applies translateToServo + CALIB at
playback and the bundle stays CALIB-independent.

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

### T:6 CMD_SET_AUTO_INVERT

Payload: `{"T": 6, "enabled": <bool>}`

Sets whether the firmware's IMU-driven auto-flip gate is allowed to push the upside-down latch into `setInverted()`. When enabled (the default), `main.cpp` calls `setInverted` each time the IMU latch changes. When disabled, the `isInverted` flag is frozen at its current value - useful when bench-calibrating the robot upside-down without triggering mirror mode.

| Key       | Type | Description |
|-----------|------|-------------|
| `enabled` | bool | `true` = allow IMU to drive `isInverted`; `false` = freeze at current value |

Missing or non-bool `enabled` is a silent no-op (warns on serial). Firmware default: `true`. Current value is mirrored in `T:10` as `auto_invert_enabled`.

Note: the old "toggle invert / flip pose in place" behaviour is now `T:9`.

Example: `{"T": 6, "enabled": false}` - freeze invert; the IMU keeps reporting `upside_down` in telemetry but the FSM will not react.

### T:7 CMD_PLAY_CLIP (Play Animation Clip)

Payload: `{"T": 7, "c": <clip_id>}` or `{"T": 7, "c": <clip_id>, "loop": <bool>}`

| Field  | Type | Description                       | Range |
|--------|------|-----------------------------------|-------|
| `c`    | int  | Clip ID (index into FH_CLIPS[])   | 0-9   |
| `loop` | bool | Replay continuously instead of playing once (optional) | default false |

**Available clips:**

| ID | Name                  | Duration  |
|----|-----------------------|-----------|
| 0  | Twerk                 | 2500 ms   |
| 1  | body wave             | 2458 ms   |
| 2  | dancing               | 4958 ms   |
| 3  | dancing up and down   | 4000 ms   |
| 4  | dog walk              | 3000 ms   |
| 5  | lie down and stand up | 4000 ms   |
| 6  | one leg lift          | 2042 ms   |
| 7  | roll-ish              | 1750 ms   |
| 8  | wave                  | 3833 ms   |
| 9  | wiggle                | 3000 ms   |

Clip IDs and names are generated by the animation exporter; the canonical list is in `animation/exported_clips/clips_manifest.json`. Use `T:8` to query the current list from the firmware at runtime.

This command plays a one-shot authored animation clip and runs only in `STATE_ACTION`. After the clip finishes (when not looping), the robot smoothly returns to the neutral standing pose over ~500 ms (via `setServoAngleTimed`), then transitions to `STATE_IDLE`. When `loop` is `true`, the clip replays from the start instead of returning to neutral, until another motion command preempts it. Sending a new `T:7` while a clip is playing interrupts the current clip and starts the new one. Out-of-range clip IDs are silently ignored.

### T:8 CMD_LIST_CLIPS

Payload (request): `{"T": 8}` - no parameters.

Queries the firmware for all compiled-in clips. The robot replies immediately over the same WebSocket connection with the clip registry built from `FH_CLIPS[]`.

Example response:
```json
{"clips":[
  {"id":0,"name":"Twerk","ms":2500},
  {"id":1,"name":"body wave","ms":2458},
  {"id":2,"name":"dancing","ms":4958},
  {"id":3,"name":"dancing up and down","ms":4000},
  {"id":4,"name":"dog walk","ms":3000},
  {"id":5,"name":"lie down and stand up","ms":4000},
  {"id":6,"name":"one leg lift","ms":2042},
  {"id":7,"name":"roll-ish","ms":1750},
  {"id":8,"name":"wave","ms":3833},
  {"id":9,"name":"wiggle","ms":3000}
]}
```

### T:9 CMD_SET_INVERT

Payload: `{"T": 9, "inverted": <bool>}`

Sets the robot's `isInverted` flag explicitly (not a toggle). The mirror is applied on the next motion tick via `applyServos`. Use this to arm or disarm invert mid-animation without interrupting clip playback.

| Key        | Type | Description |
|------------|------|-------------|
| `inverted` | bool | `true` = inverted (upside-down), `false` = upright |

Missing or non-bool `inverted` is a silent no-op with serial warning. Also triggers `flipPoseInPlace` when not in active motion.

Example: `{"T": 9, "inverted": true}` - arm invert; next motion tick applies the mirror.

### T:11 CMD_SET_CLIP_SMOOTHING

Payload: `{"T": 11, "a": <float>}`

Sets the clip-playback per-channel EMA smoothing alpha at runtime - no reflash needed.

| Key | Type  | Description | Range |
|-----|-------|-------------|-------|
| `a` | float | EMA alpha - lower = snappier, higher = smoother but laggier | 0.0-0.95 (firmware-clamped); boot default 0.75 |

Affects clip playback only (not gaits or calibration).

Example: `{"T": 11, "a": 0.5}` - less smoothing, snappier clips.

## Telemetry (Robot to Client)

### T:10 System Status

The robot emits a `T:10` packet every 100 ms (10 Hz) as a heartbeat and connection keep-alive.

Payload: `{"T": 10, "s": <state>, "d": [<tof>...], "a": [<speed>, <gyro_x>, <gyro_y>, <gyro_z>], "g": <gait>, "pc": <progress>, "e": <error_or_null>, "pitch_deg": <float>, "roll_deg": <float>, "upside_down": <bool>, "auto_invert_enabled": <bool>}`

| Field                | Type         | Description                                              | Unit / Range                          |
|----------------------|--------------|----------------------------------------------------------|---------------------------------------|
| `s`                  | int          | Current FSM state (0-5)                                  | 0=IDLE, 1=WALK, 2=ACTION, 3=FAILSAFE, 4=REST, 5=STAND |
| `d`                  | array(int)   | ToF distance readings [FL, FR, RL, RR, Center]           | mm, 0-2000                            |
| `a`                  | array(float) | Motion: [speed, gyro_x, gyro_y, gyro_z]                  | speed: m/s, gyro: deg/s               |
| `g`                  | int          | Current gait mode (0-3)                                  | 0=NONE, 1=WALK, 2=TROT, 3=CRAB       |
| `pc`                 | float        | Motion progress (gait cycle or clip frame)               | 0.0-1.0                               |
| `e`                  | string\|null | Error message, if any                                    | e.g., `"battery < 3.0V"`             |
| `pitch_deg`          | float        | Accel-only pitch, nose-up positive                       | degrees                               |
| `roll_deg`           | float        | Accel-only roll, right-side-down positive                | degrees                               |
| `upside_down`        | bool         | Latched upside-down flag (flip >150 deg, clear <30 deg)  | -                                     |
| `auto_invert_enabled`| bool         | Current value of T:6 toggle                              | -                                     |

The four IMU fields are always present in `T:10` even when the MPU6050 is not wired - they report last-known values (zero on boot) rather than vanishing, so the schema stays stable.

Example:
```json
{
  "T": 10,
  "s": 0,
  "d": [200, 200, 200, 200, 150],
  "a": [0.6, 50, 90, 15],
  "g": 1,
  "pc": 0.7,
  "e": null,
  "pitch_deg": 1.2,
  "roll_deg": -0.5,
  "upside_down": false,
  "auto_invert_enabled": true
}
```

## Safety and Timeouts

**Movement timeout:** If no `T:1` command arrives for more than 2 seconds while in `STATE_WALK`, the robot automatically reverts to `STATE_IDLE`. Sending a `"STOP"` direction is also safe but not required.

**Angle clamp:** Every servo write, whether from IK, pose, calibration, or a clip, is clamped to 0-180 degrees. If a value exceeds this range, the servo is clamped and a warning `[WARN] servo <ch> clamped: <raw> -> <clamped>` is logged to the serial monitor.

**Frame-delta warning (authoring):** The clip exporter warns during baking if any joint moves more than ~20 degrees between consecutive frames, indicating a torque or shock risk.

## Terminology

**Clip vs. gait:** A clip is a one-shot authored gesture (for example, "wiggle" or "wave"), selected by `T:7`, that runs once and then returns to neutral standing over ~500 ms. A gait is a looping locomotion pattern (trot, crab, or walk), selected by `T:5`, driven by `T:1` direction vectors in `STATE_WALK`. Do not conflate them.

**Clip end behavior:** When a clip finishes (non-looping), the robot returns to neutral standing over ~500 ms using `setServoAngleTimed`, then transitions to `STATE_IDLE`. When `loop: true` is set, the clip replays from the start instead. Clips do not hold at the end.

**T:6 vs. T:9:** `T:6` (`CMD_SET_AUTO_INVERT`) controls whether the IMU is allowed to drive the `isInverted` flag automatically; it does not itself flip the robot. `T:9` (`CMD_SET_INVERT`) sets the `isInverted` flag directly to a given value. Neither changes the FSM state. For the old "flip pose in place" toggle behaviour, use `T:9` with the desired explicit value.

**T:7 vs. T:8:** `T:7` (`CMD_PLAY_CLIP`) starts playback of a clip by ID. `T:8` (`CMD_LIST_CLIPS`) is a query-only command that returns the full clip registry from the firmware at runtime - it does not start any motion.

**Angle spaces:** Math-space angles are abstract joint angles centered on the calibration pose (all URDF joints at 0, all pitch servos at 90 degrees). `NEUTRAL[]` is not the origin of math-space; it is one particular standing pose expressed in math-space, where each shoulder points to its outward rest direction. Servo-space angles are physical 0-180 degree values written to the PCA9685, with per-leg sign conventions applied by `translateToServo`. The angle clamp `constrain(0, 180)` is an electrical backstop applied on every path: IK, pose, calibrate, and clip.
