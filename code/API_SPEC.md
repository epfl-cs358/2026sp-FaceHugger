# FaceHugger API Specification v1.1

This document defines the JSON-based communication protocol between the **Web Dashboard** and the **ESP32 Firmware**.

## 📡 Transport Layer
- **Protocol:** WebSocket (Bi-directional)
- **Port:** 81
- **Format:** Minified JSON
---

## Direction vector type
The direction vector of the robot
- **FW (id 0):** Forward vector (0.0, 1.0)
- **BW (id 1):** Backward vector (0.0, -1.0)
- **FR (id 2):** Front right vector (1.0, 1.0)
- **FL (id 3):** Front left vector (-1.0, 1.0)
- **BR (id 4):** Backward right (1.0, -1.0)
- **BL (id 5):** Backward left (-1.0, -1.0)
- **STOP (id 6):** Stop, user finished walking so we send a message to stop and finish the action (0.0, 0.0)

## Gait Mode Type 
The gait mode the robot is currently in
- **TROT (id 0):** Trot gait mode
- **CRAB (id 1):** Crab gait mode
- **CRAWL (id 2):** Crawl gait mode

## Action Mode Type
The action mode type
- **INVERT ROBOT (id 0):** Send a message to invert the robot 

## 📥 Dashboard -> Robot (Commands)

### 1. Manual Movement (`T: 1`)
Real-time body vector and gait control.
| Key | Type  | Description              | Range          |
| :-- | :---- | :----------------------- | :------------- |
| 'd' | int | FW, BW, FR, FL, BL, BR, R, L | 0,1,2... |

**Example:** `{"T": 1, "d": 0}`

---

### 2. Finite State Machine (FSM) Transition (`T: 2`)
Requests a change in high-level behavior.
| Value (`s`) | State       | Description                                    |
| :---------- | :---------- | :--------------------------------------------- |
| **0** | **IDLE** | Static hold / Active balancing.                |
| **1** | **WALK** | Gait engine active.                            |
| **2** | **ACTION** | Triggers Wall-Flip / Specialized maneuvers.    |
| **3** | **FAILSAFE**| Emergency software interrupt.                  |
| **4** | **REST** | Flat / all-servos-90 calibration pose; clears the invert flag. Safe to power off. |
| **5** | **STAND** | Standing / neutral pose (per-leg `NEUTRAL[]`); the gait launch reference. |

Out-of-range `s` is ignored. State 4 (REST) is the pose to assume when physically
calibrating the robot: all servos go to mid-travel and you mount the links to match.

**Optional `dur_ms`** (uint, ms): on the two pose states REST (4) and STAND (5),
specifies an ease window for the transition into the pose. The firmware glides
each joint to the target over `dur_ms` ms using the same timed-move infrastructure
as the clip-return ease (`setJointAnglesTimed`). Default / absent / `0` means
snap (the original instant pose write). Clamped on the firmware to
`[0, POSE_EASE_MS_MAX]` (= 5000 ms) so a bad value can't park the robot in a
multi-minute ease. Ignored on non-pose states (IDLE / WALK / ACTION / FAILSAFE).

**Examples:**
- `{"T": 2, "s": 5}` — snap to standing neutral (existing behaviour).
- `{"T": 2, "s": 5, "dur_ms": 1000}` — ease into standing neutral over 1 s
  (the default the app sends from the REST / NEUTRAL pose buttons).

---

### 3. Body Pose / Static IK (`T: 3`)
Adjusts the orientation of the chassis while the feet stay planted. We are in `STATE_ACTION`.
Orientation of the chassis with feet planted.
| Key | Type  | Description              | Unit    |
| :-- | :---- | :----------------------- | :------ |
| `h` | int   | Chassis Height           | mm      |
| `p` | float | Pitch (Tilt forward/back) | degrees |
| `r` | float | Roll (Tilt side-to-side)  | degrees |

---

### 4. Servo Calibration (`T: 4`)
Direct angle control over a specific PCA9685 channel.
| Key  | Type | Description              | Range     |
| :--- | :--- | :----------------------- | :-------- |
| `id` | int  | leg id                   |   0 - 3   |
| `servo_id`| int | servo id (0: hip, 1: thigh, 2: knee) | 0 - 2|
| `a`  | int  |         Angle            | 0-180 |

**Example:** `{"T": 4, "id": 2, "servo_id": 2,"a": 90}` *(Move knee of leg id 2 (bottom right) to 90 degrees)*

### 5. Gait integration ('T: 5')
Gait mode change
| Key | Type | Description | Range |
| :-- | :---- | :----------------------- | :------------- |
| 'g' | int  | Gait mode ID | 0,1,2...|
**Example:** '{"T": 5, "g": 1}' *(This is needed in order to avoid sending the gait each time with the T: 1 packets as well as have a separation of concern as to what the robot should do when changing gait)*

### 6. Action phase ('T: 6')

| Key | Type | Description | Range |
| :-- | :---- | :----------------------- | :------------- |
| 'a' | int  | Action ID | 0,1,2...|
**Example:** '{"T": 6, "a": 1}'

### 9. Set Invert Flag (`T: 9`)
Set the robot's invert flag without triggering any servo movement or pose change.
The mirror is applied transparently on the next motion tick (gait, clip, or stand)
via `applyServos`. Use this when you want to arm or disarm invert mid-animation
without interrupting clip playback.

| Key        | Type | Description                              |
| :--------- | :--- | :--------------------------------------- |
| `inverted` | bool | `true` = inverted (upside-down), `false` = upright |

Missing or non-bool `inverted` key → silent no-op with Serial warning.

**Example:** `{"T": 9, "inverted": true}` *(arm invert; next motion tick applies the mirror)*

---

### 11. Set Clip Smoothing (`T: 11`)
Set the clip-playback smoothing factor (per-channel EMA alpha) at runtime — no
reflash. Lower = snappy, follows the baked frames exactly; higher = smoother but
laggier. The firmware clamps to a safe range so playback can never stall. Affects
clip playback only (not gaits or calibration).

| Key | Type  | Description | Range |
| :-- | :---- | :---------- | :---- |
| `a` | float | EMA alpha (smoothing amount) | 0.0–0.95 (clamped); boot default 0.75 |

**Example:** `{"T": 11, "a": 0.5}` *(less smoothing — snappier clips)*

---

### 7. Play Clip ('T: 7')
Play a bundled animation clip by id. By default the robot plays the clip once on
its baked timeline, then auto-returns to the neutral standing pose over 500 ms
and enters IDLE. With `loop: true` it replays from the start at each end instead
of returning, until another motion command (gait / `T: 2` / a new clip) preempts
it. Clip ids/names come from `clips_manifest.json` (generated with `clips_all.h`).
| Key    | Type | Description | Range |
| :----- | :--- | :---------- | :---- |
| 'c'    | int  | Clip id (index into FH_CLIPS[]) | 0..N-1 |
| 'loop' | bool | Replay continuously instead of playing once (optional) | default false |

**Example:** `{"T": 7, "c": 0}` *(play clip 0 once; out-of-range ids are ignored)*
**Example:** `{"T": 7, "c": 0, "loop": true}` *(loop clip 0 until preempted)*

---

### 8. List Clips (`T: 8`)
Query firmware for all compiled-in clips. The robot replies immediately over the
same WebSocket connection with the clip registry built from `FH_CLIPS[]` at runtime.
No parameters required.

**Example request:** `{"T": 8}`

**Example response:**
```json
{"clips":[{"id":0,"name":"lie down and stand up","ms":3000},
           {"id":1,"name":"one leg lift","ms":2042},
           {"id":2,"name":"tiny wiggle","ms":3000},
           {"id":3,"name":"wave","ms":1208},
           {"id":4,"name":"wiggle","ms":3000}]}
```

---

## 📤 Robot -> Dashboard (Telemetry)

### 10. System Status (`T: 10`)
| Key | Type  | Description                                      |
| :-- | :---- | :----------------------------------------------- |
| `s` | int   | Current active FSM State (0-3)                   |
| `d` | array | ToF distance readings [FL, FR, RL, RR, Center]   |
| 'a' | array | AMU array containing speed + gyroscope [Speed, Rotation x, Rotation y, Rotation z] |
| 'g' | int   | Current gait mode of the robot                   |
| 'pc'| float | The current percentage of the movement (gait) accomplished |
| `e` | string or null | Error message observed (if any)         | 

**Example:** `{"T": 10, "s": 0, "d": [200, 200, 200, 200, 150], "a": [0.6, 50, 90, 15], "g": 1, "pc": 0.7, "e": "an error message has been observed"}`
**Note:** `System should send status every 500 ms to know that we still have a connection, or use ping pong standard way in websockets`

---

## ⚠️ Safety Logic
1. **Timeout:** If no `T: 1` command is received for > 2 seconds while in `STATE_WALK`, the robot reverts to `STATE_IDLE`.
2. **Action Gate:** `STATE_ACTION` is ignored if battery voltage is under a certain threshold.
