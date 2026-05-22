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

### 7. Play Clip ('T: 7')
Play a bundled one-shot animation clip by id. The robot plays the clip once on
its baked timeline, then auto-returns to the neutral standing pose over 500 ms
and enters IDLE. Clip ids/names come from `clips_manifest.json` (generated with
`clips_all.h`).
| Key | Type | Description | Range |
| :-- | :--- | :---------- | :---- |
| 'c' | int  | Clip id (index into FH_CLIPS[]) | 0..N-1 |

**Example:** `{"T": 7, "c": 0}` *(play clip 0; out-of-range ids are ignored)*

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
