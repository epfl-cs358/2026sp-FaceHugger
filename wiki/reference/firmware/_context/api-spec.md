<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  code/API_SPEC.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/firmware/api.md
-->

> **Reference material.** Verbatim copy of `code/API_SPEC.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `code/API_SPEC.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [ok]    Transport: WebSocket port 81 (network.cpp:10 WebSocketsServer(81)); T-values match data.h CommandType (MOVE=1,STATE=2,POSE=3,CALIBRATE=4,GAIT_MODE=5,ACTION_SELECTION=6,TELEMETRY=10). T:4 calibrate keys id/servo_id/a + LEG_SERVO_CHANNEL[id][servo_id] confirmed in network.cpp:90-98.
- [drift] T:1 Manual Movement key is wrong. Doc table says key 'd' (int, IDs 0..6). Firmware reads doc["dir"] (network.cpp:105) and app sends {T:1, dir: ...} (api-messages.tsx). The wire key is "dir", not "d".
- [drift] Direction values are strings, not int IDs. app/api-types.tsx DirectionVector = "FW"/"BW"/"FW_R"/"FW_L"/"BW_R"/"BW_L"/"R"/"L"/"STOP". Doc's numbered 0..6 vector list does not match; also app adds R/L (in-place turn) not in doc's enumeration.
- [drift] Gait Mode IDs disagree. Doc: TROT=0, CRAB=1, CRAWL=2. Real GaitType (movements.h): NONE=0, WALK=1, TROT=2, CRAB=3; app GaitMode TROT=2/CRAB=3. There is no CRAWL; T:5 'g' is validated against [GAIT_NONE..GAIT_CRAB] in network.cpp:119.
- [drift] FSM has a 5th state not in doc's T:2 table: STATE_REST=4 (data.h). network.cpp CMD_STATE maps IDLE→rest(), WALK→walk(), ACTION→wallFlip(); guard only allows newState<=STATE_FAILSAFE so REST isn't reachable via T:2.
- [todo] T:3 body-pose (h/p/r), T:10 telemetry shape, and §Safety timeout/voltage gate are not implemented in network.cpp's handler (no T:1-with-h/p/r, no telemetry emit) — left for Phase B to confirm against firmware loop/sensors.
-->
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
