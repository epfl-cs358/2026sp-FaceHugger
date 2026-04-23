# FaceHugger API Specification v1.1

This document defines the JSON-based communication protocol between the **Web Dashboard** and the **ESP32 Firmware**.

## 📡 Transport Layer
- **Protocol:** WebSocket (Bi-directional)
- **Port:** 81
- **Format:** Minified JSON
---

## 📥 Dashboard -> Robot (Commands)

### 1. Manual Movement (`T: 1`)
Real-time body vector and gait control.
| Key | Type  | Description              | Range          |
| :-- | :---- | :----------------------- | :------------- |
| `x` | float | Lateral (Strafing)       | -1.0 to 1.0    |
| `y` | float | Forward / Backward       | -1.0 to 1.0    |
| `z` | float | Vertical Offset (Height) | -1.0 to 1.0    |
| `r` | float | Yaw (Rotation)           | -1.0 to 1.0    |
| `g` | int   | Gait Mode ID             | 0, 1, 2...     |

**Example:** `{"T": 1, "x": 0.0, "y": 0.5, "z": -0.2, "r": 0.1, "g": 1}`

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

---

## 📤 Robot -> Dashboard (Telemetry)

### 10. System Status (`T: 10`)
| Key | Type  | Description                                      |
| :-- | :---- | :----------------------------------------------- |
| `s` | int   | Current active FSM State (0-3)                   |
| `b` | float | Battery Voltage (e.g., 7.4)                      |
| `d` | array | ToF distance readings [FL, FR, RL, RR, Center]   |
| `a` | bool  | Stabilization/PID Status (true/false)            |
| `e` | string or null | Error message observed (if any)         | 

**Example:** `{"T": 10, "s": 0, "b": 8.1, "d": [200, 200, 200, 200, 150], "a": true, "e": "an error message has been observed"}`
**Note:** `System should send status every 1-2 seconds to know that we still have a connection, or use ping pong standard way in websockets`

---

## ⚠️ Safety Logic
1. **Timeout:** If no `T: 1` command is received for > 2 seconds while in `STATE_WALK`, the robot reverts to `STATE_IDLE`.
2. **Action Gate:** `STATE_ACTION` is ignored if battery voltage is under a certain threshold.