# FaceHugger API Specification v1.1

This document defines the JSON-based communication protocol between the **Web Dashboard** and the **ESP32 Firmware**.

## 📡 Transport Layer
- **Protocol:** WebSocket (Bi-directional)
- **Port:** 81
- **Format:** Minified JSON

---

## 📥 Dashboard -> Robot (Commands)

The robot expects a JSON object with a type key `T` (CommandType) and associated data keys.

### 1. Manual Movement (`T: 1`)
Real-time translation and rotation, primarily used in `STATE_WALK`.
| Key | Type  | Description        | Range          |
| :-- | :---- | :----------------- | :------------- |
| `x` | float | Lateral (Strafing) | -1.0 to 1.0    |
| `y` | float | Forward / Backward | -1.0 to 1.0    |
| `r` | float | Yaw (Rotation)     | -1.0 to 1.0    |

**Example:** `{"T": 1, "x": 0.0, "y": 0.5, "r": -0.2}`

---

### 2. Finite State Machine (FSM) Transition (`T: 2`)
Requests a change in the robot's high-level behavior mode.
| Value (`s`) | State         | Description                                      |
| :---------- | :------------ | :----------------------------------------------- |
| **0** | **IDLE** | Active balancing enabled; resists external tilt. |
| **1** | **WALK** | Gait engine active with dynamic leveling.        |
| **2** | **ACTION** | Triggers Wall-Flip (ToF proximity check active). |
| **3** | **FAILSAFE** | Emergency software interrupt / Power cut.        |

**Example:** `{"T": 2, "s": 1}` *(Switch to Walking mode)*

---

### 3. Body Pose / Static IK (`T: 3`)
Adjusts the orientation of the chassis while the feet stay planted. We are in `STATE_ACTION`.
| Key | Type  | Description              | Unit    |
| :-- | :---- | :----------------------- | :------ |
| `h` | int   | Chassis Height           | mm      |
| `p` | float | Pitch (Tilt forward/back) | degrees |
| `r` | float | Roll (Tilt side-to-side)  | degrees |

**Example:** `{"T": 3, "h": 55, "p": 10.0, "r": 0.0}`

---

### 4. Servo Calibration (`T: 4`) app -> robot
Direct control over a specific PWM channel for hardware bring-up and alignment.
| Key  | Type | Description              | Range     |
| :--- | :--- | :----------------------- | :-------- |
| `id` | int  | leg id                   |   0 - 3   |
| `servo_id`| int | servo id (0: hip, 1: thigh, 2: knee) | 0 - 2|
| `a`  | int  |         Angle            | 0-180 |

**Example:** `{"T": 4, "id": 2, "servo_id": 2,"a": 90}` *(Move knee of leg id 2 (bottom right) to 90 degrees)*

---

## 📤 Robot -> Dashboard (Telemetry)

The ESP32 broadcasts this packet to update the UI indicators.

### 10. System Status (`T: 10`)
| Key | Type  | Description                                      |
| :-- | :---- | :----------------------------------------------- |
| `s` | int   | Current active FSM State (0-3)                   |
| `b` | float | Battery Voltage (e.g., 7.4)                      |
| `d` | array | ToF distance readings [FL, FR, RL, RR, Center]   |
| `a` | bool  | Stabilization/PID Status (true/false)            |
| `e` | string or null | Error message observed (if any)         | 

**Example:** `{"T": 10, "s": 0, "b": 8.1, "d": [200, 200, 200, 200, 150], "a": true, "e": "an error message has been observed"}`

---

## ⚠️ Safety Logic
1. **Timeout:** If no `T: 1` command is received for > 2 seconds while in `STATE_WALK`, the robot should revert to `STATE_IDLE`.
2. **Action Gate:** `STATE_ACTION` commands will be ignored if the battery voltage (`b`) is below 6.8V.