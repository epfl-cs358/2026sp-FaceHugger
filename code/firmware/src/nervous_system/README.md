# 🦵 Nervous System Abstraction (Absolute Geometry) v2.0

This directory serves as the "Peripheral Nervous System" of the FaceHugger quadruped. It bridges the gap between pure mathematical motion planning and the quirky physical reality of hobby servo motors. 

We have transitioned from the old "80/20 Hardcoded Offset" approach to a **Universal Geometric Coordinate System**, achieving complete separation of concerns between the robot's mathematical intent and its hardware execution.

## 🏗️ Architecture & File Structure

Based on our recent kinematic discoveries, the nervous system is split into distinct functional layers to isolate hardware quirks from pure math:

### 1. The Gait Engine (`spinal_cord.cpp` / `spinal_cord.h`)
**Role:** The Choreographer.
* **Primary Function:** Orchestrates the Walk Cycle (State Machine).
* **Smooth Movement:** Utilizes Linear Interpolation (LERP) to generate micro-steps between keyframes, completely eliminating robotic "jerks" and preserving momentum.
* **Logic:** Handles high-level timing for the Static Creep Gait, deciding which legs lift, reach, plant, or sweep. 

### 2. The Math Brain (`kinematics.cpp` / `kinematics.h` & `movements.h`)
**Role:** The Universal Blueprint.
* **Logic:** All movement is planned in absolute mathematical degrees, entirely ignorant of how the motors are physically mounted.
* **Reference:** `0°` is always perfectly horizontal. Negative angles point down/backward, positive angles point up/forward.
* **Keyframes:** `movements.h` stores the different gait parameters and types, as well as the leg IDs

### 3. The Hardware Translator (`leg.cpp` / `leg.h`)
**Role:** The Symmetry Compensator.
* **Primary Function:** `setPose()` / Translating Math to Physical Angles.
* **Logic (Diagonal Symmetry):** Compensates for the physical mirroring of the robot's motors:
  * **Front-Right (FR) & Back-Left (BL):** Share one mechanical inversion (`90 - Math`).
  * **Front-Left (FL) & Back-Right (BR):** Share the opposite mechanical inversion (`90 + Math`).
* **Safety:** Implements hard `constrain(0, 180)` limits to prevent software glitches from commanding the servos to rip the plastic chassis apart.

### 4. The Muscle (`servo.cpp` / `servo.h`)
**Role:** The Hardware Interface.
* **Logic:** The lowest-level abstraction. Maps the translated 0-180° angles to PWM pulse widths for the PCA9685 via I2C.

---

## 🔗 Functional Mapping

| Phase | File | Role |
| :--- | :--- | :--- |
| **Communication** | `network.cpp` *(external)* | **Interpreter:** Receives JSON commands via WebSocket. |
| **Gait Orchestration** | `spinal_cord.cpp` | **Planner:** Calculates LERP micro-steps and manages the state machine. |
| **Pure Geometry** | `kinematics.cpp` / `movements.h` | **Math Brain:** Determines absolute target angles (e.g., Thigh -60°). |
| **Hardware Mapping** | `leg.cpp` | **Translator:** Translates math to diagonal physical pairs (0-180°). |
| **Execution** | `servo.cpp` | **Muscle:** Calls I2C commands to write specific PWM values to the 12 channels. |

---

## 📐 Calibration Philosophy (The "Superman" Pose)

We abandoned continuous software offset hunting in favor of strict physical calibration. To ensure the hardware perfectly matches the software's geometric assumptions:

1. **Software Zero:** Send exactly `90` to all 12 servo channels.
2. **Physical Zero:** While the motors hold at `90`, attach the 3D-printed leg pieces so they point **perfectly straight out horizontally** (The "Superman" or "T-Pose").
3. **Result:** A command of `90` is now mechanically locked to Mathematical `0°`. All Inverse Kinematics and geometric math work flawlessly out of the box.

---

## 📐 Coordinate System

Each leg operates in its own absolute geometric space:
* **Global Shoulders:** Shoulders are mapped to a global compass: FR (+45°), FL (+135°), BR (-45°), BL (-135°).
* **Horizontal Zero:** The Thigh and Knee treat `0°` as parallel to the floor.
* **Negative Angles:** Point towards the floor (Thigh) or tuck backward (Knee).

---

## 🛠️ Milestone: Hardware Integration (v1)

**Current Branch Goal:** Complete the **Leg Layer** and **Execution** phase.
* **Success Criteria:** A `CMD_CALIBRATE` JSON packet successfully targets a specific PCA9685 pin and moves a joint to a precise pulse width.
* **Focus:** Stabilizing the I2C communication between the ESP32 and the PCA9685 driver.


## 🛠️ Milestone: Autonomous Static Walk (v2)

**Current Status:** Successfully achieved the first continuous, autonomous static walk cycle using smoothly interpolated geometric arrays via WebSockets!

**Next Steps / Current Focus:**
* **Traction:** Add rubber/silicone to the 3D-printed feet to prevent slippage and loss of forward momentum.
* **Center of Mass (CoM) Sway:** Implement dynamic body-shifting in `spinal_cord.cpp` to properly lean the robot's weight into the support polygon *before* lifting a rear leg, preventing backwards tilting and instability.

