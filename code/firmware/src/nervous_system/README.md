# 🦵 Nervous System Abstraction (80/20 Movement Logic) v1.0

This directory serves as the "Peripheral Nervous System" of the FaceHugger quadruped. It handles the translation of high-level movement intent into electrical pulses for the 12 servo motors using a **Pose-Based** approach.

## 🏗️ 3-Layer Architecture

We utilize an "80% Hardcode / 20% Calibration" philosophy to ensure stability and rapid hardware integration:

### 1. The Movement Layer (`movements.cpp`)
**Role:** The Choreographer / Gait Engine.
* **Primary Function:** `updateGait()`
* **Logic:** Orchestrates the timing of the legs by blending between hardcoded **Poses** (Stand, Sit, Lift, Step). 
* **Interpolation:** Uses time-based smoothing to transition between poses, ensuring the robot doesn't move with "jerky" stop-and-start motions.

### 2. The Pose Layer (`movements.cpp`)
**Role:** The Keyframer (Replaces complex IK).
* **Primary Functions:** `getPoseAngles()`
* **Logic:** Instead of real-time trigonometry, this layer stores optimized angle tables for specific gait phases. 
* **80% Hardcode:** High-level movement is defined by pre-set joint angles (e.g., "Step_Forward_Phase_1").

### 3. The Leg Layer (`leg.cpp`)
**Role:** The Muscle / Hardware Interface.
* **Primary Functions:** `setLegAngles()` & `writePWM()`
* **Logic:**
    * **20% Calibration:** Applies offsets from `shared/config.h` to the hardcoded angles to correct for mechanical misalignment and 3D-printing tolerances.
    * **Inversion:** Compensates for motors mounted in opposite orientations (Left vs. Right side).
    * **Execution:** Maps calibrated degrees to PWM pulse widths (150-600) for the PCA9685 via I2C.

---

## 🔗 Functional Mapping

| Phase | File | Function | Role |
| :--- | :--- | :--- | :--- |
| **Communication** | `brain/network.cpp` | `handleParsedMessage()` | **Interpreter:** Receives JSON and routes commands. |
| **Gait Engine** | `nervous_system/movements.cpp` | `updateGait()` | **Planner:** Blends between hardcoded poses. |
| **Pose Lookup** | `nervous_system/movements.cpp` | `getPoseAngles()` | **Keyframer:** Provides the "80% Hardcoded" angles. |
| **Calibration** | `nervous_system/leg.cpp` | `setLegAngles()` | **Translator:** Applies the "20% Calibration" offsets. |
| **Execution** | `nervous_system/leg.cpp` | `writePWM()` | **Muscle:** Calls `pwm.setPWM()` for the PCA9685. |

---

## 📐 Calibration Philosophy

To achieve precise movement without continuous Inverse Kinematics:
1. **Hardcode:** Define the ideal angle in code (e.g., Femur at 90° for standing).
2. **Measure:** Observe the physical leg. If it's at 88°, the mechanical error is -2°.
3. **Calibrate:** Set an offset of `+2` in `shared/config.h`. 
4. **Result:** `Final_Angle = Hardcoded_Angle (90) + Offset (2) = 92°` (Physical result: 90°).

---

## 📐 Coordinate System

Each leg operates in its own local Cartesian space to simplify the Inverse Kinematics math:
* **X-Axis:** Lateral movement (side-to-side).
* **Y-Axis:** Longitudinal movement (forward/backward).
* **Z-Axis:** Vertical movement (up/down).



---

## 🛠️ Milestone: Hardware Integration (v1)

**Current Branch Goal:** Complete the **Leg Layer** and **Execution** phase.
* **Success Criteria:** A `CMD_CALIBRATE` JSON packet successfully targets a specific PCA9685 pin and moves a joint to a precise pulse width.
* **Focus:** Stabilizing the I2C communication between the ESP32 and the PCA9685 driver.

---

### 💡 Implementation Notes
* **Safety:** Software limits should be hardcoded in `leg.cpp` to prevent the servos from exceeding the physical mechanical limits of the 3D-printed parts.
* **Latency:** All functions in this directory must remain non-blocking to ensure the `updateNetwork()` loop in `main.cpp` stays responsive.