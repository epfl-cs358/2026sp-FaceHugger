# 🦵 Nervous System Abstraction

This directory serves as the "Peripheral Nervous System" of the FaceHugger quadruped. It handles the transformation of high-level movement intent into electrical pulses for the 12 servo motors.

## 🏗️ 3-Layer Architecture

To manage the complexity of a 12-DOF (Degree of Freedom) robot, we use a tiered abstraction approach:

### 1. The Movement Layer (`movements.cpp`)
**Role:** The Choreographer / Gait Engine.
* **Primary Function:** `updateGait()`
* **Logic:** Orchestrates the timing of all four legs. It determines the "Swing" (leg in air) and "Stance" (leg on ground) phases to maintain balance and achieve the velocity vector $(v_x, v_y, \omega)$ requested by the user.

### 2. The Kinematics Layer (`movements.cpp`)
**Role:** The Mathematician.
* **Primary Function:** `solveIK(float x, float y, float z)`
* **Logic:** Implements **Inverse Kinematics (IK)** using the Law of Cosines. It translates a target 3D coordinate $(X, Y, Z)$ relative to the hip into three specific joint angles: **Coxa (Hip)**, **Femur**, and **Tibia**.


### 3. The Leg Layer (`leg.cpp`)
**Role:** The Muscle / Hardware Interface.
* **Primary Functions:** `setLegAngles()` & `writePWM()`
* **Logic:**
    * **Calibration:** Applies offsets from `shared/config.h` to correct for mechanical assembly errors.
    * **Inversion:** Compensates for motors mounted in opposite orientations (Left vs. Right side).
    * **Execution:** Maps degrees to PWM pulse widths (usually 150-600) and communicates with the PCA9685 via I2C.

---

## 🔗 Functional Mapping

The following table tracks which file is responsible for each step in the movement pipeline:

| Phase | File | Function | Role |
| :--- | :--- | :--- | :--- |
| **Communication** | `brain/network.cpp` | `handleParsedMessage()` | **Interpreter:** Receives JSON and routes commands. |
| **Path Planning** | `nervous_system/movements.cpp` | `updateGait()` | **Planner:** Decides foot placement trajectories. |
| **IK Solving** | `nervous_system/movements.cpp` | `solveIK()` | **Solver:** Converts 3D points to angles. |
| **Mapping** | `nervous_system/leg.cpp` | `setLegAngles()` | **Translator:** Applies offsets and maps degrees to pulses. |
| **Execution** | `nervous_system/leg.cpp` | `writePWM()` | **Muscle:** Calls `pwm.setPWM()` for the PCA9685. |

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