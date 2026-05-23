# Parts & Materials

This page details everything required to build a FaceHugger v7.5 quadruped. It
covers the 3D-printed chassis components, the required electronics and power
distribution hardware, and a complete breakdown of every screw and fastener used
in the assembly.

---

## Bill of materials (summary)

| Category | Item | Qty | Notes |
|---|---|---|---|
| Electronics | ESP32 (upesy_wroom) | 1 | Main microcontroller and Wi-Fi Access Point |
| Electronics | PCA9685 Multiplexer | 1 | 16-channel PWM driver for the servos |
| Electronics | DSS 230mg Servo | 12 | Metal gear, high-torque — requires servo horns |
| Electronics | I²C Sensors & UI | 5 | 1× OLED screen, 1× IMU, 3× ToF sensors |
| Power | 2S LiPo Battery | 1 | Main power source |
| Power | Buck Converter | 1 | Steps LiPo voltage down to 5V for logic |
| Power | BMS Module | 1 | Battery Management System for protection |
| Power | 2200µF 35V Capacitor | 1 | Wired in parallel to multiplexer to absorb inrush current |
| Fasteners | Metric Screws & Nuts | 148 | M2 to M4 — see full breakdown below |
| Fasteners | Ball Bearings | 12 | Fits inside the printed joints |
| Printed | FaceHugger Chassis | 18 | Frame, legs, and mounts — see full breakdown below |
| Misc | Zip Ties & Elastic Bands | 8 | Cable management and foot traction |

---

## Electronics

The electronic ecosystem is split into two domains: high-current muscle (servos)
and low-current brains (logic).

**Microcontroller**

- 1× ESP32 (upesy_wroom) — handles WebSocket communication, Inverse Kinematics,
  and gait state machines

**Actuators**

- 12× DSS 230mg metal-gear servos — 3 per leg (Hip, Thigh, Knee)
- 12× compatible servo horns (one per servo)

**Power distribution**

- 1× 2S LiPo battery
- 1× BMS (Battery Management System) — sits between the LiPo and the rest of
  the power chain for protection
- 1× Step-down Buck Converter — 5V output for logic boards
- 1× 2200µF 35V electrolytic capacitor — wired in parallel to the PCA9685 to
  absorb inrush current and prevent logic brownouts

!!! tip
    Use terminal blocks (Domino blocks) to split the raw LiPo current to the
    servos directly, bypassing the PCA9685 power traces. The PCA9685 traces are
    not rated for the combined stall current of 12 servos.

**Peripherals (I²C bus)**

- 1× 128×64 OLED screen
- 1× IMU (Inertial Measurement Unit)
- 3× Time-of-Flight (ToF) distance sensors

**Cabling**

- Female-to-Female jumper wire spools
- Custom spliced I²C cable trees — 4 trees (VCC 3.3V, GND, SDA, SCL), each
  with 1 female connector on the ESP32 side splitting to 3 female connectors
  on the peripheral side

---

## Hardware & fasteners

!!! warning
    Always insert hex nuts into their printed retaining slots **before** driving
    any screws. Once a component is installed the slots are inaccessible.

### Master fastener count

| Size | Variant | Qty |
|---|---|---|
| M2 | M2×4 screws | 6 |
| M2 | M2×10 screws | 4 |
| M2 | Hex nuts | 4 |
| M2.5 | M2.5×2 screws | 2 |
| M2.5 | M2.5×5 screws | 3 |
| M2.5 | M2.5×6 screws | 11 |
| M2.5 | M2.5×8 screws | 16 |
| M2.5 | Hex nuts | 16 |
| M3 | M3×8 screws | 24 |
| M3 | M3×12 screws | 32 |
| M3 | Hex nuts | 44 |
| M4 | M4×12 screws | 12 |
| M4 | Hex nuts | 12 |

### Fasteners by assembly step

| Component | Fasteners required | Notes |
|---|---|---|
| Mounts (×4) | 16× M2.5×8, 16× M2.5 hex nuts | Attach the main leg mounts to the body shell |
| Hip joints | 12× M4×12, 12× M4 hex nuts, 4× M3×8, 4× M3 hex nuts | Heavy load-bearing joints |
| Thigh joints | 16× M3×12, 4× M3×8, 20× M3 hex nuts | |
| Knee joints | 16× M3×12, 4× M3×8, 20× M3 hex nuts | |
| Servo horns (×12) | 12× M3×8 | Attaches printed links to servo output shafts |
| Bridges | 7× M2.5×6 | Internal chassis sensor mounts |
| OLED screen | 4× M2×10, 4× M2 hex nuts | |
| ESP32 | 1× M2.5×5 | |
| ToF sensors (×3) | 6× M2×4 | |
| IMU | 2× M2.5×2 | |
| PCA9685 Multiplexer | 4× M2.5×6 | |
| Buck Converter | 2× M2.5×5 | |

---

## Printed parts

All parts are designed to be printed without supports where possible. Specific
print settings (infill, wall count, material) are documented in
[3D Printing](printing.md).

!!! warning "Left/Right mirroring"
    Pay close attention to L/R mirroring requirements when slicing the leg
    components. Printing two identical sides instead of a mirrored pair is a
    common mistake that will require a full reprint.

**Body components**

| Part | Qty |
|---|---|
| Shell (main chassis) | 1 |
| LiPo cage | 1 |
| Leg mounts | 4 |
| Bridge mounts (internal sensor structures) | 2 |

**Leg components — print 4 sets**

| Part | Qty (per set) | Qty (total) | Notes |
|---|---|---|---|
| Link 1 — Hip | 1 | 4 | Check L/R orientation |
| Link 2 — Thigh | 1 | 4 | |
| Link 3 — Knee / Foot | 1 | 4 | |

---

## Tools required

| Tool | Use |
|---|---|
| Hex drivers / Allen keys (M2, M2.5, M3, M4) | Driving all socket head screws |
| Soldering iron & solder | BMS, Buck Converter, and I²C cable trees |
| Wire strippers & crimping tool | Custom length servo cables and Dupont connectors |
| Needle-nose pliers | Seating hex nuts into tight printed pockets |
| Multimeter | Verifying 5V Buck Converter output and checking for short circuits before power-on |

## Key components

This section gives a plain-language introduction to the more complex components
used in FaceHugger. If you are already familiar with embedded electronics, skip
ahead to [Electronics](#electronics).

---

### ESP32

The ESP32 is the brain of the robot. It is a small, low-power microcontroller
with a dual-core processor, built-in Wi-Fi, and a large number of configurable
I/O pins. Unlike a Raspberry Pi, it has no operating system — it runs a single
compiled C++ program in a continuous loop, which is exactly what you want for
hard real-time control of 12 servos simultaneously.

In FaceHugger, the ESP32 handles everything: it creates the Wi-Fi hotspot,
receives commands from the app over WebSockets, runs the gait state machine, and
drives the PCA9685 over I²C.

---

### PCA9685 — PWM Multiplexer

Servos are controlled by a PWM signal (Pulse Width Modulation) — a precise
pulse sent up to 50 times per second whose width encodes the target angle. The
ESP32 has a limited number of hardware PWM channels and cannot reliably generate
12 independent, jitter-free servo signals on its own while also running Wi-Fi and
gait math.

The PCA9685 solves this by offloading the PWM generation entirely. The ESP32
sends a single I²C command ("set channel 4 to 1500µs") and the PCA9685 handles
the precise timing for all 16 channels independently, freeing the ESP32 to focus
on higher-level logic.

---

### DSS 230mg Servos

A servo is a self-contained actuator: a DC motor, a gearbox, a position sensor,
and a control circuit all in one package. You send it a PWM pulse and it moves
to the corresponding angle and holds it against external force.

The DSS 230mg are **metal-gear, high-voltage digital servos**. Metal gears matter
here because the hip and thigh joints sustain significant shock loads every time a
foot strikes the ground — plastic gears would strip within hours. The 230mg torque
rating (roughly 2.3 Nm) is the minimum viable torque for a robot of this weight at
the leg geometry used.

FaceHugger uses 3 servos per leg — one for each joint (Hip, Thigh, Knee) — for
12 servos total.

---

### 2S LiPo Battery

LiPo (Lithium Polymer) batteries offer a very high energy density, making them
the standard choice for mobile robotics. A **2S** pack contains two cells wired in
series, giving a nominal voltage of 7.4V and a fully charged voltage of 8.4V.

!!! danger "LiPo safety"
    LiPos are not like AA batteries. Mishandling them — over-discharging,
    puncturing, shorting, or charging without a proper LiPo charger — can cause
    fire. Always store them at storage voltage (~3.8V per cell) when not in use,
    never leave them charging unattended, and inspect the pouch for swelling before
    each use. A swollen LiPo must be safely discharged and disposed of immediately.

---

### BMS — Battery Management System

The BMS sits between the LiPo and the rest of the electronics and acts as an
automatic safety gate. It monitors cell voltage and current in real time and
cuts the circuit if it detects:

- **Over-discharge** — cell voltage dropping below a safe threshold (permanently
  damages LiPo cells)
- **Overcharge** — cell voltage exceeding safe limits (fire risk)
- **Short circuit** — a sudden current spike indicating a wiring fault

Skipping the BMS to save space is a common and dangerous mistake. Without it, a
single wiring error or a over-discharged battery has nothing preventing a
thermal runaway.

---

### Buck Converter

The LiPo outputs between 7.4V and 8.4V depending on charge state. The ESP32,
PCA9685, OLED, IMU, and ToF sensors all run on 3.3V or 5V logic. Feeding raw
LiPo voltage into any of these will instantly destroy them.

The Buck Converter is a switching voltage regulator that efficiently steps the
LiPo voltage down to a stable 5V output regardless of the battery's charge state.
"Efficiently" is important here — a linear regulator would do the same job but
would waste the excess voltage as heat, which is unacceptable in a sealed chassis
drawing significant current.

---

### 2200µF Capacitor

When multiple servos move simultaneously, they draw a large burst of current in
a very short time. This sudden demand can cause the supply voltage to dip
momentarily — a **brownout** — which can reset the ESP32 mid-gait or corrupt I²C
communication with the PCA9685.

The capacitor acts as a local energy reservoir: it charges slowly from the supply
and discharges instantly into the circuit when a current spike occurs, smoothing
out the voltage dip before it reaches the logic boards. It is wired in parallel
with the PCA9685's onboard capacitor to increase the total reserve.

---

### IMU — Inertial Measurement Unit

The IMU (an MPU-6050) combines a 3-axis accelerometer and a 3-axis gyroscope on
a single I²C chip. It measures the robot's orientation and angular velocity in
real time. In FaceHugger, its primary role is detecting when the robot has been
flipped upside-down and triggering the `STATE_FAILSAFE` state to cut movement and
switch the OLED to the confused expression.

---

### ToF Sensors — Time-of-Flight

ToF sensors measure distance by emitting an infrared laser pulse and timing how
long it takes to reflect back from a surface. Three sensors are mounted on the
body bridges, giving the robot basic spatial awareness of its immediate
surroundings. Unlike ultrasonic sensors, ToF sensors have a very narrow beam and
high precision at short range, making them well suited for detecting obstacles at
leg-height.