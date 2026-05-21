<!-- Sections below are suggestions, not requirements - keep what's useful, drop or merge the rest. -->
# Software

!!! todo "Stub - to be written"
    Brief overview of the firmware and host-side tooling, then how to set it up
    and run it. Deep detail lives in
    [Reference → Firmware](../reference/firmware/index.md).

## Overview

!!! todo
    What runs where: ESP32 firmware (the robot) vs host-side tooling (simulation,
    animation, remote-control app). The brain / nervous_system / shared split.

## How it works

!!! todo
    Accessible version of the algorithms. Inverse kinematics (foot target ->
    three joint angles), angle conventions, and the finite state machine.

    \[
    \tau_{knee} = F_{tip} \cdot L_3 \cdot \cos\theta_{knee}
    \]

    ```mermaid
    stateDiagram-v2
        [*] --> Idle
        Idle --> Standing
        Standing --> Walking
        Walking --> Standing
        Standing --> Idle
    ```

    (placeholders - replace the equation and FSM with the real ones)

## Setup

!!! todo
    PlatformIO + flashing the ESP32.

```bash
pio run                 # build for the upesy_wroom ESP32
pio run -t upload       # flash
pio device monitor      # 115200 baud
```

## Running

!!! todo
    Power-on sequence and connecting the
    [remote-control app](../reference/remote-control/index.md). Host-side
    simulation:

```bash
python facehugger.py sim          # GUI, standing pose
python facehugger.py sim --walk   # walk gait
python facehugger.py blender --rigged
```
