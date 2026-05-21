# How it works

!!! todo "Stub - to be written"
    Accessible walkthrough of the algorithms that make the robot move. This is
    the friendly version; the full treatment is in
    [Reference → Firmware → Kinematics](../../reference/firmware/kinematics.md).

## Inverse kinematics

!!! todo
    How a foot target becomes three joint angles, explained simply. The math:

    \[
    \tau_{knee} = F_{tip} \cdot L_3 \cdot \cos\theta_{knee}
    \]

    (placeholder equation - confirm/replace during authoring)

## Angle conventions

!!! todo
    Zero pose, sign of each joint, the per-leg shoulder rest derivation.

## Finite state machine

!!! todo
    The robot's control states (idle / standing / walking / …) and the
    transitions between them. A Mermaid state diagram:

    ```mermaid
    stateDiagram-v2
        [*] --> Idle
        Idle --> Standing
        Standing --> Walking
        Walking --> Standing
        Standing --> Idle
    ```

    (placeholder - replace with the real FSM during authoring)
