<!-- Sections below are suggestions, not requirements. Keep what's useful, drop or merge the rest. -->
# Design


## The concept

FaceHugger is a four-legged robot with an unsual stance and gait pattern. It is not built like a mammal (think about dogs or horses). Instead, its limbs are spread at 45° and it moves either like a crab, or with a diagonal trot.

On top of that, the FaceHugger can flip its legs upside down, in order to stand and walk on its back. 

To determine its orientation, we equipped it with multiple Time Of Flight sensors. But unfortunately, we couldn't make them work on time. 

A small IMU is connected, should you ever want to get precise gyroscopic measurements for movement.

Finally, we use a screen to display cute expressions.

## How a leg is built

Each leg is comprised of 3 links: the hip, connecting the leg to the body; the femur, connected to the hip; and the knee, connected to the femur. Each leg has 3 motors, 2 in the hip and 1 in the knee. This gives each leg 3 degrees of freedom.

The hip moves in yaw in respect to the body, the femur moves in pitch in respect to the hip, and the knee moves in pitch in respect to the femur.

## How a leg moves

!!! todo
    Plain-language intuition: which joint moves the foot where; forward vs
    inverse kinematics; the zero pose and sign conventions at a glance. Full
    treatment in the [kinematics reference](../reference/firmware/kinematics.md)
    and [servo conventions](../reference/conventions.md).

## Sizing & loads

!!! todo
    Why the links and servos are sized the way they are; how much weight a leg
    holds. Full torque model in
    [Reference → Torque analysis](../reference/simulation/torque-analysis.md).

The servos have a torque of 30 kgcm. This means they can, at full power, move a load of 30kg, 1cm away from the axis of rotation. This may seem a lot, but given a leg of total length, say 20 cm, the furthermost servo would be able to move a load of 1.5kg. We had to shorten the legs to the most we could, while keeping the flip gimmick.