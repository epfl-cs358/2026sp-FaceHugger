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

Each joint moves the foot in a predictable way: the hip yaw swings the whole leg left or right, the femur pitch lifts or lowers the foot in an arc, and the knee pitch extends or retracts it. Walking gaits are built from coordinated sweeps of those two pitch joints, while yaw steers.

**Inverse kinematics** (IK) is the reverse problem: given a desired foot position in space, compute the three joint angles that reach it. For a 3-DOF leg, the hip and knee angles are solved geometrically (law of cosines on the two-link chain), and the shoulder yaw is found with `atan2` from the target's horizontal position relative to the body. This gives a closed-form solution with no iteration needed.

IK is fully implemented in the simulation pipeline (Python) and in the Blender rig (used by the animation exporter). The firmware, however, does not run IK at runtime. It works from pre-baked angle tables: gaits are fixed pose sequences, and animations are authored offline and exported as frame arrays. The vision of a closed-loop IK controller (where an IMU loop reads body tilt, computes corrective foot targets, and resolves them to servo angles in real time to keep the robot level) was designed but not implemented in the firmware within the scope of this project. The building blocks are all there; the runtime loop is the missing piece.

For the servo sign conventions and the exact coordinate frame, see [Reference → Conventions](../reference/conventions.md).

## Sizing & loads

!!! todo
    Why the links and servos are sized the way they are; how much weight a leg
    holds. Full torque model in
    [Reference → Torque analysis](../reference/simulation/torque-analysis.md).

The servos have a torque of 30 kgcm. This means they can, at full power, move a load of 30kg, 1cm away from the axis of rotation. This may seem a lot, but given a leg of total length, say 20 cm, the furthermost servo would be able to move a load of 1.5kg. We had to shorten the legs to the most we could, while keeping the flip gimmick.