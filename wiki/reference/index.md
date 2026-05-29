# Reference

The reference explains how FaceHugger works under the hood. Where the [build guide](../guide/design.md) tells you how to assemble the robot, this section tells you how the software and the conventions fit together, for someone who wants to understand the system or build on it.

If you read one page first, make it **Conventions**. The coordinate frames, angle spaces, and the math-to-servo transform defined there are the shared contract that the firmware, the animation pipeline, and the simulation all rely on. Everything else makes more sense once that is clear. After that, jump to whichever area you need.

<div class="grid cards" markdown>

-   :material-axis-arrow:{ .lg .middle } __Conventions__

    ---

    The coordinate frames, the math-space and servo-space angle spaces, leg naming, the `NEUTRAL[]` standing pose, and the per-leg `translateToServo` transform. The contract everything else depends on.

    [:octicons-arrow-right-24: Conventions](conventions.md)

-   :material-protocol:{ .lg .middle } __WebSocket API__

    ---

    The `T:` command protocol: how a client moves the robot, selects a gait, plays a clip, calibrates a servo, and reads telemetry.

    [:octicons-arrow-right-24: API](api.md)

-   :material-chip:{ .lg .middle } __Firmware__

    ---

    The ESP32 motion engine: how gaits and the baked-clip player produce motion, the state machine, the per-leg hardware mapping, and the main loop. Learn how a command becomes leg movement.

    [:octicons-arrow-right-24: Firmware](firmware/index.md)

-   :material-movie-open:{ .lg .middle } __Animation pipeline__

    ---

    The path from CAD to motion: 3D model to URDF, the Blender rig built from that URDF, and how authored clips are exported and replayed on the robot.

    [:octicons-arrow-right-24: Animation](animation/index.md)

-   :material-robot-industrial:{ .lg .middle } __Simulation__

    ---

    The PyBullet simulator: the `facehugger.py` CLI, and how it runs the exact compiled firmware in the loop on the generated URDF, so you can validate motion before it reaches hardware.

    [:octicons-arrow-right-24: Simulation](simulation/index.md)

-   :material-cellphone-link:{ .lg .middle } __Remote control__

    ---

    The mobile app and browser panel: how they connect over WebSocket and drive either the robot or the simulator.

    [:octicons-arrow-right-24: Remote control](remote-control/index.md)

</div>

For what is planned next and where the project could go, see the [roadmap](roadmap.md) and the build guide's [next steps](../guide/extending.md).
