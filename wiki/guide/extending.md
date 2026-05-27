# Next steps and extending FaceHugger

FaceHugger is a working robot, but it is also a starting point. This page collects what we would do next and how someone else could build on it. If you are picking the project up, the [Reference](../reference/conventions.md) section is where the architecture is explained in depth; this page is the shorter "what now" view.

## Future directions

FaceHugger walks, but there is a lot of room to take it further. These are the directions we would explore next. Each is a project in its own right, sketched here as a starting point rather than a finished plan.

- **More sensing.** Add sensors beyond the current ToF and IMU, and close the loop on them so the robot reacts to its surroundings rather than just reading them.
- **IMU stabilization and correction.** Use the IMU to keep the body level and adjust the gait in real time, so the robot stays balanced on uneven ground or when nudged.
- **Better walk cycles and animation.** Refine the gaits and author smoother, more lifelike clips for richer and more natural motion.
- **A camera for navigation.** Add a camera so the robot can perceive and navigate its environment instead of being driven blind.
- **A redesign around the center of gravity.** Revisit the mechanical design to bring the motors closer to the body and lower the center of gravity, which would make balance and dynamic motion much easier.

These are deliberately high level. We would expand each into its own design and milestones before building it.

## Tips for expanding the project

The architecture is built so each piece can be extended without touching the others. A few entry points:

- **Add a new gesture (clip).** Pose it on the Blender rig, save it as a clip in the FH Clip Panel, export, copy the bundled `clips_all.h` into the firmware tree, and reflash. The full path is in [Blender to robot clips](../reference/animation/clip-panel.md).
- **Add or tune a gait.** Gaits are phase-based angle schedules in the firmware (`tickGait` / `tickTrot`), parameterized by step length, height, period, and duty. The [firmware reference](../reference/firmware/index.md) explains the motion engine.
- **Extend the control API.** Add a new `T:N` command by handling it in the firmware's `network.cpp`. Because the simulator's `serve` runs the exact compiled firmware, your new command works in the sim immediately, with no separate mock to update. See the [WebSocket API](../reference/remote-control/websocket-api.md).
- **Validate in simulation first.** Anything you change in the firmware or the clips can be checked in PyBullet before it reaches hardware, including driving the sim from the real mobile app. See the [simulation section](../reference/simulation/index.md).

## Make it yours

If you fork or extend FaceHugger, the conventions in the [Reference](../reference/conventions.md) are the contract that keeps the CAD, firmware, animation, and simulation in agreement. Keep that contract and the rest of the system stays consistent as you build on it.
