# Next steps and extending FaceHugger

FaceHugger is a working robot, but it is also a starting point. This page collects what we would do next and how someone else could build on it. If you are picking the project up, the [Reference](../reference/conventions.md) section is where the architecture is explained in depth; this page is the shorter "what now" view.

## Future recommendations

These are the things we know we would tackle next, roughly in priority order.

- **Finish change B on the hardware.** The front-left shoulder was regularized in code so servo 90 points outward like the other three legs, but the physical horn remount and a clip re-export are still pending. Until both happen, the running robot expects the old FL standing pose. See [Conventions](../reference/conventions.md).
- **Finalize the pinout and the servo map.** The wiring pinout is still a proposal, and the flat servo-id scheme (`servo_mapping.yaml` / a firmware `SERVO_CONFIG[]`) has not been committed. Locking these down removes the "confirm against `config.h`" caveat that runs through the wiring and convention docs.
- **The on-board animation player.** Clips today are baked frames the firmware replays. The planned richer on-board player (a `.fhc` format with runtime IK and IMU correction) would replace that and the legacy textbook IK. It is designed but not implemented; see the [roadmap](../reference/roadmap.md).
- **Revisit servo sizing or clip speed.** The simulation shows that fast clip moves demand roughly twice the servo's stall torque in transients, so those moves saturate on real hardware. Either slow the clips (cap joint velocity) or size up the servos. See [Controlling the simulation](../reference/simulation/pybullet-control.md).
- **Body-pose IK (`T:3`).** The `CMD_POSE` command is documented but has no firmware handler yet, so the robot cannot yet hold a commanded chassis pose. It is a natural next feature.
- **Bring the sensors into the loop.** The ToF and IMU are wired but the telemetry and any closed-loop use of them are not simulated or fully used yet.

## Tips for expanding the project

The architecture is built so each piece can be extended without touching the others. A few entry points:

- **Add a new gesture (clip).** Pose it on the Blender rig, save it as a clip in the FH Clip Panel, export, copy the bundled `clips_all.h` into the firmware tree, and reflash. The full path is in [Blender to robot clips](../reference/animation/clip-panel.md).
- **Add or tune a gait.** Gaits are phase-based angle schedules in the firmware (`tickGait` / `tickTrot`), parameterized by step length, height, period, and duty. The [firmware reference](../reference/firmware/index.md) explains the motion engine.
- **Extend the control API.** Add a new `T:N` command by handling it in the firmware's `network.cpp`. Because the simulator's `serve` runs the exact compiled firmware, your new command works in the sim immediately, with no separate mock to update. See the [WebSocket API](../reference/remote-control/websocket-api.md).
- **Validate in simulation first.** Anything you change in the firmware or the clips can be checked in PyBullet before it reaches hardware, including driving the sim from the real mobile app. See the [simulation section](../reference/simulation/index.md).

## Make it yours

If you fork or extend FaceHugger, the conventions in the [Reference](../reference/conventions.md) are the contract that keeps the CAD, firmware, animation, and simulation in agreement. Keep that contract and the rest of the system stays consistent as you build on it.
