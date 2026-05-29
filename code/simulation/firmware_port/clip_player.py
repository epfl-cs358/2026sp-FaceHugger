# code/simulation/firmware_port/clip_player.py
"""Drives PyBullet joints from animation clip data.

The pipeline for each frame:
  1. Interpolate clip at elapsed_ms → math-space angles a[12]
  2. translate_to_servo(leg_id, sh, th, kn) → ServoTriple (0-180°)
  3. clamp_clip_servos(servo)  → keeps joints off mechanical stops
  4. servo_to_radians(deg)     → PyBullet joint angle (rad)
  5. apply_joint_targets(robot_id, joint_map, targets, force, velocity)

This mirrors the firmware tickClip() in
code/firmware/src/nervous_system/spinal_cord.cpp:413-457.

EMA smoothing (CLIP_EMA_ALPHA=0.75 in firmware) is intentionally
NOT applied here. The simulator is used to validate clip geometry,
not to reproduce smoothing artifacts.
"""

from .clip_loader import ClipData, ClipFrame
from .servo_convention import (
    LEG_FL,
    LEG_FR,
    LEG_ID_TO_SIM_NAME,
    LEG_ID_TO_URDF_AXIS_SIGN,
    LEG_RL,
    LEG_RR,
    clamp_clip_servos,
    servo_to_radians,
    translate_to_servo,
)

# Firmware LegId iteration order (matches a[12] layout in ClipFrame).
_LEG_IDS = (LEG_FR, LEG_FL, LEG_RR, LEG_RL)


def frame_to_joint_targets(a: list[float]) -> dict[str, float]:
    """Convert one clip frame's math-space angles to PyBullet joint targets.

    Input:  a[12] — pre-scaled math-space degrees in firmware LegId order:
            a[0..2]=FR(sh,th,kn), a[3..5]=FL, a[6..8]=RR, a[9..11]=RL.
    Output: dict mapping URDF joint names to radians, e.g.
            {"fr_link1_joint": 0.0, "fr_link2_joint": -1.047, ...}

    Does NOT apply EMA smoothing (firmware CLIP_EMA_ALPHA). Pure geometry.
    Does apply clamp_clip_servos to match firmware clip-path clamping.

    All three joints are multiplied by the per-leg URDF axis sign (±1 from the
    joint's <axis> direction in facehugger.urdf): link1 (yaw) and link2 (hip)
    by +axis, link3 (knee) by -axis. link1 needs the factor too — its <axis z>
    follows the same {fl,br}=+ / {fr,bl}=− diagonal pattern as link2, and
    without it fr/bl shoulder yaw renders backwards in the sim.

    Mirrors the inner loop of tickClip() in spinal_cord.cpp:427-437.
    """
    targets: dict[str, float] = {}
    for leg_id in _LEG_IDS:
        sh = a[leg_id * 3]
        th = a[leg_id * 3 + 1]
        kn = a[leg_id * 3 + 2]
        servo = clamp_clip_servos(leg_id, translate_to_servo(leg_id, sh, th, kn))
        urdf_name = LEG_ID_TO_SIM_NAME[leg_id]
        axis = LEG_ID_TO_URDF_AXIS_SIGN[leg_id]  # see docstring: link1 needs it too
        targets[f"{urdf_name}_link1_joint"] = axis * servo_to_radians(servo.hip)
        targets[f"{urdf_name}_link2_joint"] = axis * servo_to_radians(servo.thigh)
        targets[f"{urdf_name}_link3_joint"] = -axis * servo_to_radians(servo.knee)
    return targets


def _interpolate_frame(frames: list[ClipFrame], elapsed_ms: float) -> list[float]:
    """Linear interpolation between adjacent keyframes at elapsed_ms.

    Mirrors firmware clipPoseAt() in motion_math.cpp:62-90.

    Before first frame → first frame.
    After last frame   → last frame.
    Between frames     → linear interpolation.

    Input:  sorted ClipFrame list, elapsed time in ms.
    Output: a[12] interpolated math-space angles.
    """
    if not frames:
        return [90.0] * 12
    if elapsed_ms <= frames[0].t_ms or len(frames) == 1:
        return list(frames[0].a)
    if elapsed_ms >= frames[-1].t_ms:
        return list(frames[-1].a)
    lo_idx = 0
    for i in range(len(frames) - 1):
        if frames[i + 1].t_ms <= elapsed_ms:
            lo_idx = i + 1
        else:
            break
    lo = frames[lo_idx]
    hi = frames[lo_idx + 1]
    span = hi.t_ms - lo.t_ms
    f = 0.0 if span == 0 else (elapsed_ms - lo.t_ms) / span
    return [lo.a[j] + (hi.a[j] - lo.a[j]) * f for j in range(12)]


class ClipPlayer:
    """Drives a PyBullet robot through one animation clip.

    Does NOT import pybullet at module level so that clip_loader and
    servo_convention can be tested without a PyBullet connection.
    PyBullet is imported lazily inside play_blocking().

    Usage:
        player = ClipPlayer(robot_id, joint_map, clip_data)
        player.play_blocking(gui=True)
    """

    def __init__(
        self,
        robot_id: int,
        joint_map: dict[str, int],
        clip: ClipData,
        force: float = 10.0,
        velocity: float = 5.0,
    ) -> None:
        """Set up clip player.

        robot_id:  PyBullet body ID from pybullet.loadURDF().
        joint_map: {joint_name: pybullet_joint_index} from helpers.build_joint_map().
        clip:      ClipData from clip_loader.load_clips_all_h().
        force:     max joint force (N·m), passed to setJointMotorControl2.
        velocity:  max joint velocity (rad/s), passed to setJointMotorControl2.
        """
        self._robot_id = robot_id
        self._joint_map = joint_map
        self._clip = clip
        self._force = force
        self._velocity = velocity

    def step(self, elapsed_ms: float) -> None:
        """Apply clip pose at elapsed_ms to the PyBullet robot.

        Interpolates the clip, translates to servo degrees, converts to
        radians, and calls pybullet.setJointMotorControl2 for each joint.

        Input: elapsed_ms — time since clip start (float, milliseconds).
        """
        import pybullet as p

        a = _interpolate_frame(self._clip.frames, elapsed_ms)
        targets = frame_to_joint_targets(a)
        for joint_name, rad in targets.items():
            joint_idx = self._joint_map.get(joint_name)
            if joint_idx is None:
                continue
            p.setJointMotorControl2(
                self._robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=rad,
                force=self._force,
                maxVelocity=self._velocity,
            )

    def play_blocking(self, gui: bool = True, loop: bool = False, on_step=None) -> None:
        """Play the clip in real time, blocking until done.

        Default: after the last frame, hold the final pose every frame until
        the user closes the window (GUI) or return immediately (headless) —
        mirrors firmware hold-at-end.

        loop=True (GUI only): instead of holding, restart from frame 0 and
        replay continuously until the window is closed. Physics state is NOT
        reset between loops, so you can watch cumulative effects (drift,
        foot slip, tipping) build up over time. Ignored when gui=False so
        headless/CI runs still terminate.

        on_step: optional callable(step_index) invoked after every
        stepSimulation — used for torque/current monitoring (run_clip wires it
        up). Kept generic so this module stays decoupled from the sim tooling.

        Headless mode (gui=False): plays the clip once in wall-clock time
        then returns immediately (for CI / automated testing).
        """
        import time

        import pybullet as p

        step_s = 1.0 / 240.0
        start = time.monotonic()
        step_index = 0

        while True:
            elapsed_ms = (time.monotonic() - start) * 1000.0
            clamped_ms = min(elapsed_ms, float(self._clip.duration_ms))
            self.step(clamped_ms)
            p.stepSimulation()
            if on_step is not None:
                on_step(step_index)
            step_index += 1

            if elapsed_ms >= self._clip.duration_ms:
                if not gui:
                    return
                if not p.isConnected():
                    return
                if loop:
                    start = (
                        time.monotonic()
                    )  # replay from frame 0 (physics carries over)
                    continue
                time.sleep(step_s)
                continue

            deadline = start + elapsed_ms / 1000.0 + step_s
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
