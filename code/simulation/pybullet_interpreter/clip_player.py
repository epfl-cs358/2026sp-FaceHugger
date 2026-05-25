# code/simulation/pybullet_interpreter/clip_player.py
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
    LEG_RL,
    LEG_RR,
)

# Firmware LegId iteration order (matches a[12] layout in ClipFrame).
_LEG_IDS = (LEG_FR, LEG_FL, LEG_RR, LEG_RL)


def frame_to_joint_targets(a: list[float]) -> dict[str, float]:
    """Convert one clip frame's math-space angles to PyBullet joint targets.

    Input:  a[12] — pre-scaled math-space degrees in firmware LegId order:
            a[0..2]=FR(sh,th,kn), a[3..5]=FL, a[6..8]=RR, a[9..11]=RL.
    Output: dict mapping URDF joint names to radians, e.g.
            {"fr_link1_joint": 0.0, "fr_link2_joint": 1.047, ...}

    Does NOT apply EMA smoothing (firmware CLIP_EMA_ALPHA). Pure geometry.
    Does apply clamp_clip_servos to match firmware clip-path clamping.

    Mirrors the inner loop of tickClip() in spinal_cord.cpp:427-437.
    """
    raise NotImplementedError


def _interpolate_frame(frames: list[ClipFrame], elapsed_ms: float) -> list[float]:
    """Linear interpolation between adjacent keyframes at elapsed_ms.

    Mirrors firmware clipPoseAt() in motion_math.cpp:62-90.

    Before first frame → first frame.
    After last frame   → last frame.
    Between frames     → linear interpolation.

    Input:  sorted ClipFrame list, elapsed time in ms.
    Output: a[12] interpolated math-space angles.
    """
    raise NotImplementedError


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
        raise NotImplementedError

    def step(self, elapsed_ms: float) -> None:
        """Apply clip pose at elapsed_ms to the PyBullet robot.

        Interpolates the clip, translates to servo degrees, converts to
        radians, and calls pybullet.setJointMotorControl2 for each joint.

        Input: elapsed_ms — time since clip start (float, milliseconds).
        """
        raise NotImplementedError

    def play_blocking(self, gui: bool = True) -> None:
        """Play the clip in real time, blocking until done, then hold final pose.

        Mirrors firmware hold-at-end behaviour: after the last frame,
        the final pose is re-sent every frame until the user closes the
        window (GUI mode) or function returns (headless mode).

        Headless mode (gui=False): plays the clip in wall-clock time
        then returns immediately (for CI / automated testing).
        """
        raise NotImplementedError
