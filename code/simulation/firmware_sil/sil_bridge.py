"""Drive PyBullet from the EXACT firmware control code (software-in-the-loop).

Loads the compiled `fh_sim` module (the real firmware SpinalCord, built by
firmware_sil/CMakeLists.txt) and converts the 12 servo angles it computes into
URDF joint targets — so the sim moves on the firmware's own output, with no
Python re-port in the loop.

The servo-deg -> URDF-radian mapping reuses firmware_port.servo_convention (the
same servo_to_radians + per-leg URDF axis sign the clip re-port uses); only the
servo *degrees* differ in origin — here they come straight from the firmware.

Build the module first:
    cd code/simulation/firmware_sil
    cmake -S . -B build -DPython_EXECUTABLE=$(which python) && cmake --build build
"""

import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_BUILD = _HERE / "build"

# firmware LegId order (FR, FL, RR/BR, RL/BL); each leg = (hip, thigh, knee).
from firmware_port.servo_convention import (  # noqa: E402
    LEG_FR,
    LEG_FL,
    LEG_RR,
    LEG_RL,
    LEG_ID_TO_SIM_NAME,
    LEG_ID_TO_URDF_AXIS_SIGN,
    servo_to_radians,
)

_LEG_IDS = (LEG_FR, LEG_FL, LEG_RR, LEG_RL)


def load_fh_sim():
    """Import the compiled fh_sim module, with a helpful error if it isn't built."""
    if str(_BUILD) not in sys.path:
        sys.path.insert(0, str(_BUILD))
    try:
        import fh_sim
    except ImportError as e:
        raise ImportError(
            "fh_sim (the compiled firmware) is not built. From code/simulation/"
            "firmware_sil/ run:\n"
            "  cmake -S . -B build -DPython_EXECUTABLE=$(which python)\n"
            "  cmake --build build\n"
            f"(underlying import error: {e})"
        ) from e
    return fh_sim


def servo_angles_to_joint_targets(angles12):
    """12 firmware servo degrees -> {urdf_joint_name: radians}.

    Mirrors firmware_port.clip_player.frame_to_joint_targets, but the servo
    degrees come straight from the firmware (already post-translateToServo), so
    this is only the servo-deg -> radian + URDF-axis-sign half.
    """
    targets = {}
    for leg_id in _LEG_IDS:
        hip = angles12[leg_id * 3]
        thigh = angles12[leg_id * 3 + 1]
        knee = angles12[leg_id * 3 + 2]
        name = LEG_ID_TO_SIM_NAME[leg_id]
        axis = LEG_ID_TO_URDF_AXIS_SIGN[leg_id]
        targets[f"{name}_link1_joint"] = axis * servo_to_radians(hip)
        targets[f"{name}_link2_joint"] = axis * servo_to_radians(thigh)
        targets[f"{name}_link3_joint"] = -axis * servo_to_radians(knee)
    return targets


class FirmwareSILDriver:
    """Plays clips through the real firmware code, driving PyBullet joints."""

    def __init__(self):
        self._fh = load_fh_sim()
        self._fc = self._fh.FirmwareControl()

    def clip_names(self):
        return self._fc.clip_names()

    def joint_targets_now(self):
        """Current firmware servo angles as {joint_name: radians}."""
        return servo_angles_to_joint_targets(self._fc.servo_angles())

    def play_clip_blocking(
        self,
        robot_id,
        joint_map,
        clip_name,
        force,
        velocity,
        gui=True,
        on_step=None,
        timestep=1.0 / 240.0,
    ):
        """Play `clip_name` through the firmware, stepping PyBullet at 240 Hz.

        Headless (gui=False) runs once for the clip's duration and returns; GUI
        holds the final pose until the window closes.
        """
        import pybullet as p

        cid = self._fc.clip_id_by_name(clip_name)
        if cid < 0:
            raise KeyError(f"clip {clip_name!r} not in {self._fc.clip_names()}")
        duration_ms = self._fc.clip_duration_ms(cid)

        self._fc.set_clock_ms(0)
        self._fc.play_clip(cid)

        step = 0
        while p.isConnected():
            t_ms = int(step * 1000.0 / 240.0)
            self._fc.tick(t_ms)  # advance firmware time + run one control tick
            for name, rad in servo_angles_to_joint_targets(
                self._fc.servo_angles()
            ).items():
                idx = joint_map.get(name)
                if idx is not None:
                    p.setJointMotorControl2(
                        robot_id,
                        idx,
                        p.POSITION_CONTROL,
                        targetPosition=rad,
                        force=force,
                        maxVelocity=velocity,
                    )
            p.stepSimulation()
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(timestep)
            step += 1
            if t_ms >= duration_ms and not gui:
                break
