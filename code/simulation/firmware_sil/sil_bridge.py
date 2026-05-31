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

import re
import sys
import time

from firmware_port.servo_convention import (  # noqa: E402
    LEG_FL,
    LEG_FR,
    LEG_ID_TO_SIM_NAME,
    LEG_ID_TO_URDF_AXIS_SIGN,
    LEG_RL,
    LEG_RR,
    servo_to_radians,
)

from .build import _BUILD, _FW_SRC, _HERE, _compiled_so, _newest_source_mtime  # noqa: F401
from .build import build, load_fh_sim, staleness
from .imu import _IMU_CLEAR_COS, _IMU_FLIP_COS  # noqa: F401
from .imu import _body_up_z, imu_hysteresis_step, imu_pitch_roll_deg  # noqa: F401
from .imu import update_imu_from_pybullet
from .telemetry import _BAND_RGBA, _SIM_NAME_TO_LEG_ID, _TELEM_JOINTS, _TELEM_LEGS  # noqa: F401
from .telemetry import apply_torque_colors, build_telemetry_frame, torque_color  # noqa: F401
from .trace import _GAIT_NAME_TO_ID, trace_clip, trace_gait  # noqa: F401

_LEG_IDS = (LEG_FR, LEG_FL, LEG_RR, LEG_RL)

# ── out-of-range ([OOR]) capture ──────────────────────────────────────────────
# The firmware itself prints "[OOR] servo <channel> requested <deg>" from its
# setJointAngles / setServoAngle guards (visible on hardware too). We parse the
# lines the firmware emitted; we don't synthesize them.
_OOR_RE = re.compile(r"\[OOR\] servo (\d+) requested ([-\d.]+)")


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


def parse_oor(lines):
    """Serial lines -> {pca_channel: requested_deg} (last request per channel)."""
    out = {}
    for ln in lines:
        m = _OOR_RE.search(ln)
        if m:
            out[int(m.group(1))] = float(m.group(2))
    return out


def channel_to_joint(fc):
    """{pca_channel: urdf_joint_name} from the firmware's own channel map."""
    chans = list(fc.servo_channels())  # firmware order: leg*3 + (0 hip,1 thigh,2 knee)
    out = {}
    for leg_id in _LEG_IDS:
        leg = LEG_ID_TO_SIM_NAME[leg_id]
        for j, link in enumerate((1, 2, 3)):
            out[chans[leg_id * 3 + j]] = f"{leg}_link{link}_joint"
    return out


class FirmwareSILDriver:
    """Plays clips and gaits through the real firmware code, driving PyBullet joints."""

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
        imu_state = False  # latched upside-down flag, threaded across ticks
        while p.isConnected():
            t_ms = int(step * 1000.0 / 240.0)
            # Emulate the MPU6050: read PyBullet body orientation and push the
            # hysteresed upside-down latch + pitch/roll into the firmware
            # BEFORE tick(), so SpinalCord/Network see the IMU state this tick.
            imu_state = update_imu_from_pybullet(self._fc, p, robot_id, imu_state)
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
            if step % 12 == 0:  # 240 Hz / 12 = 20 Hz visual update
                apply_torque_colors(p, robot_id, joint_map)
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(timestep)
            step += 1
            if t_ms >= duration_ms and not gui:
                break

    def run_gait_blocking(
        self,
        robot_id,
        joint_map,
        gait_name,
        force,
        velocity,
        direction="FW",
        gui=True,
        on_step=None,
        timestep=1.0 / 240.0,
        duration_s=None,
    ):
        """Run a firmware gait through PyBullet at 240 Hz (the exact tickGait/tickTrot).

        Same per-tick PyBullet drive as play_clip_blocking, but instead of a clip it
        sets the gait (T:5) once and re-issues the move (T:1) every tick so STATE_WALK
        stays armed and the 500 ms deadman never fires. Gaits have no duration: GUI runs
        until the window closes; pass duration_s (used headless) for a finite smoke run.
        """
        import json

        import pybullet as p

        gait_id = _GAIT_NAME_TO_ID[gait_name]  # KeyError for an unknown gait name
        self._fc.set_clock_ms(0)
        self._fc.handle_message(json.dumps({"T": 5, "g": gait_id}))

        step = 0
        imu_state = False  # latched upside-down flag, threaded across ticks
        while p.isConnected():
            t_ms = int(step * 1000.0 / 240.0)
            # Full CMD_MOVE path each tick: walk() (STATE_WALK) + processCommand(dir),
            # keeping the deadman fed — literally the app's command stream at 240 Hz.
            self._fc.handle_message(json.dumps({"T": 1, "dir": direction}))
            # Emulate the MPU6050 from PyBullet (same hysteresis as the
            # firmware), so a tilt during a gait surfaces via imuIsInverted()
            # the same way it would on hardware.
            imu_state = update_imu_from_pybullet(self._fc, p, robot_id, imu_state)
            self._fc.tick(t_ms)
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
            if step % 12 == 0:
                apply_torque_colors(p, robot_id, joint_map)
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(timestep)
            step += 1
            if duration_s is not None and t_ms >= duration_s * 1000.0:
                break


def _main():
    """CI / pre-run check:  python -m firmware_sil.sil_bridge [--check]

    Default: rebuild fh_sim if stale, then report. --check: report staleness and
    exit non-zero if stale (no build) — handy as a CI gate.
    """
    import argparse

    ap = argparse.ArgumentParser(description="fh_sim freshness check / build")
    ap.add_argument(
        "--check",
        action="store_true",
        help="only report; exit 1 if stale (do not build)",
    )
    args = ap.parse_args()

    stale, reason = staleness()
    if args.check:
        print(f"[sil] {reason}")
        sys.exit(1 if stale else 0)
    if stale:
        print(f"[sil] {reason} — building...")
        build()
        print("[sil] fh_sim rebuilt.")
    else:
        print(f"[sil] {reason}.")


if __name__ == "__main__":
    _main()
