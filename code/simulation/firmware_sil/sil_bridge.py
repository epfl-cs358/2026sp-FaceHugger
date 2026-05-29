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

import os
import re
import subprocess
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_BUILD = _HERE / "build"
_FW_SRC = (_HERE / ".." / ".." / "firmware" / "src").resolve()

# Source trees whose changes invalidate the compiled module.
_SOURCE_DIRS = [_FW_SRC, _HERE / "hal"]
_SOURCE_FILES = [_HERE / "bindings.cpp", _HERE / "CMakeLists.txt"]
_SOURCE_EXTS = {".cpp", ".c", ".h", ".hpp", ".inl", ".txt"}

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

# Firmware GaitType enum (movements.h): NONE=0, WALK=1, TROT=2, CRAB=3.
_GAIT_NAME_TO_ID = {"walk": 1, "trot": 2, "crab": 3}


def _compiled_so():
    """The built module file, or None if not present."""
    hits = sorted(_BUILD.glob("fh_sim*.so")) + sorted(_BUILD.glob("fh_sim*.pyd"))
    return hits[0] if hits else None


def _newest_source_mtime():
    """mtime of the most recently changed firmware / hal / binding source."""
    newest = 0.0
    for d in _SOURCE_DIRS:
        if d.is_dir():
            for f in d.rglob("*"):
                if f.suffix in _SOURCE_EXTS and f.is_file():
                    newest = max(newest, f.stat().st_mtime)
    for f in _SOURCE_FILES:
        if f.is_file():
            newest = max(newest, f.stat().st_mtime)
    return newest


def staleness():
    """(is_stale, reason) — is the compiled .so missing or older than its sources?"""
    so = _compiled_so()
    if so is None:
        return True, "fh_sim is not built"
    src = _newest_source_mtime()
    if src > so.stat().st_mtime:
        return True, "firmware/hal/binding sources are newer than the built fh_sim"
    return False, "fh_sim is up to date"


def build(quiet=False):
    """Configure (if needed) and compile fh_sim via CMake. Raises on failure."""
    out = subprocess.DEVNULL if quiet else None
    if not (_BUILD / "CMakeCache.txt").is_file():
        subprocess.run(
            [
                "cmake",
                "-S",
                str(_HERE),
                "-B",
                str(_BUILD),
                f"-DPython_EXECUTABLE={sys.executable}",
            ],
            check=True,
            stdout=out,
            stderr=out,
        )
    subprocess.run(
        ["cmake", "--build", str(_BUILD)], check=True, stdout=out, stderr=out
    )


def load_fh_sim(auto_build=True):
    """Import the compiled fh_sim module, rebuilding it first if it is stale.

    --sil must never run a stale firmware: by default this checks the .so against
    the firmware/hal/binding sources and, if older or missing, recompiles before
    importing. Set auto_build=False (or env FH_SIL_NO_BUILD=1) to skip the rebuild
    and instead warn loudly / error if stale.
    """
    if os.environ.get("FH_SIL_NO_BUILD"):
        auto_build = False

    stale, reason = staleness()
    if stale:
        if auto_build:
            print(f"[sil] {reason} — rebuilding fh_sim...", flush=True)
            try:
                build()
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                raise ImportError(
                    f"[sil] auto-build failed ({e}). Build it manually:\n"
                    "  cd code/simulation/firmware_sil\n"
                    "  cmake -S . -B build -DPython_EXECUTABLE=$(which python)\n"
                    "  cmake --build build\n"
                    "(needs CMake >= 3.15 + a C++17 compiler + pybind11)"
                ) from e
        else:
            print(
                f"\n*** [sil] WARNING: {reason}. The .so may not reflect the current "
                "firmware. Rebuild with `cmake --build firmware_sil/build`, or unset "
                "FH_SIL_NO_BUILD to auto-rebuild. ***\n",
                file=sys.stderr,
                flush=True,
            )

    if str(_BUILD) not in sys.path:
        sys.path.insert(0, str(_BUILD))
    try:
        import fh_sim
    except ImportError as e:
        raise ImportError(
            "fh_sim (the compiled firmware) could not be imported. Build it:\n"
            "  cd code/simulation/firmware_sil\n"
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


# ── out-of-range ([OOR]) capture ──────────────────────────────────────────────
# The firmware itself prints "[OOR] servo <channel> requested <deg>" from its
# setJointAngles / setServoAngle guards (visible on hardware too). We parse the
# lines the firmware emitted; we don't synthesize them.
_OOR_RE = re.compile(r"\[OOR\] servo (\d+) requested ([-\d.]+)")


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


# ── PyBullet -> firmware IMU emulation ────────────────────────────────────────
# The host has no MPU6050. We synthesise the firmware's "upside-down" latch from
# PyBullet's body orientation each tick and push it through the FirmwareControl
# binding, so SpinalCord sees the same imuIsInverted() it would see on hardware.
#
# Hysteresis matches code/firmware/src/brain/imu_hysteresis.h:
#   flip:  tilt > 150 deg     (body_up_z < cos(150 deg) ≈ -0.866)
#   clear: tilt <  30 deg     (body_up_z > cos( 30 deg) ≈  0.866)
# The tilt angle is acos(body_up_z) where body_up_z is the world-z component
# of the body frame's local +z axis (the rotated "up"). cos is monotone
# decreasing on [0,180] so the inequality direction reverses against the dot
# product. Boundary values (cos(150°), cos(30°)) are NOT crossed — strict
# inequalities, same as the firmware.
import math as _math  # noqa: E402  (kept private; the public re-export is `math`)

_IMU_FLIP_COS = _math.cos(_math.radians(150.0))  # ≈ -0.866
_IMU_CLEAR_COS = _math.cos(_math.radians(30.0))  # ≈  0.866


def _body_up_z(quat):
    """World-frame z-component of the body-frame +z axis, from quaternion (x,y,z,w).

    The rotation matrix's third column is the rotated +z; we only need the z
    row of that column. body_up_z == 1.0 upright, -1.0 upside-down.
    """
    x, y, z, w = quat
    return 1.0 - 2.0 * (x * x + y * y)


def imu_hysteresis_step(body_up_z, prev_state):
    """Apply the firmware hysteresis to a single body_up_z sample."""
    if prev_state:
        # currently inverted: clear only when we drop below 30° tilt
        return not (body_up_z > _IMU_CLEAR_COS)
    # currently upright: flip only when we cross 150° tilt
    return body_up_z < _IMU_FLIP_COS


def imu_pitch_roll_deg(quat):
    """(pitch_deg, roll_deg) from quaternion (x,y,z,w) using the same accel-only
    convention as the firmware: pitch = atan2(ay, sqrt(ax^2+az^2)),
    roll = atan2(-ax, az), where (ax,ay,az) is the body's local gravity vector
    (== the world -z rotated INTO the body frame). With gravity pointing -z
    in the world, the body-frame gravity is the third row of R^T == third
    column of R, negated."""
    x, y, z, w = quat
    # body-frame gravity unit vector: -R^T @ ẑ  (rows of R as columns of R^T).
    # The "body up" components (third column of R) point opposite to gravity.
    ax = 2.0 * (x * z - w * y)  # = -(-2(xz - wy)) third row of R^T col 0... see below
    ay = 2.0 * (y * z + w * x)
    az = 1.0 - 2.0 * (x * x + y * y)
    # Match firmware sign convention (pitch nose-up positive, roll right-down positive).
    pitch = _math.atan2(ay, _math.sqrt(ax * ax + az * az)) * 180.0 / _math.pi
    roll = _math.atan2(-ax, az) * 180.0 / _math.pi
    return pitch, roll


def update_imu_from_pybullet(fc, p, robot_id, prev_state):
    """Read PyBullet body orientation, hysteresise, push into firmware.

    Returns the new latched state so the caller can thread it across ticks.
    """
    _, quat = p.getBasePositionAndOrientation(robot_id)
    up_z = _body_up_z(quat)
    new_state = imu_hysteresis_step(up_z, prev_state)
    pitch, roll = imu_pitch_roll_deg(quat)
    fc.set_imu_upside_down(new_state)
    fc.set_imu_pitch_deg(pitch)
    fc.set_imu_roll_deg(roll)
    return new_state


# ── torque-based link coloring (PyBullet visual feedback) ─────────────────────
_BAND_RGBA = {
    "green": (0.2, 0.8, 0.2, 1.0),  # safe continuous hold
    "yellow": (0.9, 0.7, 0.1, 1.0),  # burst-only: over continuous, under stall
    "red": (0.9, 0.2, 0.1, 1.0),  # saturated: at/over stall, can't track
}


def torque_color(torque_nm):
    """rgba for a joint's applied torque, via the shared sim_monitor.band() — so the
    link colors mean the same thing as the --monitor [CONT]/[STALL] flags. Brief
    fast-clip spikes show yellow (honest burst); red is reserved for true saturation
    at the effort cap, not merely 'fast'."""
    from pybullet_sim import sim_monitor

    return _BAND_RGBA[sim_monitor.band(torque_nm)]


def apply_torque_colors(p, robot_id, joint_map):
    """Tint each leg link by its current applied torque (call every ~12 steps)."""
    for name, idx in joint_map.items():
        if idx < 0:
            continue
        torque = p.getJointState(robot_id, idx)[3]
        p.changeVisualShape(robot_id, idx, rgbaColor=torque_color(torque))


# ── per-step telemetry frame (the control panel's "Sim telemetry" table) ──────
# Joint order the panel renders: FL, FR, BL, BR x shoulder, hip, knee. Each joint
# carries BOTH the firmware servo-space command (0-180, what the real servos get)
# and that same command in URDF-joint degrees, so the delta vs the measured joint
# angle is a true tracking error rather than a constant ~90 offset between spaces.
_TELEM_LEGS = ("fl", "fr", "bl", "br")
_TELEM_JOINTS = (("sh", 1), ("hip", 2), ("knee", 3))  # label -> URDF link number
_SIM_NAME_TO_LEG_ID = {v: k for k, v in LEG_ID_TO_SIM_NAME.items()}


def build_telemetry_frame(fc, p, robot_id, joint_map, t_s, oor=None):
    """One telemetry frame: {t, joints:[12 x {servo/joint cmd, actual, delta, ...}]}.

    `oor` is the {pca_channel: requested_deg} from parse_oor(fc.drain_serial()) for
    this tick (the firmware's pre-clamp report); pre_clamp_deg is null where a servo
    had no [OOR] line. Reads firmware servo angles + channels from `fc`, measured
    joint state from `p` (any object exposing getJointState).
    """
    from math import degrees

    from pybullet_sim import sim_monitor

    oor = oor or {}
    angles = fc.servo_angles()
    targets = servo_angles_to_joint_targets(angles)  # urdf joint name -> radians
    channels = list(fc.servo_channels())

    joints = []
    for leg in _TELEM_LEGS:
        leg_id = _SIM_NAME_TO_LEG_ID[leg]
        for label, link in _TELEM_JOINTS:
            servo_idx = leg_id * 3 + (link - 1)
            urdf = f"{leg}_link{link}_joint"
            commanded_servo = float(angles[servo_idx])
            commanded_joint = degrees(targets[urdf])
            idx = joint_map.get(urdf)
            if idx is not None and idx >= 0:
                state = p.getJointState(robot_id, idx)
                actual_joint = degrees(state[0])
                torque = state[3]
                delta = commanded_joint - actual_joint
            else:
                actual_joint = delta = None
                torque = 0.0
            joints.append(
                {
                    "name": f"{leg}_{label}",
                    "commanded_servo_deg": round(commanded_servo, 2),
                    "commanded_joint_deg": round(commanded_joint, 2),
                    "actual_joint_deg": None
                    if actual_joint is None
                    else round(actual_joint, 2),
                    "delta_deg": None if delta is None else round(delta, 2),
                    "torque_nm": round(torque, 4),
                    "current_a": round(sim_monitor.estimate_current_a(torque), 3),
                    "pre_clamp_deg": oor.get(channels[servo_idx]),
                }
            )
    return {"t": round(t_s, 4), "joints": joints}


def trace_clip(fc, clip_name, record_every=24, step_hz=240, preroll_ms=200):
    """Deterministic servo-angle trace of a clip through the firmware.

    Ticks the firmware at `step_hz` over the clip's duration plus `preroll_ms`
    (t_ms = step*1000/hz) and records the 12 servo angles (whole degrees, as the
    robot receives them) every `record_every` steps. The window includes the clip
    pre-roll (the eased glide from the live pose into frame 0 that playClip starts
    with), so the full motion is captured. Used by both the golden generator and
    the clip suite, so they tick the *identical* sequence — the trace is a pure
    function of the firmware code + the clip data, making any servo-angle change
    detectable. `preroll_ms` must match the firmware CLIP_PREROLL_MS.

    Returns: list of [t_ms, [12 int degrees]].
    """
    cid = fc.clip_id_by_name(clip_name)
    if cid < 0:
        raise KeyError(f"clip {clip_name!r} not found")
    duration_ms = fc.clip_duration_ms(cid)
    end_ms = duration_ms + preroll_ms
    fc.set_clock_ms(0)
    fc.play_clip(cid)
    samples = []
    step = 0
    while True:
        t_ms = int(step * 1000.0 / step_hz)
        fc.tick(t_ms)
        if step % record_every == 0:
            samples.append([t_ms, [int(round(a)) for a in fc.servo_angles()]])
        if t_ms >= end_ms:
            break
        step += 1
    return samples


def trace_gait(fc, gait_name, direction="FW", steps=480, step_hz=240, record_every=24):
    """Deterministic servo-angle trace of a firmware gait through the SIL.

    Drives the gait the way the app/robot does: set the gait once (T:5), then
    RE-ISSUE the move (T:1) every tick — the full CMD_MOVE path arms STATE_WALK and
    resets the 500 ms deadman, so the gait keeps running instead of graceful-stopping.
    Records the 12 servo angles (whole degrees) every `record_every` steps. Shared
    driving sequence with FirmwareSILDriver.run_gait_blocking, minus PyBullet.

    Returns: list of [t_ms, [12 int degrees]]. Raises KeyError for an unknown gait.
    """
    import json

    gait_id = _GAIT_NAME_TO_ID[gait_name]  # KeyError for an unknown gait name
    fc.set_clock_ms(0)
    fc.handle_message(json.dumps({"T": 5, "g": gait_id}))
    samples = []
    for step in range(steps):
        t_ms = int(step * 1000.0 / step_hz)
        fc.handle_message(json.dumps({"T": 1, "dir": direction}))  # beat the deadman
        fc.tick(t_ms)
        if step % record_every == 0:
            samples.append([t_ms, [int(round(a)) for a in fc.servo_angles()]])
    return samples


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
        """Run a firmware GAIT through PyBullet at 240 Hz (the exact tickGait/tickTrot).

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
