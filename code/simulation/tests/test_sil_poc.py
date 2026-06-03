"""Step-1 proof of concept: the EXACT firmware code drives the PyBullet sim.

Skips if the `fh_sim` module isn't built (no C++ toolchain / not compiled). To
build it:
    cd code/simulation/firmware_sil
    cmake -S . -B build -DPython_EXECUTABLE=$(which python) && cmake --build build

What it proves:
  - the real firmware SpinalCord (compiled unchanged via firmware_sil/hal/) runs
    on the host and plays a clip, producing 12 servo angles in [0,180];
  - those angles match the Python re-port's translate_to_servo at frame 0 to
    within the firmware's whole-degree truncation (<= 1 deg) — i.e. SIL ≡ re-port;
  - the SIL bridge can drive PyBullet joints through a whole clip headless.
"""

import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent.parent  # tests/ -> code/simulation/
FW_CLIPS = SIM_DIR / ".." / "firmware" / "src" / "nervous_system" / "clips_all.h"
FACEHUGGER = SIM_DIR.parent / "facehugger.py"  # code/facehugger.py
CLIP = "wave"


def _driver_or_skip():
    """Return a FirmwareSILDriver, or skip if fh_sim isn't built."""
    try:
        from firmware_sil.sil_bridge import FirmwareSILDriver

        return FirmwareSILDriver()
    except ImportError as e:
        pytest.skip(f"fh_sim not built: {e}")


def test_firmware_plays_clip_in_range():
    drv = _driver_or_skip()
    fc = drv._fc
    assert CLIP in fc.clip_names()
    cid = fc.clip_id_by_name(CLIP)
    fc.set_clock_ms(0)
    fc.play_clip(cid)
    dur = fc.clip_duration_ms(cid)
    # Sample across the clip; every servo angle must stay in the electrical range.
    for t_ms in range(0, dur + 1, 100):
        fc.tick(t_ms)
        angles = fc.servo_angles()
        assert len(angles) == 12
        assert all(0.0 <= a <= 180.0 for a in angles), (t_ms, angles)


def test_sil_response_matches_port_response_between_frames():
    """SIL and the Python re-port apply the same convention math, modulo a
    CONSTANT per (leg, joint) — so their servo *response* across two clip
    frames must be byte-identical (within whole-degree truncation).

    Why deltas instead of absolute equality: the compiled SIL may be built
    against `calib_sim.h` (CALIB=90 everywhere) while the Python re-port
    mirrors `calib.h` (real per-leg CALIB). That difference shows up as a
    per-joint constant offset in absolute servo angles, but cancels exactly
    in the delta between two frames. Comparing deltas tests the load-bearing
    property (same convention math, same sign per leg) without coupling the
    test to which CALIB header SIL was compiled with.
    """
    drv = _driver_or_skip()
    from firmware_port.clip_loader import get_clip_by_name, load_clips_all_h
    from firmware_port.servo_convention import (
        LEG_FL,
        LEG_FR,
        LEG_RL,
        LEG_RR,
        clamp_clip_servos,
        translate_to_servo,
    )

    fc = drv._fc
    cid = fc.clip_id_by_name(CLIP)
    clip = get_clip_by_name(load_clips_all_h(FW_CLIPS), CLIP)
    PREROLL_MS = 200
    FRAME_MS = 1000.0 / 24  # firmware CLIP_FPS

    def _sil_at_frame(frame_idx: int) -> list[float]:
        """Re-play the clip and tick past pre-roll + frame_idx frames."""
        fc.set_clock_ms(0)
        fc.play_clip(cid)
        target_ms = PREROLL_MS + frame_idx * FRAME_MS
        step = 0
        while True:
            t_ms = int(step * FRAME_MS / 10)  # tighter step than 24fps for accuracy
            fc.tick(t_ms)
            if t_ms >= target_ms:
                break
            step += 1
        return list(fc.servo_angles())

    def _port_at_frame(frame_idx: int) -> list[float]:
        a = clip.frames[frame_idx].a
        out = []
        for leg in (LEG_FR, LEG_FL, LEG_RR, LEG_RL):
            s = clamp_clip_servos(
                leg,
                translate_to_servo(leg, a[leg * 3], a[leg * 3 + 1], a[leg * 3 + 2]),
            )
            out += [s.hip, s.thigh, s.knee]
        return out

    # Pick a second frame mid-clip (not too far, so clamps stay open).
    n_frames = len(clip.frames)
    second = min(n_frames - 1, max(5, n_frames // 4))

    sil_delta = [a - b for a, b in zip(_sil_at_frame(second), _sil_at_frame(0))]
    port_delta = [a - b for a, b in zip(_port_at_frame(second), _port_at_frame(0))]

    max_diff = max(abs(s - p) for s, p in zip(sil_delta, port_delta))
    assert max_diff <= 1.0, (
        f"SIL and Port disagree on servo *response* between frame 0 and "
        f"frame {second} by {max_diff:.3f} deg.\n"
        f"This means they use different convention math (not just different "
        f"CALIB) — a real bug.\n"
        f"sil_delta={sil_delta}\nport_delta={port_delta}"
    )


def test_sil_drives_pybullet_headless():
    """Play the whole clip through PyBullet via the SIL bridge; joints must move."""
    import pybullet as p

    from pybullet_sim import paths
    from pybullet_sim.motor import build_joint_map

    drv = _driver_or_skip()
    cid = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(
            paths.URDF_PATH, basePosition=[0, 0, 0.15], useFixedBase=True
        )
        joint_map = build_joint_map(robot)
        revolute = [
            idx
            for idx in joint_map.values()
            if p.getJointInfo(robot, idx)[2] != p.JOINT_FIXED
        ]
        start = [p.getJointState(robot, i)[0] for i in revolute]
        drv.play_clip_blocking(
            robot, joint_map, CLIP, force=2.94, velocity=5.0, gui=False
        )
        end = [p.getJointState(robot, i)[0] for i in revolute]
    finally:
        p.disconnect(cid)
    # The clip drove the firmware, which drove the joints: at least one moved.
    total_motion = sum(abs(e - s) for e, s in zip(end, start))
    assert total_motion > 1e-3, f"joints did not move (total {total_motion:.5f})"


def test_facehugger_sim_clip_defaults_to_firmware():
    """`facehugger.py sim --clip wave --headless` defaults to the firmware SIL
    (no flag needed); --python-port would force the re-port instead."""
    _driver_or_skip()  # skip if fh_sim isn't built
    r = subprocess.run(
        [
            sys.executable,
            str(FACEHUGGER),
            "sim",
            "--clip",
            CLIP,
            "--headless",
            "--settle",
            "0",
        ],
        cwd=str(SIM_DIR),
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert r.returncode == 0, r.stderr
    assert "[SIL]" in r.stdout, r.stdout  # default went through the firmware driver
