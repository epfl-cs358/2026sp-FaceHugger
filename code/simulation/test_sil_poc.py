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

from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent
FW_CLIPS = SIM_DIR / ".." / "firmware" / "src" / "nervous_system" / "clips_all.h"
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


def test_sil_matches_python_report_at_frame0():
    """SIL servo angles at t=0 == re-port translate_to_servo(frame0), within the
    firmware's whole-degree truncation. Uses the firmware's OWN clips_all.h on
    both sides so the comparison is pure convention-math (C++ vs Python port)."""
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
    fc.set_clock_ms(0)
    fc.play_clip(cid)
    fc.tick(0)
    sil = list(fc.servo_angles())

    a = get_clip_by_name(load_clips_all_h(FW_CLIPS), CLIP).frames[0].a
    port = []
    for leg in (LEG_FR, LEG_FL, LEG_RR, LEG_RL):
        s = clamp_clip_servos(
            translate_to_servo(leg, a[leg * 3], a[leg * 3 + 1], a[leg * 3 + 2])
        )
        port += [s.hip, s.thigh, s.knee]

    max_diff = max(abs(s - p) for s, p in zip(sil, port))
    assert max_diff <= 1.0, (
        f"SIL vs re-port differ by {max_diff:.3f} deg\nSIL={sil}\nport={port}"
    )


def test_sil_drives_pybullet_headless():
    """Play the whole clip through PyBullet via the SIL bridge; joints must move."""
    import pybullet as p

    from pybullet_sim import constants, helpers

    drv = _driver_or_skip()
    cid = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(
            constants.URDF_PATH, basePosition=[0, 0, 0.15], useFixedBase=True
        )
        joint_map = helpers.build_joint_map(robot)
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
