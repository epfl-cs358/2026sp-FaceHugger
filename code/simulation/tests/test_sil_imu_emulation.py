"""SIL tests for the PyBullet -> firmware IMU emulation.

The host has no MPU6050; sil_bridge synthesises the firmware's upside-down latch
from PyBullet's body orientation each tick with the same hysteresis as the
firmware (flip > 150°, clear < 30°). These tests assert the emulation feeds the
firmware's sensors.h accessor (imuIsInverted) correctly, plus the pure helpers
(hysteresis + body_up_z + pitch/roll) match the firmware convention.

Skips if fh_sim isn't built / pybullet isn't installed.
"""

import math

import pytest

pytest.importorskip("pybullet")


def _fc_or_skip():
    try:
        from firmware_sil.sil_bridge import _compiled_so, load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim().FirmwareControl()


# ── pure helpers (no PyBullet, no fh_sim) ─────────────────────────────────────


def test_body_up_z_identity_quaternion_is_upright():
    from firmware_sil.sil_bridge import _body_up_z

    # Identity quaternion (x,y,z,w) = (0,0,0,1): body +Z aligned with world +Z.
    assert _body_up_z((0.0, 0.0, 0.0, 1.0)) == pytest.approx(1.0, abs=1e-9)


def test_body_up_z_180_roll_is_upside_down():
    from firmware_sil.sil_bridge import _body_up_z

    # 180° rotation about X-axis: q = (sin(π/2), 0, 0, cos(π/2)) = (1,0,0,0).
    assert _body_up_z((1.0, 0.0, 0.0, 0.0)) == pytest.approx(-1.0, abs=1e-9)


def test_hysteresis_matches_firmware_thresholds():
    from firmware_sil.sil_bridge import imu_hysteresis_step

    # Upright (up_z = +1, tilt = 0): below 150° threshold, never flips.
    assert imu_hysteresis_step(1.0, False) is False
    # Sideways (up_z ≈ 0, tilt = 90°): inside the dead zone; state is held.
    assert imu_hysteresis_step(0.0, False) is False
    assert imu_hysteresis_step(0.0, True) is True
    # Upside-down (up_z = -1, tilt = 180°): crosses 150° -> flips.
    assert imu_hysteresis_step(-1.0, False) is True
    # Already inverted, comes back upright (tilt < 30°): clears.
    assert imu_hysteresis_step(1.0, True) is False
    # Boundary: cos(150°) ≈ -0.866. The firmware uses strict > 150°, so
    # exactly at the threshold the state is held (no flip).
    cos_150 = math.cos(math.radians(150.0))
    assert imu_hysteresis_step(cos_150, False) is False
    # And exactly at 30°, the latch holds (no clear).
    cos_30 = math.cos(math.radians(30.0))
    assert imu_hysteresis_step(cos_30, True) is True


# ── end-to-end: PyBullet pose -> bridge -> firmware ───────────────────────────


def test_inverted_pose_drives_firmware_imuIsInverted():
    """180° roll body, repeated bridge updates, fc.imu_is_inverted() flips True."""
    import pybullet as p

    from firmware_sil.sil_bridge import update_imu_from_pybullet
    from pybullet_sim import paths
    from pybullet_sim.motor import build_joint_map

    fc = _fc_or_skip()
    cid = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(paths.URDF_PATH, basePosition=[0, 0, 0.1], useFixedBase=True)
        _ = build_joint_map(robot)

        # Flip the body 180° about X (upside-down).
        p.resetBasePositionAndOrientation(
            robot, [0, 0, 0.1], p.getQuaternionFromEuler([math.pi, 0, 0])
        )
        imu_state = False
        for _ in range(5):  # a handful of bridge updates + ticks
            imu_state = update_imu_from_pybullet(fc, p, robot, imu_state)
            fc.tick(0)
        assert imu_state is True, "hysteresis didn't latch inverted"
        assert fc.imu_is_inverted() is True, (
            "firmware imuIsInverted() didn't see the bridge-written state"
        )

        # And reverse: back to upright should clear the latch.
        p.resetBasePositionAndOrientation(
            robot, [0, 0, 0.1], p.getQuaternionFromEuler([0, 0, 0])
        )
        for _ in range(5):
            imu_state = update_imu_from_pybullet(fc, p, robot, imu_state)
            fc.tick(0)
        assert imu_state is False
        assert fc.imu_is_inverted() is False
    finally:
        p.disconnect(cid)


def test_set_imu_upside_down_binding_round_trips():
    """The minimal binding contract: set_imu_upside_down + imu_is_inverted match."""
    fc = _fc_or_skip()
    fc.set_imu_upside_down(True)
    assert fc.imu_is_inverted() is True
    fc.set_imu_upside_down(False)
    assert fc.imu_is_inverted() is False
