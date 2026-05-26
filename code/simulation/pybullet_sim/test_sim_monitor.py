"""Unit tests for sim_monitor pure helpers (no PyBullet)."""

from pybullet_sim import sim_monitor as m


def test_estimate_current_at_stall_is_stall_current():
    assert abs(m.estimate_current_a(m.STALL_TORQUE_NM) - m.STALL_CURRENT_A) < 1e-9


def test_estimate_current_is_linear_and_uses_abs():
    half = m.estimate_current_a(m.STALL_TORQUE_NM / 2)
    assert abs(half - m.STALL_CURRENT_A / 2) < 1e-9
    assert m.estimate_current_a(-1.0) == m.estimate_current_a(1.0)


def test_total_current_sums_all_channels():
    torques = [m.STALL_TORQUE_NM] * 4  # 4 servos at full stall -> 4 * STALL_CURRENT_A
    assert abs(m.total_current_a(torques) - 4 * m.STALL_CURRENT_A) < 1e-9


def test_format_status_flags_overcurrent_and_stall():
    # 12 joints all at stall torque -> 12 * STALL_CURRENT_A total (> 10 A budget),
    # and all above STALL_WARN_NM -> all stalled.
    torques = {f"j{i}": m.STALL_TORQUE_NM for i in range(12)}
    line = m.format_status(1.0, torques)
    assert "[WARN >10A]" in line
    assert "[STALL]" in line
    assert f"est_I={12 * m.STALL_CURRENT_A:.1f}A" in line


def test_format_status_quiet_when_within_limits():
    torques = {f"j{i}": 0.1 for i in range(12)}  # tiny torques
    line = m.format_status(0.0, torques)
    assert "[WARN" not in line
    assert "[STALL]" not in line


def test_format_status_includes_per_leg_angles_when_given():
    torques = {"fr_link2_joint": 1.0}
    pos = {
        "fr_link1_joint": 0.0,
        "fr_link2_joint": -41.0,
        "fr_link3_joint": -57.0,
    }
    line = m.format_status(2.3, torques, pos)
    assert "FR:sh=+0 th=-41 kn=-57" in line
