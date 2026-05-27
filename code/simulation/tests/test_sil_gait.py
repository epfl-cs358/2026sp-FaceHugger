"""SIL gait tests — the firmware gait (tickGait/tickTrot) driven in the sim.

Locks the firmware-gait driving sequence used by FirmwareSILDriver.run_gait_blocking:
set the gait (T:5), then re-issue the move (T:1) every tick so the 500 ms deadman
never fires and STATE_WALK stays armed. Verifies the EXACT firmware actually walks
(servo angles sweep, in range) — the source-of-truth the sim's --walk/--trot now use.

Runs against the compiled firmware (fh_sim); skips if it isn't built.
"""

import pytest

pytest.importorskip("pybullet")

STATE_WALK = 1


def _fc_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim().FirmwareControl()


@pytest.mark.parametrize("gait_name", ["walk", "trot"])
def test_firmware_gait_runs_and_sweeps_the_legs(gait_name):
    from firmware_sil.sil_bridge import trace_gait

    fc = _fc_or_skip()
    samples = trace_gait(fc, gait_name, direction="FW", steps=480)  # ~2 s @ 240 Hz

    assert samples, "trace_gait produced no samples"
    # The gait keeps STATE_WALK armed (move re-issued each tick beats the deadman).
    assert fc.robot_state() == STATE_WALK

    # Every commanded servo angle stays in the electrical range.
    for t_ms, angles in samples:
        assert len(angles) == 12
        assert all(0 <= a <= 180 for a in angles), f"out of range @ {t_ms}ms: {angles}"

    # The legs actually move: at least one servo sweeps a meaningful arc over the
    # cycle (a static/neutral hold would have ~zero range on every channel).
    per_servo_range = [
        max(s[1][i] for s in samples) - min(s[1][i] for s in samples) for i in range(12)
    ]
    assert max(per_servo_range) > 5, (
        f"{gait_name}: no servo swept >5° — gait not running? ranges={per_servo_range}"
    )


# Servo index of each leg's shoulder (hip) in the firmware's 12-angle vector
# (leg_id*3, firmware order FR,FL,BR,BL).
_HIP = {"FR": 0, "FL": 3, "BR": 6, "BL": 9}


def test_trot_rear_shoulders_mirror_for_straight_walk():
    """Straight forward trot: the two rear shoulders must mirror each other.

    BR(RR) and BL(RL) are an anti-phase pair (offsets 0.0 / 0.5), so at a MATCHED
    stride phase their shoulder deviation from neutral (90) must be opposite-signed
    — otherwise both rear feet sweep the same rotational way and the back veers.
    This regressed when BR was un-mirrored (2026-05-25): translateToServo flipped
    but tickTrot's rear sweep wasn't, so both swept the same way. PERIOD_S=1.5s, so
    samples 750 ms apart are half a cycle = matched leg phase.
    """
    from firmware_sil.sil_bridge import trace_gait

    fc = _fc_or_skip()
    # record_every=180 @240Hz = 750 ms = half the 1.5 s trot period.
    samples = trace_gait(fc, "trot", direction="FW", steps=900, record_every=180)
    # Skip the first sample (gait just armed, motion still ramping).
    pairs = [
        (samples[i], samples[i + 1])
        for i in range(1, len(samples) - 1)
        if samples[i + 1][0] - samples[i][0] == 750
    ]
    assert pairs, "need samples 750 ms apart to compare matched stride phases"
    for (t0, a0), (t1, a1) in pairs:
        dev_br = a0[_HIP["BR"]] - 90
        dev_bl_next = a1[_HIP["BL"]] - 90  # BL half a cycle later = BR's leg phase
        assert abs(dev_br + dev_bl_next) <= 4, (
            f"rear shoulders not mirrored: BR@{t0}ms dev={dev_br:+d}, "
            f"BL@{t1}ms dev={dev_bl_next:+d} (expected opposite)"
        )


def test_trot_front_left_anchored_near_neutral():
    """FL front shoulder must sit at/above neutral (90) through the trot, not be
    pulled 25-60° inward. Change B remapped FL's servo neutral 75->90 (horn
    remount) but left tickTrot's HIP table stale, parking FL at servo ~30-65. The
    fix re-anchors the old sweep delta to the new neutral, so FL stays >= ~90.
    """
    from firmware_sil.sil_bridge import trace_gait

    fc = _fc_or_skip()
    samples = trace_gait(fc, "trot", direction="FW", steps=480)
    fl_hip = [a[_HIP["FL"]] for _t, a in samples]
    assert min(fl_hip) >= 88, (
        f"FL shoulder pulled off neutral: min servo {min(fl_hip)} "
        f"(range [{min(fl_hip)}, {max(fl_hip)}]); expected anchored near 90"
    )


# Servo index of each leg's thigh in the 12-angle vector (leg_id*3 + 1).
_THIGH = {"FR": 1, "FL": 4, "BR": 7, "BL": 10}


def test_trot_sideways_engages_the_thighs():
    """A sideways trot command adds a lateral thigh sweep on top of the forward
    motion, so the thighs sweep noticeably more than during a forward trot (where
    the thigh only moves for foot lift). Confirms the activeX/crab path in tickTrot
    is wired; with activeX==0 the forward trot is unchanged (see the mirror test).
    """
    from firmware_sil.sil_bridge import trace_gait

    def max_thigh_range(direction):
        fc = _fc_or_skip()
        s = trace_gait(fc, "trot", direction=direction, steps=720, record_every=30)
        return max(
            max(x[1][i] for x in s) - min(x[1][i] for x in s) for i in _THIGH.values()
        )

    fwd = max_thigh_range("FW")
    side = max_thigh_range("R")
    assert side > fwd + 5, (
        f"sideways thigh sweep ({side}) not greater than forward ({fwd}); "
        f"lateral (activeX) path not engaging"
    )


def test_unknown_gait_name_rejected():
    from firmware_sil.sil_bridge import trace_gait

    fc = _fc_or_skip()
    with pytest.raises(KeyError):
        trace_gait(fc, "moonwalk")
