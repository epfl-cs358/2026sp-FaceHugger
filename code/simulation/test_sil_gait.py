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


def test_unknown_gait_name_rejected():
    from firmware_sil.sil_bridge import trace_gait

    fc = _fc_or_skip()
    with pytest.raises(KeyError):
        trace_gait(fc, "moonwalk")
