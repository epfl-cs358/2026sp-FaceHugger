"""SIL tests for invert behaviour + state-machine landing (drives the EXACT firmware).

Regression guards for the two root causes fixed in
.work/plans/2026-05-26-invert-and-state-machine.md:

- ROOT CAUSE A: stopping a gait while inverted must HOLD the mirrored neutral and
  land in STATE_STAND — not snap to the upright default pose in inert STATE_IDLE.
- ROOT CAUSE B: T:6 is a toggle; a duplicate within the debounce window must not
  net a double-flip.

Runs against the compiled firmware (fh_sim); skips if it isn't built.
"""

import pytest

pytest.importorskip("pybullet")

STATE_WALK = 1
STATE_STAND = 5


def _fc_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim().FirmwareControl()


def _neutral_servo_angles(inverted):
    """The 12 servo angles the SIL itself holds at neutral, optionally inverted.

    Queries a fresh FirmwareControl instance so the result is whatever the
    *compiled* SIL considers neutral — independent of which CALIB header the
    SIL was built against (calib.h vs calib_sim.h). This keeps the test
    asserting a SIL behavioural property (gait STOP lands in the inverted
    neutral) without coupling it to the Python re-port's CALIB table.
    """
    from firmware_sil.sil_bridge import load_fh_sim

    helper = load_fh_sim().FirmwareControl()
    if inverted:
        helper.invert_robot()
    helper.stand()
    helper.tick(0)
    return [round(a) for a in helper.servo_angles()]


def test_gait_stop_while_inverted_holds_mirrored_neutral_and_stands():
    fc = _fc_or_skip()
    fc.set_gait(2)  # GAIT_TROT
    fc.walk()  # STATE_WALK

    t = 0
    for _ in range(240):  # ~1 s of walking, re-issuing the move each tick
        fc.process_command("FW")
        fc.tick(t)
        t += 4
    assert fc.robot_state() == STATE_WALK, "should be walking before the stop"

    fc.invert_robot()  # flip mid-walk; robot keeps walking, mirrored

    fc.process_command("STOP")
    landed = False
    for _ in range(4000):  # let the deadman + graceful-stop fire on a phase boundary
        fc.tick(t)
        t += 4
        if fc.robot_state() != STATE_WALK:
            landed = True
            break
    assert landed, "gait never reached its graceful stop"

    # Root Cause A: must hold the INVERTED neutral, in STATE_STAND (not upright/IDLE).
    assert fc.robot_state() == STATE_STAND
    got = [round(a) for a in fc.servo_angles()]
    assert got == _neutral_servo_angles(inverted=True)


def test_invert_toggle_debounced_against_double_fire():
    fc = _fc_or_skip()

    fc.set_clock_ms(1000)
    fc.invert_robot()  # accepted -> inverted
    fc.set_clock_ms(1100)
    fc.invert_robot()  # +100 ms < debounce window -> ignored (no net second flip)

    fc.stand()  # writes neutral through applyServos -> mirrored iff still inverted
    got = [round(a) for a in fc.servo_angles()]
    assert got == _neutral_servo_angles(inverted=True), (
        "duplicate T:6 within the debounce window double-flipped back to upright"
    )

    fc.set_clock_ms(2000)  # well past the window
    fc.invert_robot()  # accepted -> back to upright
    fc.stand()
    got = [round(a) for a in fc.servo_angles()]
    assert got == _neutral_servo_angles(inverted=False)
