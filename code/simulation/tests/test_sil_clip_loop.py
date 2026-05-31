"""SIL test for clip looping (T:7 with "loop": true).

A one-shot clip returns to the neutral stand (STATE_STAND) after its duration;
a looping clip keeps replaying (stays in STATE_ACTION) until another motion
command preempts it. Runs against the compiled firmware (fh_sim); skips if it
isn't built.
"""

import json

import pytest

pytest.importorskip("pybullet")

STATE_ACTION = 2
STATE_STAND = 5
CLIP_ID = 6  # "one leg lift", ~2042 ms — a short clip keeps the test quick


def _fc_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim().FirmwareControl()


def _play_and_settle(loop):
    """Play clip CLIP_ID (optionally looping) and tick ~1.5 s past its end."""
    fc = _fc_or_skip()
    fc.set_clock_ms(0)
    fc.handle_message(json.dumps({"T": 7, "c": CLIP_ID, "loop": loop}))
    dur_ms = 2042
    end = 200 + dur_ms + 1500  # preroll + duration + margin past the return ease
    t = 0
    while t <= end:
        fc.tick(t)
        t += 4
    return fc.robot_state()


def test_one_shot_clip_returns_to_stand():
    assert _play_and_settle(loop=False) == STATE_STAND


def test_looping_clip_keeps_playing_past_its_duration():
    assert _play_and_settle(loop=True) == STATE_ACTION
