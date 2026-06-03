"""SIL test for clip looping (T:7 with "loop": true).

A one-shot clip returns to the neutral stand (STATE_STAND) after its duration;
a looping clip keeps replaying (stays in STATE_ACTION) until another motion
command preempts it. Runs against the compiled firmware (fh_sim); skips if it
isn't built or if the target clip is no longer compiled in.
"""

import json
import warnings

import pytest

pytest.importorskip("pybullet")

STATE_ACTION = 2
STATE_STAND = 5

# Name-based lookup — robust to clip list reorders as the library evolves.
# If this clip is removed from the firmware, the test skips with a warning
# rather than failing.
CLIP_NAME = "one leg lift"


def _fc_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim().FirmwareControl()


def _find_clip(fc):
    """Return (id, ms) for CLIP_NAME from the firmware's live clip list, or None."""
    resp = fc.handle_message(json.dumps({"T": 8}))
    for c in json.loads(resp)["clips"]:
        if c["name"] == CLIP_NAME:
            return c["id"], c["ms"]
    return None


def _play_and_settle(loop):
    """Play CLIP_NAME (optionally looping) and tick ~1.5 s past its end."""
    fc = _fc_or_skip()
    clip = _find_clip(fc)
    if clip is None:
        warnings.warn(
            f"clip '{CLIP_NAME}' is not compiled into the current firmware build; "
            "test skipped. Re-export and rebuild to include it.",
            UserWarning,
            stacklevel=2,
        )
        pytest.skip(f"clip '{CLIP_NAME}' not compiled in")
    clip_id, dur_ms = clip
    fc.set_clock_ms(0)
    fc.handle_message(json.dumps({"T": 7, "c": clip_id, "loop": loop}))
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
