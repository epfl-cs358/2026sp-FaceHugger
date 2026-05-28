"""SIL test for the runtime clip-smoothing knob (T:11, CMD_SET_SMOOTHING).

The EMA alpha that smooths clip playback is runtime-tunable over the WebSocket:
{T:11, a:<0..1>}. A low alpha plays the raw frames snappily; a high alpha lags
and smooths. This checks the knob actually changes playback (and survives an
out-of-range value via the firmware clamp). Skips if fh_sim isn't built.
"""

import json

import pytest

pytest.importorskip("pybullet")

CLIP_ID = 2  # "one leg lift", ~2042 ms


def _fc_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    from firmware_sil.sil_bridge import _compiled_so

    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim().FirmwareControl()


def _trace(alpha):
    fc = _fc_or_skip()
    fc.set_clock_ms(0)
    fc.handle_message(json.dumps({"T": 11, "a": alpha}))  # set smoothing first
    fc.handle_message(json.dumps({"T": 7, "c": CLIP_ID}))
    samples, t = [], 0
    while t <= 200 + 2042:
        fc.tick(t)
        if t % 200 == 0:
            samples.append(list(fc.servo_angles()))
        t += 4
    return samples


def test_smoothing_knob_changes_playback():
    snappy = _trace(0.0)  # no smoothing — follows raw frames
    smooth = _trace(0.9)  # heavy smoothing — laggy
    diffs = sum(1 for r0, r9 in zip(snappy, smooth) for x, y in zip(r0, r9) if x != y)
    assert diffs > 0, "smoothing alpha had no effect on clip playback"


def test_out_of_range_alpha_is_clamped_not_crashing():
    fc = _fc_or_skip()
    # 5.0 is far above the [0, 0.95] range; the firmware must clamp, not crash.
    fc.handle_message(json.dumps({"T": 11, "a": 5.0}))
    fc.set_clock_ms(0)
    fc.handle_message(json.dumps({"T": 7, "c": CLIP_ID}))
    t = 0
    while t <= 200 + 2042:
        fc.tick(t)
        t += 4
    # Reaching here without error and producing in-range servos is the assertion.
    assert all(0 <= a <= 180 for a in fc.servo_angles())
