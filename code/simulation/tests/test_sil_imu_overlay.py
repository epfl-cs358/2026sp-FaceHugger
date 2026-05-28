"""Unit tests for the PyBullet GUI live IMU overlay text helper.

The overlay itself (addUserDebugText calls) is GUI-only visual feedback that
can't be reasonably tested without a real PyBullet window. What we *can* test
is the pure formatter that builds the multi-line readout string — that's the
piece most likely to silently drift if someone tweaks formatting.
"""

from firmware_sil.ws_sim import format_imu_overlay


def test_format_imu_overlay_upright():
    text = format_imu_overlay(12.3, -5.7, False)
    lines = text.split("\n")
    assert len(lines) == 2
    # Pitch / Roll on the first line, signed, one decimal place.
    assert "Pitch:" in lines[0]
    assert "Roll:" in lines[0]
    assert "+12.3" in lines[0]
    assert "-5.7" in lines[0]
    assert "°" in lines[0]
    # upside_down: false on the second line when upright.
    assert lines[1] == "upside_down: false"


def test_format_imu_overlay_inverted():
    text = format_imu_overlay(0.0, 0.0, True)
    lines = text.split("\n")
    assert lines[1] == "upside_down: true"


def test_format_imu_overlay_signs():
    # Positive pitch must show a leading '+'; negative pitch shows '-'.
    pos = format_imu_overlay(45.0, 0.0, False)
    neg = format_imu_overlay(-45.0, 0.0, False)
    assert "+45.0" in pos
    assert "-45.0" in neg


def test_format_imu_overlay_zero_is_signed():
    # The %+ format on 0.0 yields '+0.0' — keeps the column width stable as the
    # robot wobbles around level. Worth pinning so a future refactor doesn't
    # silently drop the sign.
    text = format_imu_overlay(0.0, 0.0, False)
    assert "+0.0" in text
