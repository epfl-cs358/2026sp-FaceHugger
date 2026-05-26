# code/simulation/firmware_port/tests/test_clip_player.py
"""Tests for clip_player.py — pure-math tests only (no PyBullet needed).

Tests verify:
  - frame_to_joint_targets produces correct joint names and angles
  - _interpolate_frame handles boundary and interior cases
  - translate_to_servo → clamp → servo_to_radians pipeline is correct
"""

import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from firmware_port.clip_loader import (
    DEFAULT_CLIPS_H,
    get_clip_by_name,
    load_clips_all_h,
)
from firmware_port.clip_player import (
    _interpolate_frame,
    frame_to_joint_targets,
)
from firmware_port.servo_convention import (
    LEG_FR,
    NEUTRAL,
    servo_to_radians,
)


# ─── frame_to_joint_targets ─────────────────────────────────────────────────


def test_frame_to_joint_targets_key_count():
    """Must produce exactly 12 joint targets (4 legs × 3 joints)."""
    a = [0.0] * 12
    targets = frame_to_joint_targets(a)
    assert len(targets) == 12


def test_frame_to_joint_targets_joint_names():
    """All 12 expected joint names must be present."""
    a = [0.0] * 12
    targets = frame_to_joint_targets(a)
    expected_names = {
        f"{leg}{suffix}"
        for leg in ("fr", "fl", "br", "bl")
        for suffix in ("_link1_joint", "_link2_joint", "_link3_joint")
    }
    assert set(targets.keys()) == expected_names


def test_frame_to_joint_targets_neutral_fr():
    """At NEUTRAL pose FR, fr_link1_joint must be 0 rad (servo 90)."""
    # NEUTRAL for FR: sh=45, th=-60, kn=-37
    n = NEUTRAL[LEG_FR]
    # Build a[12] with FR at NEUTRAL, others at zero
    a = [n.sh, n.th, n.kn] + [0.0] * 9
    targets = frame_to_joint_targets(a)
    # FR shoulder servo: 90 + (45 - 45) = 90 → 0 rad
    assert abs(targets["fr_link1_joint"] - 0.0) < 1e-9, targets["fr_link1_joint"]


def test_frame_to_joint_targets_values_are_radians():
    """All output values must be in radian range (roughly -π/2 to +π/2)."""
    # Use NEUTRAL for all legs
    a = []
    for leg_id in range(4):
        n = NEUTRAL[leg_id]
        a += [n.sh, n.th, n.kn]
    targets = frame_to_joint_targets(a)
    for name, rad in targets.items():
        assert -math.pi <= rad <= math.pi, f"{name}={rad:.4f} out of radian range"


def test_frame_to_joint_targets_clamping_applied():
    """Values that exceed servo limits must be clamped before conversion.

    Drive FR shoulder very low (sh=-200) so hip servo would be < 38
    without clamping. With clamping, hip must equal servo_to_radians(38).
    """
    a = [-200.0, 0.0, 0.0] + [0.0] * 9
    targets = frame_to_joint_targets(a)
    # FR hip: 90 + (-200 - 45) = -155 → clamped to 38. The sim applies the
    # URDF link1 axis sign (FR = -1) to the commanded joint angle.
    expected = -servo_to_radians(38.0)
    assert abs(targets["fr_link1_joint"] - expected) < 1e-9


def test_neutral_all_hips_negative():
    """At NEUTRAL, every link2 (hip/thigh) must be negative — LEG_ID_TO_URDF_AXIS_SIGN applied."""
    a = []
    for leg_id in range(4):
        n = NEUTRAL[leg_id]
        a += [n.sh, n.th, n.kn]
    targets = frame_to_joint_targets(a)
    for leg in ("fr", "fl", "br", "bl"):
        joint = f"{leg}_link2_joint"
        assert targets[joint] < 0, (
            f"{joint}={targets[joint]:.4f} should be negative at NEUTRAL"
        )


def test_neutral_all_knees_negative():
    """At NEUTRAL, every link3 (knee) must be negative — LEG_ID_TO_URDF_AXIS_SIGN applied."""
    a = []
    for leg_id in range(4):
        n = NEUTRAL[leg_id]
        a += [n.sh, n.th, n.kn]
    targets = frame_to_joint_targets(a)
    for leg in ("fr", "fl", "br", "bl"):
        joint = f"{leg}_link3_joint"
        assert targets[joint] < 0, (
            f"{joint}={targets[joint]:.4f} should be negative at NEUTRAL"
        )


# ─── _interpolate_frame ──────────────────────────────────────────────────────


def _make_frames(timestamps_and_angles):
    from firmware_port.clip_loader import ClipFrame

    return [ClipFrame(t_ms=t, a=a) for t, a in timestamps_and_angles]


def test_interpolate_before_first_frame():
    """Before t=0 should return first frame exactly."""
    frames = _make_frames([(0, [1.0] * 12), (100, [3.0] * 12)])
    result = _interpolate_frame(frames, elapsed_ms=-10)
    assert result == [1.0] * 12


def test_interpolate_at_first_frame():
    frames = _make_frames([(0, [1.0] * 12), (100, [3.0] * 12)])
    result = _interpolate_frame(frames, elapsed_ms=0)
    assert result == [1.0] * 12


def test_interpolate_at_last_frame():
    frames = _make_frames([(0, [1.0] * 12), (100, [3.0] * 12)])
    result = _interpolate_frame(frames, elapsed_ms=100)
    assert result == [3.0] * 12


def test_interpolate_after_last_frame():
    """After clip end should hold last frame."""
    frames = _make_frames([(0, [1.0] * 12), (100, [3.0] * 12)])
    result = _interpolate_frame(frames, elapsed_ms=999)
    assert result == [3.0] * 12


def test_interpolate_midpoint():
    """At midpoint, result must be the average of both frames."""
    frames = _make_frames([(0, [0.0] * 12), (100, [10.0] * 12)])
    result = _interpolate_frame(frames, elapsed_ms=50)
    for v in result:
        assert abs(v - 5.0) < 1e-9


def test_interpolate_real_clip():
    """Interpolating within a real clip must not crash and must produce 12 values."""
    clips = load_clips_all_h(DEFAULT_CLIPS_H)
    clip = get_clip_by_name(clips, "tiny wiggle")
    result = _interpolate_frame(clip.frames, elapsed_ms=500)
    assert len(result) == 12
    assert all(isinstance(v, float) for v in result)
