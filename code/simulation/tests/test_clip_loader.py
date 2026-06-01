# code/simulation/tests/test_clip_loader.py
"""Tests for clip_loader.py.

Uses the real animation/exported_clips/clips_all.h as test fixture —
it's the exact file the firmware is built from, so if we can parse it
correctly, we can validate clips against the firmware.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from firmware_port.clip_loader import (
    DEFAULT_CLIPS_H,
    get_clip_by_name,
)

# `all_clips`, `manifest`, and `sample_clip` are provided by conftest.py so
# test_clip_player.py can share them too.


def test_clips_all_h_exists():
    assert DEFAULT_CLIPS_H.exists(), f"clips_all.h not found at {DEFAULT_CLIPS_H}"


def test_load_matches_manifest_count(all_clips, manifest):
    """Parsed clip count must equal clips_manifest.json."""
    assert len(all_clips) == len(manifest)


def test_clip_names(all_clips, manifest):
    """Clip names + order must match clips_manifest.json."""
    assert [c.name for c in all_clips] == [m["name"] for m in manifest]


def test_clip_frame_counts(all_clips, manifest):
    """Frame counts must match clips_manifest.json."""
    expected = {m["name"]: m["frame_count"] for m in manifest}
    for clip in all_clips:
        assert clip.frame_count == expected[clip.name], clip.name
        assert len(clip.frames) == expected[clip.name], clip.name


def test_clip_duration_ms(all_clips, manifest):
    """Duration must match clips_manifest.json."""
    expected = {m["name"]: m["duration_ms"] for m in manifest}
    for clip in all_clips:
        assert clip.duration_ms == expected[clip.name], clip.name


def test_frame_has_12_values(all_clips):
    """Every frame must have exactly 12 float values."""
    for clip in all_clips:
        for frame in clip.frames:
            assert len(frame.a) == 12, f"{clip.name} frame@{frame.t_ms}"


def test_first_frame_timestamp_is_zero(all_clips):
    """All clips must start at t_ms=0."""
    for clip in all_clips:
        assert clip.frames[0].t_ms == 0, clip.name


def test_frames_in_ascending_order(all_clips):
    """Frames must be in strictly ascending t_ms order."""
    for clip in all_clips:
        for i in range(1, len(clip.frames)):
            assert clip.frames[i].t_ms > clip.frames[i - 1].t_ms, (
                f"{clip.name} frame {i}: t_ms not ascending"
            )


def test_first_frame_lie_down_fr_shoulder(all_clips):
    """Spot-check FR shoulder first frame of 'lie down and stand up' — the
    historical regression value (~45.71 = FR neutral 45 plus the rest-yaw
    residual under the uniform-math-space yaw convention). If that clip
    has been retired from the catalog, this regression-guard skips rather
    than failing, since the value is meaningless without that specific clip.
    See docs/CLIP_SHOULDER_CONVENTION.md."""
    try:
        clip = get_clip_by_name(all_clips, "lie down and stand up")
    except KeyError:
        pytest.skip("'lie down and stand up' is no longer in the clip catalog")
    assert abs(clip.frames[0].a[0] - 45.7052) < 0.01


def test_get_clip_by_name_case_insensitive(all_clips, sample_clip):
    """Lookup is case-insensitive — exercise against an arbitrary real clip
    rather than a hardcoded name (catalog churns as animations are authored)."""
    name = sample_clip.name
    assert get_clip_by_name(all_clips, name.upper()).name == name
    assert get_clip_by_name(all_clips, name.lower()).name == name


def test_get_clip_by_name_not_found(all_clips):
    with pytest.raises(KeyError):
        get_clip_by_name(all_clips, "nonexistent clip")
