# code/simulation/pybullet_sim/interpreter/tests/test_clip_loader.py
"""Tests for clip_loader.py.

Uses the real animation/exported_clips/clips_all.h as test fixture —
it's the exact file the firmware is built from, so if we can parse it
correctly, we can validate clips against the firmware.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from pybullet_sim.interpreter.clip_loader import (
    DEFAULT_CLIPS_H,
    get_clip_by_name,
    load_clips_all_h,
)


@pytest.fixture(scope="module")
def all_clips():
    return load_clips_all_h(DEFAULT_CLIPS_H)


@pytest.fixture(scope="module")
def manifest():
    """clips_manifest.json sits next to clips_all.h and is the export's own
    record of clip id/name/frame_count/duration_ms. Validate the parsed
    clips against it rather than hardcoding counts — the clip set changes
    as animations are authored."""
    import json

    path = DEFAULT_CLIPS_H.parent / "clips_manifest.json"
    return json.loads(path.read_text())["clips"]


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
    """Spot-check FR shoulder first frame of 'lie down and stand up'.

    a[0] = FR shoulder ≈ 45.71 — FR neutral (45) plus the small rest-yaw
    residual under the uniform-math-space yaw convention (the exporter cancels
    the rig bone axis_sign). Earlier values: ~14.29 (delta bug), then ~44.29
    before the axis-cancel fix. See docs/CLIP_SHOULDER_CONVENTION.md.
    """
    clip = get_clip_by_name(all_clips, "lie down and stand up")
    assert abs(clip.frames[0].a[0] - 45.7052) < 0.01


def test_get_clip_by_name_case_insensitive(all_clips):
    clip = get_clip_by_name(all_clips, "Tiny Wiggle")
    assert clip.name == "tiny wiggle"


def test_get_clip_by_name_not_found(all_clips):
    with pytest.raises(KeyError):
        get_clip_by_name(all_clips, "nonexistent clip")
