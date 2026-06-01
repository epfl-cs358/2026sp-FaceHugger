"""Shared fixtures for firmware_port/tests/.

The tests parse `animation/exported_clips/clips_all.h` and walk it as a real
clip catalog. The set of clips changes whenever the user re-exports from
Blender, so tests should depend on *catalog properties* — "there is at
least one clip", "the first clip has 12 floats per frame" — and let the
fixture pick which specific clip to exercise.

`sample_clip` is the canonical "give me any real clip" handle for any test
that just wants real frame data without caring which clip it is.
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from firmware_port.clip_loader import DEFAULT_CLIPS_H, load_clips_all_h


@pytest.fixture(scope="module")
def all_clips():
    return load_clips_all_h(DEFAULT_CLIPS_H)


@pytest.fixture(scope="module")
def manifest():
    """clips_manifest.json — the exporter's own record of clip
    id/name/frame_count/duration_ms. Use it to validate parsing without
    hardcoding any counts."""
    path = DEFAULT_CLIPS_H.parent / "clips_manifest.json"
    return json.loads(path.read_text())["clips"]


@pytest.fixture(scope="module")
def sample_clip(all_clips):
    """An arbitrary clip from the current catalog — for tests that exercise
    real frame data but don't care which clip in particular. Prefers a clip
    whose name contains at least one letter (so case-insensitive lookup
    tests aren't degenerate); falls back to the first clip otherwise.
    Skips when the catalog is empty."""
    if not all_clips:
        pytest.skip("clips_all.h has no clips")
    for c in all_clips:
        if any(ch.isalpha() for ch in c.name):
            return c
    return all_clips[0]
