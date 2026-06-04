"""Shared fixtures for code/simulation/tests/.

All sim tests live here now (animation_tools, pybullet_sim, urdf_gen, SIL).
Sub-package tests folded in during Phase 6 of the sim-reorg.

Fixtures:

  - `all_clips`, `manifest`, `sample_clip` — clip catalog access. Parsed
    from animation/exported_clips/clips_all.h. Use `sample_clip` when a
    test wants real frame data but doesn't care which clip it is — the
    catalog churns as animations are authored.

  - `fh_sim`, `firmware_control`, `sil_driver` — SIL build gates. Each
    one skips the test cleanly when the compiled `fh_sim` module isn't
    built. Use these instead of inline `_fc_or_skip()` / `_driver_or_skip()`
    helpers when writing new tests.

  - `pybullet_direct` — PyBullet DIRECT-mode connection that auto-disconnects
    after the test runs. Wraps the `p.connect(p.DIRECT) ... p.disconnect()`
    boilerplate that many SIL tests repeat by hand.
"""

import json
import sys
from pathlib import Path

import pytest

# Put code/simulation/ on sys.path so the test files can import
# animation_tools, pybullet_sim, firmware_sil, urdf_gen without per-file
# sys.path tinkering. (Tests retain their own inserts as defensive
# fallback for direct-script invocation.)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from animation_tools.clip_loader import DEFAULT_CLIPS_H, load_clips_all_h


# ── Clip catalog ────────────────────────────────────────────────────────────


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


# ── SIL build gate ──────────────────────────────────────────────────────────


@pytest.fixture(scope="session")
def fh_sim():
    """The compiled fh_sim pybind11 module, or skip if it isn't built.
    Session-scoped — `load_fh_sim()` is idempotent but importing the
    compiled module once per test is wasteful."""
    try:
        from firmware_sil.sil_bridge import _compiled_so, load_fh_sim
    except ImportError as e:
        pytest.skip(f"fh_sim not importable: {e}")
    if _compiled_so() is None:
        pytest.skip("fh_sim not built")
    return load_fh_sim()


@pytest.fixture
def firmware_control(fh_sim):
    """A fresh FirmwareControl instance per test."""
    return fh_sim.FirmwareControl()


@pytest.fixture
def sil_driver():
    """A FirmwareSILDriver — auto-builds fh_sim if stale, skips on
    ImportError. The driver wraps a FirmwareControl plus the PyBullet
    joint-targets bridge."""
    try:
        from firmware_sil.sil_bridge import FirmwareSILDriver

        return FirmwareSILDriver()
    except ImportError as e:
        pytest.skip(f"fh_sim not built: {e}")


# ── PyBullet ────────────────────────────────────────────────────────────────


@pytest.fixture
def pybullet_direct():
    """PyBullet DIRECT-mode connection, auto-disconnects after the test."""
    pytest.importorskip("pybullet")
    import pybullet as p_

    cid = p_.connect(p_.DIRECT)
    try:
        yield cid
    finally:
        if p_.isConnected(physicsClientId=cid):
            p_.disconnect(physicsClientId=cid)
