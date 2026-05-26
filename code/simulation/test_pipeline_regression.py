"""End-to-end regression net for the simulation pipeline.

Exercises the features documented in docs/.work/pybullet-sim/REPORT.md through
the *stable* interfaces (the `facehugger.py` CLI and the helper scripts), so they
keep working byte-for-byte across the code/simulation/ reorganization into
pybullet_sim/ (runtime) + urdf_gen/ (build).

These are characterization tests: they pass on the pre-migration layout and must
keep passing after. They drive subprocesses rather than importing modules, so a
change of internal import paths can't silently make them lie.

Requires pybullet (conda env `facehugger`); the module skips if it is absent.
"""

import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent
FACEHUGGER = SIM_DIR / "facehugger.py"
URDF = SIM_DIR / "generated" / "facehugger.urdf"
CLIP = "wave"


def _run(args, cwd=SIM_DIR, timeout=120):
    return subprocess.run(
        [sys.executable, *[str(a) for a in args]],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _find(name):
    """Locate a script by name anywhere under SIM_DIR (layout-robust)."""
    hits = [p for p in SIM_DIR.rglob(name) if "__pycache__" not in p.parts]
    assert hits, f"{name} not found under {SIM_DIR}"
    return hits[0]


def test_urdf_regenerates_byte_identical():
    """`facehugger.py urdf` reproduces the committed URDF exactly (generation is
    deterministic and the checked-in artifact is in sync)."""
    before = URDF.read_bytes()
    r = _run([FACEHUGGER, "urdf"])
    assert r.returncode == 0, r.stderr
    assert URDF.read_bytes() == before


def test_cli_help_lists_subcommands_without_view():
    r = _run([FACEHUGGER, "--help"])
    assert r.returncode == 0, r.stderr
    for cmd in ("urdf", "sim", "blender", "all"):
        assert cmd in r.stdout, f"missing subcommand {cmd}"
    assert "view" not in r.stdout  # removed this session


def test_clip_playback_headless_runs():
    """Full interpreter + pybullet pipeline: load clips_all.h, build config from
    the URDF, connect DIRECT, play the clip, disconnect."""
    r = _run([FACEHUGGER, "sim", "--clip", CLIP, "--headless", "--settle", "0"])
    assert r.returncode == 0, r.stderr
    assert CLIP in r.stdout  # "[clip] playing 'wave' (... ms)"


def test_clip_monitor_prints_current_status():
    r = _run(
        [FACEHUGGER, "sim", "--clip", CLIP, "--headless", "--settle", "0", "--monitor"]
    )
    assert r.returncode == 0, r.stderr
    assert "est_I=" in r.stdout  # the torque/current status line


def test_parity_check_passes():
    script = _find("verify_export_parity.py")
    r = _run([script])
    assert r.returncode == 0, r.stdout + r.stderr
    assert "agree" in r.stdout
