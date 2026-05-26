"""SIL clip suite — golden-trace regression for every firmware clip.

Replays each clip through the EXACT firmware code (fh_sim) and asserts its full
servo-angle trace matches the committed golden under firmware_sil/golden/. This
is the exporter/firmware guard: a firmware change (or a clip re-export) that
alters ANY servo angle by >=1 deg fails here, so it can't reach the robot
unreviewed. Regenerate goldens deliberately with `python -m firmware_sil.gen_golden`.

Skips if fh_sim isn't built (no C++ toolchain). Build: see firmware_sil/README.md.
"""

import json
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

SIM_DIR = Path(__file__).resolve().parent
GOLDEN_DIR = SIM_DIR / "firmware_sil" / "golden"
GOLDEN_FILES = sorted(p for p in GOLDEN_DIR.glob("*.json") if p.name != "index.json")


def _fh_or_skip():
    try:
        from firmware_sil.sil_bridge import load_fh_sim

        return load_fh_sim()
    except ImportError as e:
        pytest.skip(f"fh_sim not built: {e}")


@pytest.mark.parametrize("golden_path", GOLDEN_FILES, ids=lambda p: p.stem)
def test_clip_matches_golden_trace(golden_path):
    """The clip's servo-angle trace is bit-identical to its committed golden."""
    from firmware_sil.sil_bridge import trace_clip

    fh = _fh_or_skip()
    golden = json.loads(golden_path.read_text())
    fc = fh.FirmwareControl()
    got = trace_clip(fc, golden["clip"], record_every=golden["record_every"])

    assert len(got) == len(golden["samples"]), (
        f"{golden['clip']}: sample count {len(got)} != golden {len(golden['samples'])}"
    )
    for (gt_ms, g_angles), (t_ms, angles) in zip(golden["samples"], got):
        assert t_ms == gt_ms, f"{golden['clip']}: t_ms {t_ms} != golden {gt_ms}"
        if angles != g_angles:
            diffs = [
                (i, g, a) for i, (g, a) in enumerate(zip(g_angles, angles)) if g != a
            ]
            pytest.fail(
                f"{golden['clip']} @ t={t_ms}ms: servo angles changed vs golden "
                f"(idx, golden, got): {diffs}\n"
                f"If this is an intentional firmware/clip change, regenerate with "
                f"`python -m firmware_sil.gen_golden` and review the diff."
            )


@pytest.mark.parametrize("golden_path", GOLDEN_FILES, ids=lambda p: p.stem)
def test_clip_angles_in_range(golden_path):
    """No servo in any clip is commanded outside the electrical [0,180] range."""
    golden = json.loads(golden_path.read_text())
    for t_ms, angles in golden["samples"]:
        for i, a in enumerate(angles):
            assert 0 <= a <= 180, (
                f"{golden['clip']} @ t={t_ms}ms servo[{i}]={a} out of [0,180]"
            )


def test_every_firmware_clip_has_a_golden():
    """A newly added firmware clip must ship a golden (no silent gaps in coverage)."""
    fh = _fh_or_skip()
    fc = fh.FirmwareControl()
    covered = {json.loads(p.read_text())["clip"] for p in GOLDEN_FILES}
    missing = [n for n in fc.clip_names() if n not in covered]
    assert not missing, (
        f"clips without a golden trace: {missing} — run `python -m firmware_sil.gen_golden`"
    )
