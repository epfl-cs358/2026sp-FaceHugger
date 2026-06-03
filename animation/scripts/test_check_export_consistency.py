# === Plain Python — no Blender required ===
# Run via pytest.
"""Unit tests for check_export_consistency.

`check_frame` is exercised against a fixed math-space row (wiggle frame 0) but
the expected servo values are computed dynamically from `_frame_to_servo` so
the test survives any CALIB / convention tweak without a fixture refresh.
"""

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).parent
ADDONS_DIR = SCRIPTS_DIR.parent / "addons"
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(ADDONS_DIR))

from check_export_consistency import (
    _frame_to_servo,
    _scale_from_neutral,
    check_convention,
    check_frame,
    check_standing_neutral,
    h_row_to_dict,
)

CONVENTION = {
    "neutral_joint_deg": {
        "fr": [45, -60, -37],
        "fl": [135, -60, -40],
        "br": [-45, -50, -50],
        "bl": [-135, -60, -35],
    },
    "scale": 0.6667,
}

H_ROW_F0 = [
    -1.0577,
    -32.2288,
    -67.1672,  # fl
    -1.0577,
    -32.2288,
    -67.1672,  # fr
    -1.0577,
    -32.2288,
    -67.1672,  # bl
    -1.0577,
    -32.2288,
    -67.1673,  # br
]


def test_h_row_to_dict_maps_bones_correctly():
    row = h_row_to_dict(H_ROW_F0)
    assert row["fl_link1"] == H_ROW_F0[0]
    assert row["fl_link2"] == H_ROW_F0[1]
    assert row["fl_link3"] == H_ROW_F0[2]
    assert row["fr_link1"] == H_ROW_F0[3]
    assert row["bl_link1"] == H_ROW_F0[6]
    assert row["br_link1"] == H_ROW_F0[9]


def test_check_frame_passes_when_js_matches_computed():
    """check_frame returns no errors when js_frame == _scale_from_neutral(row).
    Both sides are math-space floats now (post-CALIB-decouple); the expected
    values come from _scale_from_neutral itself so the test exercises the
    identity branch and survives any convention tweak."""
    row = h_row_to_dict(H_ROW_F0)
    js = _scale_from_neutral(row, CONVENTION)
    errors = check_frame(row, js, CONVENTION, frame_idx=0)
    assert errors == [], f"Unexpected errors: {errors}"


def test_check_frame_detects_mismatch():
    """check_frame surfaces a single error when one math-space value disagrees
    by more than the float tolerance."""
    row = h_row_to_dict(H_ROW_F0)
    js = _scale_from_neutral(row, CONVENTION)
    js["fr"] = [js["fr"][0] + 5.0, js["fr"][1], js["fr"][2]]  # corrupt one joint
    errors = check_frame(row, js, CONVENTION, frame_idx=0)
    assert len(errors) == 1
    assert "fr" in errors[0] and "hip" in errors[0]


def test_parse_js_frame_with_negative_floats():
    """parse_js_file accepts math-space negative floats (post-T:12 format)."""
    from check_export_consistency import parse_js_file
    import tempfile
    import os

    js_content = """// Wire shape: T:12 CMD_STREAM_FRAME
const CLIP = [
  { t: 0, fr:[44.2948,-41.4850,-57.1125], fl:[134.2948,-41.4850,-58.1124], br:[-45.7052,-38.1520,-61.4454], bl:[-135.7052,-41.4850,-56.4459] },
];"""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".js", delete=False) as f:
        f.write(js_content)
        path = f.name
    try:
        frames = parse_js_file(Path(path))
        assert len(frames) == 1, f"Expected 1 frame, got {len(frames)}"
        for j, want in enumerate([44.2948, -41.4850, -57.1125]):
            assert abs(frames[0]["fr"][j] - want) < 1e-6, (
                f"fr[{j}]: got {frames[0]['fr'][j]} want {want}"
            )
        assert abs(frames[0]["br"][0] - -45.7052) < 1e-6
    finally:
        os.unlink(path)


def test_parse_js_file_skips_stale_t4_files():
    """A legacy T:4 servo-space integer .js (no T:12 marker) raises
    StaleClipFormatError with a clear re-export hint, so check_clip can
    SKIP rather than fail noisily."""
    from check_export_consistency import StaleClipFormatError, parse_js_file
    import tempfile
    import os
    import pytest

    js_content = """// FaceHugger clip: fallingRobot
// Generated 2026-05-25 from Blender animation
const CLIP = [
  { t: 0, fr:[91,131,33], fl:[89,49,148], br:[89,52,151], bl:[91,131,34] },
];"""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".js", delete=False) as f:
        f.write(js_content)
        path = f.name
    try:
        with pytest.raises(StaleClipFormatError) as exc:
            parse_js_file(Path(path))
        msg = str(exc.value)
        assert "Re-export" in msg or "re-export" in msg
    finally:
        os.unlink(path)


def test_check_convention_passes_with_current_neutral():
    """convention.json's NEUTRAL must put every shoulder at servo 90 and
    every hip/knee off 90 (the 'servo 90 = outward' guarantee)."""
    errors = check_convention(CONVENTION)
    assert errors == [], f"convention check should pass, got: {errors}"


def test_check_convention_detects_shoulder_drift():
    """If a shoulder neutral drifts so translateToServo != 90, fail loudly.
    fl shoulder neutral 135 -> 75 reintroduces the pre-Change-B clamp bug."""
    drifted = {
        "neutral_joint_deg": {**CONVENTION["neutral_joint_deg"], "fl": [75, -60, -40]},
        "scale": CONVENTION["scale"],
    }
    errors = check_convention(drifted)
    assert any("fl shoulder" in e for e in errors), (
        f"expected an fl shoulder drift error, got: {errors}"
    )


def test_standing_neutral_shoulders_90_hipknee_not_90():
    """Standing pose: shoulders servo 90, hip/knee at NEUTRAL[] values (NOT 90)."""
    servo = _frame_to_servo_at_neutral(CONVENTION)
    for leg in ("fr", "fl", "br", "bl"):
        sh, hip, kn = servo[leg]
        assert sh == 90, f"{leg} shoulder should be 90 at standing, got {sh}"
        assert hip != 90 and kn != 90, (
            f"{leg} hip/knee should NOT be 90 at standing, got {hip}/{kn}"
        )
    assert check_standing_neutral(CONVENTION) == []


def _frame_to_servo_at_neutral(convention):
    """Helper: servo values when raw == NEUTRAL (the standing pose)."""

    neutral = convention["neutral_joint_deg"]
    row = {}
    for leg in ("fr", "fl", "br", "bl"):
        sh, th, kn = neutral[leg]
        row[f"{leg}_link1"], row[f"{leg}_link2"], row[f"{leg}_link3"] = sh, th, kn
    return _frame_to_servo(row, convention)


def test_body_rotation_uniform_mathspace_yaw():
    """Canon (convention PNG / DRAFT-delta-conventions.md §2): math-space +sh =
    CCW yaw, the SAME direction for all four legs ("unit circle, same rotation";
    the exact angles are illustrative, the direction is the convention).

    A pure body yaw rotates every foot the same way, so the exported math-space
    shoulder delta-from-neutral must be IDENTICAL for all four legs. The rig's
    link1 driver writes the bone-local Z rotation = the true CCW delta times the
    bone axis_sign (URDF link1 <axis z>: fr -1, fl +1, br +1, bl -1); the export
    must cancel that sign so math-space comes out uniform.

    Since BR's shoulder was un-mirrored (2026-05-25, slope -1 -> +1), all four
    shoulder SERVOS now move the same direction for a body yaw too. Regression
    for both the FL inversion and the BR-mirror.
    """
    import sys
    from pathlib import Path as _Path

    _lib = str(_Path(__file__).resolve().parents[1] / "lib")
    if _lib not in sys.path:
        sys.path.insert(0, _lib)
    from servo_math import (
        _link1_delta_to_absolute,
        _scale_from_neutral,
    )

    _fw = str(_Path(__file__).resolve().parents[2] / "code" / "simulation")
    if _fw not in sys.path:
        sys.path.insert(0, _fw)
    from firmware_port.exporter_parity import _frame_to_servo

    axis_sign = {"fr": -1, "fl": +1, "br": +1, "bl": -1}
    n = CONVENTION["neutral_joint_deg"]
    ccw = -12.0  # feet counter-rotate for a body CCW yaw

    row = {}
    for leg in ("fr", "fl", "br", "bl"):
        row[f"{leg}_link1"] = axis_sign[leg] * ccw  # raw bone-local reading
        row[f"{leg}_link2"] = n[leg][1]
        row[f"{leg}_link3"] = n[leg][2]
    _link1_delta_to_absolute(row, CONVENTION)
    scaled = _scale_from_neutral(row, CONVENTION)

    sh_delta = {leg: scaled[leg][0] - n[leg][0] for leg in ("fr", "fl", "br", "bl")}
    spread = max(sh_delta.values()) - min(sh_delta.values())
    assert spread < 1e-6, (
        f"math-space shoulder yaw must be uniform across all legs (convention "
        f"direction); got deltas {sh_delta}"
    )

    # Servo consequence after un-mirroring BR: ALL four move the same direction.
    servo = _frame_to_servo(row, CONVENTION, warn=False)
    base = _frame_to_servo(
        {**{f"{leg}_link1": n[leg][0] for leg in n}, **_neutral_pitch(n)}, CONVENTION
    )
    sdelta = {leg: servo[leg][0] - base[leg][0] for leg in ("fr", "fl", "br", "bl")}
    first = sdelta["fr"] > 0
    assert all((sdelta[leg] > 0) == first for leg in ("fl", "br", "bl")), (
        f"all four shoulder servos must move the same direction now; got {sdelta}"
    )


def _neutral_pitch(n):
    return {f"{leg}_link{j}": n[leg][j - 1] for leg in n for j in (2, 3)}
