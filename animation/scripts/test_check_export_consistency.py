"""Unit test using wiggle clip frame 0 as a known fixture.

Frame 0 of wiggle.h (bone order: fl_link1..3, fr_link1..3, bl_link1..3, br_link1..3):
  .h row: [-1.0577, -32.2288, -67.1672, -1.0577, -32.2288, -67.1672,
           -1.0577, -32.2288, -67.1672, -1.0577, -32.2288, -67.1673]

Expected servo values after _frame_to_servo with current convention.json
(fl neutral shoulder=135, Change-B formula, scale=0.6667):
  fr:[59,131,33], fl:[0,49,148], br:[61,52,151], bl:[179,131,34]
  (fl hip clamps to 0 from -0.71 — the .h raw fl shoulder -1.0577 sits just
  below neutral 135 after scaling; not a fixture error, just a pose near the
  servo floor).

NOTE: wiggle.js and the other exported .js files on disk were generated under
an older convention (fl neutral shoulder=75, pre-Change-B fl formula) and will
NOT pass check_all_clips against the current convention.json.  The clips need
re-export before the smoke test can show all-PASS.
"""

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).parent
ADDONS_DIR = SCRIPTS_DIR.parent / "addons"
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(ADDONS_DIR))

from check_export_consistency import (
    check_convention,
    check_flat_pose,
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
# Expected values computed from H_ROW_F0 via current _frame_to_servo + convention.json.
# fl hip = 0 (clamped from -0.71 — fl shoulder raw -1.0577 is just below neutral 135).
# br shoulder 61 -> 119 after BR un-mirror (2026-05-25): 90 + (sh + 45) not 90 - (sh + 45).
JS_F0 = {
    "fr": [59, 131, 33],
    "fl": [0, 49, 148],
    "br": [119, 52, 151],
    "bl": [179, 131, 34],
}


def test_h_row_to_dict_maps_bones_correctly():
    row = h_row_to_dict(H_ROW_F0)
    assert row["fl_link1"] == H_ROW_F0[0]
    assert row["fl_link2"] == H_ROW_F0[1]
    assert row["fl_link3"] == H_ROW_F0[2]
    assert row["fr_link1"] == H_ROW_F0[3]
    assert row["bl_link1"] == H_ROW_F0[6]
    assert row["br_link1"] == H_ROW_F0[9]


def test_check_frame_passes_for_wiggle_f0():
    row = h_row_to_dict(H_ROW_F0)
    errors = check_frame(row, JS_F0, CONVENTION, frame_idx=0)
    assert errors == [], f"Unexpected errors: {errors}"


def test_check_frame_detects_mismatch():
    row = h_row_to_dict(H_ROW_F0)
    bad_js = dict(JS_F0)
    bad_js["fr"] = [0, 131, 33]  # wrong hip value
    errors = check_frame(row, bad_js, CONVENTION, frame_idx=0)
    assert len(errors) == 1
    assert "fr" in errors[0] and "hip" in errors[0]


def test_parse_js_frame_with_negative_servo():
    """Ensure parse_js_file handles negative servo values (e.g. fl:[-1,52,147])."""
    from check_export_consistency import parse_js_file
    import tempfile
    import os

    js_content = """const CLIP = [
  { t: 0, fr:[59,131,33], fl:[-1,49,148], br:[61,52,151], bl:[179,131,34] },
];"""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".js", delete=False) as f:
        f.write(js_content)
        path = f.name
    try:
        frames = parse_js_file(Path(path))
        assert len(frames) == 1, f"Expected 1 frame, got {len(frames)}"
        assert frames[0]["fl"] == [-1, 49, 148], (
            f"Expected fl=[-1,49,148], got {frames[0]['fl']}"
        )
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


def test_flat_pose_all_12_servos_90():
    """Flat / calibration pose: every one of the 12 servos is 90."""
    assert check_flat_pose(CONVENTION) == []


def test_flat_and_standing_differ_on_hipknee():
    """Guard against conflating the two poses: they share shoulder=90 but
    flat hip/knee = 90 while standing hip/knee != 90 (e.g. FR thigh 90 vs 150)."""
    standing = _frame_to_servo_at_neutral(CONVENTION)
    # FR is representative: standing thigh=150, knee=53; flat thigh/knee=90.
    assert standing["fr"][1] != 90 and standing["fr"][2] != 90
    assert check_flat_pose(CONVENTION) == []  # flat hip/knee ARE 90


def _frame_to_servo_at_neutral(convention):
    """Helper: servo values when raw == NEUTRAL (the standing pose)."""
    from check_export_consistency import _frame_to_servo

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
    from check_export_consistency import _mod

    axis_sign = {"fr": -1, "fl": +1, "br": +1, "bl": -1}
    n = CONVENTION["neutral_joint_deg"]
    ccw = -12.0  # feet counter-rotate for a body CCW yaw

    row = {}
    for leg in ("fr", "fl", "br", "bl"):
        row[f"{leg}_link1"] = axis_sign[leg] * ccw  # raw bone-local reading
        row[f"{leg}_link2"] = n[leg][1]
        row[f"{leg}_link3"] = n[leg][2]
    _mod._link1_delta_to_absolute(row, CONVENTION)
    scaled = _mod._scale_from_neutral(row, CONVENTION)

    sh_delta = {leg: scaled[leg][0] - n[leg][0] for leg in ("fr", "fl", "br", "bl")}
    spread = max(sh_delta.values()) - min(sh_delta.values())
    assert spread < 1e-6, (
        f"math-space shoulder yaw must be uniform across all legs (convention "
        f"direction); got deltas {sh_delta}"
    )

    # Servo consequence after un-mirroring BR: ALL four move the same direction.
    servo = _mod._frame_to_servo(row, CONVENTION, warn=False)
    base = _mod._frame_to_servo(
        {**{f"{leg}_link1": n[leg][0] for leg in n}, **_neutral_pitch(n)}, CONVENTION
    )
    sdelta = {leg: servo[leg][0] - base[leg][0] for leg in ("fr", "fl", "br", "bl")}
    first = sdelta["fr"] > 0
    assert all((sdelta[leg] > 0) == first for leg in ("fl", "br", "bl")), (
        f"all four shoulder servos must move the same direction now; got {sdelta}"
    )


def _neutral_pitch(n):
    return {f"{leg}_link{j}": n[leg][j - 1] for leg in n for j in (2, 3)}
