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

from check_export_consistency import h_row_to_dict, check_frame

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
JS_F0 = {
    "fr": [59, 131, 33],
    "fl": [0, 49, 148],
    "br": [61, 52, 151],
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
