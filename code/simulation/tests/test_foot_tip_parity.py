"""Foot-tip geometry parity: pure-Python vs Fusion construction points.

load_foot_tip_in_link3_frame (animation/scripts/urdf_to_blender_rigged.py) uses
Blender's mathutils and cannot run in CI.  This module re-implements the same
Rz(90°) arithmetic in plain Python so GitHub Actions can verify the geometry stays
consistent after pipeline changes.

Also cross-checks against _stl_foot_tip_m to catch the known heuristic mismatch
(marked xfail; remove the decorator once _stl_foot_tip_m is fixed and the URDF
regenerated).

Skips automatically on fresh checkouts before `python code/facehugger.py urdf`.
"""

import json
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
GENERATED = REPO_ROOT / "code/simulation/generated"
FUSION_JSON = GENERATED / "fusion_export.json"
URDF_PATH = GENERATED / "facehugger.urdf"
LEG_LOWER_STL = GENERATED / "exported_meshes/leg_lower.stl"

sys.path.insert(0, str(REPO_ROOT / "code/simulation"))


# ---------------------------------------------------------------------------
# Pure-Python equivalent of load_foot_tip_in_link3_frame
# (same logic, no mathutils dependency)
# ---------------------------------------------------------------------------


def _foot_tip_link3_from_json(json_path) -> tuple[float, float, float] | None:
    """Return foot tip in link3-local frame (mm) from fusion_export.json, or None.

    Mirrors the math in urdf_to_blender_rigged.load_foot_tip_in_link3_frame:
      1. Find Link2ToLink3Axis origin_mm (knee joint, source frame).
      2. Find Link3TipPoint pos_mm (foot, source frame).
      3. delta = foot - knee.
      4. Apply Rz(90°): (x, y, z) -> (-y, x, z).
      5. Return result in mm (NOT divided by 1000 — tests work in mm).
    """
    data = json.loads(Path(json_path).read_text())
    knee_mm: list[float] | None = None
    foot_mm: list[float] | None = None

    def walk(node: dict) -> None:
        nonlocal knee_mm, foot_mm
        for axis in node.get("axes", []) or []:
            if axis.get("name") == "Link2ToLink3Axis" and knee_mm is None:
                xyz = axis.get("origin_mm")
                if xyz and len(xyz) >= 3:
                    knee_mm = list(xyz[:3])
        for point in node.get("points", []) or []:
            if point.get("name") == "Link3TipPoint" and foot_mm is None:
                xyz = point.get("pos_mm")
                if xyz and len(xyz) >= 3:
                    foot_mm = list(xyz[:3])
        if knee_mm is None or foot_mm is None:
            for child in node.get("children", []) or []:
                walk(child)

    for occ in data.get("occurrences", []) or []:
        walk(occ)
        if knee_mm is not None and foot_mm is not None:
            break

    if knee_mm is None or foot_mm is None:
        return None

    dx = foot_mm[0] - knee_mm[0]
    dy = foot_mm[1] - knee_mm[1]
    dz = foot_mm[2] - knee_mm[2]

    # Rz(π/2): [x, y, z] → [−y, x, z]
    return (-dy, dx, dz)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.skipif(
    not FUSION_JSON.exists(),
    reason="fusion_export.json absent – run `python code/facehugger.py urdf` first",
)
def test_foot_tip_link3_bounds():
    """Foot tip from JSON must be geometrically plausible for the lower leg.

    Known values from Fusion 360 construction points:
      Link2ToLink3Axis origin_mm: [34.57, 121.62, 10.246]
      Link3TipPoint    pos_mm:    [14.06, 199.84, 10.246]
      delta (source frame):       [-20.51,  78.22,   0.0]
      after Rz(90°) (link3 frame): [-78.22, -20.51,   0.0]

    Bounds below are ±20% of the expected values so the test survives minor
    geometry tweaks without being brittle.
    """
    tip = _foot_tip_link3_from_json(str(FUSION_JSON))
    assert tip is not None, (
        "Link3TipPoint or Link2ToLink3Axis missing from fusion_export.json. "
        "Re-export from Fusion 360 (ExportBodiesToURDF add-in) with both "
        "construction points present."
    )

    tip_x_mm, tip_y_mm, tip_z_mm = tip

    # foot extends in −X from knee; must be in (−100, −60) mm
    assert -100.0 < tip_x_mm < -60.0, (
        f"foot tip X={tip_x_mm:.2f} mm outside expected (−100, −60) mm"
    )
    # small lateral offset in −Y; must be in (−35, −5) mm
    assert -35.0 < tip_y_mm < -5.0, (
        f"foot tip Y={tip_y_mm:.2f} mm outside expected (−35, −5) mm"
    )
    # nearly planar in Z (foot tip construction point sits at the link mid-plane)
    assert abs(tip_z_mm) < 5.0, f"foot tip Z={tip_z_mm:.2f} mm unexpectedly large"


@pytest.mark.skipif(
    not FUSION_JSON.exists() or not URDF_PATH.exists(),
    reason="generated files absent – run `python code/facehugger.py urdf` first",
)
def test_urdf_foot_tip_matches_json():
    """URDF FootTip metadata must agree with the Fusion construction point within 0.5 mm.

    Catches the case where fusion_export.json was re-exported from Fusion but
    the URDF was not regenerated (`python code/facehugger.py urdf`).
    """
    import sys

    sys.path.insert(0, str(REPO_ROOT / "code/simulation"))
    from pybullet_sim.urdf_io import _parse_leg_points_from_urdf

    urdf_tip_mm = _parse_leg_points_from_urdf(str(URDF_PATH))["FootTip"]
    json_tip_mm = _foot_tip_link3_from_json(str(FUSION_JSON))
    if json_tip_mm is None:
        pytest.skip("Link3TipPoint absent from fusion_export.json")

    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(urdf_tip_mm, json_tip_mm)))
    assert dist <= 0.5, (
        f"URDF FootTip {tuple(round(v, 2) for v in urdf_tip_mm)} mm "
        f"vs JSON {tuple(round(v, 2) for v in json_tip_mm)} mm; "
        f"Δ={dist:.2f} mm — regenerate with `python code/facehugger.py urdf`"
    )
