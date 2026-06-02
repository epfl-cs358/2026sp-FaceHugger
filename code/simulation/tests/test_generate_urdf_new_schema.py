"""Tests for generate_urdf.py new JSON schema field name support.

New schema uses:
  - world_transform  (was world_transform_rm_cm)
  - axis_dir_world   (was axis_dir_local_unit)

Old names are still accepted as fallback.
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from urdf_gen.lib.fusion_export import _find_occ_rot
from urdf_gen.lib.mesh_servo import _servo_rot_by_role
from urdf_gen.lib.physics_fallback import get_physics


def _cad_axis_dir(cad: dict):
    return cad.get("axis_dir_world") or cad.get("axis_dir_local_unit")


FIXTURES = Path(__file__).parent / "fixtures"


# ---------------------------------------------------------------------------
# Test 1: world_transform accepted by _servo_rot_by_role and _find_occ_rot
# ---------------------------------------------------------------------------


def _identity_4x4():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _identity_3x3():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def test_servo_rot_by_role_accepts_world_transform():
    export = {
        "occurrences": [{"name": "Servo:1", "world_transform": _identity_4x4()}],
        "mesh_files": {
            "_servo_role_assignment": {"shoulder": "Servo:1"},
        },
    }
    result = _servo_rot_by_role(export, "shoulder")
    assert result == _identity_3x3()


def test_find_occ_rot_accepts_world_transform():
    occs = [{"name": "Servo:1", "world_transform": _identity_4x4()}]
    result = _find_occ_rot(occs, "Servo:1")
    assert result == _identity_3x3()


# ---------------------------------------------------------------------------
# Test 2: axis_dir_world read from new-schema joints fixture
# ---------------------------------------------------------------------------


def test_axis_dir_world_in_fixture():
    data = json.loads((FIXTURES / "scene_dump_minimal.json").read_text())
    for joint in data["joints"]:
        assert _cad_axis_dir(joint) is not None
        assert len(_cad_axis_dir(joint)) == 3
    link1 = next(j for j in data["joints"] if j["name"] == "Link1Revolute")
    assert _cad_axis_dir(link1) == [0.0, 0.0, -1.0]


def test_axis_dir_world_preferred_over_local():
    # New-schema joint: axis_dir_world takes precedence
    assert _cad_axis_dir({"axis_dir_world": [0.0, 0.0, -1.0]}) == [0.0, 0.0, -1.0]
    # Old-schema joint: axis_dir_local_unit used as fallback
    assert _cad_axis_dir({"axis_dir_local_unit": [0.0, 1.0, 0.0]}) == [0.0, 1.0, 0.0]
    # Both present: axis_dir_world wins
    assert _cad_axis_dir(
        {"axis_dir_world": [0.0, 0.0, -1.0], "axis_dir_local_unit": [0.0, 1.0, 0.0]}
    ) == [0.0, 0.0, -1.0]
    # Neither: returns None
    assert _cad_axis_dir({}) is None


# ---------------------------------------------------------------------------
# Test 3: get_physics(None, ...) triggers UserWarning and returns fallback
# ---------------------------------------------------------------------------


def test_get_physics_none_warns_and_returns_fallback():
    with pytest.warns(UserWarning, match="No physics data"):
        mass, com, inertia = get_physics(None, comp_name="PCB_ESP32")

    assert mass == pytest.approx(0.05)
    assert com == [0, 0, 0]
    assert inertia["ixx"] == pytest.approx(1e-5)


# ---------------------------------------------------------------------------
# Test 4: get_physics with physics.error triggers UserWarning and returns fallback
# ---------------------------------------------------------------------------


def test_get_physics_error_node_warns_and_returns_fallback():
    node = {"physics": {"error": "no material"}}
    with pytest.warns(UserWarning, match="No physics data"):
        mass, com, inertia = get_physics(node, comp_name="PCB_ESP32")

    assert mass == pytest.approx(0.05)
    assert com == [0, 0, 0]
    assert inertia["iyy"] == pytest.approx(1e-5)
