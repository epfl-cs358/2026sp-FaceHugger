"""Tests for _primary_link_occurrence and _mesh_shift helper functions.

Both live in urdf_gen/generate_urdf.py and are loaded via importlib to avoid
pulling in heavy optional deps (pybullet, etc.) at import time.
"""

import importlib.util
from pathlib import Path


_GU_PATH = Path(__file__).parents[1] / "urdf_gen/generate_urdf.py"
_spec = importlib.util.spec_from_file_location("_generate_urdf", _GU_PATH)
_gu = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_gu)

_primary_link_occurrence = _gu._primary_link_occurrence
_mesh_shift = _gu._mesh_shift


# ---------------------------------------------------------------------------
# _primary_link_occurrence
# ---------------------------------------------------------------------------


def test_primary_link_occurrence_none_input():
    assert _primary_link_occurrence(None) is None


def test_primary_link_occurrence_empty_dict():
    assert _primary_link_occurrence({}) is None


def test_primary_link_occurrence_body_rule():
    entry = {
        "source_type": "body",
        "source_occurrences": ["RootComp:1/Chassis:1", "RootComp:1/Chassis:2"],
    }
    assert _primary_link_occurrence(entry) == "RootComp:1/Chassis:1"


def test_primary_link_occurrence_combined_rule():
    entry = {
        "source_type": "combined",
        "parts": [
            {"occurrence": "RootComp:1/Shoulder:1"},
            {"occurrence": "RootComp:1/HipServo:1"},
        ],
    }
    assert _primary_link_occurrence(entry) == "RootComp:1/Shoulder:1"


def test_primary_link_occurrence_combined_empty_parts():
    entry = {
        "source_type": "combined",
        "parts": [],
    }
    assert _primary_link_occurrence(entry) is None


def test_primary_link_occurrence_body_empty_source_occurrences():
    entry = {
        "source_type": "body",
        "source_occurrences": [],
    }
    assert _primary_link_occurrence(entry) is None


def test_primary_link_occurrence_combined_part_missing_occurrence_key():
    entry = {
        "source_type": "combined",
        "parts": [{"name": "Shoulder"}],  # no 'occurrence' key
    }
    assert _primary_link_occurrence(entry) is None


# ---------------------------------------------------------------------------
# _mesh_shift
# ---------------------------------------------------------------------------


def test_mesh_shift_missing_mesh_name():
    export = {"mesh_files": {"other_mesh.stl": {"origin_shift_mm": [1.0, 2.0, 3.0]}}}
    assert _mesh_shift(export, "missing.stl") == [0.0, 0.0, 0.0]


def test_mesh_shift_entry_no_origin_shift():
    export = {"mesh_files": {"chassis.stl": {"source_type": "body"}}}
    assert _mesh_shift(export, "chassis.stl") == [0.0, 0.0, 0.0]


def test_mesh_shift_returns_actual_shift():
    export = {"mesh_files": {"leg.stl": {"origin_shift_mm": [10.0, -5.0, 3.5]}}}
    assert _mesh_shift(export, "leg.stl") == [10.0, -5.0, 3.5]


def test_mesh_shift_no_mesh_files_key():
    export = {}
    assert _mesh_shift(export, "chassis.stl") == [0.0, 0.0, 0.0]


def test_mesh_shift_returns_copy():
    stored = [10.0, -5.0, 3.5]
    export = {"mesh_files": {"leg.stl": {"origin_shift_mm": stored}}}
    result = _mesh_shift(export, "leg.stl")
    assert result == stored
    assert result is not stored
